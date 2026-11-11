"""Run released robots with their policies' own noise, as released and with the loophole closed.

For each robot: one deterministic episode, as evogym-datasets/robots_with_policies.py runs it,
and N episodes in which the policy samples its actions (torch seeds 0..N-1). Each runs in the
simulator as released (built with the counting probe, which changes no arithmetic) and with the
loophole closed, the policy reacting to whichever it is in. For every episode: the reward, how
high the centre of mass got above its start, and whether the episode ended early; as released,
also the deepest point left inside fixed terrain with no contact force on it.

    python scope/fetch.py
    python scope/noise.py --uid cca8b49a 51f55c20 --noise 200 --out released_launchers.csv
    python scope/noise.py --uid 1585210c 73613b25 1eead7ad 8145f538 --noise 10 --out released_deep_climber_v1.csv

--task NAME runs every released robot of a task instead.
"""

import os, sys, io, csv, json, glob, ctypes, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REL = os.path.join(ROOT, 'build', 'released')


def episodes(job):
    """The deterministic episode and N noisy ones of one robot in one build tree."""
    tree, rob, N = job
    for p in (os.path.join(ROOT, 'build', tree), os.path.join(REL, 'evogym-datasets')):
        if p not in sys.path:
            sys.path.insert(0, p)
    import torch
    torch.set_num_threads(1)
    import gymnasium as gym
    import evogym.envs  # noqa: F401
    import evogym
    assert os.path.realpath(evogym.__file__).startswith(os.path.realpath(os.path.join(ROOT, 'build', tree)))
    from stable_baselines3.common.running_mean_std import RunningMeanStd  # noqa: F401  (needed to load the policies)
    from a2c_ppo_acktr.model import Policy  # noqa: F401  (needed to load the policies)
    probe = None
    if tree == 'probe':
        probe = ctypes.CDLL(glob.glob(os.path.join(os.path.dirname(evogym.__file__), 'simulator_cpp*.so'))[0])
    body, conn = np.array(rob['body'], dtype=np.int64), np.array(rob['connections'], dtype=np.int64)
    blob = open(os.path.join(REL, 'policies', rob['uid'] + '.pt'), 'rb').read()
    policy, obs_rms = torch.load(io.BytesIO(blob), weights_only=False, map_location='cpu')
    out = dict(reward=[], rise=[], early=[], depth=[])
    for k in range(-1, N):                  # -1 is the deterministic episode
        env = gym.make(rob['env_name'], body=body, connections=conn)
        hidden = torch.zeros(1, policy.recurrent_hidden_state_size)
        masks = torch.zeros(1, 1)
        if probe:
            probe.evogym_probe_reset()
        obs, _ = env.reset()
        u = env.unwrapped
        height = lambda: float(np.mean(u.object_pos_at_time(u.get_time(), 'robot'), 1)[1])
        start = top = height()
        torch.manual_seed(max(k, 0))
        total = 0.0
        while True:
            with torch.no_grad():
                o = np.clip((obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8), -10.0, 10.0)
                _, action, _, hidden = policy.act(torch.tensor(o, dtype=torch.float32).unsqueeze(0), hidden, masks,
                                                  deterministic=(k < 0))
            obs, reward, terminated, truncated, _ = env.step(action + 1.0)
            done = terminated or truncated
            masks.fill_(0.0 if done else 1.0)
            total += reward
            top = max(top, height())
            if done:
                break
        out['reward'].append(float(total))
        out['rise'].append(top - start)
        out['early'].append(int(bool(terminated)))
        if probe:
            steps, depth, pairs = ctypes.c_long(), ctypes.c_double(), ctypes.c_long()
            probe.evogym_probe_read(ctypes.byref(steps), ctypes.byref(depth), ctypes.byref(pairs))
            out['depth'].append(depth.value)
        env.close()
    return tree, rob['uid'], out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--task', nargs='+', default=[], help='every released robot of these tasks')
    ap.add_argument('--uid', nargs='+', default=[], help='robots whose identifier starts with one of these')
    ap.add_argument('--noise', type=int, default=20, help='episodes with the policy\'s noise per robot and simulator (default 20)')
    ap.add_argument('--out', default='released_noise.csv', help='file name under results/')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--tree', help=argparse.SUPPRESS)
    a = ap.parse_args()
    index = json.load(open(os.path.join(REL, 'index.json')))
    robots = [r for r in index if r['env_name'] in a.task or any(r['uid'].startswith(u) for u in a.uid)]
    part = lambda tree: os.path.join(ROOT, 'build', 'noise_' + tree + '_' + a.out + '.json')
    if a.tree:                              # one build of the simulator per process
        import multiprocessing as mp
        got = {}
        with mp.get_context('spawn').Pool(a.workers, maxtasksperchild=20) as pool:
            for i, (tree, uid, out) in enumerate(pool.imap_unordered(episodes, [(a.tree, r, a.noise) for r in robots])):
                got[uid] = out
                if i % 50 == 0:
                    print(a.tree, i, len(robots), flush=True)
        json.dump(got, open(part(a.tree), 'w'))
        sys.exit(0)
    import subprocess
    which = (['--task'] + a.task if a.task else []) + (['--uid'] + a.uid if a.uid else [])
    running = [subprocess.Popen([sys.executable, os.path.abspath(__file__), '--tree', tree, '--noise', str(a.noise), '--out', a.out,
                                 '--workers', str(max(1, a.workers // 2))] + which)
               for tree in ('probe', 'closed')]
    if any(p.wait() for p in running):
        sys.exit('a run failed')
    P, C = json.load(open(part('probe'))), json.load(open(part('closed')))
    fmt = lambda v, d: ' '.join(f'{x:.{d}f}' for x in v)
    rows = []
    for r in sorted(robots, key=lambda r: (r['env_name'], r['uid'])):
        p, c = P[r['uid']], C[r['uid']]
        rows.append(dict(uid=r['uid'], task=r['env_name'], generated_by=r['generated_by'], cells=int((np.array(r['body']) != 0).sum()),
                         body='/'.join(''.join(str(v) for v in row) for row in r['body']),
                         dataset_reward=repr(r['reward']),
                         released=repr(p['reward'][0]), released_rise_m=repr(p['rise'][0]), released_ended_early=p['early'][0],
                         released_deepest_unpushed_m=repr(p['depth'][0]),
                         closed=repr(c['reward'][0]), closed_rise_m=repr(c['rise'][0]), closed_ended_early=c['early'][0],
                         released_noise=fmt(p['reward'][1:], 4), released_noise_rise_m=fmt(p['rise'][1:], 3),
                         released_noise_ended_early=' '.join(map(str, p['early'][1:])),
                         released_noise_deepest_unpushed_m=fmt(p['depth'][1:], 4),
                         closed_noise=fmt(c['reward'][1:], 4), closed_noise_rise_m=fmt(c['rise'][1:], 3),
                         closed_noise_ended_early=' '.join(map(str, c['early'][1:]))))
    path = os.path.join(ROOT, 'results', a.out)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('written', path, len(rows), 'robots,', 1 + a.noise, 'episodes each in each simulator')
