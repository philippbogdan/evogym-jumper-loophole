"""Run every robot the benchmark's authors released, in its own task, as released and with each closing.

One deterministic episode per robot and simulator, as evogym-datasets/robots_with_policies.py runs it:
as released, with patches/close-loophole.patch, and with patches/close-loophole-top.patch, the last two
with counters that change no arithmetic (patches/probe-fix.patch, patches/probe-top.patch). For each
closing: the score, how many times it pushed a point that the released simulator leaves without a force,
and the deepest point it pushed (its distance to the edge it was pushed through). Python's generator is
seeded with 0 before every episode, as in scan.py.

    python scope/fetch.py
    python scope/fix.py                    # writes results/released_fix.csv
"""

import os, sys, io, csv, json, glob, ctypes, random, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REL = os.path.join(ROOT, 'build', 'released')
TREES = {'released': 'stock', 'fixed': 'closed_probe', 'top': 'top_probe'}


def one(job):
    tree, rob = job
    for p in (os.path.join(ROOT, 'build', tree), os.path.join(REL, 'evogym-datasets')):
        if p not in sys.path:
            sys.path.insert(0, p)
    import torch
    torch.set_num_threads(1)
    import gymnasium as gym
    import evogym.envs  # noqa: F401
    import evogym
    assert os.path.realpath(evogym.__file__).startswith(os.path.realpath(os.path.join(ROOT, 'build', tree)) + os.sep)
    from stable_baselines3.common.running_mean_std import RunningMeanStd  # noqa: F401  (needed to load the policies)
    from a2c_ppo_acktr.model import Policy  # noqa: F401  (needed to load the policies)
    probe = None
    if tree != 'stock':
        probe = ctypes.CDLL(glob.glob(os.path.join(os.path.dirname(evogym.__file__), 'simulator_cpp*.so'))[0])
    body, conn = np.array(rob['body'], dtype=np.int64), np.array(rob['connections'], dtype=np.int64)
    try:
        blob = open(os.path.join(REL, 'policies', rob['uid'] + '.pt'), 'rb').read()
        policy, obs_rms = torch.load(io.BytesIO(blob), weights_only=False, map_location='cpu')
        random.seed(0)
        env = gym.make(rob['env_name'], body=body, connections=conn)
        hidden = torch.zeros(1, policy.recurrent_hidden_state_size)
        masks = torch.zeros(1, 1)
        if probe:
            probe.evogym_fallback_reset()
        obs, _ = env.reset()
        total = 0.0
        while True:
            with torch.no_grad():
                o = np.clip((obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8), -10.0, 10.0)
                _, action, _, hidden = policy.act(torch.tensor(o, dtype=torch.float32).unsqueeze(0), hidden, masks,
                                                  deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action + 1.0)
            done = terminated or truncated
            masks.fill_(0.0 if done else 1.0)
            total += reward
            if done:
                break
        count, depth = ctypes.c_long(0), ctypes.c_double(0.0)
        if probe:
            probe.evogym_fallback_read(ctypes.byref(count), ctypes.byref(depth))
        env.close()
        return rob['uid'], dict(reward=repr(float(total)), pushes=count.value, deepest=repr(depth.value), error='')
    except Exception as e:      # one unreadable row must not lose the scan
        return rob['uid'], dict(reward='', pushes=0, deepest='', error=f'{type(e).__name__}: {e}'[:200])


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--workers', type=int, default=4)
    a = ap.parse_args()
    index = json.load(open(os.path.join(REL, 'index.json')))
    import multiprocessing as mp
    got = {}
    for name, tree in TREES.items():        # one pool per build: a process can hold only one build of the simulator
        with mp.get_context('spawn').Pool(a.workers, maxtasksperchild=40) as pool:
            for i, (uid, r) in enumerate(pool.imap_unordered(one, [(tree, rob) for rob in index], chunksize=2)):
                got[name, uid] = r
                if i % 100 == 0:
                    print(name, i, len(index), r['error'], flush=True)
    rows = []
    for rob in index:
        u = rob['uid']
        R, F, T = (got[k, u] for k in TREES)
        rows.append(dict(uid=u, task=rob['env_name'], generated_by=rob['generated_by'], cells=int((np.array(rob['body']) != 0).sum()),
                         released=R['reward'], fixed=F['reward'], fixed_pushes=F['pushes'], fixed_deepest_m=F['deepest'],
                         top=T['reward'], top_pushes=T['pushes'], top_deepest_m=T['deepest'],
                         error=R['error'] or F['error'] or T['error']))
    rows.sort(key=lambda r: (r['task'], r['uid']))
    path = os.path.join(ROOT, 'results', 'released_fix.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('written', path, len(rows), 'robots;', sum(1 for r in rows if r['error']), 'could not be run')
