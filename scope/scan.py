"""Run every robot the benchmark's authors released, in its own task, and count unpushed points.

One deterministic episode per robot, as evogym-datasets/robots_with_policies.py runs it, in the
simulator as released plus a counting probe (patches/probe.patch, which changes no arithmetic).
For each episode: how many point-steps a robot point spent strictly inside fixed terrain with no
contact force on it, and how deep the deepest such point was (its distance to the nearest surface
of the fixed terrain, the underside of the world excepted). One task, Catcher-v0, places its box
at a random offset: Python's generator is seeded with 0 before every episode.

    python scope/fetch.py
    python scope/scan.py                   # writes results/released_scan.csv
"""

import os, sys, io, csv, json, glob, ctypes, random, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REL = os.path.join(ROOT, 'build', 'released')
TREE = os.path.join(ROOT, 'build', 'probe')


def one(rob):
    for p in (TREE, os.path.join(REL, 'evogym-datasets')):
        if p not in sys.path:
            sys.path.insert(0, p)
    import torch
    torch.set_num_threads(1)
    import gymnasium as gym
    import evogym.envs  # noqa: F401
    import evogym
    from stable_baselines3.common.running_mean_std import RunningMeanStd  # noqa: F401  (needed to load the policies)
    from a2c_ppo_acktr.model import Policy  # noqa: F401  (needed to load the policies)
    probe = ctypes.CDLL(glob.glob(os.path.join(os.path.dirname(evogym.__file__), 'simulator_cpp*.so'))[0])
    body, conn = np.array(rob['body'], dtype=np.int64), np.array(rob['connections'], dtype=np.int64)
    out = dict(uid=rob['uid'], task=rob['env_name'], generated_by=rob['generated_by'], cells=int((body != 0).sum()),
               dataset_reward=repr(rob['reward']))
    try:
        blob = open(os.path.join(REL, 'policies', rob['uid'] + '.pt'), 'rb').read()
        policy, obs_rms = torch.load(io.BytesIO(blob), weights_only=False, map_location='cpu')
        random.seed(0)
        env = gym.make(rob['env_name'], body=body, connections=conn)
        hidden = torch.zeros(1, policy.recurrent_hidden_state_size)
        masks = torch.zeros(1, 1)
        probe.evogym_probe_reset()
        obs, _ = env.reset()
        total, t = 0.0, 0
        while True:
            with torch.no_grad():
                o = np.clip((obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8), -10.0, 10.0)
                _, action, _, hidden = policy.act(torch.tensor(o, dtype=torch.float32).unsqueeze(0), hidden, masks,
                                                  deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action + 1.0)
            done = terminated or truncated
            masks.fill_(0.0 if done else 1.0)
            total += reward
            t += 1
            if done:
                break
        steps, depth, pairs = ctypes.c_long(), ctypes.c_double(), ctypes.c_long()
        probe.evogym_probe_read(ctypes.byref(steps), ctypes.byref(depth), ctypes.byref(pairs))
        env.close()
        out.update(replay_reward=repr(float(total)), commands=t, ended_early=int(bool(terminated)),
                   unpushed_point_steps=steps.value, deepest_unpushed_m=repr(depth.value), error='')
    except Exception as e:      # one unreadable row must not lose the scan
        out.update(replay_reward='', commands=0, ended_early=0, unpushed_point_steps=0, deepest_unpushed_m='',
                   error=f'{type(e).__name__}: {e}'[:200])
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--task', help='scan one task only')
    a = ap.parse_args()
    index = json.load(open(os.path.join(REL, 'index.json')))
    if a.task:
        index = [r for r in index if r['env_name'] == a.task]
    import multiprocessing as mp
    rows = []
    with mp.get_context('spawn').Pool(a.workers, maxtasksperchild=40) as pool:
        for i, r in enumerate(pool.imap_unordered(one, index, chunksize=2)):
            rows.append(r)
            if i % 100 == 0:
                print(i, len(index), r['task'], r['error'], flush=True)
    rows.sort(key=lambda r: (r['task'], r['uid']))
    path = os.path.join(ROOT, 'results', 'released_scan.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('written', path, len(rows), 'robots;', sum(1 for r in rows if r['error']), 'could not be run')
