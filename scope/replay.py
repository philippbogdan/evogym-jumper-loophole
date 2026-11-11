"""Replay the Jumper-v0 robots released by the benchmark's authors, with their own policies.

Each robot runs as evogym-datasets/robots_with_policies.py runs it: observations normalised with
the stored statistics, deterministic actions, action + 1.0 sent to the environment. It runs in the
simulator as released and with the loophole closed, the policy reacting to whichever it is in.
Run 0 is that rollout exactly. Runs 1..N change only the first command, by k parts in a million:
a change that has nothing to do with contact, to show how far a score moves anyway.

The measuring engine then replays run 0 of the released simulator and reports how deep any corner
sat inside the floor with no force on it, and the episode's energy account.

    python scope/fetch.py
    python scope/replay.py                 # writes results/released_jumper.csv

Needs scope/requirements.txt as well as requirements.txt.
"""

import os, sys, io, csv, json, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REL = os.path.join(ROOT, 'build', 'released')


def robots():
    return [r for r in json.load(open(os.path.join(REL, 'index.json'))) if r['env_name'] == 'Jumper-v0']


def episodes(job):
    """Closed-loop episodes of one released robot in one build tree: scores, and the commands of run 0."""
    tree, rob, runs = job
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
    body, conn = np.array(rob['body'], dtype=np.int64), np.array(rob['connections'], dtype=np.int64)
    blob = open(os.path.join(REL, 'policies', rob['uid'] + '.pt'), 'rb').read()
    policy, obs_rms = torch.load(io.BytesIO(blob), weights_only=False, map_location='cpu')

    def episode(k):
        env = gym.make('Jumper-v0', body=body, connections=conn)
        hidden = torch.zeros(1, policy.recurrent_hidden_state_size)
        masks = torch.zeros(1, 1)
        obs, _ = env.reset()
        total, sent_all, t = 0.0, [], 0
        while True:
            with torch.no_grad():
                o = np.clip((obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8), -10.0, 10.0)
                _, action, _, hidden = policy.act(torch.tensor(o, dtype=torch.float32).unsqueeze(0), hidden, masks,
                                                  deterministic=True)
            a = action + 1.0                                   # a float32 tensor, as the released script sends it
            sent = np.asarray(a.clip(0.6, 1.6), dtype=np.float64).reshape(-1)   # what the simulator receives
            if k and t == 0:
                sent = np.where(sent > 1.1, sent * (1 - k * 1e-6), sent * (1 + k * 1e-6))
                a = sent.copy()
            sent_all.append(sent.copy())
            obs, reward, terminated, truncated, _ = env.step(a)
            done = terminated or truncated
            masks.fill_(0.0 if done else 1.0)
            total += reward
            t += 1
            if done:
                break
        env.close()
        return total, np.array(sent_all)

    scores, commands = [], None
    for k in range(runs + 1):
        s, sent = episode(k)
        scores.append(s)
        if k == 0:
            commands = sent
    return rob['uid'], np.array(scores), commands


def measure(job):
    """Engine measurements on run 0 of the released simulator."""
    rob, A = job
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    from common import RULES, closed, engine
    from energy import NAMES
    R = dict(RULES['base'])
    R['engine'] = dict(R['engine'], hi=float(np.float32(1.6)))      # commands arrive clipped in float32
    body, conn = np.array(rob['body']), np.array(rob['connections'])
    A = A if len(A) == 500 else np.vstack([A, np.zeros((500 - len(A), A.shape[1]))])
    e = engine(body, conn, R)
    replay = float(e.rollout(A, 500, False)[0])
    point_steps, depth, _ = e.unpushed()
    same = float(engine(body, conn, closed(R)).rollout(A, 500, False)[0])
    E, terms, _, _ = e.account(A, 500)
    t = dict(zip(NAMES, np.asarray(terms).sum(0)[:13]))
    return rob['uid'], dict(engine_replay=repr(replay), same_commands_closed=repr(same),
                            unpushed_point_steps=int(point_steps), deepest_unpushed_m=repr(float(depth)),
                            floor_normal_work_J=repr(float(t['ground normal'])), muscle_work_J=repr(float(t['muscle spring work'])),
                            passive_spring_work_J=repr(float(t['passive spring work'])), energy_gained_J=repr(float(E[-1] - E[0])))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', type=int, default=8, help='perturbed runs per robot and build (default 8)')
    ap.add_argument('--workers', type=int, default=4)
    a = ap.parse_args()
    import multiprocessing as mp
    ctx = mp.get_context('spawn')
    rs = robots()
    scores, commands = {}, {}
    for tree in ('stock', 'closed'):      # one pool per build: a process can hold only one build of the simulator
        with ctx.Pool(a.workers) as pool:
            for i, (uid, s, A) in enumerate(pool.imap_unordered(episodes, [(tree, r, a.runs) for r in rs])):
                scores[tree, uid] = s
                if tree == 'stock':
                    commands[uid] = A
                print(tree, i + 1, len(rs), uid, round(float(s[0]), 3), flush=True)
    with ctx.Pool(a.workers) as pool:
        measured = dict(pool.imap_unordered(measure, [(r, commands[r['uid']]) for r in rs]))
    rows = []
    for r in rs:
        u = r['uid']
        s, c = scores['stock', u], scores['closed', u]
        rows.append(dict(uid=u, generated_by=r['generated_by'], cells=int(np.count_nonzero(r['body'])),
                         dataset_reward=repr(r['reward']), released=repr(float(s[0])), closed=repr(float(c[0])),
                         released_perturbed=' '.join(repr(float(v)) for v in s[1:]),
                         closed_perturbed=' '.join(repr(float(v)) for v in c[1:]), **measured[u]))
    path = os.path.join(ROOT, 'results', 'released_jumper.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('written', path, len(rows), 'robots')
