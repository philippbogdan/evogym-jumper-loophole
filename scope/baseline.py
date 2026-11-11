"""Replay the controllers of the rerun baseline and look for the loophole in what they do.

For each of the bodies in results/baseline_robots.csv: one deterministic episode in Evolution Gym
as released (and, with --noise K, K more with the policy's own noise: torch seeds 0..K-1,
environment seed 42, as the benchmark's evaluation does). The measuring engine then replays the
commands of every episode and reports the deepest corner left inside the floor with no force on
it, the score of the same commands with the loophole closed, and the energy account.

    python scope/baseline.py --controllers DIR      # writes results/baseline_episodes.csv

DIR holds the runs as the benchmark's genetic algorithm writes them:
DIR/seedS/saved_data/jumper_ga_sS/generation_G/{structure/L.npz, controller/L.zip}.
The controllers are not part of this repository.
"""

import os, sys, csv, argparse, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'build', 'baseline')


def robots():
    return list(csv.DictReader(open(os.path.join(ROOT, 'results', 'baseline_robots.csv'))))


def paths(controllers, r):
    g = os.path.join(controllers, f"seed{r['seed']}", 'saved_data', f"jumper_ga_s{r['seed']}", f"generation_{r['generation']}")
    return os.path.join(g, 'structure', r['robot'] + '.npz'), os.path.join(g, 'controller', r['robot'] + '.zip')


def name(r):
    return f"seed{r['seed']}_g{int(r['generation']):02d}_r{int(r['robot']):02d}"


def episodes(job):
    """Closed-loop episodes of one controller in one build tree."""
    tree, controllers, r, K, keep = job
    out = os.path.join(OUT, tree, name(r) + '.npz')
    if os.path.exists(out):
        return out
    build = os.path.join(ROOT, 'build', tree)
    if build not in sys.path:
        sys.path.insert(0, build)
    import torch
    torch.set_num_threads(1)
    import gymnasium as gym
    import evogym.envs  # noqa: F401
    from stable_baselines3 import PPO
    structure, controller = paths(controllers, r)
    npz = np.load(structure)
    body, conn = npz['arr_0'], npz['arr_1']
    model = PPO.load(controller, device='cpu',
                     custom_objects={'learning_rate': 0.0, 'lr_schedule': lambda _: 0.0, 'clip_range': lambda _: 0.0})

    def episode(deterministic):
        env = gym.make('Jumper-v0', body=body, connections=conn)
        obs, _ = env.reset(seed=42)
        total, sent = 0.0, []
        while True:
            a, _ = model.predict(obs, deterministic=deterministic)
            a = np.asarray(a, dtype=np.float64)
            sent.append(a.copy())
            obs, reward, terminated, truncated, _ = env.step(a)
            total += reward
            if terminated or truncated:
                break
        env.close()
        return total, np.array(sent)

    res = dict(body=body.astype(np.int8), conn=conn.astype(np.int16))
    score, sent = episode(True)
    res['scores'] = [score]
    if keep:
        res['actions_0'] = sent
    for k in range(K):
        torch.manual_seed(k)
        score, sent = episode(False)
        res['scores'].append(score)
        if keep:
            res[f'actions_{k + 1}'] = sent
    res['scores'] = np.array(res['scores'])
    np.savez_compressed(out + '.tmp.npz', **res)
    os.replace(out + '.tmp.npz', out)
    return out


def measure(r):
    """Engine measurements on the episodes recorded in the released simulator."""
    sys.path.insert(0, os.path.join(ROOT, 'scripts'))
    from common import RULES, closed, engine
    from energy import NAMES
    S = np.load(os.path.join(OUT, 'stock', name(r) + '.npz'))
    body, conn = S['body'].astype(int), S['conn'].astype(int)
    R = RULES['base']
    e, es = engine(body, conn, R), engine(body, conn, closed(R))
    rows = []
    for k, score in enumerate(S['scores']):
        A = S[f'actions_{k}']
        A = A if len(A) == 500 else np.vstack([A, np.zeros((500 - len(A), A.shape[1]))])
        replay = float(e.rollout(A, 500, False)[0])
        point_steps, depth, _ = e.unpushed()
        same = float(es.rollout(A, 500, False)[0])
        E, terms, _, _ = e.account(A, 500)
        t = dict(zip(NAMES, np.asarray(terms).sum(0)[:13]))
        rows.append(dict(seed=r['seed'], generation=r['generation'], robot=r['robot'],
                         episode='deterministic' if k == 0 else f'noise seed {k - 1}', cells=r['cells'], fitness=r['fitness'],
                         released=repr(float(score)), engine_replay=repr(replay), same_commands_closed=repr(same),
                         unpushed_point_steps=int(point_steps), deepest_unpushed_m=repr(float(depth)),
                         floor_normal_work_J=repr(float(t['ground normal'])), muscle_work_J=repr(float(t['muscle spring work'])),
                         passive_spring_work_J=repr(float(t['passive spring work'])), energy_gained_J=repr(float(E[-1] - E[0]))))
    return rows


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--controllers', required=True, help='directory holding seed0, seed1, seed2 as the genetic algorithm wrote them')
    ap.add_argument('--noise', type=int, default=0, help='extra episodes with the policy\'s own noise per controller (default 0)')
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--top', type=int, default=0, help='only the N bodies of highest fitness (default: all)')
    ap.add_argument('--tree', help=argparse.SUPPRESS)
    a = ap.parse_args()
    import multiprocessing as mp
    rs = robots()
    if a.top:
        rs = sorted(rs, key=lambda r: -float(r['fitness']))[:a.top]
    if a.tree:
        os.makedirs(os.path.join(OUT, a.tree), exist_ok=True)
        with mp.get_context('spawn').Pool(a.workers) as pool:
            for i, _ in enumerate(pool.imap_unordered(episodes, [(a.tree, a.controllers, r, a.noise, a.tree == 'stock') for r in rs])):
                if i % 100 == 0:
                    print(a.tree, i, len(rs), flush=True)
    else:
        subprocess.run([sys.executable, os.path.abspath(__file__), '--controllers', a.controllers, '--noise', str(a.noise),
                        '--workers', str(a.workers), '--top', str(a.top), '--tree', 'stock'], check=True)      # the simulator lives in its own process
        rows = []
        with mp.get_context('spawn').Pool(a.workers) as pool:
            for part in pool.imap(measure, rs, chunksize=4):
                rows += part
        path = os.path.join(ROOT, 'results', 'baseline_episodes.csv')
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print('written', path, len(rows), 'episodes of', len(rs), 'bodies')
