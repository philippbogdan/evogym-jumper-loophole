"""Run the two-cell robot, with its commands unchanged, in every task of the benchmark.

The body and the 500 commands of designs/best-stock were found for Jumper-v0. Here they are
replayed as they are in each of the benchmark's tasks, in the simulator as released (built with
the counting probe, which changes no arithmetic) and with the loophole closed. Where a task
runs for more than 500 commands, the commands start again from the first. One task, Catcher-v0,
places its box at a random offset: Python's generator is seeded with 0 before every episode.

    python scope/tasks.py                  # writes results/other_tasks.csv

The columns for the released robots come from results/released.json, which scope/fetch.py writes.
"""

import os, sys, csv, json, glob, ctypes, random, argparse, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DESIGN = os.path.join(ROOT, 'designs', 'best-stock')


def episodes(tree):
    """One episode per task in one build tree; a list of rows."""
    sys.path.insert(0, os.path.join(ROOT, 'build', tree))
    import gymnasium as gym
    import evogym.envs  # noqa: F401
    import evogym
    probe = None
    if tree == 'probe':
        probe = ctypes.CDLL(glob.glob(os.path.join(os.path.dirname(evogym.__file__), 'simulator_cpp*.so'))[0])
    body = np.array(json.load(open(os.path.join(DESIGN, 'design.json')))['body'])
    A = np.load(os.path.join(DESIGN, 'actions.npy'))
    tasks = sorted(k for k, v in gym.registry.items() if str(v.entry_point).startswith('evogym.envs'))
    rows = []
    for task in tasks:
        random.seed(0)
        env = gym.make(task, body=body, connections=None)
        if probe:
            probe.evogym_probe_reset()
        env.reset()
        u = env.unwrapped
        height = lambda: float(np.mean(u.object_pos_at_time(u.get_time(), 'robot'), 1)[1])
        start = top = height()
        total, k, ended = 0.0, 0, False
        while True:
            _, reward, terminated, truncated, _ = env.step(np.array(A[k % len(A)], dtype=np.float64))
            total += reward
            k += 1
            top = max(top, height())
            if terminated or truncated:
                ended = bool(terminated)
                break
        row = dict(task=task, commands_allowed=env.spec.max_episode_steps, reward=repr(float(total)), commands=k,
                   ended_early=int(ended), height_start_m=repr(start), height_end_m=repr(height()), height_max_m=repr(top))
        if probe:
            steps, depth, pairs = ctypes.c_long(), ctypes.c_double(), ctypes.c_long()
            probe.evogym_probe_read(ctypes.byref(steps), ctypes.byref(depth), ctypes.byref(pairs))
            row.update(unpushed_point_steps=steps.value, deepest_unpushed_m=repr(depth.value))
        env.close()
        rows.append(row)
    return rows


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tree', help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.tree:
        print('ROWS ' + json.dumps(episodes(a.tree)))
        sys.exit(0)
    got = {}
    for tree in ('probe', 'closed'):        # each build of the simulator lives in its own process
        out = subprocess.run([sys.executable, os.path.abspath(__file__), '--tree', tree], capture_output=True, text=True)
        if out.returncode != 0:
            sys.exit(out.stderr)
        got[tree] = json.loads([line for line in out.stdout.splitlines() if line.startswith('ROWS ')][-1][5:])
    released = json.load(open(os.path.join(ROOT, 'results', 'released.json'))).get('tasks', {})
    rows = []
    for r, c in zip(got['probe'], got['closed']):
        assert r['task'] == c['task']
        rel = released.get(r['task'], {})
        rows.append(dict(task=r['task'], commands_allowed=r['commands_allowed'],
                         reward=r['reward'], commands=r['commands'], ended_early=r['ended_early'],
                         height_start_m=r['height_start_m'], height_end_m=r['height_end_m'], height_max_m=r['height_max_m'],
                         unpushed_point_steps=r['unpushed_point_steps'], deepest_unpushed_m=r['deepest_unpushed_m'],
                         closed_reward=c['reward'], closed_commands=c['commands'], closed_ended_early=c['ended_early'],
                         closed_height_max_m=c['height_max_m'],
                         released_robots=rel.get('robots', ''), best_released_reward=rel.get('best_reward', ''),
                         second_best_released_reward=rel.get('second_best_reward', '')))
    path = os.path.join(ROOT, 'results', 'other_tasks.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('written', path, len(rows), 'tasks')
    for r in rows:
        if r['best_released_reward'] != '' and float(r['reward']) > float(r['best_released_reward']):
            print(f"  {r['task']}: {float(r['reward']):.2f} as released, {float(r['closed_reward']):.2f} with the loophole closed; "
                  f"best of {r['released_robots']} released robots {float(r['best_released_reward']):.2f}")
