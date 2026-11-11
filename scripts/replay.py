"""Replay a stored design in Evolution Gym's own Jumper-v0, as released or patched.

    python scripts/replay.py stock designs/best-stock/design.json
    python scripts/replay.py closed designs/best-stock/design.json
"""

import os, sys, json, argparse
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def replay(tree, path):
    """Return score and height diagnostics from the chosen tree's own environment."""
    pkg = os.path.join(ROOT, 'build', tree)
    sys.path.insert(0, pkg)
    import gymnasium as gym
    import evogym.envs  # Register the environments on import.  # noqa: F401
    import evogym
    assert os.path.realpath(evogym.__file__).startswith(os.path.realpath(pkg)), evogym.__file__
    d = json.load(open(path))
    body = np.array(d['body'])
    conn = None if d.get('connections') is None else np.array(d['connections'])
    acts = d['actions']
    A = np.load(os.path.join(os.path.dirname(path), acts)) if isinstance(acts, str) else np.array(acts, dtype=np.float64)
    env = gym.make('Jumper-v0', body=body, connections=conn)
    env.reset()
    u = env.unwrapped
    T = env.spec.max_episode_steps
    R = 0.0
    ys = [float(np.mean(u.object_pos_at_time(0, 'robot'), 1)[1])]
    unstable = False
    t = 0
    for t in range(T):
        _, r, term, trunc, _ = env.step(np.array(A[t], dtype=np.float64).copy())
        R += r
        ys.append(float(np.mean(u.object_pos_at_time(u.get_time(), 'robot'), 1)[1]))
        if term:
            unstable = True
            break
        if trunc:
            break
    return dict(tree=tree, design=os.path.relpath(path, ROOT), ret=R, actions=t + 1, unstable=unstable,
                com_height_start=ys[0], com_height_end=ys[-1], com_height_max=max(ys))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('tree', choices=['stock', 'closed', 'plain'], help='simulator tree under build/: as released, patched, or with the plain patch')
    ap.add_argument('design', help='path to design.json; its action file is resolved beside it')
    a = ap.parse_args()
    print(json.dumps(replay(a.tree, os.path.abspath(a.design)), indent=1))
