"""See the score in Evolution Gym exactly as its authors ship it. Nothing here is built.

    pip install "evogym==2.0.0" "setuptools<70"      # Python 3.8 to 3.10
    python scripts/quick.py                          # the two-cell robot: 514.48
    python scripts/quick.py designs/best-closed/design.json

Replays a stored body and its commands in the installed package's own Jumper-v0 environment.
"""

import os, sys, json
import numpy as np
import gymnasium as gym
import evogym.envs  # noqa: F401  (registers Jumper-v0)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'designs', 'best-stock', 'design.json')
design = json.load(open(path))
commands = np.load(os.path.join(os.path.dirname(os.path.abspath(path)), design['actions']))
joins = None if design.get('connections') is None else np.array(design['connections'])

env = gym.make('Jumper-v0', body=np.array(design['body']), connections=joins)
env.reset()
height = lambda: float(np.mean(env.unwrapped.object_pos_at_time(env.unwrapped.get_time(), 'robot'), 1)[1])
start, score = height(), 0.0
for command in commands:
    _, reward, unstable, ended, _ = env.step(np.array(command, dtype=np.float64))
    score += reward
    if unstable or ended:
        break
print(f'cells {int(np.count_nonzero(design["body"]))}   score {score:.2f}   centre of mass rose {height() - start:.2f} m')
