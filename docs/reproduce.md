# Reproduce it

## One command

```
pip install "evogym==2.0.0" "setuptools<70"
python scripts/quick.py
```

This is Evolution Gym as its authors ship it, with nothing of ours but the robot and its commands. It prints `score 514.48` and a rise of 51.45 m. Python 3.8 to 3.10, where the project publishes wheels.

## The numbers

```
python3.10 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
bash scripts/setup.sh
python scripts/check.py
python -m pytest tests
```

`setup.sh` fetches Evolution Gym at commit `f63c649` and builds it six ways under `build/`: as released, with the fix, with the plain patch, with the counting probe, and with the fix and the top-edge patch each counting its pushes. It needs git, CMake, a C and a C++ compiler, and the network once. It takes a few minutes.

`check.py` replays the stored designs and compares 90 values with `results/expected.json`: the headline scores in Evolution Gym's own Jumper-v0 environment, the energy account of every best design, all three columns of the rule table, and what the plain patch does. It prints `MATCH` or `DIFF` for each and exits non-zero on any difference.

`tests/` needs no simulator. It recomputes every number quoted on the front page and in `docs/` from the tables in `results/` and `figures/data/`.

All of this reproduces what each stored design does. The searches that found the designs are not here, and no score is claimed to be the best possible.

## One design

```
python scripts/diagnose.py designs/best-stock/design.json          # jump, or launch?
python scripts/replay.py stock designs/best-stock/design.json      # in Evolution Gym's own environment
python scripts/replay.py closed designs/best-stock/design.json
python scripts/energy.py designs/best-stock/design.json
```

To check a robot of your own, write its body and commands in [the same form](../designs/README.md).

## Last bits

The launch is chaotic, so identical numbers need identical arithmetic.

- **x86-64 Linux with glibc and GCC** is the reference. `results/expected.json` was computed there, and the check matches it exactly.
- **Apple silicon** agrees to six significant figures on 87 of the 90 values: `python scripts/check.py --tol 1e-6` reports the other three and exits non-zero. They are the score of a launch design once the loophole is closed, under three of the changed rule settings, and differ in the second decimal place.

`setup.sh` switches off fused multiply-add and links a fixed `tanh`, so that neither the compiler nor the maths library decides the result. What is left comes from Evolution Gym itself: the order in which it adds contact forces depends on the C++ standard library ([why](../platform/README.md)). With `REFERENCE_MATH=0` Apple silicon still reproduces 514.48 to seven figures.

## The released robots and the other tasks

```
pip install -r scope/requirements.txt
python scope/fetch.py          # the authors' robots and policies, 208 MB, CC BY-NC 4.0
python scope/tasks.py          # the two-cell robot, unchanged, in all 32 tasks: results/other_tasks.csv
python scope/replay.py         # 78 Jumper-v0 robots, as released and patched: results/released_jumper.csv
python scope/scan.py           # all 2,553 robots in their own tasks: results/released_scan.csv
python scope/noise.py --uid cca8b49a 51f55c20 --noise 200 --out released_launchers.csv     # the best released Climber-v0 robot
python scope/fix.py            # all 2,553, as released, fixed and with the top-edge patch: results/released_fix.csv
```

`tasks.py` replays stored commands and is exact on the reference platform. On Apple silicon its three scores above the released robots' agree to eight figures, and every score in the table to within 0.00001. The policies run closed loop, so their scores are not exact across machines; those tables are one machine's run. `results/released_fix.csv` and `results/released_deep_climber_v1.csv` are Apple silicon's, the first built with `REFERENCE_MATH=0`, the arithmetic of the shipped wheel there. What a score moves by under a change of one part in a million to the first command is in the tables too.

`results/baseline_episodes.csv` is the one table that cannot be remade from here: it comes from controllers trained by the benchmark's own genetic algorithm ([how](../scope/README.md)), 359 MB that are not in the repository. The 1,345 bodies, their scores and the training settings are.
