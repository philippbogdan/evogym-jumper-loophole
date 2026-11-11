# What was checked

## Where each number comes from

Every design is stored as a body and its commands, and every score on these pages is that design replayed. `python scripts/check.py` replays them and compares 90 values with [`results/expected.json`](../results/expected.json). `python -m pytest tests` recomputes every number quoted on these pages from the tables in `results/` and `figures/data/`.

| Claim | Measured how | Where |
|---|---|---|
| 514.48 | The two-cell robot in Evolution Gym's own Jumper-v0 environment, built from source as released. The wheel on PyPI gives the same number to the last digit on x86-64 Linux | `designs/best-stock`, `scripts/replay.py`, `scripts/quick.py` |
| -1.99 | The same robot in the same environment built with the patch | `patches/close-loophole.patch` |
| 13.83 | The best design found with the loophole closed, in the patched environment. On x86-64 Linux it scores the same as released, where none of its corners is ever inside the floor without a force on it; in the shipped wheel on Apple silicon it is thrown as released, and scores 58.52 | `designs/best-closed`, `results/expected.json` |
| 97% | 1 - 13.83 / 514.48 = 97.3%. An estimate: both scores are the best a search found | `results/expected.json` |
| 59,641 J and 450 J | The energy account of the two-cell robot's episode | `scripts/energy.py` |
| 4 × 213,965 N at 9.69 cm | The contacts applied at step 6066 | `figures/data/release_contacts.csv` |
| 9.45 and 7.32 | The two-cell robot and its commands in Climber-v0 and Climber-v1 | `scope/tasks.py`, `results/other_tasks.csv` |

The reference platform is x86-64 Linux with glibc and GCC. On it the check is exact, bit for bit; [on others](reproduce.md#last-bits) it is not.

## The measuring engine

Evolution Gym's simulator does not report which force did what. [`engine/`](../engine) is its integration step written out so that it does: the same operations in the same order, on the scene the simulator's own code builds. It was checked against the simulator three ways:

- `check.py` replays the two-cell robot, as released and patched, and the best patched design in Evolution Gym's own environment and in the engine, and requires the scores to be identical to the last bit.
- 478 episodes of the released robots and of the rerun baseline, recorded in Evolution Gym's environment, were replayed in the engine. All 478 scores are identical to the last bit.
- The energy account of every best design closes: the terms sum to the change in the robot's energy with a relative residual below 2 × 10⁻¹⁵.

## The plain patch

The obvious way to close the loophole is six lines: where the routine would return for want of an edge, use the floor cell's top edge ([`patches/close-loophole-plain.patch`](../patches/close-loophole-plain.patch)). It closes the loophole. It also pushes corners that are not in the floor.

The routine runs for every point that the simulator's containment test places inside a cell, and that test has a blind spot (`Boxel.cpp:41-65`). It splits the cell into two triangles along the diagonal from its top left to its bottom right corner and compares the signs of cross products. For a point lying exactly on the line through those two corners one product is zero, and the test passes wherever on that line the point is.

A robot standing on the lattice has such points. The top left corner of a cell in its bottom row, at height 0.2 m, lies on that line of the floor cell one down and one across. As released, nothing follows: no edge qualifies and the routine returns. With the plain patch the fallback fires.

The best design found under the plain patch is a 21-cell checkerboard of muscles. Before it has moved, the patch puts 220,500 N on one of its corners, 10 cm above the floor: the contact law at a depth of one whole cell. That design scores 14.20 under the plain patch and 12.22 in the simulator as released, so 1.98 of its points are the patch's doing and not the robot's.

So [`close-loophole.patch`](../patches/close-loophole.patch) tests that the corner lies within the cell's corners before it pushes, and every number on the front page uses it. Most of the designs in [`designs/closed-search`](../designs/closed-search) were found by searches run under the plain patch; the best of them for the released task and for three other rule settings were then searched again, locally, with the loophole closed ([`scripts/retune.py`](../scripts/retune.py)). That second search was the smaller, which is one more reason 13.83 is a score that was found and not a ceiling.

## The top-edge patch

With that check, the floor cell's top edge closes the loophole and pushes nothing outside a cell ([`patches/close-loophole-top.patch`](../patches/close-loophole-top.patch)). It assumes the way out of a fixed cell is up. A point that touches a peg or a pipe's ceiling from below lies on the bottom side of a cell, no edge qualifies, and this patch pushes it up through the whole cell: 2,100,000 N/m × (0.005 m + 0.1 m), about 220,000 N.

[`scope/fix.py`](../scope/fix.py) runs every released robot once in its own task, in the shipped wheel's arithmetic on Apple silicon, as released, with this patch and with the fix ([`results/released_fix.csv`](../results/released_fix.csv)). 37 of them meet a push of this patch from more than 2 cm deep: 22 in BeamToppler-v0, 9 in Climber-v2, 4 in BeamSlider-v0 and 2 in Lifter-v0. The BeamToppler-v0 robot that scores highest as released, 9.41, scores 11.46 under this patch and 9.39 under the fix. No push of the fix starts deeper than 12.62 mm.

So [`close-loophole.patch`](../patches/close-loophole.patch) uses the nearest exposed side of the cell, and only one less than half a cell away: a point farther than that from every exposed side is nearer a side the cell shares with another.

## The other tasks

[`scope/tasks.py`](../scope/tasks.py) puts the two-cell robot and its 500 commands into each of the benchmark's 32 tasks and changes nothing else: [`results/other_tasks.csv`](../results/other_tasks.csv). One task, Catcher-v0, places its box at a random offset, and the script fixes that draw.

- In 22 tasks a corner sinks more than 4 cm into the floor with no force on it and the floor throws the robot: more than 50 m in 15 of them, between 1.6 and 16.9 m where the episode ends first or something is in the way. In the other 10 it never sinks and nothing happens.
- With the loophole closed its centre of mass never rises above where it started, in any task.
- It outscores every released robot in three: Jumper-v0, Climber-v0 and Climber-v1. A climb ends when the centre of mass passes a line, 8.6 m up in Climber-v0 and 6.5 m in Climber-v1 (`evogym/envs/climb.py:90-92` and `:148-150`), and pays the height gained plus 1. The robot crosses them at command 230 of 400 and command 223 of 600, for 9.45 and 7.32. With the loophole closed it scores -0.20 in both.
- A released Climber-v1 robot that reached the top would score at least 6.95, because its centre of mass starts at most 0.55 m up. The best of the 2,162 scores 4.13.
- One of the 3,417 released Climber-v0 robots scores 9.27, and the next 5.49. That one [is thrown from the floor too](#the-best-released-climber-v0-robot).

The commands were found for Jumper-v0 and nothing was searched for any other task. These numbers show that the loophole is there to be used in other tasks, not how much a search aimed at each would get from it.

## Who stands on the loophole

### The best released Climber-v0 robot

In the dataset `EvoGym/robots` the best Climber-v0 reward is 9.27 and the next 5.49. The best belongs to robot `cca8b49a`, which CPPN-NEAT produced: a column of four vertical muscles. It is also in `EvoGym/robots-with-policies`, where its reward is 1.77; that dataset's card says its rewards "may not exactly match" the first one's, "due to changes in the library, system architecture, etc."

[`scope/noise.py`](../scope/noise.py) runs its released controller once deterministically, as the authors' loader does, and 200 times with the controller sampling its actions, in the simulator as released and with the loophole closed ([`results/released_launchers.csv`](../results/released_launchers.csv)).

| | As released | Loophole closed |
|---|---:|---:|
| Deterministic run, score | 1.77 | -0.13 |
| Deterministic run, highest above its start | 5.76 m | 1.09 m |
| 200 noisy runs that reach the top | 2, scoring 9.24 and 9.21 | 0 |
| Best score of all 201 runs | 9.24 | 1.05 |
| Highest above its start in all 201 runs | 8.24 m | 1.10 m |

As released, the deterministic run holds a corner 85 mm inside the floor with no force on it, and the two runs that reach the top hold one 93 and 100 mm down; the two-cell robot's is 100 mm down. The column starts in the middle of the channel, two cells clear of either wall, and a point more than 50 mm from any surface can only be in the floor: the walls are one cell thick. The run that scored 9.27 is not released and cannot be replayed. What is shown is that this robot's controller reaches the top through the loophole, and with it closed gets no higher than 1.1 m of the 8.2 m the top needs.

A second column of four muscles in the same dataset, `51f55c20`, does the same: 6.74 m in its deterministic run as released, the top in 2 of 200 noisy runs, and never more than 1.23 m with the loophole closed.
### The authors' robots on Jumper-v0

The authors have released 90,563 robots with the score each reached, 2,578 of them for Jumper-v0. The best scores 10.17, a CPPN-NEAT robot; the best from the genetic algorithm scores 5.43. A jump with the loophole closed reaches 13.83, so no released score needs the loophole.

For 78 of the Jumper-v0 robots they released the trained controller. [`scope/replay.py`](../scope/replay.py) runs each as their loader does: [`results/released_jumper.csv`](../results/released_jumper.csv).

- As released, 72 of the 78 reproduce the dataset's reward to three decimals, and the largest difference is 0.10.
- Each robot then runs nine times as released and nine times with the loophole closed, the controller reacting to whichever floor it is on. The nine differ by a few parts in a million in the first command, which alone moves a robot's score by 0.29 (the median standard deviation over the nine). On x86-64 Linux the 78 average 3.40 as released and 3.38 closed, and the difference between a robot's two means averages 0.016, with a standard error of 0.049; run once each in the shipped wheel's arithmetic on Apple silicon they average 2.81 and 2.58, and the difference averages 0.23 with a standard error of 0.11 ([`results/released_fix.csv`](../results/released_fix.csv)).
- The deepest any of their corners sits inside the floor with no force on it is 12.59 mm, against 99.83 mm for the two-cell robot. 37 of the 78 have such a corner at some step.

### Every released robot, in its own task

[`scope/scan.py`](../scope/scan.py) runs all 2,553 robots that come with a controller, each once and deterministically in its own task, in the simulator as released, and records how far below the terrain's surface any of its points sits with no contact force on it ([`results/released_scan.csv`](../results/released_scan.csv)). Catcher-v0's random draw is fixed here too.

- In 29 of the 32 tasks no point is ever more than 17.11 mm down.
- In the three climbs 14 robots hold a point more than 20 mm down, and 6 of them more than 50 mm: the two columns in Climber-v0 and 4 robots in Climber-v1, which score between 1.00 and 1.25 where that task's best scores 4.13. Run again with the loophole closed, eleven times each, those four average within 0.09 of what they average as released and none rises more than 1.33 m ([`results/released_deep_climber_v1.csv`](../results/released_deep_climber_v1.csv)): whatever holds them up is not the loophole.

### The benchmark's baseline, rerun

The benchmark's baseline is a genetic algorithm over bodies with a PPO controller trained for each. It was rerun for Jumper-v0 with the benchmark's example code and the paper's settings, which differ from the paper's own run in the PPO implementation and its minibatch ([the settings](../scope/README.md)). Of three seeds one ran its 750 bodies and two stopped at 328 and 267: 1,345 bodies, each with a controller trained for 512,000 steps ([`results/baseline_robots.csv`](../results/baseline_robots.csv)). The best scores 3.66.

The controllers of the 400 fittest were replayed once each, deterministically ([`results/baseline_episodes.csv`](../results/baseline_episodes.csv)). The deepest any corner sits inside the floor with no force on it is 18.74 mm; 138 of the 400 have such a corner at some step.

### Published scores

Every score for Jumper-v0 that a search of the papers citing Evolution Gym turned up, in print or in a released dataset, with its place in the source ([`results/published_scores.csv`](../results/published_scores.csv)). The search can have missed some. The highest in a paper is 9.17 on the standard body and 14.50 on a 7 × 7 one, where the best jump found here with the loophole closed scores 17.36. No paper found reports the loophole, and on 11 November 2026 Evolution Gym's issue tracker held no report of it.

| Source | Where | What | Score | Body | Note |
|---|---|---|---:|---|---|
| [Bhatia et al., Evolution Gym (NeurIPS 2021)](https://arxiv.org/abs/2201.09863) | arXiv v1, figure 40 | genetic algorithm with PPO, best so far, mean of 3 seeds, at the end of the run | about 5 | 5 × 5 | Read off the plot: the paper prints no number for this task. |
| [Evolution Gym authors, dataset EvoGym/robots](https://huggingface.co/datasets/EvoGym/robots) | reward field, 2,578 Jumper-v0 rows | best released robot (CPPN-NEAT) | 10.17 | 5 × 5 | Best genetic-algorithm robot in the dataset: 5.43. |
| [Evolution Gym authors, dataset EvoGym/robots-with-policies](https://huggingface.co/datasets/EvoGym/robots-with-policies) | reward field, 78 Jumper-v0 rows | the same robot, evaluated again by the authors | 7.28 | 5 × 5 | Best genetic-algorithm robot: 4.86. |
| [Wang et al., CuCo (ICLR 2023)](https://openreview.net/forum?id=r9fX833CsuN) | table 1 | CuCo, mean and standard error of 7 runs | 13.08 | 7 × 7 | 7 × 7 body and its own observation and action interface; its baselines score 0.79 to 3.82. |
| [Wang et al., CuCo (ICLR 2023)](https://openreview.net/forum?id=r9fX833CsuN) | figure 16 | CuCo, one robot, mean of 5 tests | 14.50 | 7 × 7 | A block of vertical muscles. |
| [Dong et al., HERD (ICLR 2024)](https://arxiv.org/abs/2311.00462) | table 2 | HERD, mean of 4 seeds | 2.71 | 5 × 5 | Its handcrafted robot: 3.15; its other baselines: -2.49 to 0.30. |
| [Liu et al., AIEA (IEEE TEVC 2024)](https://doi.org/10.1109/TEVC.2023.3327459) | table I | genetic algorithm, mean of 5 runs of 100 evaluations | 0.81 | 5 × 5 | AIEA itself: 0.46. |
| [Wang et al., Auto-Robotist (EMNLP 2026)](https://aclanthology.org/2026.emnlp-main.368) | table 1 | Auto-Robotist, mean of 3 seeds | 6.67 | 5 × 5 | Its genetic-algorithm baseline: 5.58; with a 10 × 10 body, table 2: 7.24. |
| [Song et al., MISCo (arXiv 2609.29491)](https://arxiv.org/abs/2609.29491) | figure 5a | MISCo-A, best fitness at the end of evolution | 9.17 | 5 × 5 | Its baselines: 3.29 to 8.56. Its table II prints 27.62 to 47.47 in rows labelled Jumper-v0; those are its Flipper-v0 values under a swapped label (9.17 and 6.37 appear there as Flipper-v0). |
| [Han et al., tabular foundation models as surrogates (arXiv 2609.18130)](https://arxiv.org/abs/2609.18130) | table S-IX | best of four methods, mean of 5 runs of 100 evaluations | 2.24 | 5 × 5 | The others: 1.61, 2.03, 2.03. |
