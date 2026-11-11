# How far it reaches, and who stands on it

Four sets of measurements. What they found is on [the evidence page](../docs/evidence.md#the-other-tasks).

## 1. The two-cell robot in every task

`tasks.py` replays the two-cell robot and its 500 commands, unchanged, in each of the benchmark's 32 tasks, as released and with the loophole closed: [`results/other_tasks.csv`](../results/other_tasks.csv). Where a task runs for more than 500 commands, the commands start again from the first. The columns for the released robots are each task's count and two best rewards in the authors' dataset.

## 2. The robots the benchmark's authors released

The authors of Evolution Gym have released the robots of their paper on Hugging Face: [EvoGym/robots](https://huggingface.co/datasets/EvoGym/robots), 90,563 bodies with the score each reached, and [EvoGym/robots-with-policies](https://huggingface.co/datasets/EvoGym/robots-with-policies), 2,553 of them with the trained policy. 2,578 of the scores and 78 of the policies are for Jumper-v0.

- `fetch.py` downloads both (CC BY-NC 4.0, not redistributed here) and checks their SHA-256.
- `replay.py` runs the 78 Jumper-v0 robots with their own policies, as the authors' loader does, in the simulator as released and with the loophole closed. The policy runs closed loop in each, so it can react to the patched floor. It also repeats each run eight times with the first command changed by a few parts in a million, because these scores are chaotic and a difference means nothing without that yardstick. The measuring engine then replays the commands and reports how deep any corner sat inside the floor with no force on it.
- `scan.py` runs all 2,553 in their own tasks, in the simulator as released with a counting probe, and records for each how many point-steps a robot point spent strictly inside fixed terrain with no contact force on it, and how far the deepest such point was from the terrain's surface.
- `noise.py` takes the two Climber-v0 robots the scan finds deepest in the floor, one of them the best Climber-v0 robot in the dataset, and runs each controller once deterministically and 200 times sampling its actions, as released and with the loophole closed: [`results/released_launchers.csv`](../results/released_launchers.csv). It does the same, with 10 noisy runs, for the four Climber-v1 robots the scan finds deepest: [`results/released_deep_climber_v1.csv`](../results/released_deep_climber_v1.csv).
- `fix.py` runs all 2,553 once each, deterministically, as released, with the fix and with the top-edge patch, the last two with counters of their pushes, and records each score, how often each patch pushed a point the released simulator leaves without a force, and how deep the deepest was: [`results/released_fix.csv`](../results/released_fix.csv).

## 3. The benchmark's own baseline, rerun

The benchmark's baseline is a genetic algorithm over bodies with a PPO controller trained for each. It was rerun for Jumper-v0 with the benchmark's own example code at commit `f63c649` and the settings of the paper's appendix D:

| Setting | Value |
|---|---|
| Population, body | 25, 5 × 5 |
| Bodies evaluated per run | 750 |
| Mutation | 10% per cell |
| Survivors | 60% falling linearly to 0, at least 2 |
| PPO | 1,000 iterations of 128 steps in 4 environments: 512,000 steps per body |
| Learning rate | 2.5 × 10⁻⁴, decaying linearly to 0 |
| Clip, value coefficient, entropy coefficient | 0.1, 0.5, 0.01 |
| Evaluation | every 50 iterations; a body's fitness is its best evaluation |

[`patches/baseline-examples.patch`](../patches/baseline-examples.patch) is the whole change to the example code: a seed argument, four sampling environments and the learning-rate decay, which the paper specifies and the example hard-codes otherwise, and exact resumption of an interrupted run.

Two things are known to differ from the paper. The example at this commit trains with Stable-Baselines3, where the paper used another PPO implementation, and its minibatches are 4 samples where the paper's code used 4 minibatches of 128. And 750 evaluations for Jumper-v0 is read off the axis of the paper's figure 40, which is the only place it appears.

Three seeds were started. One ran to its 750 bodies; the other two stopped at 328 and 267. That is 1,345 bodies with a trained controller each: [`results/baseline_robots.csv`](../results/baseline_robots.csv).

`baseline.py --top 400` replays the controllers of the 400 fittest bodies once each, deterministically, in the simulator as released. The measuring engine then replays the commands it sent, and reports how deep any corner sat inside the floor with no force on it and what the same commands score with the loophole closed. A body's fitness came from an episode run with the policy's noise, which a deterministic replay does not repeat. The controllers (359 MB) are not in this repository.

## 4. Published numbers

Scores for Jumper-v0 printed in papers and released datasets are collected in [`results/published_scores.csv`](../results/published_scores.csv), each with its place in the source.
