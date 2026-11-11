# The same result under sixteen rule settings

Is the loophole a quirk of one time step, one material or one box size? Each row changes one rule of the task and nothing else. The patches are in [`patches/rule-variants`](../patches/rule-variants), the winning designs in [`designs/rule-variants`](../designs/rule-variants), and `python scripts/check.py` replays every number.

| Setting | What changes | Best found | Same design, loophole closed | Best found, loophole closed | Share from the loophole |
|---|---|---:|---:|---:|---:|
| `base` | The task as released | 514.48 | -1.99 | 13.83 | 97.3% |
| `box7` | Body box 7 × 7 instead of 5 × 5 | 514.48 | -1.99 | 17.36 | 96.6% |
| `no_rigid` | No rigid cells | 505.21 | -2.95 | 13.83 | 97.3% |
| `no_soft` | No soft cells | 514.48 | -1.99 | 13.83 | 97.3% |
| `no_hmuscle` | No horizontal muscles | 513.71 | -1.90 | 13.83 | 97.3% |
| `no_vmuscle` | No vertical muscles | 514.48 | -1.99 | 0.70 | 99.9% |
| `joins` | Any subset of joins between neighbouring cells | 514.48 | -1.99 | 13.83 | 97.3% |
| `symmetric` | Body and commands mirror-symmetric | 514.48 | -1.99 | 13.83 | 97.3% |
| `range` | Muscle range [0.475, 1.725] instead of [0.6, 1.6] | 514.48 | -1.99 | 17.97 | 96.5% |
| `control` | 1,000 commands of 15 steps | 514.48 | -1.99 | 13.83 | 97.3% |
| `horizon` | 1,000 commands of 30 steps | 521.23 | -2.88 | 12.72 | 97.6% |
| `sideways` | No penalty for sideways movement | 520.54 | -2.95 | 14.14 | 97.3% |
| `instability` | Instability never checked | 514.48 | -1.99 | 13.83 | 97.3% |
| `strain` | Strain limits doubled (50%, rigid 6%) | 495.27 | -2.87 | 19.68 | 96.0% |
| `dt` | Step halved (0.00005 s, 60 per command) | 508.45 | -0.89 | 25.13 | 95.1% |
| `contact` | Contact forces recomputed at every Runge-Kutta stage | 419.08 | -2.90 | 13.86 | 96.7% |

Every winner is a two-cell column that sinks and is thrown out. The share runs from 95.1% to 99.9%.

Three rows say something about the cause:

- **`contact`.** The simulator computes contact forces once per step and holds them. Recomputing them at every stage of the integrator costs the launch 19% and leaves it in place. The held force adds to the launch. It does not cause it.
- **`dt`.** Halving the time step does not remove it either.
- **`no_vmuscle`.** Without vertical muscles the best jump found with the loophole closed scores 0.70, and the launch still scores 514.48. A horizontal muscle is enough to squeeze the cell below it.

Each "best found" is the best of a search, not a proved maximum, and the searches with the loophole closed were the smaller ones. The designs in [`designs/closed-search`](../designs/closed-search) come from six body searches and eight two-cell searches for the released task and one or two short searches for each other setting, all run under [the plain patch](evidence.md#the-plain-patch), and from a local search of the commands of the best of them with the loophole closed, for the released task and for `strain`, `dt` and `contact` ([`scripts/retune.py`](../scripts/retune.py)). Each row takes the best of all 35 that the setting admits. The share is 1 minus the ratio of the two best-found scores. It is an estimate.
