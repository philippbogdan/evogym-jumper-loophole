# Isolate the contact mechanism

`close-loophole.patch` adds a fallback when a robot point is inside a fixed floor cell but none of that cell's surface edges qualifies for contact. It chooses the top surface edge instead of applying no force. The stock winner's reference return falls from 514.48 to -1.99.

This is an analysis variant, not a proposed benchmark fix. It changes only the robot-point case and leaves the reverse contact case, a floor point inside a robot cell, unchanged. A benchmark repair needs physical validation of both contact directions.

## Why contact is missed

In `PhysicsEngine.cpp`, `resolve_point_boxel_collisions` applies force to a point inside another object's cell only if one of the point's own cell edges crosses a surface edge of that containing cell. If no edge qualifies, it returns without applying force.

The stock winner's bottom corners start on floor-cell boundaries. The muscle squeezes the rigid cell into a trapezoid: each corner drifts into an outer neighbouring floor cell, but its side crosses the floor's top in the middle cell. The containing floor cell has no qualifying edge, so the corner sinks towards `y = 0`, where a separate ground penalty stops it. Meanwhile, the middle floor cell's top corners enter the robot cell.

At release, each bottom corner sits about 96 micrometres outside the column's original footprint, in a neighbouring floor cell. From step 6065 to step 6066, each side edge's crossing of the floor top moves from 19.7 micrometres inside the middle floor cell to 7.3 micrometres inside the corner's own cell. The [contact figure](../figures/contact-mechanism.png) stretches the horizontal axis in its bottom panels to show this change.

As the sides straighten, contact engages in both directions. Each robot corner is pushed upwards by its floor cell; each swallowed floor corner also pushes back on the robot's bottom edge. The normal force is `F = 2.1e6 * (0.005 + depth)` N, with depth in metres. At the first release step (6066), the depth is 0.0969 m, or about 0.097 m, and the four upward contacts are each 213,965 N, or about 214,000 N. Each bottom robot corner receives about 428,000 N: its own contact plus half of each of the two reverse contacts. The [contact records](../figures/data/release_contacts.csv) identify the selected edges and also include the smaller, mostly horizontal side contacts.

## Rule comparisons

`rule-variants/` contains 15 independent patches, each changing one rule relative to Evolution Gym commit `f63c649c688adda1accad5f86731aad96606388a`. Their names match `scripts/common.py` and the result tables. `base` needs no rule patch. Apply one rule patch to a stock tree; optionally add the closing patch for the paired comparison. [Reproduction instructions](../REPRODUCE.md) show patching, rebuilding and environment replay.

| Rule | Change | Best found | Best found, closed |
|---|---|---:|---:|
| `base` | Released task | 514.48 | 14.20 |
| `box7` | Body box 7×7 | 514.48 | 20.41 |
| `no_rigid` | No rigid cells | 505.21 | 14.20 |
| `no_soft` | No soft cells | 514.48 | 14.20 |
| `no_hmuscle` | No horizontal muscles | 513.71 | 13.79 |
| `no_vmuscle` | No vertical muscles | 514.48 | 0.69 |
| `joins` | Any subset of joins | 514.48 | 14.20 |
| `symmetric` | Mirror-symmetric body and actions | 514.48 | 14.20 |
| `range` | Muscle range [0.475, 1.725] | 514.48 | 18.80 |
| `control` | 1000 actions of 15 steps | 514.48 | 14.20 |
| `horizon` | 1000 actions | 521.23 | 13.09 |
| `sideways` | No sideways penalty | 520.54 | 14.81 |
| `instability` | No instability check | 514.48 | 14.20 |
| `strain` | Strain limits 50%, rigid 6% | 495.27 | 19.93 |
| `dt` | Step 5e-5 s, 60 per action | 508.45 | 25.03 |
| `contact` | Contact forces at every Runge-Kutta stage | 419.08 | 13.46 |

These are [reference-platform results](../results/expected.json). Each closed value is the best admissible replay among 31 stored closed-search designs, with actions repeated twice for `control`. They are best-found lower bounds; the closed searches were smaller. The estimated loophole share ranges from 95.1% to 99.9%. Closed scores span 13.09 to 25.03 for 15 settings; without vertical muscles, the best closed score is 0.69.

All open winners are two-cell columns using the same sink-and-release mechanism. Recalculating contact at every Runge-Kutta stage still allows a score of 419.08: holding contact force through a step is not the cause. The longer `horizon` permits several releases.

These diffs contain Evolution Gym source and retain its [MIT licence](LICENSE), copyright (c) 2022 jagdeepsb. Our changes are also offered under [CC0](../LICENSE).
