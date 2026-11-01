# Geometry and energy behind the return

These visuals are rendered from the CSV files in `data/`. The [main README](../README.md) places them beside the findings.

| Visual | Files and size |
|---|---|
| Launch | [`launch.mp4`](launch.mp4), 1600×900, 16 seconds at 24 frames/s; [`launch.gif`](launch.gif), 960×540 at 12 frames/s. |
| Contact mechanism | [`contact-mechanism.png`](contact-mechanism.png), 3000×1800; [`SVG`](contact-mechanism.svg). |
| Energy account | [`energy-account.png`](energy-account.png), 3000×1800; [`SVG`](energy-account.svg). |
| Rule variants | [`rule-variants.png`](rule-variants.png), 3000×2400; [`SVG`](rule-variants.svg). |

## Launch

Left, the two robot cells and floor cells are shown at true scale; right, centre-of-mass height spans all 500 actions for the stock winner under stock and closed contact and the closed winner under closed contact; below, bars show ground normal work per integration step through release. The 12-step release is slowed to eight MP4 frames per integration step, then the robot leaves the left view while the right plot follows its 51.45 m centre-of-mass rise.

## Contact mechanism

Top, action 200 above the floor, step 6065 sunk, and release step 6066 share one scale, with the selected edges highlighted; the filled arrow is the corner's own contact, the hollow arrows are its halves of the two reverse contacts, and the force key is 400,000 N. Bottom, the left and right corners use x offsets in micrometres from x = 3.4 m and 3.5 m against y in metres, stretching the horizontal axis; fading side edges run from the deepest sunk state at step 6056 to step 6065, and the bold edge is the release state.

## Energy account

Top, the account's signed work terms over the episode are shown in kJ. Below, ground normal work per integration step around release and cumulative work, both in kJ, share a time axis in milliseconds from release.

## Rule variants

All 16 rule settings share one linear return axis. Filled dots show the best found with the loophole open, hollow circles the best found with it closed, and crosses the open winner replayed with it closed.

## Regenerate the visuals

From the repository root:

```sh
python scripts/render_figures.py
```

This regenerates every visual from `figures/data/` in about 15 seconds, replacing the MP4, GIF, PNG and SVG outputs. It needs NumPy plus Matplotlib 3.10.9 and Pillow 12.3.0, pinned in `requirements.txt`, and `ffmpeg` on the PATH for the animation. It does not need a simulator build. Fonts fall back from Helvetica Neue to Helvetica, Arial or DejaVu Sans, so only the typeface changes on other machines.

`--data DIR` selects the CSV directory, `--out DIR` selects the output directory, and `--only hero contact energy rules` selects the visuals to render; all four are selected by default. For example, `--only contact energy` regenerates those two figures. The renderer checks geometry, snapshot alignment and cumulative work, and stops if those inputs disagree.

## Source data

`scripts/figure_data.py` exports the following CSV files under `data/`. The traces are engine replays on the local platform; the energy summary and rule table come from the current `results/numbers.json`. See [the reproduction instructions](../REPRODUCE.md#export-figure-data) before replacing the bundled CSV files.

| CSV file | Content |
|---|---|
| `body_cells.csv` | Cell index, material and four corner-point indices. |
| `episode_points_per_action.csv` | `action`, `point`, `x_m`, `y_m` for the base winner throughout the episode. |
| `release_points_per_step.csv` | `step`, `action`, `point`, positions in metres and velocities in metres per second around release. |
| `release_contacts.csv` | Contact direction, point and cell indices, selected edge and its endpoints, depth in metres, normal-force components in newtons and robot-point heights. |
| `ground_normal_work_per_step.csv` | Per-step work and cumulative work in joules over the episode. |
| `com_height_per_action.csv` | Height in metres for the stock winner under stock and closed contact, and the closed winner under closed contact. |
| `energy_terms_best_design.csv` | Signed episode work by term, retaining terms whose absolute value exceeds 1 J. |
| `rule_variants.csv` | Rule name and change, best open and closed returns, estimated share and the open winner's own closed return. |

Point snapshots include the initial state: snapshot `step = s` precedes update `s`, while work at `step = s` belongs to the update from state `s` to state `s + 1`. Action snapshots follow the same convention. Cell corners must be connected in perimeter order, top left, top right, bottom right, bottom left.

The animation interpolates positions between recorded snapshots, using per-step geometry near release and per-action geometry elsewhere. The GIF uses every second video frame, giving four frames per release step. Small energy terms have dots at their plotted values so they remain visible beside the larger bars.

Contact forces are recorded on the penetrating point. For a floor point inside the robot, the robot receives the opposite force split between the selected edge's endpoints. Blank edge endpoints denote fixed floor edges outside the exported robot-edge list. The contact log records applied contacts only; a missing row alone does not establish penetration without force.

At release, the bottom corners sit about 96 micrometres outside the column's original footprint, in neighbouring floor cells. Each side edge's crossing of the floor top shifts from 19.7 micrometres inside the middle floor cell to 7.3 micrometres inside the corner's own cell. The depth is 0.0969 m, about 0.097 m: each of the four upward contacts supplies 213,965 N, about 214,000 N, and each bottom corner receives about 428,000 N from its own contact plus half of each reverse contact. See [the mechanism](../patches/README.md#why-contact-is-missed) for the contact rule.
