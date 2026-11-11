# The visuals, and what in them is not data

Every position drawn is the simulator's own. `python scripts/figure_data.py` writes the data under `data/` from a fresh replay, and each visual is redrawn from it by one script.

| Visual | Script | Reads |
|---|---|---|
| `launch-*.gif`, `.mp4`, `.png` | `launch.py` (numpy, matplotlib, Pillow, ffmpeg) | `data/launch.npz`, `data/best_closed.npz`, `data/release_contacts.csv` |
| `inside-*.png` | `inside.py` (numpy, numba, Pillow, matplotlib for its font) | `data/launch.npz` |
| `mechanism-*.svg` | `mechanism.py` | nothing: a schematic |
| `score-*.svg`, `energy-*.svg`, `climber-*.svg` | `results.py` | `results/expected.json`, `results/released.json`, `results/released_launchers.csv` |

## The launch, one shot

One continuous shot of the two-cell robot, with the best design found with the loophole closed on the same floor, scale and clock. Not literal data:

- **Time.** The fall and the sink play 200 times slower than real time, the release 400 times slower, and the climb speeds up to real time by the end. The clock and the speed label on screen say so. It opens on the closing frame and then goes back to t = 0.5976 s.
- **Between steps.** While it speeds up, most frames fall between two simulator steps 0.1 ms apart; positions there are interpolated linearly between the two.
- **Scale.** The camera pulls back from a frame 0.5 m tall to one 58 m tall. Every frame is at one true scale.
- **The second robot** is drawn 1 m to the right. In its own episode it stands where the two-cell robot does.
- **Depth.** The simulator is two-dimensional. The cubes' depth, the shading and the shadow are drawing. The part of the robot inside the floor is drawn as a dashed outline; in the simulator the two simply overlap.
- **Arrows.** Length is proportional to force. Only the vertical contact forces on the two bottom corners are drawn. Forces on the other corners (up to 30,000 N while the robot is stopped) and sideways forces are not.
- **The trail** is the path of the centre of mass after the robot leaves the floor, with a dot every 50 ms. Its width is not to scale.
- **The numbers** are the rise of each robot's centre of mass above its own start, read from the data when the frame is drawn.

## Inside the floor

Three instants of the same episode, path-traced. Not literal data:

- **Depth.** Cells are drawn as 0.1 m cubes and the floor as a block 0.3 m deep. The front face of each block is the simulation's plane, and every corner on it is where the simulator recorded it.
- **The floor** shows three of its sixty cells, with cut ends. The three blocks are the same place at three instants; the gaps between them are layout.
- **Trails.** Behind each robot are translucent copies of its 24 previous recorded states (2.4 ms). Their opacity is a choice.
- **Light, shadows, materials and seams** are drawing.
- **The instants** are found in the data: the last state wholly above the floor; the last state before the corners reach the floor's underside; 24 steps after the step with the largest jump in speed. The times and speeds in the labels are read from the data.

## The schematic

Steps 1 and 2 of `mechanism-*.svg` draw the lean hundreds of times wider than it is. Steps 3 and 4 are to scale.
