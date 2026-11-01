# From stored design to checked result

`check.py` is the entry point: it replays the headline designs in Evolution Gym, accounts for energy, and compares every rule variant with `results/expected.json`. It writes local measurements to `results/numbers.json`, prints `MATCH` or `DIFF`, and exits unsuccessfully if any check differs.

| File | Purpose |
|---|---|
| `setup.sh` | Fetch the pinned simulator and build stock, closed and contact-logging modules. |
| `replay.py` | Run a design through the selected Evolution Gym environment; return JSON with score and heights. |
| `energy.py` | Split the discrete energy change into force and potential-energy terms; locate the main release. |
| `figure_data.py` | Export geometry, contacts, work and comparisons as CSV. |
| `render_figures.py` | Check the exported CSV inputs and render the launch MP4/GIF and three PNG/SVG figures. |
| `common.py` | Load designs, encode rule variants, test admissibility and construct engine scenes. |

From the repository root, `python scripts/render_figures.py` regenerates every visual from `figures/data/` in about 15 seconds without using the simulator. `--data DIR` changes the input directory, `--out DIR` changes the output directory, and `--only hero contact energy rules` selects which visuals to render; all four are selected by default. Existing selected outputs are replaced.

Rendering uses NumPy plus Matplotlib 3.10.9 and Pillow 12.3.0, pinned in `requirements.txt`, and needs `ffmpeg` on the PATH for the animation. Fonts fall back from Helvetica Neue to Helvetica, Arial or DejaVu Sans, so only the typeface changes on other machines. The renderer checks geometry, snapshot alignment and cumulative work, and stops if those inputs disagree.

Setup defaults to `REFERENCE_MATH=0`. Set `REFERENCE_MATH=1` to disable fused multiply-add and link the bundled reference `tanh`; `CC` selects the C compiler for that object. [The platform folder](../platform/README.md) explains the option and its licence.

Python entry points provide `--help`. `common.py` is a shared library. [Reproduction instructions](../REPRODUCE.md) specify setup, commands and platform limits. In particular, `--tol` relaxes reference comparisons only; the direct engine comparisons remain exact.
