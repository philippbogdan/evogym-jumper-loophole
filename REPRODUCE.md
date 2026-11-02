# Reproduce the results

From this directory, after setup, one command recomputes the headline returns, energy accounts and rule comparisons:

```sh
python scripts/check.py
```

It replays the stored designs, compares them with `results/expected.json`, and replaces `results/numbers.json` with the local results. The check took **31 seconds on x86-64 Linux** and **18.90 seconds on an Apple M4 Mac**. Mac setup took about **3.5 minutes**, including the clone and three builds; two complete Linux setups and checks took about **5 minutes on two CPU cores**. Timing depends on the platform.

## Setup

Use Python 3.10, git, CMake, a C++ compiler and the packages in `requirements.txt`. The native libraries needed by Evolution Gym's simulator build must also be available. Network access is needed for the initial downloads; the check itself runs locally.

```sh
python3.10 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
PYTHON=python bash scripts/setup.sh
```

The script fetches Evolution Gym at `f63c649c688adda1accad5f86731aad96606388a` and its pinned GLFW, GLEW and pybind11 submodules. It builds stock and loophole-closed copies with Evolution Gym's CMake, the accounting modules, and a contact-logging module. `PYTHON` selects the interpreter, `CXX` the C++ compiler and `BUILD_THREADS` the CMake build concurrency. `CC` selects the C compiler used for the optional reference-maths object. Rerunning setup replaces `build/stock` and `build/closed`.

Keep the `setuptools<70` requirement: this Evolution Gym revision imports `pkg_resources`, which newer setuptools releases no longer ship. A fresh Linux environment failed without that pin.

## Read the check

Each comparison prints `MATCH` or `DIFF`, its name, the local value and the reference value, followed by a match count and the output path. Differences include their signed magnitude. The check exits with status 1 if any comparison fails, after writing the results.

It covers the stock winner in Evolution Gym's own environment, that design with the loophole closed, the stored closed winner, their engine counterparts, the estimated share, every variant's energy closure and every variant's best open and closed returns. The closed comparison replays all admissible candidates in `designs/closed-search/`; it does not run a new search.

The default Linux build finished with:

```text
62 of 62 match; results written to results/numbers.json
```

Of those checks, 46 compare values for exact equality by default; the other 16 require an energy-account residual below `1e-12`.

### Platform agreement

The reference platform is **x86-64 Linux with glibc**. The reference values were computed on one such machine and reproduced bit for bit on an Intel Core i9-13900K, which supports fused multiply-add (FMA), running Ubuntu 22.04, glibc 2.35, GCC 11.4 and Python 3.10.12. Both the default build and `REFERENCE_MATH=1` gave **62 of 62 MATCH**, with every equality comparison exact. The two runs also produced identical `numbers.json` contents. The bundled `results/numbers.json` is from that default Linux run.

The re-implemented engine was also checked against Evolution Gym's simulator on random bodies and actions for every rule variant, including final positions, velocities and returns. That agreement is platform-specific.

With Evolution Gym's default build on an Apple M4 Mac (macOS, arm64, Apple clang, Python 3.10.20):

| Measurement | x86-64 Linux reference | Mac default build |
|---|---:|---:|
| Best stock design | 514.4803489235601 | 514.4803330560252 |
| Same design, loophole closed | -1.987423 | -1.987423 |
| Stored best closed design, loophole closed | 14.20 | 11.85 |
| `no_rigid` winner | 505.2 | 40.9 |
| `sideways` winner | 520.5 | 43.6 |

The main launch agrees to seven significant figures. Its closed return differs by only `1.6e-11`, its energy terms agree to `1e-4 J`, and the same 12-step release occurs. All Mac energy accounts have relative residuals below `4e-15`. But the two soft-bottom designs, `no_rigid` and `sideways`, do not launch as they do on Linux, and the stored closed winner's jump changes substantially. The exact check therefore reports **17 of 62 MATCH** and exits with status 1.

On arm64 the compiler fuses multiply-adds by default, and Apple's `tanh` differs from glibc's in the last bit for some inputs. Contact friction uses `tanh`; a last-bit change can decide which step makes contact and grow into a different trajectory. The main two-cell launch survives these differences, but bit identity is not guaranteed across platforms.

### Reference maths option

The default is `REFERENCE_MATH=0`, using Evolution Gym's own build and the system maths library. To disable fused multiply-add and use the bundled reference `tanh`, rebuild with:

```sh
REFERENCE_MATH=1 PYTHON=python bash scripts/setup.sh
python scripts/check.py
```

This adds `-ffp-contract=off` to the simulator and accounting builds, including contact logging, and links `platform/ref_tanh.c`. The file ports the fdlibm `tanh`/`expm1` algorithm with the operation order used by glibc's generic double-precision code. It removes the friction calculation's dependence on the system `tanh` without modifying Evolution Gym's source. The reference-maths object also needs a C compiler; see [its purpose and licence](platform/README.md).

`REFERENCE_MATH=1` has so far been verified only on x86-64 Linux, where the result is exact as above. It was not measured on the Apple M4. On Apple silicon, the default build reproduces the main launch to seven significant figures, but not the sensitive designs listed above; the reference maths option has not yet been shown to resolve those differences.

### Tolerance-based comparisons

For a tolerance-based reference comparison:

```sh
python scripts/check.py --tol 1e-4
```

For these comparisons, `--tol T` accepts `abs(local - reference) <= T * max(1, abs(reference))`; the default is exact equality. It does not relax the two direct engine-versus-environment comparisons, the release-step count, or the separate energy-closure threshold. A tolerance therefore need not produce a successful exit on another platform. It also cannot establish agreement for a design whose trajectory has changed substantially.

## What the score measures

Jumper-v0 sums rewards over 500 actions. Each reward is `10 * rise - 5 * abs(sideways movement)`, using the centre of mass of the robot's points and distances in metres. Each muscle receives one action value, clipped to `[0.6, 1.6]` and held for 30 integration steps of `1e-4 s`. The body occupies a 5×5 grid of 0.1 m cells, forms one connected piece and contains at least one muscle. The [rule variants](patches/README.md#rule-comparisons) change one rule at a time.

All reported best scores are lower bounds on what a search could find, not proved ceilings. The estimated loophole share is `1 - best_closed / best_stock`, or 97.2% for the released task. It is not a bound: the closed searches were smaller, and better closed designs may exist. The base closed searches comprised six body searches and eight two-cell searches; each other variant had one short body search, with two for `no_vmuscle` and `horizon`.

## Replay one design

```sh
python scripts/replay.py stock designs/best-stock/design.json
python scripts/replay.py closed designs/best-stock/design.json
python scripts/replay.py closed designs/best-closed/design.json
```

Each command uses the selected tree's own Gymnasium `Jumper-v0` and prints JSON: return, completed actions, instability status and initial, final and maximum centre-of-mass heights in metres. Run each tree in a separate Python process so imported simulator modules cannot be mixed. The reference stock winner scores 514.4803489235601; closing the loophole reduces it to -1.9874. The stored closed winner scores 14.2010 on the reference platform.

## Account for the energy

```sh
python scripts/energy.py designs/best-stock/design.json
python scripts/energy.py designs/rule-variants/contact/design.json --rules contact
```

The account uses `E = sum(|v|² / 2 + 110 y)` over robot points, each of mass 1. It splits the kinetic-energy change by each force's contribution to the Runge-Kutta velocity update, then includes gravitational potential changes from integration and position projection. These terms sum to the change in E up to round-off. Spring reset charges and discarded spring energy are separate diagnostics, not extra terms to add to that sum.

For the best stock design, the reference episode account is:

| Contribution | Work (J, rounded) |
|---|---:|
| Ground normal force | +59,641 |
| Passive springs | -17,740 |
| Muscle springs | +450 |
| Ground damping | -6,057 |
| Drag | -2,323 |
| Ground friction | -13 |
| Separate ground penalty below `y = 0` | +146 |

The full account also includes gravity work and the potential-energy changes. The robot gains 34,105 J, from 264 J to 34,369 J; its relative closure residual is `2.4e-16`. The ground normal force supplies 56,883 J in the 12-step release, 95% of its episode total. Across all 16 winners, the reference residuals range from `1.1e-16` to `1.3e-15`. Closure establishes a decomposition of the simulator's update rule, not physical conservation. Full-precision terms are in [the reference results](results/expected.json).

The JSON includes signed work in joules, starting and ending E, the relative closure residual, and the integration steps carrying 95% of ground normal work. The script selects steps by descending work, then reports their earliest and latest indices; they need not be consecutive for every design. For the base design they are consecutive, steps 6066 to 6077 within action 202. Indices start at zero.

## Export figure data

After a check has written `results/numbers.json`:

```sh
python scripts/figure_data.py
```

This writes `figures/data/*.csv`: point positions, cell corners, contact records, ground normal work, centre-of-mass histories, energy terms and rule comparisons. See [the data formats](figures/README.md). Traces are fresh engine replays, while energy and variant summaries are read from `numbers.json`. Keep those results and the build consistent when exporting; they are local-platform values, not an automatic copy of the reference results.

## Render the visuals

To regenerate every visual from the existing `figures/data/`, without rebuilding or replaying the simulator:

```sh
python scripts/render_figures.py
```

This replaces the MP4, GIF, PNG and SVG files in `figures/` and takes about 15 seconds, depending on the machine. It uses NumPy plus Matplotlib 3.10.9 and Pillow 12.3.0, pinned in `requirements.txt`; the animation also needs `ffmpeg` on the PATH. Fonts fall back from Helvetica Neue to Helvetica, Arial or DejaVu Sans, so only the typeface changes on other machines.

`--data DIR` selects the input CSV directory, `--out DIR` selects the output directory, and `--only hero contact energy rules` selects the visuals to render; pass fewer names to select a subset, such as `--only contact energy`. The default renders all four. The renderer checks geometry, snapshot alignment and cumulative work, and stops if those inputs disagree. See [the figure descriptions](figures/README.md) for scales, timing and symbols.

## Replay a rule variant in Evolution Gym

The check uses engine flags for the rule table. For an independent environment replay, make a separate copy of the stock tree and apply one `patches/rule-variants/<name>.patch` from its root with `patch -p0`, just like the closing patch. For example:

```sh
cp -R build/stock build/variant-contact
patch -d build/variant-contact -p0 < patches/rule-variants/contact.patch
cmake -S build/variant-contact/evogym/simulator \
  -B build/variant-contact/cmake-variant \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="$(command -v python)" \
  -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$PWD/build/variant-contact/evogym"
cmake --build build/variant-contact/cmake-variant
PYTHONPATH=scripts python - <<'PY'
import json
from replay import replay
print(json.dumps(replay('variant-contact', 'designs/rule-variants/contact/design.json'), indent=1))
PY
```

Use a destination that does not already exist. The fresh CMake directory avoids the copied tree's old build paths. The command calls the replay function because the command-line `tree` argument accepts only `stock` and `closed`. It uses the rebuilt Evolution Gym simulator, not the copied accounting module.

This example uses default maths. For reference maths, run setup with `REFERENCE_MATH=1` first, then add these arguments to the CMake configure command above so the new build uses the same arithmetic:

```sh
-DCMAKE_CXX_FLAGS="-ffp-contract=off" \
-DCMAKE_MODULE_LINKER_FLAGS="$PWD/build/ref_tanh.o" \
-DCMAKE_SHARED_LINKER_FLAGS="$PWD/build/ref_tanh.o"
```

To test the variant with the loophole closed, also apply `patches/close-loophole.patch` before configuring and rebuilding. Use a separate copy to retain both versions. Substitute the chosen variant's name and design path throughout. For closed candidates, select one admissible under that variant; `control` needs 30-step actions repeated twice, and `box7` needs the body embedded in its larger box as in `scripts/check.py`.

## Rerun from public inputs

Start with the stored `design.json` and `actions.npy` files, the pinned Evolution Gym revision and the supplied patches. Setup, check, figure export and rendering regenerate the reported replays, accounts, CSV data and visuals. The optimisation searches that found these designs are not included; this reproduces their measured outcomes, not the search history or a claim of optimality.
