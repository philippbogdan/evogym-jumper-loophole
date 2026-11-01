# Reference values and local measurements

`expected.json` holds the reference values quoted in the main README. `rules` maps each rule name to its best open return, best admissible closed return, selected closed design, estimated share, the open winner's own closed return, and energy diagnostics. `share` is `1 - best_closed / best`, an estimate from best-found results rather than a bound.

The bundled `numbers.json` comes from the default x86-64 Linux build (glibc 2.35, GCC 11.4, Python 3.10.12, Intel Core i9-13900K). All 62 checks passed, with every equality comparison exact; rebuilding with `REFERENCE_MATH=1` produced identical contents. Each run of `scripts/check.py` replaces this file with local measurements.

Its headline fields are direct Evolution Gym replays; `energy` contains the engine accounts and `variants` contains engine comparisons. In particular, `best_closed` replays the stored closed winner, whereas `variants.base.best_closed` selects the best local replay across all admissible stored candidates. They can differ on another platform.

Work and energy are in joules, heights in metres, and returns are benchmark scores. `closure` is the absolute energy-account residual divided by the largest absolute account term. Release steps and actions are indexed from zero. [Platform notes](../REPRODUCE.md) explain why local results may differ from the reference.
