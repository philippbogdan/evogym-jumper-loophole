# Patches to Evolution Gym

All apply to commit `f63c649` with `patch -p0` from the root of its tree. `scripts/setup.sh` applies them for you.

- `close-loophole.patch` closes the loophole, and adds a test of it to Evolution Gym's own. A free point that lies within a fixed cell and has no qualifying edge is pushed out through the cell's nearest surface edge, if that edge is less than half a cell away, instead of receiving no force: [what it does and does not do](../docs/mechanism.md#7-the-patch).
- `close-loophole-plain.patch` uses the cell's top edge, in six lines, without the check that the point lies within the cell. It also pushes corners that are not in the floor: [the plain patch](../docs/evidence.md#the-plain-patch).
- `close-loophole-top.patch` uses the cell's top edge, with that check. It also pushes a point that touches terrain from below up through the whole cell: [the top-edge patch](../docs/evidence.md#the-top-edge-patch).
- `probe.patch` counts, without changing any arithmetic, the robot points that sit inside fixed terrain with no contact force on them, and records how far the deepest is from the terrain's surface.
- `probe-fix.patch` and `probe-top.patch`, applied after `close-loophole.patch` and `close-loophole-top.patch`, count the pushes each makes where the released simulator makes none, and record the deepest, without changing any arithmetic.
- `rule-variants/` holds fifteen patches, each changing one rule of the task. [The table.](../docs/rules.md)
- `baseline-examples.patch` applies to the benchmark's `examples/` and is what the [rerun of its baseline](../scope/README.md) used.

These are diffs of Evolution Gym's source and keep its [MIT licence](LICENSE). Our changes are also offered under [CC0](../LICENSE).
