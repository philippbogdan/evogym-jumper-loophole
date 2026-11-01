# Inspect the simulator's energy transfers

This re-implementation exists to separate the energy change at each integration step, replay rule variants and record contacts. `fastsim.h` follows Evolution Gym's binary64 operation order for robot points. `fastjump.cpp` exposes it to Python and uses Evolution Gym's own scene construction, preserving point, edge and cell order. `ground_scene.txt` describes the Jumper-v0 floor and walls.

`Engine.rollout` returns a score and optional centre-of-mass histories. `trace` records point positions and velocities at every integration step. `account` returns energy and signed update terms. `rollout_source` calls Evolution Gym's own `Sim::step`; the separate `scripts/replay.py` runs its Gymnasium environment. The contact-logging build also exposes `contacts`.

Final positions, velocities and returns were verified bit-identical to the source simulator on the reference platform across random bodies and actions for every rule variant. That claim is platform-specific; [reproduction notes](../REPRODUCE.md) describe floating-point sensitivity.

The account measures kinetic plus gravitational potential energy, not stored spring energy. It includes integration and position-projection terms, with spring reset and discard diagnostics reported separately. Agreement of that account establishes a decomposition of the update rule, not physical conservation.

Derived from Evolution Gym. Its [MIT licence](LICENSE) applies; our changes are also offered under [CC0](../LICENSE).
