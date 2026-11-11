# The step, rewritten to be measured

`fastsim.h` is Evolution Gym's integration step for the robot's points, written as plain loops in the same order of operations as the source, in binary64. `fastjump.cpp` wraps it for Python and builds the scene with Evolution Gym's own `ObjectCreator`, so points, edges and cells keep the source's indices. It exists to do three things the source cannot: split each step's energy change among the forces, replay a design under a changed rule without rebuilding, and report every contact.

| Call | Returns |
|---|---|
| `Engine(scene, params).rollout(actions, T)` | score, commands completed, instability flag |
| `.account(actions, T)` | energy after each command and the signed work of every term of the update |
| `.trace(actions, T)` | every point's position and velocity at every step |
| `.unpushed()` | after a replay: point-steps spent inside the floor with no floor force, and the deepest such point |
| `.rollout_source(actions, T)` | the same replay through Evolution Gym's own `Sim::step` |
| `.contacts()` | every contact applied at the current state (debug build) |

Positions, velocities and scores agree with the source simulator to the last bit on the reference platform, for random bodies and commands under every rule setting. `python scripts/check.py` compares the two on the headline designs on yours.

Derived from Evolution Gym: its [MIT licence](LICENSE) applies. Our changes are also offered under [CC0](../LICENSE).
