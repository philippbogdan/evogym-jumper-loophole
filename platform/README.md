# Reference maths for replay

`ref_tanh.c` keeps the simulator's friction calculation independent of the system maths library. It ports the fdlibm `tanh` and `expm1` algorithm with the operation order used by glibc's generic double-precision code. Last-bit differences can change contact timing enough to alter a launch.

Setup uses system maths by default (`REFERENCE_MATH=0`). With `REFERENCE_MATH=1`, it compiles this file with `CC` (default `cc`), disables fused multiply-add with `-ffp-contract=off`, and links the object into the simulator and accounting modules, including contact logging. Evolution Gym's source is unchanged. [Reproduction instructions](../REPRODUCE.md#reference-maths-option) give the command and platform results; the option has so far been verified only on x86-64 Linux, where it reproduced the reference values exactly. It was not measured on the Apple M4; the default Apple silicon build reproduces the main launch to seven significant figures but not the sensitive designs.

This is fdlibm-derived code, not Evolution Gym code. It carries the Sun Microsystems permission notice at the top of [ref_tanh.c](ref_tanh.c): use, copying, modification and distribution are permitted provided that notice is preserved. Its licence is separate from the repository's CC0 dedication and Evolution Gym's MIT licence; see [third-party notices](../THIRD_PARTY_NOTICES.md).
