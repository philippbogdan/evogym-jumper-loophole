# Reference maths

The launch is chaotic: a difference in the last bit of one force can move a score. Three things make last bits differ between machines. `scripts/setup.sh` removes two of them by default:

- fused multiply-add, which some compilers use when the processor has it: switched off with `-ffp-contract=off`;
- the system's `tanh`, which the simulator's friction calls: replaced by `ref_tanh.c`, the fdlibm algorithm in the order of operations of glibc's generic double-precision code.

The third cannot be removed without changing Evolution Gym. The simulator lists an object's surface cells by iterating over a `std::unordered_set` (`SimObject.cpp:237-305`) and builds its collision tree in that order, so the order in which two contact forces on one point are added depends on the C++ standard library. GNU's and LLVM's differ. For the two-cell robot the same contacts are applied in a different order from step 1755, and the first bit of its state differs at step 1757.

So x86-64 Linux with glibc and GNU's library is the reference, and its numbers are exact there. On Apple silicon 87 of the 90 checked values agree to six significant figures. The other three are the score of a launch design once the loophole is closed, under the rule settings `no_hmuscle`, `horizon` and `dt`: the robot tumbles to rest, and where it rests differs in the second decimal place (-1.85 for -1.90, -2.89 for -2.88, -0.88 for -0.89).

Evolution Gym's source is not changed. `REFERENCE_MATH=0 bash scripts/setup.sh` builds with the system's maths and the compiler's defaults instead.

`ref_tanh.c` is derived from fdlibm, not from Evolution Gym. It carries the Sun Microsystems notice at its top: use, copying, modification and distribution are permitted provided the notice is preserved. See [third-party notices](../THIRD_PARTY_NOTICES.md).
