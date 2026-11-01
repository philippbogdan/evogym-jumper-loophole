# Third-party notices

Our original code, data, figures and text are offered under [CC0 1.0](LICENSE). Third-party material retains its own licence.

## Evolution Gym

`engine/` re-implements Evolution Gym's integration step and includes a floor scene derived from its world file. `patches/` contains diffs of its source. Both retain the MIT licence and copyright below, also supplied in `engine/LICENSE` and `patches/LICENSE`. Our changes are additionally offered under CC0; this does not remove the upstream notice requirement.

Source: [EvolutionGym/evogym](https://github.com/EvolutionGym/evogym), commit `f63c649c688adda1accad5f86731aad96606388a`.

Reference: J. S. Bhatia, H. Jackson, Y. Tian, J. Xu and W. Matusik. *Evolution Gym: A Large-Scale Benchmark for Evolving Soft Robots*. Advances in Neural Information Processing Systems 34, 2021. [arXiv:2201.09863](https://arxiv.org/abs/2201.09863).

```text
MIT License

Copyright (c) 2022 jagdeepsb

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## fdlibm reference maths

[`platform/ref_tanh.c`](platform/ref_tanh.c) ports the fdlibm `tanh`/`expm1` algorithm with the operation order used by glibc's generic double-precision code. It is not Evolution Gym code and is not covered by that project's MIT notice or the CC0 dedication for our original work. The Sun Microsystems permission notice is preserved in the file and reproduced here:

```text
Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.

Developed at SunSoft, a Sun Microsystems, Inc. business.
Permission to use, copy, modify, and distribute this
software is freely granted, provided that this notice
is preserved.
```

The optional `REFERENCE_MATH=1` build links this file so the simulator's friction calculation does not depend on the system `tanh`. See [the platform README](platform/README.md).

## Dependencies fetched during setup

The publication files do not bundle these dependencies. Setup downloads them into the local build or Python environment, where their own notices apply.

| Dependency | Licence |
|---|---|
| Evolution Gym | MIT |
| Eigen | MPL-2.0 |
| GLFW | zlib |
| GLEW | Modified BSD and MIT |
| pybind11, simulator submodule and Python package | BSD-3-Clause |
| NumPy | BSD-3-Clause |
| Gymnasium | MIT |
| CMake | BSD-3-Clause |
