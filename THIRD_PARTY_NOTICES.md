# Third-party notices

What is ours here (code, data, figures, text) is offered under [CC0 1.0](LICENSE). What derives from other work keeps that work's licence.

## Evolution Gym

`engine/` re-implements Evolution Gym's integration step and includes a floor scene derived from its world file. `patches/` holds diffs of its source. Both keep the MIT licence and copyright below, also supplied as `engine/LICENSE` and `patches/LICENSE`. Our changes are additionally offered under CC0, which does not remove the notice requirement.

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

## fdlibm

[`platform/ref_tanh.c`](platform/ref_tanh.c) ports the fdlibm `tanh` and `expm1` algorithm in the order of operations of glibc's generic double-precision code. It is not Evolution Gym code and is covered neither by that project's notice nor by our CC0 dedication. Its notice is kept in the file and reproduced here:

```text
Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.

Developed at SunSoft, a Sun Microsystems, Inc. business.
Permission to use, copy, modify, and distribute this
software is freely granted, provided that this notice
is preserved.
```

## The released robots

The robots and policies replayed in `scope/` are the datasets [EvoGym/robots](https://huggingface.co/datasets/EvoGym/robots) and [EvoGym/robots-with-policies](https://huggingface.co/datasets/EvoGym/robots-with-policies), released by the authors of Evolution Gym under CC BY-NC 4.0. They are not copied into this repository: `scope/fetch.py` downloads them. The tables in `results/` hold our measurements on those robots, keyed by the datasets' identifiers, with the datasets' own reward beside each and, for six robots, the body. Those identifiers, rewards and bodies are quoted from the datasets and stay under their licence.

## Fetched during setup, not bundled

| Dependency | Licence |
|---|---|
| Evolution Gym | MIT |
| Eigen | MPL-2.0 |
| GLFW | zlib |
| GLEW | Modified BSD and MIT |
| pybind11 | BSD-3-Clause |
| NumPy | BSD-3-Clause |
| Gymnasium | MIT |
| CMake | BSD-3-Clause |
| PyTorch, Stable-Baselines3, PyArrow (for `scope/` only) | BSD-3-Clause, MIT, Apache-2.0 |
| [evogym-datasets](https://github.com/EvolutionGym/evogym-datasets) (for `scope/` only) | no licence file; fetched, not redistributed |
