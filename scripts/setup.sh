#!/bin/bash
# Fetch pinned Evolution Gym sources and build stock, closed and contact-logging modules.
# Requires the Python dependencies, git, CMake and a C++ compiler; downloads need network access.
# PYTHON selects the interpreter, CXX the C++ compiler and BUILD_THREADS the CMake build concurrency.
# REFERENCE_MATH=1 disables fused multiply-add and links platform/ref_tanh.c; default 0 uses system maths.
# CC selects the C compiler for the reference-maths object (default cc).
# Replaces build/stock and build/closed; build/src retains the pinned checkout.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PY="${PYTHON:-python}"
COMMIT=f63c649c688adda1accad5f86731aad96606388a
B="$ROOT/build"
mkdir -p "$B"
REF="${REFERENCE_MATH:-0}"
EXTRA_CXX=""; EXTRA_OBJ=""
if [ "$REF" = 1 ]; then
  ${CC:-cc} -O2 -ffp-contract=off -fno-builtin -fPIC -c "$ROOT/platform/ref_tanh.c" -o "$B/ref_tanh.o"
  EXTRA_CXX="-ffp-contract=off"; EXTRA_OBJ="$B/ref_tanh.o"
fi

if [ ! -d "$B/src/.git" ]; then
  git clone --quiet https://github.com/EvolutionGym/evogym.git "$B/src"
fi
git -C "$B/src" checkout --quiet "$COMMIT"
git -C "$B/src" submodule update --init --quiet \
  evogym/simulator/externals/glfw evogym/simulator/externals/glew evogym/simulator/externals/pybind11

build_engine () {
  local D="$1" MOD="$2" OUT="$3"; shift 3
  local S="$D/evogym/simulator"
  local SIMSRC=""
  for f in Sim Environment PhysicsEngine ObjectCreator SimObject Robot Boxel Edge BBTreeNode Snapshot; do
    SIMSRC="$SIMSRC $S/SimulatorCPP/$f.cpp"
  done
  local EXT; EXT="$("$PY" -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")"
  local FLAGS="-shared -fPIC"
  if [ "$(uname)" = "Darwin" ]; then FLAGS="$FLAGS -undefined dynamic_lookup"; fi
  mkdir -p "$OUT"
  ${CXX:-c++} -O3 -DNDEBUG -std=c++14 $EXTRA_CXX $FLAGS -fvisibility=hidden -w "$@" \
    -I"$S/SimulatorCPP" -I"$S/externals/eigen" -I"$S/externals/glew/include" -I"$S/externals/glfw/include/GLFW" \
    -DGLEW_STATIC -DMODNAME="$MOD" $("$PY" -m pybind11 --includes) \
    "$ROOT/engine/fastjump.cpp" $SIMSRC $EXTRA_OBJ -o "$OUT/$MOD$EXT"
}

build_tree () {
  local name="$1"; shift
  local D="$B/$name"
  rm -rf "$D"; mkdir -p "$D"
  (cd "$B/src" && git archive --format=tar HEAD) | tar -xf - -C "$D"
  for sub in glfw glew pybind11; do
    (cd "$B/src/evogym/simulator/externals/$sub" && git archive --format=tar HEAD) | \
      (mkdir -p "$D/evogym/simulator/externals/$sub" && tar -xf - -C "$D/evogym/simulator/externals/$sub")
  done
  for p in "$@"; do (cd "$D" && patch -s -p0 < "$p"); done
  mkdir -p "$D/cmake"
  (cd "$D/cmake" && CMAKE_POLICY_VERSION_MINIMUM=3.5 cmake "$D/evogym/simulator" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="$EXTRA_CXX" -DCMAKE_MODULE_LINKER_FLAGS="$EXTRA_OBJ" -DCMAKE_SHARED_LINKER_FLAGS="$EXTRA_OBJ" \
      -DPYTHON_EXECUTABLE="$(command -v "$PY")" -DCMAKE_LIBRARY_OUTPUT_DIRECTORY="$D/evogym" > "$D/build.log" 2>&1 \
    && cmake --build . -j"${BUILD_THREADS:-4}" >> "$D/build.log" 2>&1) || { tail -30 "$D/build.log"; exit 1; }
  ls "$D"/evogym/simulator_cpp*.so > /dev/null
  build_engine "$D" "jumpengine_$name" "$D"
  echo "built $name: $D"
}

build_tree stock
build_tree closed "$ROOT/patches/close-loophole.patch"
build_engine "$B/stock" jumpengine_stock_debug "$B/stock_debug" -DCONTACT_DEBUG
echo "built stock_debug: $B/stock_debug"
