#!/usr/bin/env bash
# End-to-end build: ORB-SLAM3 submodule + our wrapper, driven by the conda
# env described in environment.yml. Fails fast if the env is not active.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

if [ -z "${CONDA_PREFIX:-}" ]; then
    echo "ERROR: CONDA_PREFIX not set. Activate the env first:" >&2
    echo "  conda env create -f environment.yml    # first time only" >&2
    echo "  conda activate orbslam-spatial" >&2
    exit 1
fi

JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)

# Make sure Pangolin is built into the env (idempotent)
"$SCRIPT_DIR/build-pangolin.sh"

# Patches + ORB-SLAM3 build (idempotent)
"$SCRIPT_DIR/build-orb-slam3.sh"

echo
echo "==> Building wrapper (mono_tum + spatial_publisher)"
mkdir -p build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build . -- -j"$JOBS"

echo
echo "Wrapper build complete."
echo "  mono_tum -> $(pwd)/mono_tum"
