#!/usr/bin/env bash
# Build the ORB-SLAM3 submodule and its Thirdparty dependencies against
# whatever CMake prefix is already active (typically $CONDA_PREFIX set by
# `conda activate orbslam-spatial`). Idempotent: skips steps already done.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SUB="$REPO_ROOT/thirdparty/ORB_SLAM3"

if [ -z "${CONDA_PREFIX:-}" ]; then
    echo "ERROR: CONDA_PREFIX not set. Activate the env first:" >&2
    echo "  conda activate orbslam-spatial" >&2
    exit 1
fi

JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)

# Ensure patches are applied before any compilation
"$SCRIPT_DIR/apply-patches.sh"

build_subdir() {
    local name="$1"
    local dir="$2"
    shift 2
    if [ -f "$dir/build/CMakeCache.txt" ]; then
        echo "==> $name: reusing existing build dir"
    else
        echo "==> $name: configuring"
        mkdir -p "$dir/build"
        (cd "$dir/build" && cmake .. \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
            "$@")
    fi
    echo "==> $name: building (-j$JOBS)"
    cmake --build "$dir/build" -- -j"$JOBS"
}

build_subdir "DBoW2"  "$SUB/Thirdparty/DBoW2"
build_subdir "g2o"    "$SUB/Thirdparty/g2o"
# Sophus unit tests use deprecated copy-assignment patterns rejected by modern
# clang (-Werror,-Wdeprecated-copy). We only need the Sophus headers, not the
# tests, so skip them entirely.
build_subdir "Sophus" "$SUB/Thirdparty/Sophus" -DBUILD_TESTS=OFF
build_subdir "ORB_SLAM3" "$SUB"

echo
echo "ORB-SLAM3 build complete."
echo "  libORB_SLAM3 -> $SUB/lib/"
ls -lh "$SUB/lib/" 2>/dev/null | tail -5 || true
