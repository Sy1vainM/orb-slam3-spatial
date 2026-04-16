#!/usr/bin/env bash
# Build Pangolin from source and install into $CONDA_PREFIX.
# Pangolin is not on conda-forge, so we build it ourselves once the rest of
# the toolchain (cmake, GLEW, libpng, libjpeg-turbo) is available via conda.
#
# Idempotent: skips if Pangolin CMake config already exists in the env.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -z "${CONDA_PREFIX:-}" ]; then
    echo "ERROR: CONDA_PREFIX not set. Activate the env first:" >&2
    echo "  conda activate orbslam-spatial" >&2
    exit 1
fi

if [ -f "$CONDA_PREFIX/lib/cmake/Pangolin/PangolinConfig.cmake" ]; then
    echo "Pangolin already installed at $CONDA_PREFIX/lib/cmake/Pangolin — skipping."
    exit 0
fi

PANGOLIN_DIR="$REPO_ROOT/thirdparty/Pangolin"
if [ ! -d "$PANGOLIN_DIR/.git" ] && [ ! -f "$PANGOLIN_DIR/.git" ]; then
    echo "==> Cloning Pangolin (shallow)"
    git -C "$REPO_ROOT" submodule add --depth 1 \
        https://github.com/stevenlovegrove/Pangolin.git \
        thirdparty/Pangolin 2>/dev/null || true
    git -C "$REPO_ROOT" submodule update --init --depth 1 thirdparty/Pangolin
fi

JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)

mkdir -p "$PANGOLIN_DIR/build"
cd "$PANGOLIN_DIR/build"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
    -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_PANGOLIN_VIDEO=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_REALSENSE=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LIBOPENNI=OFF \
    -DBUILD_PANGOLIN_LIBOPENNI2=OFF \
    -DBUILD_PANGOLIN_PLEORA=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_TOON=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF

cmake --build . -- -j"$JOBS"
cmake --install .

echo
echo "Pangolin installed into $CONDA_PREFIX"
