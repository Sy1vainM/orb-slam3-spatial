#!/usr/bin/env bash
# End-to-end build: ORB-SLAM3 submodule + our wrapper.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)

"$SCRIPT_DIR/build-orb-slam3.sh"

echo
echo "==> Building wrapper (mono_tum + spatial_publisher)"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j"$JOBS"

echo
echo "Wrapper build complete."
echo "  mono_tum -> $(pwd)/mono_tum"
