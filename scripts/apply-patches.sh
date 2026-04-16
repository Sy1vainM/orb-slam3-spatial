#!/usr/bin/env bash
# Apply all patches to the ORB_SLAM3 submodule.
# Idempotent: safe to run repeatedly — checks if patches are already applied.
#
# Usage: scripts/apply-patches.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SUBMODULE="$REPO_ROOT/thirdparty/ORB_SLAM3"
PATCH_DIR="$REPO_ROOT/patches"

if [ ! -d "$SUBMODULE/.git" ] && [ ! -f "$SUBMODULE/.git" ]; then
    echo "ERROR: submodule not initialized. Run: git submodule update --init --recursive" >&2
    exit 1
fi

cd "$SUBMODULE"

# Check for any pre-existing local modifications
if ! git diff --quiet HEAD; then
    echo "Submodule has local modifications; assuming patches already applied. Skipping." >&2
    exit 0
fi

applied=0
for patch in "$PATCH_DIR"/*.patch; do
    name="$(basename "$patch")"
    if git apply --check "$patch" 2>/dev/null; then
        git apply "$patch"
        echo "  applied: $name"
        applied=$((applied + 1))
    else
        # Already applied or conflicts — check via reverse apply
        if git apply --reverse --check "$patch" 2>/dev/null; then
            echo "  already applied: $name"
        else
            echo "ERROR: $name cannot be applied cleanly." >&2
            echo "Likely cause: submodule upstream changed, patches need regeneration." >&2
            exit 1
        fi
    fi
done

echo "Patches status: $applied newly applied."
