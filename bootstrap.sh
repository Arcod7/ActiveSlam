#!/bin/bash
# Idempotent install helper — collapses docs/INSTALL.md into one command.
# Fast/testable path (deps + rosdep + colcon build) runs by default; the heavy
# C++ builds (patched Stonefish, vdbfusion) are opt-in via flags.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$REPO_ROOT")"
if [ "$(basename "$PARENT_DIR")" = "src" ]; then
    WS_ROOT="$(dirname "$PARENT_DIR")"
else
    WS_ROOT="$PARENT_DIR"
    echo "Note: $REPO_ROOT is not under a .../src/ directory — using $WS_ROOT" \
         "as the colcon workspace root. Clone this repo into <ws>/src/ActiveSlam" \
         "for the standard layout (see docs/INSTALL.md)." >&2
fi

BUILD_STONEFISH=false
STONEFISH_DIR="$REPO_ROOT/../stonefish"
STONEFISH_UPSTREAM="https://github.com/patrykcieslak/stonefish.git"
STONEFISH_BASE_COMMIT="09208f913daf688cc8abc775d45742c860c81f19"

WITH_VDBFUSION=false
VDBFUSION_DIR="$REPO_ROOT/../vdbfusion"
VDBFUSION_UPSTREAM="https://github.com/PRBonn/vdbfusion.git"

MESHES_FROM=""

usage() {
    cat <<EOF
Usage: ./bootstrap.sh [options]

Fast path (default): pip deps, rosdep, colcon build.

  --build-stonefish        Clone + patch the pinned Stonefish commit and print
                            its own build instructions (heavy C++ build, not
                            run automatically — see sim/stonefish_patches/README.md).
  --stonefish-dir <path>   Where to clone Stonefish (default: $STONEFISH_DIR).
  --with-vdbfusion         Clone + pip install vdbfusion (needed for mapper:=tsdf;
                            heavy — needs OpenVDB + a C++ toolchain).
  --vdbfusion-dir <path>   Where to clone vdbfusion (default: $VDBFUSION_DIR).
  --meshes-from <path>     Copy sim/world/data/obj/ from an existing checkout.
  -h, --help               Show this help.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --build-stonefish) BUILD_STONEFISH=true; shift ;;
        --stonefish-dir) STONEFISH_DIR="$2"; shift 2 ;;
        --with-vdbfusion) WITH_VDBFUSION=true; shift ;;
        --vdbfusion-dir) VDBFUSION_DIR="$2"; shift 2 ;;
        --meshes-from) MESHES_FROM="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

echo "==> 1/6 Sanity checks"
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not found on PATH. Enter the ROS 2 Jazzy environment first" \
         "(e.g. 'distrobox enter ros2-jazzy && source /opt/ros/jazzy/setup.zsh')" \
         "and re-run this script." >&2
    exit 1
fi
if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdep not found. Install it first:" \
         "'sudo apt install python3-rosdep && sudo rosdep init && rosdep update'" \
         "(see docs/INSTALL.md)." >&2
    exit 1
fi

echo "==> 2/6 Python dependencies (requirements.txt)"
pip install -r "$REPO_ROOT/requirements.txt"

echo "==> 3/6 rosdep (workspace root: $WS_ROOT)"
( cd "$WS_ROOT" && rosdep install --from-paths src -i -y )

echo "==> 4/6 Patched Stonefish"
if [ "$BUILD_STONEFISH" = true ]; then
    if [ -d "$STONEFISH_DIR" ]; then
        echo "$STONEFISH_DIR already exists, skipping clone/patch."
    else
        git clone "$STONEFISH_UPSTREAM" "$STONEFISH_DIR"
        ( cd "$STONEFISH_DIR" && \
          git checkout "$STONEFISH_BASE_COMMIT" && \
          git am "$REPO_ROOT"/sim/stonefish_patches/*.patch )
    fi
    echo "Stonefish cloned and patched at $STONEFISH_DIR." \
         "Build it following Stonefish's own CMake instructions" \
         "(see sim/stonefish_patches/README.md) — not run automatically here."
else
    echo "Skipped (pass --build-stonefish to clone + patch it)."
fi

echo "==> 5/6 vdbfusion (mapper:=tsdf)"
if [ "$WITH_VDBFUSION" = true ]; then
    if [ -d "$VDBFUSION_DIR" ]; then
        echo "$VDBFUSION_DIR already exists, skipping clone."
    else
        git clone "$VDBFUSION_UPSTREAM" "$VDBFUSION_DIR"
    fi
    ( cd "$VDBFUSION_DIR" && pip install . )
else
    echo "Skipped (pass --with-vdbfusion; only needed for mapper:=tsdf)."
fi

echo "==> Scene meshes"
MESH_DIR="$REPO_ROOT/sim/world/data/obj"
if [ -n "$MESHES_FROM" ]; then
    mkdir -p "$MESH_DIR"
    cp -r "$MESHES_FROM"/. "$MESH_DIR"/
    echo "Copied meshes from $MESHES_FROM into $MESH_DIR."
elif [ -d "$MESH_DIR" ] && [ -n "$(ls -A "$MESH_DIR" 2>/dev/null)" ]; then
    echo "$MESH_DIR already populated, skipping."
else
    echo "$MESH_DIR is empty. Mesh provenance is not automated yet" \
         "(see sim/world/data/README.md) — pass --meshes-from <path> to copy" \
         "them from an existing checkout, or place them there manually" \
         "before building." >&2
fi

echo "==> 6/6 colcon build"
( cd "$WS_ROOT" && colcon build --symlink-install --cmake-args -Wno-dev )

echo "==> Done. source $WS_ROOT/install/setup.zsh, then: python3 launcher.py"
