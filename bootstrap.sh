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

# Stonefish and vdbfusion are git submodules under external/, pinned to exact
# commits, so there is nothing to clone or patch by hand and the versions cannot
# drift from what this repo was tested against.
BUILD_STONEFISH=false
STONEFISH_DIR="$REPO_ROOT/external/stonefish"
STONEFISH_BUILD_JOBS="$(nproc 2>/dev/null || echo 2)"

WITH_VDBFUSION=false
VDBFUSION_DIR="$REPO_ROOT/external/vdbfusion"

MESHES_FROM=""

usage() {
    cat <<EOF
Usage: ./bootstrap.sh [options]

Fast path (default): pip deps, rosdep, colcon build.

  --build-stonefish        Build the patched Stonefish submodule and install it
                            (heavy C++ build; needs Stonefish's own 3rdparty
                            dependencies).
  --stonefish-dir <path>   Use an existing Stonefish checkout instead of the
                            submodule (default: $STONEFISH_DIR).
  --with-vdbfusion         Build + pip install the vdbfusion submodule (needed
                            for mapper:=tsdf; heavy — needs OpenVDB and a C++
                            toolchain). The only option on aarch64, where no
                            wheel is published.
  --vdbfusion-dir <path>   Use an existing vdbfusion checkout instead of the
                            submodule (default: $VDBFUSION_DIR).
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

echo "==> 1/7 Sanity checks"
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

echo "==> 2/7 Submodules (pinned Stonefish + vdbfusion sources)"
if [ -f "$REPO_ROOT/.gitmodules" ] && [ -d "$REPO_ROOT/.git" ] || \
   [ -f "$REPO_ROOT/.git" ]; then
    ( cd "$REPO_ROOT" && git submodule update --init --recursive )
else
    echo "Not a git checkout, skipping submodule init."
fi

echo "==> 3/7 Python dependencies (requirements.txt)"
pip install -r "$REPO_ROOT/requirements.txt"

echo "==> 4/7 rosdep (workspace root: $WS_ROOT)"
( cd "$WS_ROOT" && rosdep install --from-paths src -i -y )

echo "==> 5/7 Patched Stonefish"
if [ "$BUILD_STONEFISH" = true ]; then
    if [ ! -d "$STONEFISH_DIR/Library" ]; then
        echo "$STONEFISH_DIR looks empty. Run" \
             "'git submodule update --init external/stonefish' first," \
             "or point --stonefish-dir at an existing checkout." >&2
        exit 1
    fi
    cmake -S "$STONEFISH_DIR" -B "$STONEFISH_DIR/build" \
          -DCMAKE_BUILD_TYPE=Release
    cmake --build "$STONEFISH_DIR/build" -j "$STONEFISH_BUILD_JOBS"
    echo "Stonefish built at $STONEFISH_DIR/build." \
         "Install it (sudo cmake --install) if it is not already on the" \
         "library path — see docs/INSTALL.md."
else
    echo "Skipped (pass --build-stonefish to build the pinned submodule)."
fi

echo "==> 6/7 vdbfusion (mapper:=tsdf)"
if [ "$WITH_VDBFUSION" = true ]; then
    if [ ! -f "$VDBFUSION_DIR/setup.py" ] && [ ! -f "$VDBFUSION_DIR/pyproject.toml" ]; then
        echo "$VDBFUSION_DIR looks empty. Run" \
             "'git submodule update --init external/vdbfusion' first," \
             "or point --vdbfusion-dir at an existing checkout." >&2
        exit 1
    fi
    pip install "$VDBFUSION_DIR"
else
    echo "Skipped (pass --with-vdbfusion; only needed for mapper:=tsdf)."
fi

echo "==> Scene meshes"
MESH_DATA_DIR="$REPO_ROOT/sim/world/data"
MESH_DIR="$MESH_DATA_DIR/obj"
MESH_MANIFEST="$MESH_DATA_DIR/obj.sha256"
if [ -n "$MESHES_FROM" ]; then
    mkdir -p "$MESH_DIR"
    cp -r "$MESHES_FROM"/. "$MESH_DIR"/
    echo "Copied meshes from $MESHES_FROM into $MESH_DIR."
fi
# Check against the manifest rather than just testing that the directory is
# non-empty: a partial or corrupted copy otherwise fails much later, inside the
# simulator, with no indication of which file is at fault.
if [ -f "$MESH_MANIFEST" ]; then
    if ( cd "$MESH_DATA_DIR" && sha256sum -c --quiet "$MESH_MANIFEST" 2>/dev/null ); then
        echo "All meshes present and matching $(basename "$MESH_MANIFEST")."
    else
        echo "Mesh assets are missing or do not match the manifest:" >&2
        ( cd "$MESH_DATA_DIR" && sha256sum -c "$MESH_MANIFEST" 2>&1 \
            | grep -vE ': OK$' | sed 's/^/  /' ) >&2
        echo "" >&2
        echo "obj/ is gitignored and distributed out of band. Copy it from an" \
             "existing checkout with --meshes-from <path>, or see" \
             "sim/world/data/README.md." >&2
    fi
else
    echo "No mesh manifest at $MESH_MANIFEST, skipping the integrity check." >&2
fi

echo "==> 7/7 colcon build"
( cd "$WS_ROOT" && colcon build --symlink-install --cmake-args -Wno-dev )

echo "==> Done. source $WS_ROOT/install/setup.zsh, then: python3 launcher.py"
