#!/bin/bash
# Idempotent install helper — collapses docs/INSTALL.md into one command.
# Fast/testable path (deps + rosdep + colcon build) runs by default; the heavy
# C++ builds (patched Stonefish, vdbfusion, Open3D) are opt-in via flags.
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

# Stonefish, vdbfusion and Open3D are git submodules under external/, pinned to exact
# commits, so there is nothing to clone or patch by hand and the versions cannot
# drift from what this repo was tested against.
BUILD_STONEFISH=false
STONEFISH_DIR="$REPO_ROOT/external/stonefish"
STONEFISH_BUILD_JOBS="$(nproc 2>/dev/null || echo 2)"

WITH_VDBFUSION=false
VDBFUSION_DIR="$REPO_ROOT/external/vdbfusion"

WITH_OPEN3D=false
OPEN3D_DIR="$REPO_ROOT/external/open3d"
OPEN3D_BUILD_JOBS="$(nproc 2>/dev/null || echo 2)"

MESHES_FROM=""

# Python deps go into a uv-managed virtualenv: Ubuntu 24.04 (ROS 2 Jazzy's
# target) marks its system Python externally-managed, so a plain
# `pip install` there fails with PEP 668. --system-site-packages keeps the
# distro's ROS Python packages visible inside it.
VENV_DIR="$WS_ROOT/.venv"

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
  --with-open3d            Build + pip install the Open3D submodule (FPFH
                            descriptors and point-cloud registration; heavy
                            C++ build). The only option on aarch64, where no
                            wheel is published.
  --open3d-dir <path>      Use an existing Open3D checkout instead of the
                            submodule (default: $OPEN3D_DIR).
  --venv <path>            Virtualenv for the Python dependencies
                            (default: $VENV_DIR). Ignored when a virtualenv is
                            already active.
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
        --with-open3d) WITH_OPEN3D=true; shift ;;
        --open3d-dir) OPEN3D_DIR="$2"; shift 2 ;;
        --venv) VENV_DIR="$2"; shift 2 ;;
        --meshes-from) MESHES_FROM="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

echo "==> 1/8 Sanity checks"
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

echo "==> 2/8 Submodules (pinned Stonefish + vdbfusion + Open3D sources)"
if [ -f "$REPO_ROOT/.gitmodules" ] && [ -d "$REPO_ROOT/.git" ] || \
   [ -f "$REPO_ROOT/.git" ]; then
    ( cd "$REPO_ROOT" && git submodule update --init --recursive )
else
    echo "Not a git checkout, skipping submodule init."
fi

echo "==> 3/8 Python dependencies (uv virtualenv + requirements.txt)"
if [ -n "${VIRTUAL_ENV:-}" ]; then
    VENV_DIR="$VIRTUAL_ENV"
    echo "Using the already-active virtualenv at $VENV_DIR."
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "uv not found, installing it to ~/.local/bin"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "uv still not on PATH after install. Add ~/.local/bin to PATH" \
         "(or install uv yourself: https://docs.astral.sh/uv/) and re-run." >&2
    exit 1
fi
# --allow-existing keeps this script re-runnable; --clear would discard
# whatever the user has already installed into the venv.
# --python /usr/bin/python3 is not optional: uv otherwise bases the venv on its
# own managed CPython, whose --system-site-packages does not include Debian's
# dist-packages — so rclpy imports and then dies on a missing PyYAML.
uv venv --python /usr/bin/python3 --system-site-packages --allow-existing "$VENV_DIR"
VENV_PY="$VENV_DIR/bin/python"
uv pip install --python "$VENV_PY" -r "$REPO_ROOT/requirements.txt"

echo "==> 4/8 rosdep (workspace root: $WS_ROOT)"
( cd "$WS_ROOT" && rosdep install --from-paths src -i -y )

echo "==> 5/8 Patched Stonefish"
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

echo "==> 6/8 vdbfusion (mapper:=tsdf)"
if [ "$WITH_VDBFUSION" = true ]; then
    if [ ! -f "$VDBFUSION_DIR/setup.py" ] && [ ! -f "$VDBFUSION_DIR/pyproject.toml" ]; then
        echo "$VDBFUSION_DIR looks empty. Run" \
             "'git submodule update --init external/vdbfusion' first," \
             "or point --vdbfusion-dir at an existing checkout." >&2
        exit 1
    fi
    uv pip install --python "$VENV_PY" "$VDBFUSION_DIR"
else
    echo "Skipped (pass --with-vdbfusion; only needed for mapper:=tsdf)."
fi

echo "==> 7/8 Open3D (FPFH descriptors)"
if [ "$WITH_OPEN3D" = true ]; then
    if [ ! -f "$OPEN3D_DIR/CMakeLists.txt" ]; then
        echo "$OPEN3D_DIR looks empty. Run" \
             "'git submodule update --init external/open3d' first," \
             "or point --open3d-dir at an existing checkout." >&2
        exit 1
    fi
    # setuptools and wheel are build-time only (the pip-package target runs
    # setup.py), so they live here rather than in requirements.txt.
    uv pip install --python "$VENV_PY" setuptools wheel
    # BUILD_CUDA_MODULE=OFF: no CUDA on the target machines, and its absence
    # otherwise fails configuration rather than degrading.
    # Python3_EXECUTABLE: without it CMake picks whichever python3 is first on
    # PATH, which is not necessarily the venv the wheel gets installed into.
    cmake -S "$OPEN3D_DIR" -B "$OPEN3D_DIR/build" \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_CUDA_MODULE=OFF \
          -DBUILD_GUI=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_UNIT_TESTS=OFF \
          -DBUILD_PYTHON_MODULE=ON \
          -DPython3_EXECUTABLE="$VENV_PY"
    cmake --build "$OPEN3D_DIR/build" -j "$OPEN3D_BUILD_JOBS" --target pip-package
    uv pip install --python "$VENV_PY" \
        "$OPEN3D_DIR"/build/lib/python_package/pip_package/open3d-*.whl
else
    echo "Skipped (pass --with-open3d; only needed for FPFH descriptor work)."
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

echo "==> 8/8 colcon build"
( cd "$WS_ROOT" && colcon build --symlink-install --cmake-args -Wno-dev )

echo "==> Done. Activate the virtualenv, source the workspace, then launch:"
echo "      source $VENV_DIR/bin/activate"
echo "      source $WS_ROOT/install/setup.zsh"
echo "      python3 launcher.py"
