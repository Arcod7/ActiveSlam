#!/bin/bash
# Idempotent install helper — collapses docs/INSTALL.md into one command.
# Everything a default run needs happens without flags, Stonefish included;
# vdbfusion and Open3D are opt-in, being heavy builds nothing needs by default.
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

# Stonefish, vdbfusion and Open3D are git submodules under external/, pinned to
# exact commits, so there is nothing to clone or patch by hand and the versions
# cannot drift from what this repo was tested against.
# Stonefish is not optional — the workspace does not build without it — so it is
# built on demand rather than behind a flag: skipped when already installed,
# built when not. --skip-stonefish forces the skip for an install CMake cannot find.
SKIP_STONEFISH=false
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
SYSTEM_PYTHON="/usr/bin/python3.12"

usage() {
    cat <<EOF
Usage: ./bootstrap.sh [options]

With no options it does everything a default run needs: submodules, Python
deps, rosdep, the patched Stonefish (unless already installed), colcon build.

  --skip-stonefish         Do not build or install Stonefish. Use when it is
                            installed somewhere CMake finds but this script
                            does not look (it checks for StonefishConfig.cmake
                            under the usual prefixes).
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
        --skip-stonefish) SKIP_STONEFISH=true; shift ;;
        # Accepted for compatibility: building Stonefish is now the default.
        --build-stonefish) shift ;;
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
if [ "${ROS_DISTRO:-}" != "jazzy" ]; then
    echo "ActiveSlam requires ROS 2 Jazzy, but ROS_DISTRO is" \
         "'${ROS_DISTRO:-unset}'. Enter a Jazzy/Ubuntu 24.04 shell, source" \
         "'/opt/ros/jazzy/setup.bash' (or setup.zsh), and re-run this script." >&2
    exit 1
fi
if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdep not found. Install it first:" \
         "'sudo apt install python3-rosdep && sudo rosdep init && rosdep update'" \
         "(see docs/INSTALL.md)." >&2
    exit 1
fi

# A venv isolates packages but keeps the Python implementation/version it was
# created from. The supported Jazzy/Ubuntu 24.04 environment supplies Python
# 3.12, and we select that exact interpreter below. Catch an
# Ubuntu 22.04/Humble-style Python 3.10 environment before uv produces a much
# less actionable dependency-resolution error.
python_is_expected() {
    "$1" -c 'import sys; raise SystemExit(sys.version_info[:2] != (3, 12))'
}
if [ ! -x "$SYSTEM_PYTHON" ] || ! python_is_expected "$SYSTEM_PYTHON"; then
    echo "ActiveSlam requires $SYSTEM_PYTHON from the supported ROS 2 Jazzy /" \
         "Ubuntu 24.04 environment. A virtualenv does not change the Python" \
         "version it is created from. Enter that environment and re-run this" \
         "script." >&2
    exit 1
fi

echo "==> 2/8 Submodules (patched Stonefish + ROS 2 bridge)"
# Only the two required submodules by default. vdbfusion and Open3D are init'd
# by --with-vdbfusion/--with-open3d instead: Open3D alone is ~350 MB, which is
# a long clone to impose on everyone for an optional feature.
REQUIRED_SUBMODULES="external/stonefish sim/stonefish_ros2"
if [ -f "$REPO_ROOT/.gitmodules" ] && [ -d "$REPO_ROOT/.git" ] || \
   [ -f "$REPO_ROOT/.git" ]; then
    # shellcheck disable=SC2086
    ( cd "$REPO_ROOT" && git submodule update --init --recursive $REQUIRED_SUBMODULES )
else
    echo "Not a git checkout, skipping submodule init."
fi

echo "==> 3/8 Python dependencies (uv virtualenv + requirements.txt)"
if [ -n "${VIRTUAL_ENV:-}" ]; then
    VENV_DIR="$VIRTUAL_ENV"
    echo "Using the already-active virtualenv at $VENV_DIR."
fi
UV="$(command -v uv || true)"
if [ -z "$UV" ]; then
    echo "uv not found, installing it to ~/.local/bin"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    UV="$HOME/.local/bin/uv"
fi
# Deliberately an absolute path rather than PATH="$HOME/.local/bin:$PATH":
# prepending that directory puts any uv-managed CPython in it ahead of
# /usr/bin, and CMake's FindPython3 then picks an interpreter without
# catkin_pkg, breaking the colcon build in step 8.
if [ ! -x "$UV" ]; then
    echo "uv is still not executable at $UV. Install it yourself" \
         "(https://docs.astral.sh/uv/) and re-run." >&2
    exit 1
fi
# --allow-existing keeps this script re-runnable; --clear would discard
# whatever the user has already installed into the venv.
# --python /usr/bin/python3.12 is not optional: uv otherwise bases the venv on its
# own managed CPython, whose --system-site-packages does not include Debian's
# dist-packages — so rclpy imports and then dies on a missing PyYAML.
"$UV" venv --python "$SYSTEM_PYTHON" --system-site-packages --allow-existing "$VENV_DIR"
VENV_PY="$VENV_DIR/bin/python"
if ! python_is_expected "$VENV_PY"; then
    VENV_PY_VERSION="$("$VENV_PY" --version 2>&1 || echo unavailable)"
    echo "$VENV_DIR uses $VENV_PY_VERSION instead of the required Python 3.12." \
         "Deactivate it, move or remove that venv, and re-run bootstrap from" \
         "the supported Jazzy/Ubuntu 24.04 environment so it is recreated with" \
         "$SYSTEM_PYTHON." >&2
    exit 1
fi
"$UV" pip install --python "$VENV_PY" -r "$REPO_ROOT/requirements.txt"

echo "==> 4/8 rosdep (workspace root: $WS_ROOT)"
( cd "$WS_ROOT" && rosdep install --from-paths src -i -y )

echo "==> 5/8 Patched Stonefish"
stonefish_installed() {
    for prefix in /usr/local /usr /opt/stonefish; do
        [ -f "$prefix/lib/cmake/Stonefish/StonefishConfig.cmake" ] && return 0
    done
    return 1
}
if [ "$SKIP_STONEFISH" = true ]; then
    echo "Skipped (--skip-stonefish)."
elif stonefish_installed; then
    echo "Already installed, skipping the build. Force a rebuild by removing" \
         "$STONEFISH_DIR/build and the installed StonefishConfig.cmake."
else
    if [ ! -d "$STONEFISH_DIR/Library" ]; then
        echo "$STONEFISH_DIR looks empty. Run" \
             "'git submodule update --init external/stonefish' first," \
             "or point --stonefish-dir at an existing checkout." >&2
        exit 1
    fi
    # Stonefish's own dependencies (its README): glm, SDL2, Freetype, OpenGL.
    # rosdep does not cover them — it only reads this workspace's package.xml files.
    sudo apt-get install -y libglm-dev libsdl2-dev libfreetype-dev libgl1-mesa-dev
    cmake -S "$STONEFISH_DIR" -B "$STONEFISH_DIR/build" \
          -DCMAKE_BUILD_TYPE=Release
    cmake --build "$STONEFISH_DIR/build" -j "$STONEFISH_BUILD_JOBS"
    # Installing is not optional: stonefish_ros2 does find_package(Stonefish),
    # which only resolves against an installed StonefishConfig.cmake.
    sudo cmake --install "$STONEFISH_DIR/build"
    echo "Stonefish built and installed from $STONEFISH_DIR/build."
fi

echo "==> 6/8 vdbfusion (mapper:=tsdf)"
if [ "$WITH_VDBFUSION" = true ]; then
    if [ "$VDBFUSION_DIR" = "$REPO_ROOT/external/vdbfusion" ]; then
        ( cd "$REPO_ROOT" && git submodule update --init external/vdbfusion )
    fi
    if [ ! -f "$VDBFUSION_DIR/setup.py" ] && [ ! -f "$VDBFUSION_DIR/pyproject.toml" ]; then
        echo "$VDBFUSION_DIR has no vdbfusion sources in it." >&2
        exit 1
    fi
    "$UV" pip install --python "$VENV_PY" "$VDBFUSION_DIR"
else
    echo "Skipped (pass --with-vdbfusion; only needed for mapper:=tsdf)."
fi

echo "==> 7/8 Open3D (FPFH descriptors)"
if [ "$WITH_OPEN3D" = true ]; then
    if [ "$OPEN3D_DIR" = "$REPO_ROOT/external/open3d" ]; then
        echo "Fetching the Open3D submodule (~350 MB)..."
        ( cd "$REPO_ROOT" && git submodule update --init external/open3d )
    fi
    if [ ! -f "$OPEN3D_DIR/CMakeLists.txt" ]; then
        echo "$OPEN3D_DIR has no Open3D sources in it." >&2
        exit 1
    fi
    # setuptools and wheel are build-time only (the pip-package target runs
    # setup.py), so they live here rather than in requirements.txt.
    "$UV" pip install --python "$VENV_PY" setuptools wheel
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
    "$UV" pip install --python "$VENV_PY" \
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
        # || true: sha256sum's failure is the expected case here, and errexit
        # would otherwise abort before the hint below and the build.
        ( cd "$MESH_DATA_DIR" && sha256sum -c "$MESH_MANIFEST" 2>&1 \
            | grep -vE ': OK$' | sed 's/^/  /' ) >&2 || true
        echo "" >&2
        echo "obj/ is gitignored and distributed out of band. Copy it from an" \
             "existing checkout with --meshes-from <path>, or see" \
             "sim/world/data/README.md." >&2
    fi
else
    echo "No mesh manifest at $MESH_MANIFEST, skipping the integrity check." >&2
fi

echo "==> 8/8 colcon build"
# Built with the venv first on PATH — exactly what `source .venv/bin/activate`
# does, and the same build a user gets by doing that themselves. It matters
# twice over: the ament_python entry points take their shebang from whichever
# interpreter runs colcon, and only the venv can import gtsam and the other
# wheels; and CMake's FindPython3 takes the first python3 on PATH, which on a
# machine with a uv- or pyenv-managed Python in ~/.local/bin is one without
# catkin_pkg. The venv answers both, being /usr/bin/python3.12 with
# --system-site-packages.
( cd "$WS_ROOT" && PATH="$VENV_DIR/bin:$PATH" \
    colcon build --symlink-install --cmake-args -Wno-dev )

echo "==> Done. Activate the virtualenv, source the workspace, then launch:"
echo "      source $VENV_DIR/bin/activate"
echo "      source $WS_ROOT/install/setup.zsh"
echo "      python3 launcher.py"
