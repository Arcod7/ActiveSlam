#!/bin/bash
# Idempotent install helper — collapses docs/INSTALL.md into one command.
# Everything a default run needs happens without flags — Stonefish and
# vdbfusion included. Open3D stays opt-in: nothing in a default run uses it.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPENDENCY_CONFIG="$REPO_ROOT/dependencies.conf"
if [ ! -r "$DEPENDENCY_CONFIG" ]; then
    echo "Missing dependency manifest: $DEPENDENCY_CONFIG" >&2
    exit 1
fi
# shellcheck source=dependencies.conf
source "$DEPENDENCY_CONFIG"
ORIGINAL_ARGS=("$@")
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
STONEFISH_ROS2_DIR="$REPO_ROOT/sim/stonefish_ros2"
STONEFISH_BUILD_JOBS="$(nproc 2>/dev/null || echo 2)"

# vdbfusion backs mapper:=tsdf, which the launcher offers by default, so it is
# installed like GTSAM rather than behind a flag: wheel first, source only where
# no wheel matches. --skip-vdbfusion opts out.
SKIP_VDBFUSION=false
VDBFUSION_DIR="$REPO_ROOT/external/vdbfusion"

WITH_OPEN3D=false
OPEN3D_DIR="$REPO_ROOT/external/open3d"
OPEN3D_BUILD_JOBS="$(nproc 2>/dev/null || echo 2)"

MESHES_FROM=""
GTSAM_DIR="$REPO_ROOT/$GTSAM_SUBMODULE_PATH"
# The generated Python bindings have a high peak memory use.  A serial build is
# reliable in constrained containers; callers with enough RAM can override it.
GTSAM_BUILD_JOBS="${ACTIVESLAM_GTSAM_BUILD_JOBS:-1}"

# Python deps go into a uv-managed virtualenv. --system-site-packages keeps
# the distro's ROS Python packages visible inside it (and avoids PEP 668 on
# Ubuntu 24.04).
VENV_DIR="$WS_ROOT/.venv"
DISTROBOX_NAME="activeslam-jazzy"
DISTROBOX_IMAGE="quay.io/toolbx/ubuntu-toolbox:24.04"

select_python_for_ros_distro() {
    case "${ROS_DISTRO:-}" in
        jazzy)
            SYSTEM_PYTHON="/usr/bin/python3.12"
            EXPECTED_PYTHON_VERSION="3.12"
            ;;
        humble)
            SYSTEM_PYTHON="/usr/bin/python3.10"
            EXPECTED_PYTHON_VERSION="3.10"
            ;;
        *)
            SYSTEM_PYTHON=""
            EXPECTED_PYTHON_VERSION=""
            return 1
            ;;
    esac
}

python_is_expected() {
    "$1" -c "import sys; raise SystemExit(sys.version_info[:2] != (${EXPECTED_PYTHON_VERSION%.*}, ${EXPECTED_PYTHON_VERSION#*.}))"
}

gtsam_installed_version() {
    "$VENV_PY" -c 'import importlib.metadata as m; print(m.version("gtsam"))' 2>/dev/null
}

install_gtsam() {
    # Prefer a prebuilt wheel. The pinned source submodule is the portable
    # fallback for interpreters and architectures no wheel covers.
    #
    # Version-checked rather than merely importable: a GTSAM left behind by an
    # earlier run — a source build, or a wheel from a since-changed pin — keeps
    # importing happily, so testing importability alone pinned every machine to
    # whatever it installed first and made re-running this a no-op.
    GTSAM_INSTALLED="$(gtsam_installed_version)"
    case "$GTSAM_INSTALLED" in
        "$GTSAM_WHEEL_VERSION"|"$GTSAM_SOURCE_REF")
            echo "GTSAM $GTSAM_INSTALLED already installed."
            return
            ;;
    esac
    if [ -n "$GTSAM_INSTALLED" ]; then
        echo "GTSAM $GTSAM_INSTALLED is installed, but the pins are" \
             "$GTSAM_WHEEL_VERSION (wheel) and $GTSAM_SOURCE_REF (source)." \
             "Replacing it with the pinned wheel."
        if "$UV" pip install --python "$VENV_PY" --reinstall \
                "gtsam==$GTSAM_WHEEL_VERSION"; then
            return
        fi
        # Rebuilding from source here would repeat a long build on every run
        # for platforms no wheel covers, so say so and leave it alone.
        echo "No $GTSAM_WHEEL_VERSION wheel for this platform; keeping GTSAM" \
             "$GTSAM_INSTALLED. Rebuild it from $GTSAM_SUBMODULE_PATH by hand" \
             "if it misbehaves." >&2
        return
    fi

    if "$UV" pip install --python "$VENV_PY" "gtsam==$GTSAM_WHEEL_VERSION"; then
        return
    fi

    echo "No compatible GTSAM $GTSAM_WHEEL_VERSION wheel; building" \
         "$GTSAM_SOURCE_REF from the $GTSAM_SUBMODULE_PATH submodule."
    sudo apt-get install -y build-essential cmake libboost-all-dev libtbb-dev python3-dev
    if [ -f "$REPO_ROOT/.git" ] || [ -d "$REPO_ROOT/.git" ]; then
        ( cd "$REPO_ROOT" && git submodule update --init --recursive "$GTSAM_SUBMODULE_PATH" )
    fi
    if [ ! -f "$GTSAM_DIR/CMakeLists.txt" ]; then
        echo "$GTSAM_DIR has no GTSAM sources. Initialise the" \
             "$GTSAM_SUBMODULE_PATH submodule and re-run." >&2
        exit 1
    fi
    cmake -S "$GTSAM_DIR" -B "$GTSAM_DIR/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DGTSAM_BUILD_PYTHON=ON \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF \
        -DGTSAM_PYTHON_VERSION="$EXPECTED_PYTHON_VERSION" \
        -DPython_EXECUTABLE="$SYSTEM_PYTHON" \
        -DPYTHON_EXECUTABLE="$SYSTEM_PYTHON"
    cmake --build "$GTSAM_DIR/build" -j "$GTSAM_BUILD_JOBS"
    sudo cmake --install "$GTSAM_DIR/build"
    sudo ldconfig

    # GTSAM's regular CMake install intentionally omits the Python module;
    # its generated package lives in build/python. Copy it first: setuptools
    # writes egg-info beside setup.py, and that build tree may contain files
    # created by a prior sudo install. This places the prebuilt binding in the
    # venv without compiling it again.
    GTSAM_PYTHON_PACKAGE="$(mktemp -d)"
    cp -R "$GTSAM_DIR/build/python/." "$GTSAM_PYTHON_PACKAGE/"
    # Keep this build's cache under the selected venv. A shared uv cache may
    # have entries created by sudo during an earlier container install.
    UV_CACHE_DIR="$VENV_DIR/.uv-cache" \
        "$UV" pip install --python "$VENV_PY" "$GTSAM_PYTHON_PACKAGE"
    rm -rf "$GTSAM_PYTHON_PACKAGE"
    "$VENV_PY" -c 'import gtsam; print("GTSAM Python bindings:", gtsam.__file__)'
}

install_vdbfusion() {
    # Same shape as install_gtsam: a wheel where one exists, the pinned source
    # submodule otherwise. vdbfusion publishes x86_64 wheels only, and only up
    # to CPython 3.10, so Humble on x86_64 gets the wheel and everything else
    # builds — which needs OpenVDB and a C++ toolchain.
    if "$VENV_PY" -c 'import vdbfusion' >/dev/null 2>&1; then
        echo "Already installed."
        return
    fi

    if "$UV" pip install --python "$VENV_PY" "vdbfusion==$VDBFUSION_WHEEL_VERSION"; then
        return
    fi

    echo "No compatible vdbfusion $VDBFUSION_WHEEL_VERSION wheel; building from" \
         "the external/vdbfusion submodule."
    sudo apt-get install -y build-essential cmake libeigen3-dev libtbb-dev \
        libblosc-dev libboost-iostreams-dev
    if [ "$VDBFUSION_DIR" = "$REPO_ROOT/external/vdbfusion" ] \
       && { [ -f "$REPO_ROOT/.git" ] || [ -d "$REPO_ROOT/.git" ]; }; then
        ( cd "$REPO_ROOT" && git submodule update --init external/vdbfusion )
    fi
    if [ ! -f "$VDBFUSION_DIR/setup.py" ] && [ ! -f "$VDBFUSION_DIR/pyproject.toml" ]; then
        echo "$VDBFUSION_DIR has no vdbfusion sources in it. Initialise the" \
             "external/vdbfusion submodule and re-run." >&2
        exit 1
    fi
    "$UV" pip install --python "$VENV_PY" "$VDBFUSION_DIR"
    "$VENV_PY" -c 'import vdbfusion; print("vdbfusion:", vdbfusion.__file__)'
}

setup_distrobox_and_continue() {
    if [ -e /run/.containerenv ] || [ -e /.dockerenv ]; then
        echo "This shell is already inside a container. Exit it and re-run" \
             "bootstrap.sh from the Ubuntu host to create $DISTROBOX_NAME." >&2
        exit 1
    fi

    if ! command -v podman >/dev/null 2>&1; then
        echo "==> Installing Podman (Distrobox's container engine)"
        sudo apt-get update
        sudo apt-get install -y podman
    fi

    if ! command -v distrobox >/dev/null 2>&1; then
        if ! command -v curl >/dev/null 2>&1; then
            sudo apt-get update
            sudo apt-get install -y curl
        fi
        echo "==> Installing Distrobox with its official installer"
        curl -fsSL https://raw.githubusercontent.com/89luca89/distrobox/main/install \
            | sudo sh
    fi

    DISTROBOX_FLAGS=(
        --name "$DISTROBOX_NAME"
        --image "$DISTROBOX_IMAGE"
        --yes
    )
    ACTIVESLAM_EXPECT_NVIDIA=0
    if command -v nvidia-smi >/dev/null 2>&1 || [ -d /proc/driver/nvidia ]; then
        DISTROBOX_FLAGS+=(--nvidia)
        ACTIVESLAM_EXPECT_NVIDIA=1
        echo "NVIDIA drivers detected; enabling Distrobox NVIDIA integration."
    fi

    if ! podman container exists "$DISTROBOX_NAME"; then
        echo "==> Creating Ubuntu 24.04 Distrobox: $DISTROBOX_NAME"
        DBX_CONTAINER_MANAGER=podman distrobox create "${DISTROBOX_FLAGS[@]}"
    else
        echo "==> Reusing existing Distrobox: $DISTROBOX_NAME"
    fi

    echo "==> Entering $DISTROBOX_NAME and preparing ROS 2 Jazzy"
    DBX_CONTAINER_MANAGER=podman distrobox enter --name "$DISTROBOX_NAME" -- \
        env ACTIVESLAM_DISTROBOX_BOOTSTRAP=1 \
        ACTIVESLAM_EXPECT_NVIDIA="$ACTIVESLAM_EXPECT_NVIDIA" \
        bash "$REPO_ROOT/tools/setup_jazzy_distrobox_guest.sh" \
        "$REPO_ROOT" "${ORIGINAL_ARGS[@]}"
    exit $?
}

unsupported_environment() {
    if [ ! -t 0 ] || [ ! -t 1 ]; then
        return 0
    fi

    echo "ActiveSlam needs ROS 2 Humble (Ubuntu 22.04, Python 3.10) or" \
         "ROS 2 Jazzy (Ubuntu 24.04, Python 3.12)."
    echo "This shell has ROS_DISTRO='${ROS_DISTRO:-unset}' and" \
         "$(/usr/bin/python3 --version 2>&1 || echo 'no /usr/bin/python3')."
    echo ""
    echo "Distrobox setup will install Podman if needed, run Distrobox's official"
    echo "curl installer with sudo, create an Ubuntu 24.04 container, enable NVIDIA"
    echo "integration when detected, install ROS 2 Jazzy, and resume bootstrap."
    printf "Set up and use the '%s' Distrobox now? [y/N] " "$DISTROBOX_NAME"
    read -r DISTROBOX_REPLY
    case "$DISTROBOX_REPLY" in
        y|Y|yes|YES|Yes) setup_distrobox_and_continue ;;
    esac
    return 0
}

usage() {
    cat <<EOF
Usage: ./bootstrap.sh [options]

With no options it does everything a default run needs: submodules, Python
deps, rosdep, the patched Stonefish (unless already installed), vdbfusion,
colcon build.

  --skip-stonefish         Do not build or install Stonefish. Use when it is
                            installed somewhere CMake finds but this script
                            does not look (it checks the usual prefixes for a
                            StonefishConfig.cmake whose headers carry the API
                            stonefish_ros2 needs, and rebuilds when they do
                            not).
  --stonefish-dir <path>   Use an existing Stonefish checkout instead of the
                            submodule (default: $STONEFISH_DIR).
  --skip-vdbfusion         Do not install vdbfusion. mapper:=tsdf needs it, and
                            without it that mapper exits on import. Installed
                            from a wheel where one matches; elsewhere (aarch64,
                            CPython 3.11+) built from the submodule, which is
                            heavy — it needs OpenVDB and a C++ toolchain.
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
        --skip-vdbfusion) SKIP_VDBFUSION=true; shift ;;
        # Accepted for compatibility: installing vdbfusion is now the default.
        --with-vdbfusion) shift ;;
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
if ! select_python_for_ros_distro || ! command -v ros2 >/dev/null 2>&1 || \
   [ ! -x "$SYSTEM_PYTHON" ] || ! python_is_expected "$SYSTEM_PYTHON"; then
    unsupported_environment
    echo "Unsupported environment. Enter ROS 2 Humble on Ubuntu 22.04 or ROS 2" \
         "Jazzy on Ubuntu 24.04, source its setup.bash (or setup.zsh), and re-run" \
         "this script." >&2
    exit 1
fi
if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdep not found. Install it first:" \
         "'sudo apt install python3-rosdep && sudo rosdep init && rosdep update'" \
         "(see docs/INSTALL.md)." >&2
    exit 1
fi

echo "==> 2/8 Submodules (patched Stonefish + ROS 2 bridge)"
# Only the two required submodules by default. vdbfusion is init'd on demand,
# by the fallback in install_vdbfusion, and Open3D by --with-open3d: Open3D
# alone is ~350 MB, a long clone to impose on everyone for an optional feature.
REQUIRED_SUBMODULES="external/stonefish sim/stonefish_ros2"
if [ -f "$REPO_ROOT/.gitmodules" ] && [ -d "$REPO_ROOT/.git" ] || \
   [ -f "$REPO_ROOT/.git" ]; then
    # shellcheck disable=SC2086
    ( cd "$REPO_ROOT" && git submodule update --init --recursive $REQUIRED_SUBMODULES )
else
    echo "Not a git checkout, skipping submodule init."
fi

# The bridge is a fork already, but keep distro compatibility corrections as
# explicit, idempotent patches beside this repository. This lets the pinned
# submodule remain a reproducible upstream commit.
if [ -d "$STONEFISH_ROS2_DIR/.git" ] || [ -f "$STONEFISH_ROS2_DIR/.git" ]; then
    for patch_file in "$REPO_ROOT/$STONEFISH_ROS2_PATCH_DIR"/*.patch; do
        [ -e "$patch_file" ] || continue
        if git -C "$STONEFISH_ROS2_DIR" apply --reverse --check "$patch_file" \
            >/dev/null 2>&1; then
            continue
        fi
        git -C "$STONEFISH_ROS2_DIR" apply "$patch_file"
        echo "Applied $(basename "$patch_file") to stonefish_ros2."
    done
fi

echo "==> 3/8 Python dependencies (uv virtualenv + requirements.txt)"
if [ -n "${VIRTUAL_ENV:-}" ]; then
    VENV_DIR="$VIRTUAL_ENV"
    echo "Using the already-active virtualenv at $VENV_DIR."
fi
if [ -x "$VENV_DIR/bin/python" ] && ! python_is_expected "$VENV_DIR/bin/python"; then
    VENV_PY_VERSION="$("$VENV_DIR/bin/python" --version 2>&1 || echo unknown)"
    if [ "${ACTIVESLAM_DISTROBOX_BOOTSTRAP:-}" = "1" ]; then
        VENV_BACKUP="${VENV_DIR}.pre-${ROS_DISTRO}-$(date +%Y%m%d-%H%M%S)"
        mv "$VENV_DIR" "$VENV_BACKUP"
        echo "Moved incompatible $VENV_PY_VERSION venv to $VENV_BACKUP."
    else
        echo "$VENV_DIR uses $VENV_PY_VERSION instead of the required Python $EXPECTED_PYTHON_VERSION." \
             "Deactivate it, move or remove that venv, and re-run bootstrap." >&2
        exit 1
    fi
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
# --python $SYSTEM_PYTHON is not optional: uv otherwise bases the venv on its
# own managed CPython, whose --system-site-packages does not include Debian's
# dist-packages — so rclpy imports and then dies on a missing PyYAML.
"$UV" venv --python "$SYSTEM_PYTHON" --system-site-packages --allow-existing "$VENV_DIR"
VENV_PY="$VENV_DIR/bin/python"
if ! python_is_expected "$VENV_PY"; then
    VENV_PY_VERSION="$("$VENV_PY" --version 2>&1 || echo unavailable)"
    echo "$VENV_DIR uses $VENV_PY_VERSION instead of the required Python $EXPECTED_PYTHON_VERSION." \
        "Deactivate it, move or remove that venv, and re-run bootstrap from" \
        "a supported ROS environment so it is recreated with" \
        "$SYSTEM_PYTHON." >&2
    exit 1
fi
"$UV" pip install --python "$VENV_PY" -r "$REPO_ROOT/requirements.txt"
install_gtsam

echo "==> 4/8 rosdep (workspace root: $WS_ROOT)"
( cd "$WS_ROOT" && rosdep install --from-paths src -i -y )

echo "==> 5/8 Patched Stonefish"
stonefish_prefix() {
    for prefix in /usr/local /usr /opt/stonefish; do
        [ -f "$prefix/lib/cmake/Stonefish/StonefishConfig.cmake" ] \
            && { echo "$prefix"; return 0; }
    done
    return 1
}

# Methods stonefish_ros2 calls that exist only in the patched fork. An older
# Stonefish satisfies find_package(Stonefish) just as well, so testing only
# that something is installed lets a stale one through — and it surfaces as a
# compile error deep in the bridge rather than here.
stonefish_missing_api() {
    for pair in "sensors/vision/Camera.h:getLastCaptureTime" \
                "sensors/vision/DepthCamera.h:getVerticalFOV"; do
        grep -q "${pair##*:}" "$1/include/Stonefish/${pair%%:*}" 2>/dev/null \
            || echo "${pair##*:}"
    done
}

STONEFISH_PREFIX="$(stonefish_prefix || true)"
STONEFISH_MISSING_API=""
if [ -n "$STONEFISH_PREFIX" ]; then
    STONEFISH_MISSING_API="$(stonefish_missing_api "$STONEFISH_PREFIX" \
        | paste -sd' ' -)"
fi
if [ "$SKIP_STONEFISH" = true ]; then
    echo "Skipped (--skip-stonefish)."
elif [ -n "$STONEFISH_PREFIX" ] && [ -z "$STONEFISH_MISSING_API" ]; then
    echo "Already installed at $STONEFISH_PREFIX, skipping the build."
else
    if [ -n "$STONEFISH_PREFIX" ]; then
        echo "The Stonefish installed at $STONEFISH_PREFIX predates the" \
             "patched fork — it has no $STONEFISH_MISSING_API, which" \
             "stonefish_ros2 calls. Rebuilding and installing over it."
    fi
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
    # Reinstalling over an older copy leaves the linker cache pointing at it.
    sudo ldconfig
    echo "Stonefish built and installed from $STONEFISH_DIR/build."
fi

echo "==> 6/8 vdbfusion (mapper:=tsdf)"
if [ "$SKIP_VDBFUSION" = true ]; then
    echo "Skipped (--skip-vdbfusion). mapper:=tsdf will not run."
else
    install_vdbfusion
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
# The BlueROV2 meshes are tracked in git. Of what stays out of band, only this
# one is loaded by a scenario (scenario/waterlinked.scn); the rest are
# unreferenced, so a checkout without them is still complete.
REQUIRED_OUT_OF_BAND_MESHES="obj/off_shore_station.obj"
if [ -n "$MESHES_FROM" ]; then
    mkdir -p "$MESH_DIR"
    cp -r "$MESHES_FROM"/. "$MESH_DIR"/
    echo "Copied meshes from $MESHES_FROM into $MESH_DIR."
fi
# Hash what is actually on disk instead of running sha256sum -c over the whole
# manifest: the manifest also lists the unreferenced meshes, and those would be
# reported as failures purely for being absent.
MESH_INTACT=true
if [ -f "$MESH_MANIFEST" ]; then
    MESH_CHECKLIST="$(mktemp)"
    while read -r mesh_sum mesh_path; do
        case "$mesh_sum" in ''|'#'*) continue ;; esac
        [ -f "$MESH_DATA_DIR/$mesh_path" ] \
            && printf '%s  %s\n' "$mesh_sum" "$mesh_path"
    done < "$MESH_MANIFEST" > "$MESH_CHECKLIST"
    if [ -s "$MESH_CHECKLIST" ]; then
        # sha256sum has already named the file at fault; errexit would abort
        # before the hint below and the build.
        if ! ( cd "$MESH_DATA_DIR" && sha256sum -c --quiet "$MESH_CHECKLIST" ) >&2
        then
            MESH_INTACT=false
            echo "Recopy the mesh above with --meshes-from <path>." >&2
        fi
    fi
    rm -f "$MESH_CHECKLIST"
else
    echo "No mesh manifest at $MESH_MANIFEST, skipping the integrity check." >&2
fi
MESH_MISSING=""
for mesh in $REQUIRED_OUT_OF_BAND_MESHES; do
    [ -f "$MESH_DATA_DIR/$mesh" ] || MESH_MISSING="$MESH_MISSING $mesh"
done
if [ -n "$MESH_MISSING" ]; then
    echo "Missing scene meshes:$MESH_MISSING" >&2
    echo "These are the meshes git does not carry. Copy them from an existing" \
         "checkout with --meshes-from <path>, or see" \
         "sim/world/data/README.md." >&2
elif [ "$MESH_INTACT" = true ]; then
    echo "Scene meshes present."
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
