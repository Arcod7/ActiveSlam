#!/bin/bash
# Runs inside the Ubuntu 24.04 Distrobox created by bootstrap.sh.
set -euo pipefail

REPO_ROOT="$1"
shift

if [ ! -f "$REPO_ROOT/bootstrap.sh" ]; then
    echo "The ActiveSlam checkout is not visible inside Distrobox at:" \
         "$REPO_ROOT" >&2
    exit 1
fi

# Do not carry a sourced host Humble environment or its venv into Jazzy.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO \
      ROS_VERSION ROS_PYTHON_VERSION VIRTUAL_ENV || true

# shellcheck disable=SC1091
source /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
    echo "Expected Ubuntu 24.04 (noble), found '${VERSION_CODENAME:-unknown}'." >&2
    exit 1
fi

# Some Ubuntu 22.04 NVIDIA installations leave old libnvidia-container,
# nvidia-container-runtime and nvidia-docker apt feeds behind. Distrobox's
# --nvidia setup bind-mounts those definitions into the Noble guest without
# their legacy signing key, making every apt update fail. A bind mount must not
# be renamed or edited: that could affect the host. Instead, point apt at a
# container-local directory of symlinks to every guest source except those
# legacy NVIDIA feeds. --nvidia still exposes the host driver libraries.
ACTIVESLAM_APT_SOURCE_VIEW="/etc/apt/activeslam.sources.list.d"
ACTIVESLAM_APT_MAIN_VIEW="/etc/apt/activeslam.sources.list"
ACTIVESLAM_APT_SOURCE_CONFIG="/etc/apt/apt.conf.d/99activeslam-sourceparts"

refresh_apt_source_view() {
    local source source_name

    sudo mkdir -p "$ACTIVESLAM_APT_SOURCE_VIEW"
    # This directory belongs to ActiveSlam. Rebuild only its symlinks so a
    # source that later becomes invalid cannot survive here from an older run.
    sudo find "$ACTIVESLAM_APT_SOURCE_VIEW" -mindepth 1 -maxdepth 1 \
        -type l -delete
    for source in \
        /etc/apt/sources.list.d/*.list \
        /etc/apt/sources.list.d/*.sources; do
        [ -f "$source" ] || continue
        source_name="$(basename "$source")"
        if grep -Eq \
            'nvidia\.github\.io/(libnvidia-container|nvidia-container-runtime|nvidia-docker)' \
            "$source"; then
            echo "Ignoring host-mounted legacy NVIDIA apt source: $source"
            continue
        fi
        sudo ln -sfn "$source" "$ACTIVESLAM_APT_SOURCE_VIEW/$source_name"
    done

    # sources.list is normally empty on Noble (ubuntu.sources is used), but
    # filter a legacy main file into a container-local copy as well.
    if [ -f /etc/apt/sources.list ]; then
        sed -E \
            '\|nvidia\.github\.io/(libnvidia-container|nvidia-container-runtime|nvidia-docker)|d' \
            /etc/apt/sources.list \
            | sudo tee "$ACTIVESLAM_APT_MAIN_VIEW" >/dev/null
    else
        printf '' | sudo tee "$ACTIVESLAM_APT_MAIN_VIEW" >/dev/null
    fi
}

refresh_apt_source_view
printf 'Dir::Etc::sourcelist "%s";\nDir::Etc::sourceparts "%s";\n' \
    "$ACTIVESLAM_APT_MAIN_VIEW" "$ACTIVESLAM_APT_SOURCE_VIEW" \
    | sudo tee "$ACTIVESLAM_APT_SOURCE_CONFIG" >/dev/null

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "==> Installing ROS 2 Jazzy Desktop inside Distrobox"
    sudo apt-get update
    sudo apt-get install -y locales software-properties-common curl
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo add-apt-repository -y universe

    ACTIVESLAM_ROS_APT_VERSION="$(
        curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
            | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' \
            | head -n 1
    )"
    if [ -z "$ACTIVESLAM_ROS_APT_VERSION" ]; then
        echo "Could not determine the latest ros2-apt-source release." >&2
        exit 1
    fi
    ACTIVESLAM_ROS_APT_DEB="$(
        mktemp --suffix=.deb /tmp/activeslam-ros-apt-source.XXXXXX
    )"
    trap 'rm -f "$ACTIVESLAM_ROS_APT_DEB"' EXIT
    curl -fL -o "$ACTIVESLAM_ROS_APT_DEB" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ACTIVESLAM_ROS_APT_VERSION}/ros2-apt-source_${ACTIVESLAM_ROS_APT_VERSION}.noble_all.deb"
    sudo dpkg -i "$ACTIVESLAM_ROS_APT_DEB"
    # ros2-apt-source just added a legitimate file to sources.list.d; expose
    # it through the filtered container-local view before the next apt update.
    refresh_apt_source_view
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-desktop ros-dev-tools python3-rosdep
fi

if ! command -v rosdep >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y python3-rosdep
fi
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# setup.bash is generated code and is not guaranteed to be nounset-clean.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

if [ "${ACTIVESLAM_EXPECT_NVIDIA:-0}" = "1" ]; then
    if command -v nvidia-smi >/dev/null 2>&1; then
        echo "NVIDIA integration available inside Distrobox:"
        nvidia-smi --query-gpu=name --format=csv,noheader || true
    else
        echo "Warning: nvidia-smi is unavailable inside Distrobox. If this" \
             "container was created before NVIDIA integration was enabled," \
             "recreate it with 'distrobox create --nvidia'." >&2
    fi
fi

exec "$REPO_ROOT/bootstrap.sh" "$@"
