# Install

Full instructions. See the root [`README.md`](../README.md) for the quick version.

## 1. ROS 2 Jazzy

Targets Ubuntu 24.04. Follow the official ROS 2 Jazzy installation guide (search
"ROS 2 Jazzy installation" — the `desktop` variant includes RViz, which this
project uses). On another Linux distro, a container tool like
[distrobox](https://github.com/89luca89/distrobox) is one way to get an
Ubuntu 24.04 userspace for it; that's a host-OS detail, not part of this guide.

## 2. `rosdep`

Not always preinstalled alongside ROS 2 itself:

```bash
sudo apt install python3-rosdep
sudo rosdep init   # only if this is the first time rosdep has been set up on the machine
rosdep update
```

## 3. Patched Stonefish

This project runs a patched build of [Stonefish](https://github.com/patrykcieslak/stonefish),
not the stock package. It comes in as a git submodule at
`external/stonefish`, pinned to an exact commit of the patched fork, so
there is nothing to clone or patch by hand:

```bash
git submodule update --init external/stonefish
./bootstrap.sh --build-stonefish
```

[`sim/stonefish_patches/`](../sim/stonefish_patches/) still carries the patches
as plain files. They document what the fork changes relative to upstream and
record the base commit; the submodule is what actually gets built.

**Submodules are required, not optional.** A plain `git clone` does not fetch
them, and the workspace will not build without `external/stonefish` (the
library `stonefish_ros2` compiles against) or `sim/stonefish_ros2` (a package
`stonefish_groundtruth_mapping` depends on). Clone with
`--recurse-submodules`, or run `git submodule update --init --recursive`
afterwards — `./bootstrap.sh` does this for you. `external/vdbfusion` is the
one genuinely optional submodule; it is only needed for `mapper:=tsdf`.

The ROS 2 bridge is a patched fork too — see
[`stonefish_ros2_fork.md`](stonefish_ros2_fork.md).

## 4. Scene meshes

`sim/world/data/obj/` (~313 MB) is gitignored — see
[`sim/world/data/README.md`](../sim/world/data/README.md). Must be present on
disk (not just cloned) before the demo will load the scenario.

## 5. Workspace + this repo

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone --recurse-submodules git@github.com:Arcod7/ActiveSlam.git
cd ~/ros_ws
rosdep install --from-paths src -i -y
colcon build --symlink-install --cmake-args -Wno-dev
source install/setup.zsh
```

`rosdep` resolves every dependency declared in each package's `package.xml`
(`depth_image_proc`, `octomap_server`, `rviz2`, `python3-numpy`,
`python3-scipy`, ...).

`-Wno-dev` silences CMake dev warnings from PCL's own cmake modules
(`CMP0144`/`CMP0074` about `_ROOT` variables) — noise from upstream PCL, not
this project.

## 6. Python dependencies not covered by `rosdep`

Python dependencies are split by who can install them:

- **`rosdep` handles anything packaged for apt** — `numpy`, `scipy` and
  `matplotlib` are declared in the `package.xml` of each package that imports
  them, so step 5's `rosdep install` already pulled them in. They are
  deliberately not repeated in `requirements.txt`.
- **[`requirements.txt`](../requirements.txt) is only what `rosdep` cannot
  provide** — install with:

```bash
pip install -r requirements.txt   # or --break-system-packages / inside a venv
```

`gtsam` and `small-gicp` (needed only for `slam:=slam`) and `pymavlink` (needed
only for the real ArduSub adapter) are PyPI wheels and install cleanly this way
on aarch64 as well as x86_64. `vdbfusion` (needed only for `mapper:=tsdf`) is
the exception — see below, it needs a source build instead.

**`vdbfusion` is NOT installable via this file.**
PyPI only publishes wheels up to Python 3.10, x86_64
only — there is no wheel for Python 3.11/3.12 (Ubuntu 24.04's default) on any
architecture, so `pip install vdbfusion` fails everywhere on a fresh Jazzy
setup, and on aarch64 (Apple Silicon, Raspberry Pi) a source build is the only
option at all. It is pinned as a submodule for that reason:

```bash
git submodule update --init external/vdbfusion
./bootstrap.sh --with-vdbfusion     # or: pip install external/vdbfusion
```

Follow [vdbfusion's own `INSTALL.md`](https://github.com/PRBonn/vdbfusion/blob/main/INSTALL.md)
first if the build fails — it needs OpenVDB and a C++ toolchain, which aren't
part of this project's own dependency list. Skip this entirely if you only
plan to run the default `mapper:=octomap`.

## Verify

```bash
ros2 launch bringup demo.launch.py
```

Should bring up Stonefish, RViz, and print a reminder to run
`ros2 run launch_tools my_keyboard` in another terminal (the default mode is
`teleop`). See the root README's *Run* section for the `mode`/`mapper`/`slam`
switches.
