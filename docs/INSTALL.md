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
not the stock package. See [`sim/stonefish_patches/README.md`](../sim/stonefish_patches/README.md)
for the upstream commit, the patches themselves, and build instructions
(CMake + Stonefish's own 3rdparty dependencies — follow Stonefish's own build
docs for those).

## 4. Scene meshes

`sim/world/data/obj/` (~313 MB) is gitignored — see
[`sim/world/data/README.md`](../sim/world/data/README.md). Must be present on
disk (not just cloned) before the demo will load the scenario.

## 5. Workspace + this repo

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone git@github.com:Arcod7/ActiveSlam.git
cd ~/ros_ws
rosdep install --from-paths src -i -y
pip install -r src/ActiveSlam/requirements.txt
colcon build --symlink-install
source install/setup.zsh
```

`rosdep` resolves the ROS package dependencies declared in each package's
`package.xml` (`depth_image_proc`, `octomap_server`, `rviz2`, `python3-numpy`,
`python3-scipy`, ...). `requirements.txt` covers the Python dependencies
`rosdep` doesn't know about (`numpy`, `scipy`; `vdbfusion` needs a separate
step — see below).

**Expect `rosdep install` to end with an error, and that's fine:**

```
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
stonefish_groundtruth_mapping: Cannot locate rosdep definition for [ament_python]
...
stonefish_ros2: Cannot locate rosdep definition for [pcl]
```

`ament_python` has no rosdep key at all (it ships with the ROS distro itself,
not as a separate system package) and bare `pcl` isn't a resolvable key either
(`pcl_conversions`, which *is* resolvable, pulls in what's actually needed).
Every real runtime dependency (`octomap-server`, `depth-image-proc`, `rviz2`,
etc.) installs before rosdep reports this — the nonzero exit code and error
block are noise, not a broken install.

**`pip install -r requirements.txt` may refuse to run** on a stock Ubuntu
24.04 with `error: externally-managed-environment` (PEP 668). Either add
`--break-system-packages`, or use a virtualenv:

```bash
python3 -m venv ~/ros_ws/venv --system-site-packages
source ~/ros_ws/venv/bin/activate
pip install -r src/ActiveSlam/requirements.txt
```

(`--system-site-packages` is needed so the venv can still see `rclpy` and the
other ROS Python packages installed system-wide.)

**`vdbfusion` (needed only for `mapper:=tsdf`) isn't covered by
`requirements.txt`.** PyPI only publishes wheels up to Python 3.10, x86_64
only — there is no wheel for Python 3.11/3.12 (Ubuntu 24.04's default) on any
architecture, so `pip install vdbfusion` fails everywhere on a fresh Jazzy
setup. Build it from source instead:

```bash
git clone https://github.com/PRBonn/vdbfusion.git
cd vdbfusion
pip install .   # or --break-system-packages / inside the venv above
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
`teleop`). See the root README's *Run* section for the `mode`/`mapper`
switches.
