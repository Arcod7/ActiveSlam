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
`python3-scipy`, ...). `requirements.txt` covers the one Python dependency
`rosdep` doesn't know about: `vdbfusion` (used only by `tsdf_mapper`, the
TSDF map backend — needed for `mapper:=tsdf`, not for the default `octomap`
mapper).

## Verify

```bash
ros2 launch bringup demo.launch.py
```

Should bring up Stonefish, RViz, and print a reminder to run
`ros2 run launch_tools my_keyboard` in another terminal (the default mode is
`teleop`). See the root README's *Run* section for the `mode`/`mapper`
switches.
