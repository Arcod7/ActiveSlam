# ActiveSlam

Active SLAM for underwater volumetric exploration, extending Suresh, Sodhi, Mangelson, Wettergreen & Kaess, *"Active SLAM using 3D Submap Saliency for Underwater Volumetric Exploration"* (IEEE ICRA 2020), along three axes:
- a wide-FoV 3D sonar (90°×40°) in place of their 29°×1° profiling sonar
- FPFH submap descriptors in place of SHOT
- OctoMap→TSDF map backend.

Simulation-only (ROS 2 Jazzy + [Stonefish](https://github.com/patrykcieslak/stonefish)), BlueROV2.

MSc dissertation project, Heriot-Watt University — supervisor Yvan Petillot, Ocean Systems Lab.

See [`docs/ROADMAP.md`](docs/ROADMAP.md) for the phased plan and current status.

## What's in the box

A BlueROV2 in a simulated underwater scene, sensed by a depth camera
configured as a wide-FoV 3D sonar proxy (elevation aperture wired through a
[patched Stonefish](sim/stonefish_patches/)), building a map while either
teleoperated or exploring autonomously via frontier detection.

| Package | Role |
|---|---|
| [`bringup`](bringup) | Unified `demo.launch.py` (mode + mapper switches) and the demo RViz config |
| [`sim/world`](sim/world) | Stonefish scenario: BlueROV2 model, environment meshes, `.scn` config |
| [`sim/stonefish_ros2`](sim/stonefish_ros2) | ROS 2 bridge to the simulator (patched fork) |
| [`slam/stonefish_groundtruth_mapping`](slam/stonefish_groundtruth_mapping) | TF chain, depth image → point cloud, OctoMap/TSDF mapping |
| [`planner/frontier_slam`](planner/frontier_slam) | Frontier-based autonomous exploration (the optional mode) |
| [`tools/launch_tools`](tools/launch_tools) | Keyboard teleop |

## Install

Requires ROS 2 Jazzy and a patched build of Stonefish (patches + build
instructions: [`sim/stonefish_patches/README.md`](sim/stonefish_patches/README.md)).
The scene meshes (~313 MB, gitignored — see
[`sim/world/data/README.md`](sim/world/data/README.md)) must be present on
disk separately; they don't come from `git clone`.

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone git@github.com:Arcod7/ActiveSlam.git
cd ~/ros_ws
rosdep install --from-paths src -i -y
pip install -r src/ActiveSlam/requirements.txt
colcon build --symlink-install
source install/setup.zsh
```

Full details (including if `rosdep` isn't already set up): [`docs/INSTALL.md`](docs/INSTALL.md).

**Not on Ubuntu?** ROS 2 Jazzy targets Ubuntu 24.04. On another Linux distro,
a container tool like [distrobox](https://github.com/89luca89/distrobox) is
one way to get an Ubuntu 24.04 userspace for it — this is a host-OS detail,
not part of the build above; everything from `colcon build` onward is the
same once you have a Jazzy environment.

## Run

One command brings up Stonefish + TF + point cloud + mapper + RViz:

```bash
ros2 launch bringup demo.launch.py
```

Two independent switches, each defaulting to the first value:

```bash
ros2 launch bringup demo.launch.py mode:=teleop|frontier      # operator mode
ros2 launch bringup demo.launch.py mapper:=octomap|tsdf       # map backend
ros2 launch bringup demo.launch.py rviz:=false                # headless
```

`mode:=teleop` (the default) brings up sim+mapper and prints a reminder to
run teleop yourself in another terminal:

```bash
ros2 run launch_tools my_keyboard
```

(Raw keyboard input needs a real terminal — `ros2 launch` can't hand one to
a launched node, so this can't be bundled into the single command above.
`mode:=frontier` doesn't have this problem and launches fully inside
`demo.launch.py`.)

Override the world path with `export STONEFISH_WORLD_DIR=/path/to/sim/world`
if not running from the default checkout location.

Individual pieces are still directly launchable if you don't want the whole
stack — see [`slam/stonefish_groundtruth_mapping/launch/`](slam/stonefish_groundtruth_mapping/launch)
(`tf` → `pointcloud` → `octomap`/`tsdf`, each including the one before it).
