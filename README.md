# ActiveSlam

Active SLAM for underwater volumetric exploration, extending Suresh,
Sodhi, Mangelson, Wettergreen & Kaess, *"Active SLAM using 3D Submap
Saliency for Underwater Volumetric Exploration"* (IEEE ICRA 2020), along
three axes: a wide-FoV 3D sonar (90°×40°) in place of their 29°×1°
profiling sonar, FPFH submap descriptors in place of SHOT, and an
OctoMap→TSDF map backend. Simulation-only (ROS 2 Jazzy + [Stonefish](https://github.com/patrykcieslak/stonefish)),
BlueROV2. MSc dissertation project, Heriot-Watt University — supervisor
Yvan Petillot, Ocean Systems Lab.

See [`docs/ROADMAP.md`](docs/ROADMAP.md) for the phased plan and current status.

## What's in the box

A BlueROV2 in a simulated underwater scene, sensed by a depth camera
configured as a wide-FoV 3D sonar proxy (elevation aperture wired through a
[patched Stonefish](sim/stonefish_patches/)), building a map while either
teleoperated or exploring autonomously via frontier detection.

| Package | Role |
|---|---|
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
colcon build --symlink-install
source install/setup.zsh
```

## Run

No single bring-up command yet — that's in progress
([`docs/ROADMAP.md`](docs/ROADMAP.md), Sprint 1). For now, three steps
launch the sim + mapping stack, then pick an operator mode:

```bash
ros2 launch stonefish_groundtruth_mapping step1_tf.launch.py          # Stonefish + TF chain
ros2 launch stonefish_groundtruth_mapping step2_pointcloud.launch.py  # + depth → point cloud
ros2 launch stonefish_groundtruth_mapping step3_octomap.launch.py     # + OctoMap

# then, in another terminal:
ros2 run launch_tools my_keyboard                    # teleop
# or
ros2 launch frontier_slam frontier_slam.launch.py    # autonomous frontier exploration
```

Override the world path with `export STONEFISH_WORLD_DIR=/path/to/sim/world`
if not running from the default checkout location.
