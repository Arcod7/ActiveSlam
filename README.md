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
teleoperated or exploring autonomously via frontier detection. Pose can come
from ground truth (default) or from a GTSAM pose-graph SLAM backend
correcting simulated pressure/IMU/DVL sensor noise, with a benchmarking node
logging ATE/RPE against ground truth.

| Package | Role |
|---|---|
| [`bringup`](bringup) | Unified `demo.launch.py` (mode/mapper/slam switches) and the three RViz configs it picks between |
| [`sim/world`](sim/world) | Stonefish scenario: BlueROV2 model, environment meshes, `.scn` config |
| [`sim/stonefish_ros2`](sim/stonefish_ros2) | ROS 2 bridge to the simulator (patched fork, submodule — see [`docs/stonefish_ros2_fork.md`](docs/stonefish_ros2_fork.md)) |
| [`slam/stonefish_groundtruth_mapping`](slam/stonefish_groundtruth_mapping) | TF chain, depth image → point cloud, OctoMap/TSDF mapping |
| [`slam/slam_backend`](slam/slam_backend) | Simulated pressure/IMU/DVL sensors + dead-reckoning fusion + GTSAM iSAM2 pose-graph SLAM |
| [`planner/frontier_slam`](planner/frontier_slam) | Frontier-based autonomous exploration (the optional mode) |
| [`eval/eval_tools`](eval/eval_tools) | ATE/RPE benchmarking against ground truth, TUM trajectory export, offline plotting |
| [`tools/launch_tools`](tools/launch_tools) | Keyboard teleop |

## Install

Needs ROS 2 Jazzy (Ubuntu 24.04). The robot meshes are in the repo; the scene
mesh is distributed out of band — see
[`sim/world/data/README.md`](sim/world/data/README.md).

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone git@github.com:Arcod7/ActiveSlam.git
cd ActiveSlam && ./bootstrap.sh
```

`bootstrap.sh` is idempotent and does the whole job: submodules, Python
dependencies in a uv virtualenv, rosdep, the patched Stonefish (built and
installed unless it already is), and the colcon build. Copy the out-of-band
meshes in with `--meshes-from <path-to-an-existing-checkout>/sim/world/data/obj`, and add
`--with-open3d` if you want FPFH descriptors. `./bootstrap.sh --help` lists
the rest.

Then, in every new shell:

```bash
source ~/ros_ws/.venv/bin/activate
source ~/ros_ws/install/setup.zsh
```

Details, prerequisites and what to do when a step fails:
[`docs/INSTALL.md`](docs/INSTALL.md).

## Run

```bash
python3 launcher.py
```

A terminal UI that brings the stack up, switches options while it runs without
restarting the simulator, arms the fail-closed motion gate, and drives the
vehicle from the keyboard — always live on the QWEASD cluster (AZERTY
supported), with an optional follow-the-point mode on `y`. Or launch it
directly:

```bash
ros2 launch bringup demo.launch.py                       # teleop + OctoMap, ground-truth pose
ros2 launch bringup demo.launch.py mode:=frontier        # autonomous frontier exploration
ros2 launch bringup demo.launch.py slam:=slam            # GTSAM pose graph instead of ground truth
```

`mode`, `mapper`, `slam` and the rest are independent axes. Nothing moves until
the motion gate is armed.

- Every switch, benchmarking, RViz views: [`docs/RUN.md`](docs/RUN.md)
- Worked examples, easiest first: [`docs/DEMOS.md`](docs/DEMOS.md)
- Plan and current status: [`docs/ROADMAP.md`](docs/ROADMAP.md)
- Out of scope, and why: [`docs/FUTURE_WORK.md`](docs/FUTURE_WORK.md)
