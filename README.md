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
| [`sim/stonefish_ros2`](sim/stonefish_ros2) | ROS 2 bridge to the simulator (patched fork) |
| [`slam/stonefish_groundtruth_mapping`](slam/stonefish_groundtruth_mapping) | TF chain, depth image → point cloud, OctoMap/TSDF mapping |
| [`slam/slam_backend`](slam/slam_backend) | Simulated pressure/IMU/DVL sensors + dead-reckoning fusion + GTSAM iSAM2 pose-graph SLAM |
| [`planner/frontier_slam`](planner/frontier_slam) | Frontier-based autonomous exploration (the optional mode) |
| [`eval/eval_tools`](eval/eval_tools) | ATE/RPE benchmarking against ground truth, TUM trajectory export, offline plotting |
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
colcon build --symlink-install --cmake-args -Wno-dev
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

Independent switches, each defaulting to the first value:

```bash
ros2 launch bringup demo.launch.py mode:=teleop|frontier          # operator mode
ros2 launch bringup demo.launch.py mapper:=octomap|tsdf           # map backend
ros2 launch bringup demo.launch.py slam:=none|slam                # pose source
ros2 launch bringup demo.launch.py noise_profile:=ideal|realistic|degraded  # slam:=slam only
ros2 launch bringup demo.launch.py rviz:=false                    # headless
```

Benchmarking switches (all `slam:=slam` only, all default to today's behavior):

```bash
ros2 launch bringup demo.launch.py slam:=slam loop_closure:=false             # A/B: no loop closure
ros2 launch bringup demo.launch.py slam:=slam mapper:=tsdf map_rebuild:=true  # rebuild belief TSDF after big closures
ros2 launch bringup demo.launch.py slam:=slam noise_seed:=7                   # reproducible, decorrelated noise draws
ros2 launch bringup demo.launch.py slam:=slam output_dir:=/path/to/run       # label eval output instead of a timestamp
```

`slam:=none` (default) broadcasts pose from ground truth, as before.
`slam:=slam` replaces that with a GTSAM iSAM2 pose graph correcting
simulated pressure/IMU/DVL/sonar sensor noise (see
[`slam/slam_backend`](slam/slam_backend)), and starts a benchmark node
that logs ATE/RPE against ground truth plus a map_metrics node that
scores the belief map against the ground-truth reference map (IoU/coverage
for `mapper:=octomap`, chamfer distance/coverage for `mapper:=tsdf`),
writing TUM trajectory files + CSVs to `eval/runs/<timestamp>/` — plot them
with:

```bash
ros2 run eval_tools plot_results eval/runs/<timestamp>/
```

To compare multiple configurations/seeds unattended, use the batch
orchestrator (a plain script, run directly with `python3` after sourcing
`install/setup.bash` — see [`eval/eval_tools/scripts/run_matrix.py`](eval/eval_tools/scripts/run_matrix.py)
and the example matrices in [`eval/eval_tools/config/`](eval/eval_tools/config)):

```bash
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_smoke.yaml  # ~5min pre-flight check
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_full.yaml   # 35 runs, ~5.3h
```

RViz view switches automatically with `mapper`/`slam` (`slam:=slam` wins if both
apply — see [`bringup/rviz/`](bringup/rviz)):
- default (`mapper:=octomap slam:=none`): unchanged base view.
- `mapper:=tsdf`: TSDF surface/voxels in place of the OctoMap displays (which
  would just sit empty — `octomap_server` isn't launched in this mode).
- `slam:=slam`: ground truth (green) vs SLAM (blue) vs raw dead-reckoning (red)
  paths, a live drift arrow + text HUD (error/ATE/RPE/keyframes/loop
  closures/D-optimality, from `/eval/markers`), pose-graph edges, and
  covariance ellipsoids — plus a second map built from the exact simulator
  pose (`/gt/...` topics) overlaid against the SLAM-estimate map, so you can
  see where the belief map actually diverges from reality, not just how far
  the path has drifted.

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
(`tf` → `pointcloud` → `octomap`/`tsdf`, each including the one before it) and
[`slam/slam_backend/launch/`](slam/slam_backend/launch) (`sensors_only` for
just the simulated pressure/IMU/DVL sensors, `slam` adds the pose graph on top).
