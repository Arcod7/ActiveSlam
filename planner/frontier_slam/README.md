# frontier_slam

**Frontier-based exploration baseline for underwater autonomous inspection.**

This package implements a classical frontier SLAM stack for the BlueROV2 running in the
[Stonefish](https://github.com/patrykcieslak/stonefish) underwater simulator under ROS 2 Jazzy.
Its primary purpose is to establish a **performance baseline** against which more sophisticated
exploration algorithms — in particular the saliency-driven revisit policy from Suresh et al.
(ICRA 2020) — will be compared.

## Goal

Autonomous 3-D inspection of underwater structures (offshore platforms, pipelines) requires
the robot to efficiently cover unknown space while maintaining an accurate map for loop
closure. Greedy frontier exploration is the canonical baseline: it is simple, well-understood,
and exposes the fundamental trade-offs that more complex methods aim to improve.

By measuring how well the frontier baseline explores the structure (coverage %, path length,
time to map closure, number of stuck events), we get a concrete reference point. Any proposed
improvement must outperform this baseline on those same metrics to be considered worthwhile.

## Sensor model

The simulation uses a Stonefish depth camera to approximate the
**WaterLinked Sonar 3D-15** (low-frequency mode):

| Parameter | Real sonar | Simulation |
|---|---|---|
| Horizontal FOV | 90° | 90° |
| Vertical FOV | 40° | ~22.5° (aspect-ratio limited) |
| Max range | 15 m | 15 m |
| Min range | 20 cm | 20 cm |
| H angular resolution | 0.85° → ~106 beams | 128 px (0.70°/px) |
| V angular resolution | 1.60° → ~25 beams | 32 px |
| Update rate | 5 Hz | 5 Hz |

Depth camera config in the `.scn` file:
```xml
<sensor name="Dcam" rate="5.0" type="depthcamera">
    <specs resolution_x="128" resolution_y="32"
           horizontal_fov="90.0"
           depth_min="0.2" depth_max="15.0"/>
    ...
</sensor>
```

## How it works

```
/projected_map (OccupancyGrid)           /sensor_msgs/image_depth
        │                                        │
        ▼                                        │
frontier_extractor ──────────────────────────── │
  - detect occupied/unknown boundaries          │
  - cluster frontiers (scipy.ndimage)           │
  - score: cluster_size / distance              │
  - GoalManager: commit-until-done,            │
    stuck detection, blacklisting              │
  - A* path planning (3-zone cost grid)        │
        │                                        │
        │ /frontier_slam/path (Path)             │
        │ /frontier_slam/goal (PointStamped)     │
        ▼                                        ▼
waypoint_controller
  - follow A* waypoints (advance at 1.5 m radius)
  - depth hold: locked setpoint from first odom
  - surge ramp-down from 1.5 m, back-surge below 0.4 m
        │
        ▼
/bluerov2/controller/thruster_setpoints_sim
```

## Package structure

```
frontier_slam/
├── frontier_extractor.py   # ROS node: map → goal + path publisher
├── waypoint_controller.py  # ROS node: path → thruster setpoints
├── frontier_detection.py   # pure: OccupancyGrid → list[Cluster]
├── goal_manager.py         # state: commitment, stuck detection, blacklist
├── path_planner.py         # pure: A* on 3-zone cost grid (CostGrid API)
├── control_utils.py        # pure: thruster mixing, yaw helpers
├── session_log.py          # shared: timestamped CSV logging
├── launch/
│   └── frontier_slam.launch.py
├── logs/                   # auto-generated per-session CSVs (gitignored)
├── Progress.md             # change log with observed impact per session
└── FutureWork.md           # candidate improvements and their rationale
```

## ROS interface

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/projected_map` | `OccupancyGrid` | 2-D OctoMap projection |
| in | `/StoneFish/Odometry` | `Odometry` | robot pose + velocity |
| in | `/sensor_msgs/image_depth` | `Image` (32FC1) | forward depth camera (sonar proxy) |
| out | `/frontier_slam/goal` | `PointStamped` | current exploration goal |
| out | `/frontier_slam/path` | `Path` | A\* waypoint sequence |
| out | `/frontier_slam/frontiers` | `MarkerArray` | RViz frontier markers |
| out | `/frontier_slam/inflated_map` | `OccupancyGrid` | 3-zone cost map (debug) |
| out | `/frontier_slam/debug_image` | `Image` | top-down composite view (debug) |
| out | `/bluerov2/controller/thruster_setpoints_sim` | `Float64MultiArray` | 6 thruster commands |

## Key parameters

| Parameter | Value | Where |
|---|---|---|
| `REPLAN_HZ` | 3 Hz | `frontier_extractor.py` |
| `UPDATE_HZ` | 0.5 Hz | `frontier_extractor.py` |
| `MIN_CLUSTER_CELLS` | 1 | `frontier_extractor.py` |
| `HARD_INFLATION_M` | 0.20 m | `path_planner.py` |
| `INFLATION_M` | 0.75 m | `path_planner.py` |
| `PLAN_INFLATION_M` | 1.50 m | `path_planner.py` |
| `STUCK_TIMEOUT` | 30 s | `frontier_extractor.py` |
| `MAX_SURGE` | 0.35 m/s | `waypoint_controller.py` |

## Build and run

```bash
distrobox enter ros2-jazzy
cd ~/delivery/slam_ws
colcon build --symlink-install
source install/setup.zsh
ros2 launch frontier_slam frontier_slam.launch.py
```

The simulation must be running first (steps 1–3 from `CLAUDE.md`).

## Session logs

Both nodes write CSV files to `logs/` at startup:

- `YYYY-MM-DD_HH-MM-SS_extractor.csv` — frontier selection at 0.5 Hz
- `YYYY-MM-DD_HH-MM-SS_controller.csv` — control state at 1 Hz

See `Progress.md` for a full history of changes and their measured impact.

## Roadmap

| Step | Status |
|---|---|
| Greedy nearest-frontier baseline | ✅ done |
| Info-gain scoring (`cluster_size / distance`) | ✅ done |
| Goal blacklisting (arrival + stuck + A\*-unreachable) | ✅ done |
| 2-D A\* path planning (3-zone cost grid) | ✅ done |
| Occupied↔unknown frontier definition | ✅ done |
| Wall-normal targeting | future work §1 |
| Realistic odometry (drift + noise) | future work §2 |
| Realistic sonar noise model | future work §3 |
| Full SLAM pipeline | future work §4 |
| 3-D frontier detection | future work §5 |
| Active SLAM: saliency + information gain | future work §6 |

See `FutureWork.md` for detailed notes on each step.

## Reference

> Suresh, S., Yogamani, S., & Ganesan, K. (2020). *Active SLAM using 3D Submap Saliency
> for Underwater Volumetric Exploration*. IEEE ICRA 2020.
