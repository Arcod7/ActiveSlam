# Demo ladder

Each step adds one difficulty over the previous. All commands run inside the
ROS2 workspace after `colcon build --symlink-install` and `source install/setup.zsh`.

Noise profiles: `ideal` (near-perfect everything), `sonar_only` (realistic sonar,
near-ideal nav sensors), `odom_pos_only` (realistic DVL/pressure position,
exact Stonefish orientation and sonar), `odom_only` (ground-truth sonar,
realistic nav sensors), `realistic` (datasheet values everywhere), `degraded`
(turbid water / magnetic interference). Add `noise_seed:=42` to any SLAM run
for reproducibility.

## 1. Base demo — teleop + OctoMap, ground-truth pose

```bash
ros2 launch bringup demo.launch.py
ros2 run launch_tools my_keyboard   # second terminal (teleop needs a real TTY)
```

## 2. Autonomous exploration — frontier + TSDF

```bash
ros2 launch bringup demo.launch.py mode:=frontier mapper:=tsdf
```

## 3. Wall-oriented path execution (simple)

```bash
ros2 launch bringup demo.launch.py mode:=frontier mapper:=tsdf motion:=walloriented \
    wall_orientation_offset_deg:=30 wall_orientation_lookahead_m:=3.0
```

This keeps the normal A* path and forward speed control. The vehicle yaw is
offset from the path bearing toward whichever side contains the nearest mapped
wall, while surge+sway keep the actual travel direction on the path. It uses
`/tsdf/surface_cloud` with `mapper:=tsdf` and
`/octomap_point_cloud_centers` with `mapper:=octomap`. If the selected map cloud
is missing or stale, it temporarily behaves like direct forward control.

With `mapper:=tsdf`, the frontier selector also rejects a 2-D frontier whose
point at the vehicle's depth is inside a confidently solid TSDF voxel. It is a
strict containment check, not a wall-clearance buffer, so valid frontiers on a
surface boundary remain selectable.

It also offsets each valid TSDF frontier goal by `tsdf_frontier_standoff_m`
(default `1.0`) along the outward horizontal TSDF surface normal, keeping the
vehicle in free space rather than targeting the surface itself. Set it to `0.0`
to retain the original surface-centred goal.

`motion:=wall_oriented` is accepted as an alias. The offset is capped below
90 degrees so the controller remains forward-oriented.

Path translation is 10% slower (the controller constants are `0.315` rather
than `0.35`) to give scan matching more overlap between keyframes. Yaw control
is unchanged. Set `wall_orientation_lookahead_m` to a positive radius (for
example `3.0`) to use the tangent where that circle first intersects the
forward A* path for wall-side and viewing heading. Translation still follows
the current path waypoint; `0.0` keeps the exact former heading behaviour.

## 3b. Wall-looking path execution (wall-normal/standoff controller)

```bash
ros2 launch bringup demo.launch.py mode:=frontier mapper:=tsdf motion:=walllooking
```

Requires `mode:=frontier mapper:=tsdf`: frontier still chooses the goal and
A* path; wall-looking is the selected path executor. It reports `BLOCKED` to
the planner if no mapped wall can make the requested route progress. Drop
`motion:=walllooking` from any later command to compare against direct motion.

`wall_standoff:=1.5` is the desired wall distance in metres. The live node
parameter is also adjustable during a run: `ros2 param set /wall_looking
standoff_m 2.0`. When a nearby goal cannot be reached along the current wall,
wall-follow performs an odometry-confirmed 180° sweep to inspect and select
another wall; set its distance gate at launch with
`wall_switch_goal_distance:=6.0`.

The travel vector and viewing heading are separate. `wall_path_influence`
blends wall-tangent travel with the planned path. For viewing,
`wall_path_look_offset_deg` turns the planner bearing toward the wall and
`wall_normal_offset_deg` turns the wall-facing normal toward the path; then
`wall_path_heading_weight` blends the two resulting headings (`0` = wall,
`1` = path). For example:

```bash
ros2 launch bringup demo.launch.py mode:=frontier mapper:=tsdf motion:=walllooking \
    wall_path_look_offset_deg:=30 wall_normal_offset_deg:=0 \
    wall_path_heading_weight:=0.35
```

## 4. SLAM, noisy sonar + ground-truth odometry, no loop closure

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=sonar_only loop_closure:=false
```

Isolates sonar noise: dead reckoning stays near-perfect, only the map degrades.

## 5. Loop closures, ground-truth sonar + odometry

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=ideal
```

Sanity check: closures fire but corrections should be near-zero.

## 6. Loop closures, noisy sonar + ground-truth odometry

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=sonar_only
```

## 6b. Loop closures, ground-truth sonar + noisy odometry

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=odom_only
```

Isolates odometry drift: the map stays sharp, dead reckoning wanders on its
own and loop closures should visibly pull the trajectory back. A/B against
`loop_closure:=false` to see the drift closures are correcting.

## 6c. Position-only drift, exact Stonefish orientation and sonar

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=odom_pos_only motion:=walloriented \
    wall_orientation_offset_deg:=60
```

Uses realistic DVL noise/bias/scale error for integrated XY and realistic
pressure noise for Z. Roll, pitch and yaw come from Stonefish without added
noise or drift; sonar is also an exact passthrough. This isolates position
estimation error from orientation and sonar error.

All non-exact profiles fuse high-rate IMU yaw increments with a lower-rate
absolute compass heading before dead reckoning. The compass limits long-term
heading drift while the IMU preserves short-term turns. In `odom_pos_only`,
both sources are exact Stonefish orientation.

## 7. Loop closures, noisy sonar + noisy odometry

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=realistic
```

The benchmark baseline (600 s, seed 42: ATE 0.54 m). A/B against
`loop_closure:=false` to quantify what closures buy.

## 8. Scripted leave-and-return — watch a loop closure fire

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=realistic scenario:=drift_return
```

## 9. Uncertainty-triggered revisit (active SLAM)

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=realistic
```

`revisit:=true` is the default for `slam:=slam mode:=frontier`. It suspends
exploration, selects a previously mapped keyframe likely to form a loop, and
uses the normal A* path planner to return to it when pose uncertainty
exceeds what the mission allows. The trigger is the ratio
`U_r = D(Σ)/D(Σ_allow)` of Suresh et al. (2020) eq. 5, so the threshold is
stated as an allowable covariance in metres and radians
(`sigma_allow_xy_m`, `sigma_allow_yaw_rad`) rather than as a bare determinant.
`D(Σ)` is scored over the drifting DoF only — x, y and heading — since depth,
pitch and roll are directly observed. It resumes only after a loop closure,
reduced uncertainty, timeout, or sterile arrival; use `revisit:=false` to
disable it. Force it early by tightening the allowance:
`ros2 param set /revisit_planner sigma_allow_xy_m 0.02`.

## 10. Belief-map rebuild after large closures

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=realistic map_rebuild:=true
```

TSDF only (prints a warning and does nothing with `mapper:=octomap`).

## 11. Stress test — degraded conditions

```bash
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier mapper:=tsdf \
    noise_profile:=degraded revisit:=true
```

## Switch reference

| Switch | Values (default first) | Needs |
|---|---|---|
| `mode` | `teleop`, `frontier` | |
| `mapper` | `octomap`, `tsdf` | |
| `motion` | `default`, `walloriented` (`wall_oriented` alias), `walllooking` (`wallfollow` alias) | `walllooking` needs `mapper:=tsdf` |
| `wall_orientation_offset_deg` | `30.0` | `motion:=walloriented`; yaw offset toward nearest mapped wall |
| `wall_orientation_lookahead_m` | `0.0` | `motion:=walloriented`; future path-heading radius |
| `tsdf_frontier_standoff_m` | `1.0` | `mapper:=tsdf`; free-space goal offset along outward surface normal |
| `wall_standoff` | `1.5` | `motion:=walllooking`; target wall distance (m) |
| `wall_switch_goal_distance` | `6.0` | `motion:=walllooking`; nearby-goal wall-switch gate (m) |
| `wall_switch_scan_angle` | `3.14159` | `motion:=walllooking`; sweep angle (rad) |
| `wall_switch_scan_yaw` | `0.08` | `motion:=walllooking`; sweep yaw command |
| `wall_path_influence` | `0.70` | `motion:=walllooking`; travel blend: 0=wall tangent, 1=path |
| `wall_path_look_offset_deg` | `30.0` | `motion:=walllooking`; turn path-derived look heading toward wall |
| `wall_normal_offset_deg` | `0.0` | `motion:=walllooking`; turn wall-derived look heading toward path |
| `wall_path_heading_weight` | `0.35` | `motion:=walllooking`; look blend: 0=wall-derived, 1=path-derived |
| `slam` | `none`, `slam` | |
| `noise_profile` | `realistic`, `ideal`, `sonar_only`, `odom_pos_only`, `odom_only`, `degraded` | `slam:=slam` |
| `loop_closure` | `true`, `false` | `slam:=slam` |
| `noise_seed` | `-1` (profile default), any int | `slam:=slam` |
| `scenario` | `none`, `drift_return` | `mode:=frontier` |
| `revisit` | `true`, `false` | `slam:=slam mode:=frontier`; uncertainty-triggered loop-closure revisit |
| `map_rebuild` | `false`, `true` | `slam:=slam mapper:=tsdf` |
| `rviz` | `true`, `false` | |
| `output_dir` | timestamped under `eval/runs/` | `slam:=slam` |

Batch evaluation across configs/seeds: `eval/eval_tools/scripts/run_matrix.py`
(see `STATE.md` "Launch usage").
