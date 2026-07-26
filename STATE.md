# ActiveSlam SLAM Backend — Current State

Mutable snapshot. Overwrite, never append. Last updated: 2026-07-25.

Change log → `Progress.md`. Detailed design + as-built deltas → `docs/SLAM_PLAN.md`.

## RViz view

`rviz/demo.rviz` is the only config; `bringup/demo.launch.py` and the launcher
both pass it in every mode. A display whose topic has no publisher in the
current mode draws nothing, so nothing has to be picked — and there is no
second copy left to drift out of sync (three hand-edited configs were merged
into this one; the `mapper`/`slam` branch in both launch paths is gone).

Both launch paths hand RViz a **scratch copy** under
`$TMPDIR/activeslam_rviz/` (`session_rviz_config` in `demo.launch.py`,
`_rviz_config` in `launcher_core.py`). RViz rewrites its entire config when a
session ends, and the installed path is a symlink into the source tree under
`--symlink-install`, so every run used to edit this tracked file: a display
ticked on in one session came back off from another, and marker namespace
lists and window geometry churned the diff. The copy absorbs that. To keep a
change made inside a session, copy its scratch file back over
`bringup/rviz/demo.rviz` — the launch prints the path.

One representation per map backend, so the two backends never draw over each
other. Enabled by default:
- `Sonar PointCloud` (`/cloud_in`) and `Octomap` (`/occupied_cells_vis_array`)
  — the default `mapper:=octomap` view: the cloud building the 3-D octomap.
  `ProjectedMapSlice` (`/projected_map`, the 2-D planning/rqt band) is present
  but off; it draws a flat plane through the scene.
- `TSDFVoxels` (`/tsdf/voxels`) for `mapper:=tsdf`. `TSDFSurface` (orange,
  `/tsdf/surface_cloud`) is present but off.
- `OcTree (free)`/`OcTree (occupied)` on `/octomap_binary` — `octomap_server`
  only, i.e. `mapper:=octomap`. `tsdf_to_octomap` publishes its octree on
  `/tsdf/octomap_binary` instead (see "TSDF → OcTree"), so these stay empty
  under `mapper:=tsdf` rather than overlaying octree voxels on `TSDFVoxels`.
  `OcTree (occupied)` is off — `Octomap` already shows the same cells.
- Ground truth vs. belief, always in a representation that does not hide the
  belief map: `Octopoints_GroundTruth` (green points,
  `/gt/octomap_point_cloud_centers`) against `Octomap`, and
  `TSDFSurface_GroundTruth` (green, `/gt/tsdf/surface_cloud`) against
  `TSDFVoxels`. The marker-based `TSDFVoxels_GroundTruth` is present but off.
  Both GT topics only publish under `slam:=slam` — see "Ground-truth
  reference map".
- `RobotState` (`/motion/robot_marker`) — the vehicle arrow plus a text label
  above it, both published by `motion_safety_gate` at its tick rate from the
  pose it already watches (ground truth under `slam:=none`, `/slam/odometry`
  under `slam:=slam`). One `_marker_state()` resolves label and colour
  together so they cannot disagree: purple `MOTION DISABLED` (gate state
  wins), cyan `REVISITING` when `revisit_planner` reports `revisiting` on
  `/frontier_slam/revisit_state`, white `INITIAL SCAN`, otherwise green with
  the current `/frontier_slam/activity` spelled out (`DRIVING TO WAYPOINT`,
  `SCANNING FOR FRONTIERS`, …). Both inputs publish at 1 Hz and are ignored
  past `marker_state_timeout_s` (3 s), so a stopped planner or executor falls
  back to green `MOTION ENABLED` instead of latching. An RViz Odometry display
  carries a fixed colour, so the old `GroundTruth` arrow could not show gate
  state and is now off; the launcher's goto-mode target sphere follows the
  same green/purple rule.
- Ground truth (green) vs SLAM (blue) vs raw dead-reckoning (red) paths,
  pose-graph edges, covariance ellipsoids, and `LiveDrift` — a GT→estimate
  line labelled `error x.xx m`, at 10 Hz off `/slam/odometry`
  (`/eval/markers_live`); a line, not an arrow, because at small drift the
  arrowhead swallowed the shaft. All from `slam:=slam`. The scalar metrics
  (`err`/`ATE`/`RPE` translation+rotation/keyframe count/loop-closure
  count/D-optimality) come from `eval_tools/benchmark.py`'s `/eval/markers`
  and render in the Eval HUD panel docked at the bottom, under a state row
  that mirrors the `RobotState` label and its colour.
- One image view, `Sonar DepthMap` (`/cloud_in/range_image`, published in
  every mode), enabled and docked in the saved window state — a second image
  display would tab into the same dock slot where only the front tab renders.

## Ground-truth reference map (`slam:=slam` only)

`gt_map.launch.py` runs a second mapping stack fed from the exact simulator
pose, in parallel with the SLAM-estimate map, so belief vs. reality can be
visually compared. Only included under `slam:=slam` — under `slam:=none`
the primary map already IS ground truth.

TF can't hold two transforms for one frame at once, so it's a fully parallel
chain, not a toggle on the existing one:
```
world_ned → bluerov2/base_link_gt → bluerov2/Dcam_gt      (always ground truth,
                                                             odom_tf_sync --target_frame)
/cloud_in_raw → cloud_relabel → /gt/cloud_in              (pre-noise sensor data,
                                                             frame_id swapped — stays
                                                             clean even when the belief
                                                             map is fed through sonar_noise)
/gt/cloud_in → octomap_server (namespace=gt) | tsdf_mapper_gt (remapped)
             → /gt/octomap_binary | /gt/tsdf/{surface_cloud,voxels}
```
Matches whichever backend `mapper:=` selected for the belief map.

## Architecture

```
/StoneFish/Odometry (GT, 100Hz)
    ├──► pressure_sim  ──► /slam/sensors/pressure_depth   (10Hz, absolute)
    ├──► imu_sim       ──► /slam/sensors/imu_orientation  (50Hz, absolute)
    └──► dvl_sim       ──► /slam/sensors/dvl_velocity     (10Hz, body-frame velocity)
                                    │
                    dead_reckoning (fuses all three; only X/Y integrates —
                                    Z/attitude are absolute, non-drifting)
                                    │
                        /slam/sensors/dead_reckoned_odom
                                    │
depth_image_proc ─► /cloud_in_raw ─► sonar_noise ─► /cloud_in (5Hz) ───┤
  (slam:=slam only: sonar_noise    (datasheet noise:                  ▼
   inserted, see below)             range/lateral/dropout/     pose_graph (GTSAM iSAM2)
                                    outlier — WaterLinked            │
                                    Sonar 3D-15, 1.2MHz mode)  ┌──────┼──────┬─────────────┐
                                                               ▼      ▼      ▼             ▼
                                                         TF broadcast /slam/pose  /slam/rebuild/*
                                                        (world_ned →  (+cov)    (map_rebuild:=true
                                                         base_link)     │        only, paced)
                                                                        ▼
                                                                  benchmark (eval_tools)
                                                           ATE / RPE / TUM export / /eval/markers
                                                                        +
                                                                  map_metrics (eval_tools)
                                                          belief vs /gt/... map: IoU/coverage/chamfer
```

Motion path (independent of the mapping/SLAM chain above):

```
waypoint_controller / wall executors ──► /motion/body_command
                                              │
                                     safety_gate (fail-closed)
                                      start_enabled=false by default;
                                      zeroes on stale/invalid/no-odom
                                              │
                                    /motion/body_command_safe ──► thruster mixer
                                                                  or ardusub_adapter
                                                                  (MAVLink MANUAL_CONTROL)
```

Nothing moves until `/motion/enable` is published — by the RViz panel
(`tools/motion_safety_rviz`), `launcher.py`'s `m` key, or
`safety_start_enabled:=true` for headless runs. `/motion/safety_status`
reports which condition is blocking.

## Map products under `mapper:=tsdf`

`octomap_server` does not run in this mode; `tsdf_mapper` owns the belief map
and exposes it three ways, all derived from the same VDB grid so they cannot
disagree:

| Topic | Type | Consumer |
|---|---|---|
| `/tsdf/surface_cloud`, `/tsdf/voxels` | cloud, MarkerArray | RViz |
| `/tsdf/occupied_voxels`, `/tsdf/free_voxels` | PointCloud2 | frontier solid rejection; `tsdf_to_octomap` |
| `/projected_map` | OccupancyGrid | 2-D frontier detection + A* (`publish_projected_map:=true`) |
| `/tsdf/octomap_binary` | octomap_msgs/Octomap | the octree interface for future 3-D frontier/A* (`tsdf_octomap:=true`, default). Deliberately not `/octomap_binary` — that is `octomap_server`'s, and RViz's OcTree displays subscribe to it |

`slam/tsdf_octomap/` (`tsdf_to_octomap`, C++) rebuilds an `octomap::OcTree`
from the occupied + free clouds once a second — from scratch each cycle, since
the TSDF itself is reset+re-integrated after a large loop closure and an
incrementally-updated octree would keep cells the TSDF has already corrected
away. Occupied and free voxels come straight across; everything else stays
unknown. Requires the pyopenvdb grid path (the surface-vertex fallback has no
free space); `tsdf_mapper` logs an error and disables `/tsdf/free_voxels` if
it is missing.

## Packages

- `slam/slam_backend/` — sensor sims (incl. `sonar_noise`), dead-reckoning fusion, pose graph, scan matcher
- `slam/tsdf_octomap/` — `tsdf_to_octomap`: TSDF grid → `octomap::OcTree` → `/tsdf/octomap_binary`
- `eval/eval_tools/` — benchmark node, map_metrics node, TUM writer, offline plotting, batch orchestrator (`scripts/run_matrix.py`)
- `slam/stonefish_groundtruth_mapping/launch/` — layered: `core` (Stonefish
  alone) / `tf_only` / `pointcloud_only` / `mapper_only`, with
  `tf`/`pointcloud`/`octomap`/`tsdf` as thin compositions of them. Launch a
  single layer to restart it without dropping the simulator.
- `external/` — pinned submodules: the patched Stonefish fork, vdbfusion and Open3D.
  Not colcon packages (`COLCON_IGNORE`); built by `./bootstrap.sh`.

## Parameters

### Noise Profiles (`slam/slam_backend/config/`)
- `noise_ideal.yaml`: near-perfect sensors (sanity checks); sonar section all-zero (exact passthrough)
- `noise_sonar_only.yaml`: sonar section from `realistic`, nav sensors from `ideal`, seed 42 —
  isolates sonar noise from dead-reckoning drift
- `noise_odom_pos_only.yaml`: realistic DVL/pressure position, exact Stonefish IMU+compass
  orientation, and exact sonar passthrough — isolates position drift from attitude and sonar error
- `noise_odom_only.yaml`: realistic DVL/pressure/IMU+compass navigation with exact sonar passthrough
- `noise_realistic.yaml`: matches Bar30 pressure + Pathfinder DVL + gyro/compass attitude; sonar section
  derived from the WaterLinked Sonar 3D-15 datasheet (`ActiveSlam-Resources/3d-sonar`)
- `noise_realistic_no_reverb.yaml`: `realistic` with `reverb_p: 0` — the only term that puts
  returns in the near field where no surface is. Removes the spray at its source, so unlike
  `near_cutoff`/`near_fade` (which gate *all* returns under a range) a real surface the vehicle
  drives up to is still mapped. Every other sonar term and every nav section is `realistic` verbatim,
  so it is also the controlled A/B for what the reverberation term alone costs
- `noise_degraded.yaml`: turbid water / magnetic interference / degraded bottom-lock; sonar section
  worse-than-datasheet (full beam-separation lateral jitter, higher dropout/outlier rates,
  stronger specular loss, larger dropout patches, uncalibrated speed of sound)

Each profile's `seed:` (42 for `ideal`/`sonar_only`/`odom_pos_only`/`odom_only`,
-1/random for `realistic`/`realistic_no_reverb`/`degraded`) is combined with a per-sensor offset
(imu +1, dvl +2, pressure +3, sonar +4, compass +5) before seeding, so co-launched sims no
longer draw identical RNG streams off one shared seed — this changed `ideal`'s exact per-sensor
draws vs. pre-Phase-16 runs (same seed, different effective value per node); nothing previously
published used `ideal`, so no quoted numbers are affected. Override with `noise_seed:=<int>`
(`-1` = use the profile's own seed).

### `pose_graph.py` key parameters (defaults)
| Parameter | Value | Purpose |
|---|---|---|
| `keyframe_dist_m` | 1.0 | Min travel to trigger a new keyframe |
| `keyframe_angle_rad` | 0.3 | Min rotation (~17°) to trigger a new keyframe |
| `keyframe_max_per_cell` | 3 | Max keyframes sharing one position+heading cell; 0 disables |
| `keyframe_cell_radius_m` / `keyframe_cell_angle_rad` | 0.5 / 0.5 | Extent of that cell |
| `loop_closure_enabled` | true | Master on/off switch (A/B benchmarking) |
| `loop_closure_radius_m` | 5.0 | Proximity search radius for loop closure |
| `loop_closure_min_gap` | 10 | Min keyframe-index gap for a valid closure |
| `loop_closure_max_candidates` | 4 | Max registrations attempted per keyframe |
| `loop_closure_cluster_radius_m` | 1.0 | Candidates closer than this collapse to one representative |
| `loop_closure_retry_move_m` | 0.5 | Endpoint motion required before a failed pair is retried |
| `loop_closure_dopt_floor` | 0.0 | Skip detection below this D-optimality; 0 disables (see below) |
| `redetect_max_keyframes` | 5 | Most-displaced keyframes re-scanned after a closure |
| `redetect_min_interval_s` | 5.0 | Throttle between re-detection sweeps |
| `min_inlier_ratio` | 0.3 | Registration gate: fraction of source points matched |
| `min_inlier_count` | 50 | Registration gate: absolute inlier floor |
| `max_error_per_inlier` | 0.05 | Registration gate: GICP error normalized per inlier |
| `odom_sigma_rot` / `odom_sigma_trans` | 0.02 / 0.02 | Dead-reckoning BetweenFactor noise (tight, non-robust: the DVL is accurate and must not be down-weighted by a bad closure) |
| `scan_sigma_rot` / `scan_sigma_trans` | 0.08 / 0.12 | Scan-match + loop-closure BetweenFactor noise (looser, Huber-robust: sparse-sonar registration is decimeter-level) |
| `scan_voxel_size` | 0.1 | small_gicp downsampling resolution |
| `isam_relinearize_threshold` / `isam_relinearize_skip` | 0.01 / 1 | iSAM2 tuning; same values as when they were hardcoded |
| `diagnostics_stride` | 10 | Keyframes between whole-graph chi-square evaluations |
| `viz_period_s` | 1.0 | Path/marker publish period (was per keyframe) |
| `path_dr_min_move_m` | 0.05 | Travel between dead-reckoned path samples |
| `map_rebuild_enabled` | false | Rebuild the belief TSDF map after a big loop closure (TSDF only) |
| `rebuild_min_move_m` / `rebuild_min_move_rad` | 0.3 / 0.15 | Min keyframe shift to trigger a rebuild |
| `rebuild_min_interval_s` | 30.0 | Throttle between rebuild triggers |

**Station-keeping cost.** Hovering (or sweeping in place) used to make closure
candidacy degenerate to all-pairs: every co-located keyframe is inside
`loop_closure_radius_m` of every other, so registrations per keyframe grew with
the graph and the total cost was quadratic in hover length. Measured over a
synthetic hover, one keyframe at the end of the hover cost 16 / 36 / 76
registrations at 20 / 40 / 80 keyframes; with the candidate cap it is a flat 2.
Totals over the same hovers: 139 / 669 / 2929 registrations and 40 / 284 /
1044 ms, against 34 / 74 / 154 and 18 / 51 / 163 ms. `/slam/timing/keyframe_ms`,
`/slam/timing/isam_ms` and `/slam/timing/icp_calls` publish this per keyframe.

`loop_closure_dopt_floor` is the information-gain gate and defaults **off**. The
first closure on returning to a known place absorbs the accumulated drift; later
closures to the same place reuse correlated evidence, so they cost registrations
and bias the marginal covariance downward without adding information. It is off
by default because it changes SLAM behaviour and needs its own ATE A/B rather
than riding in on a performance change.

### `revisit_planner.py` key parameters (defaults; `*` = live-read via `get_parameter`
every tick, so `ros2 param set` takes effect without a restart)
| Parameter | Value | Purpose |
|---|---|---|
| `sigma_allow_xy_m` * | 0.045 | Largest horizontal position sigma the mission tolerates |
| `sigma_allow_yaw_rad` * | 0.045 | Largest heading sigma the mission tolerates |
| `ratio_trigger` * | 1.0 | `U_r = D(Σ)/D(Σ_allow)` above this → suspend + revisit (Suresh et al. 2020 eq. 5) |
| `ratio_resume` * | 0.5 | `U_r` below this while revisiting → cooldown |
| `min_keyframes` | 15 | Minimum keyframe count before a revisit can trigger |
| `min_index_gap` | 10 | Candidate keyframes must be at least this many indices old |
| `candidate_radius_m` | 5.0 | Neighbourhood radius used to score candidate density |
| `min_target_dist_m` | 3.0 | Candidates closer than this to the robot are excluded |
| `w_density` / `w_travel` | 1.0 / 0.2 | Target score = density − w_travel·dist |
| `revisit_timeout_s` * | 120 | Give up and cooldown if a revisit hasn't resolved by then |
| `arrival_radius_m` | 2.5 | "Arrived at target" threshold |
| `arrival_dwell_s` | 30 | Time spent at an arrived target with no closure before cooldown |
| `cooldown_s` | 60 | COOLDOWN → EXPLORING delay |

### Launch usage

`python3 launcher.py` (repo root) is the interactive way in: it runs the stack
as independently restartable groups, so changing the mapper, mode or pose
source only bounces the layers that depend on it and leaves Stonefish up. It
also arms the fail-closed motion gate (`m`) — nothing moves until it is armed,
and with `rviz:=false` there is no other way to do that — resets a run (`r`),
and has always-live keyboard driving built in: the QWEASD cluster (AZERTY
option too) drives with no mode key to press, and what it moves follows the
*Mode* option — `teleop` the vehicle, `goto` a target point the planner swims
to, `frontier` autonomous exploration until a drive key takes over (`Esc`
resumes it). While the gate is disarmed the footer says so instead of listing
the drive keys. `demo.launch.py` below is unchanged and drives the same launch
files (it has no `goto`: that mode is the launcher publishing
`/frontier_slam/goal` itself).

```bash
ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic mode:=frontier
ros2 launch bringup demo.launch.py                                       # unchanged default (slam:=none, ground truth)
ros2 launch slam_backend sensors_only.launch.py noise_profile:=degraded  # sensor layer only

# Benchmarking switches (all slam:=slam only, all default to current behavior):
ros2 launch bringup demo.launch.py slam:=slam loop_closure:=false                    # A/B: no loop closure
ros2 launch bringup demo.launch.py slam:=slam mapper:=tsdf map_rebuild:=true         # rebuild belief TSDF after big closures
ros2 launch bringup demo.launch.py slam:=slam noise_seed:=7                          # repeatable, decorrelated sensor-noise draws
ros2 launch bringup demo.launch.py slam:=slam output_dir:=/path/to/run              # label eval output instead of a timestamp
ros2 launch bringup demo.launch.py slam:=slam mode:=frontier revisit:=true          # Week 3: uncertainty-triggered revisit
ros2 launch bringup demo.launch.py mode:=frontier scan_style:=spin                  # pre-2026-07-21 full-revolution scan (default is sweep)
ros2 launch bringup demo.launch.py mode:=frontier scan_sweep_deg:=120.0             # narrower cable-safe sweep
ros2 launch bringup demo.launch.py mode:=frontier rviz:=false safety_start_enabled:=true  # headless: arm the motion gate at startup
ros2 launch bringup demo.launch.py mapper:=tsdf carve_no_return:=true               # measured negative, see below — stays off

# Batch evaluation (plain script, not a console_script -- needs
# `source install/setup.bash` first so eval_tools.plot_results is importable):
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_smoke.yaml  # ~5min pre-flight check
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_full.yaml   # 35 runs, ~5.3h
python3 eval/eval_tools/scripts/run_matrix.py --aggregate-only eval/runs/<batch_dir>    # re-aggregate only
```

`carve_no_return` frees the voxels along no-return sonar rays. It is **off and
should stay off**: vdbfusion has no carve-only ray API, so each synthesized
pseudo-point also writes a surface at `carve_range_m`, and on a moving vehicle
that artefact lands inside the volume mapped from earlier poses. Measured on
one seed (Progress.md Phase 29): coverage 0.952 → 0.446, chamfer 0.349 → 8.33 m.

## Verification status

- ✅ Sensor fusion chain (pressure+IMU+DVL→dead_reckoning): profile-dependent drift confirmed via standalone rclpy harness
- ✅ `ScanMatcher` gating: correctly rejects a synthetic zero-overlap match despite `converged=True`
- ✅ Pose graph + loop closure logic: synthetic square-loop test (wiring/logic sanity check only, not representative of real numbers — see caveat below)
- ⚠️ **Historical real benchmark** (`demo.launch.py slam:=slam mode:=frontier
  noise_profile:=realistic noise_seed:=42`, real Stonefish sim, 600s / 94 metrics
  rows / 35 loop closures, post re-detect fix and incl. the Phase 15 sonar noise
  model): final cumulative ATE **0.5356 m**, final instantaneous error 0.7908 m,
  mean RPE (translation) 0.1137 m, map coverage 0.9651, occupied-cell IoU 0.5194.
  This predates Phase 36 and sampled only irregularly-spaced keyframe poses, so
  its ATE/RPE are not directly comparable with current continuous-odometry metrics.
  `noise_seed:=42` repeats the sensor RNG draws, but does not make asynchronous
  Stonefish/ROS/planner execution deterministic. The earlier 0.58 m / 157 s figure is
  superseded — it predated both the re-detect fix and the sonar noise model, so
  the two numbers happen to land close but aren't measuring the same system.
  Loop closure remains opportunistic-only — the robot never revisits anything
  on its own; closing this gap is the Week 3 active-SLAM contribution (see
  "Not yet implemented" below).
- ✅ **Evaluation audit and repair (Phase 36)**: ATE now samples corrected
  `/slam/odometry` continuously instead of weighting sparse keyframes; RPE uses a fixed
  temporal delta. Two identical pre-fix 120 s runs on commit `4011d3b`/seed 1 measured
  ATE 1.255 vs 0.709 m and 45 vs 25 loop closures, demonstrating that a noise seed is
  not an end-to-end determinism guarantee. Matrix validity now unwraps yaw, detects
  pre-shutdown child deaths/non-zero launch exits, ignores orphan nodes in other ROS
  domains, accepts `near_cutoff`, and labels its status as structural validity only.
- ✅ **TSDF cloud/TF arrival-order loss fixed (Phases 37–38)**: a read-only 65 s timing probe
  observed 302 `/cloud_in` messages. TF was available immediately for 151; of the
  remaining 151, 150 became transformable within 0.5 s and only one expired. The
  mapper now uses ROS Jazzy's maintained `Buffer.wait_for_transform_async()` and a
  10-cloud/0.5 s bounded FIFO, preserving exact timestamps and capture order without
  falling back to latest TF. In the matched 70 s post-fix run, the belief mapper
  deferred 141/290 clouds, recovered 140, expired one, and overflowed none by 60 s.
  A 180 s stress run stayed at zero overflow/future failures; a deterministic rebuild
  reset and replayed 258 retained scans in 1.32 s, then ran another 30 s cleanly.
- ⚠️ **Fixed-duration shutdown can interrupt a late rebuild**: the Phase 38 stress
  run naturally began replaying 547 scans only 1.7 s before its 180 s deadline and
  was stopped by the planned SIGINT. Its structural status was still `ok`. For map
  quality comparisons, inspect rebuild-start/completion counts and rerun or extend
  any cell that ends mid-replay; choosing automatic overtime versus invalidation is
  an evaluation-policy decision, not a mapper correctness fix.
- ⚠️ **A rebuild-correlated position jump was traced upstream to loop closure
  (Phase 39)**: in a 240 s wall-oriented/frontier/realistic-noise run, absolute
  error jumped 0.347 -> 0.862 m 36 ms after node 84's closure update and 139 ms
  before TSDF rebuild began. The graph moved 40/85 keyframes by up to 0.99 m;
  TSDF subsequently replayed all 606 scans in 2.04 s. Ten-second pre/post mean
  errors were 0.323/0.795 m, and the error remained high after replay. The mapper
  cannot change SLAM pose, so rebuild is a downstream indicator of the large graph
  correction, not its cause. Current closure gating checks local ICP convergence,
  overlap, and residual but not ICP-vs-prediction innovation or mutual consistency
  across several accepted closures. Do not add an arbitrary correction cutoff from
  this one event: it could reject legitimate drift correction. Add per-candidate
  diagnostics and validate a consistency gate across labelled closure events first.
- ✅ **Split odometry/scan noise model (Phase 40)** — the fix for the above: the
  Phase 39 jump was one symptom of a single over-confident noise model (σ=0.05 m)
  shared by DVL dead-reckoning and sonar scan-matching. The DVL is cm-accurate
  per keyframe while sonar GICP is decimeter-level, so the shared σ let scan
  constraints override good odometry — dragging the estimate via sequential
  factors (2.6× worse than raw odometry with loop closure off) and warping it via
  loop closures (peak error to 1.3 m). Split into a tight, non-robust
  `odom_sigma` (0.02) and a looser, Huber-robust `scan_sigma` (0.08/0.12).
  Validated on a wall-oriented / realistic-noise, 4-arm × 3-seed, 240 s batch:
  mean final error fell 0.454→0.150 m (loop-closure off), 0.677→0.104 m
  (clean-sonar), 0.265→0.137 m (near_cutoff), unchanged where already healthy
  (baseline 0.176→0.180 m); worst per-run peak error 1.30→0.30 m and the largest
  single closure-induced step 1.23→0.11 m, with every run now degrading gradually
  rather than jumping. Clean-sonar SLAM now beats raw dead-reckoning on all three
  seeds despite firing 111–456 closures. Caveat: n=3 per arm and asynchronous
  execution dominates run-to-run variance, so the arm means are directional, not
  precise; the tail elimination is the robust result.
- ✅ Regression check: `slam:=none` (default) unchanged, no SLAM nodes started, `odom_tf_sync` still the TF source
- ✅ Ground-truth reference map (`gt_map.launch.py`): live-verified both `mapper:=octomap`
  and `mapper:=tsdf`. Not yet visually confirmed in RViz (headless verification only, no GUI here).
- ✅ Pose-graph re-detect-after-move loop closures (was dead code, fixed in the post-Phase-14
  audit): live-verified firing repeatedly (`Re-detecting loop closures for moved keyframes [...]`).
- ✅ Sensor-sim backward-timestamp guards (`imu_sim`/`pressure_sim`/`dvl_sim`, post-Phase-14
  audit): synthetic + live verification, no more `ValueError: scale < 0` crashes.
- ✅ **Sonar noise model** (Phase 15): synthetic checks of range-noise std vs. range, dropout
  fraction vs. range, exact quantization lattice, NaN/no-return pixels untouched, and
  bytes-identical `ideal`-profile passthrough. Live: `/cloud_in_raw`/`/cloud_in`/`/gt/cloud_in`
  all ~5 Hz with correct frame IDs; `slam:=none` unaffected.
- ✅ **Geometry-aware sonar noise** (Phase 30): grazing-incidence dropout (0.024 head-on →
  0.151 at 75° on `realistic`), spatially and temporally correlated speckle/dropout fields
  (ping-to-ping +0.426, spatial lag-1 +0.559, was ~0 iid), and a per-run speed-of-sound
  range scale. Variance-preserving, so the `realistic` error *magnitude* is unchanged —
  only its structure. **Not fitted to hardware**: no real Sonar 3D-15 range images exist
  yet, so these are geometrically-motivated stress-test values, not a calibrated model.
  Note when reading raw range statistics that the multipath outlier term (~0.077 m std)
  swamps the speckle (~0.008 m) and must be rejected before the correlation terms are visible.
- ✅ **Imaging-sonar geometry** (Phase 31): an organized-image stage ahead of the per-point
  noise adds strongest-return ranging (beam-window arg-max: thin targets fade, edges bleed),
  projection-aware lateral jitter (per-pixel pinhole beam width, not a uniform constant),
  volume reverberation (correlated near-field backscatter, elevated on weak returns), and
  geometric multipath (screen-space second bounce: phantoms in concave corners, none on flat
  walls). Live-verified: `ideal` still byte-exact passthrough; `realistic` fades a floated
  blob, fires multipath on ~0.4% of the image at the concave seam only, and injects ~250
  reverb returns/ping. **The projection-aware jitter changes lateral error magnitude across
  the image and breaks strict comparability with pre-Phase-31 benchmark runs** (centre kept
  near the old value, so it is a redistribution). Multipath models in-frustum bounces only.
  Still not fitted to hardware.
- ✅ **Near-field gate** (Phase 32): `near_cutoff:=<m>` launch arg (over any profile) drops noised
  returns nearer than the threshold — clears the reverberation spray around the vehicle. Off by
  default; live-verified 429 → 0 near-field returns/ping at `near_cutoff:=1.6`, far geometry
  untouched. Hides reverb rather than retuning it; lowering `reverb_p` is the alternative.
  Exposed in the launcher TUI as **"Noise Attenuation"** under the SLAM section (Phase 33).
- ✅ **Near-field fade** (Phase 55): `near_fade:=<p> near_fade_range:=<m>` — the soft form of the
  gate. Drop probability `near_fade_p * (1 - r/range)^exp`, drawn per beam rather than from the
  correlated field, so a close surface thins instead of disappearing and refills over pings. Off by
  default. Measured on `realistic` against a 6 m wall: spray 245 → 53 returns/ping (vs 0 for the
  cut), while a 0.5 m wall keeps 43% of its points where the cut leaves 0.1%; exact no-op beyond
  its range. Third **Noise Attenuation** value `fade_close`; `near_cutoff_m` is now
  **"Near-field distance (m)"** for both modes, plus advanced **"Fade strength"** (`near_fade_p`,
  default 0.8). Not yet exercised in a full sim run.
- ✅ **Range-image view** (Phase 34): `range_image` node renders the noised cloud back to a 2D
  depth-camera-style Image. `/cloud_in/range_image` (what SLAM gets) and `/cloud_in_raw/range_image`
  (clean) publish from `pointcloud_only.launch.py`; RViz layouts show them as "SLAM input (noised
  range)" and "Clean cloud (range)". Toggling Noise Attenuation changes the SLAM-input image live.
  **(Phase 43)** All three RViz configs previously had two or three of these Image displays
  enabled at once, tabbed together in the same dock slot — only the front tab renders, so which
  view actually appeared on launch depended on the saved (opaque, hand-uneditable) `QMainWindow
  State` blob, not on anything the launch args controlled. **(Phase 44)** The single `demo.rviz`
  enables exactly one, `SLAM input (noised range)`; `DepthCamera` and `Clean cloud (range)` are
  present but off. ⚠️ On this machine RViz ignores that at startup — every `Image` display comes
  up disabled with no dock, because RViz ties an `Image` display's enabled state to its dock
  widget's visibility and the dock is created while the main window is still hidden. Verified
  against unmodified configs from before Phase 44, and with the `QMainWindow State` blob removed
  entirely, so it is neither the merge nor the saved layout. Enable the image view with one click
  in the Displays tree after launch.
- ✅ **Eval HUD panel** (Phase 43, `tools/eval_hud_rviz`): RViz panel plugin (mirrors
  `motion_safety_rviz`'s structure) subscribing to `/eval/markers` and showing the `eval_hud`
  namespace's text (err/ATE/RPE/KF/LC/D-opt) in a panel docked at the bottom of the window — a
  fixed on-screen readout instead of a marker that floats above the robot in world space and
  moves with the camera. Outside `slam:=slam` it shows a placeholder line, since nothing
  publishes `/eval/markers` there. **(Phase 44)** Phase 43 removed the `Time` panel and added the
  panel to the `Panels:` list, but left the `QMainWindow State` blob still naming `Time` in the
  bottom dock slot and never naming `Eval HUD` — Qt turns the unmatched name into a placeholder
  and leaves the real panel wherever `addPane` put it, which is why the metrics never appeared.
  The blob now carries `Eval HUD` in that slot (the name is a length-prefixed UTF-16BE string
  inside the hex blob, so it is patchable without re-saving from the GUI). Confirmed live: the
  panel docks along the bottom, full width, and shows its placeholder outside `slam:=slam`.
  The panel now carries two rows: the vehicle state on top — text and colour taken straight off
  the `motion_state_text` marker on `/motion/robot_marker`, so it cannot drift from the arrow,
  and it works in every mode, not just `slam:=slam` — over the metrics row.
- ✅ **Benchmarking switches** (Phase 16): `loop_closure:=false` keeps
  `/slam/loop_closure_count` at 0 (default still closes loops); `noise_seed:=7` reaches all
  four sensor nodes; a forced-threshold live run fired 4 map-rebuild cycles cleanly
  (49/49, 54/54 scans re-integrated), ground-truth instance unaffected. **Map rebuild is
  TSDF-only** — `mapper:=octomap` + `map_rebuild:=true` prints a warning and does nothing,
  since `octomap_server` has no clean way to replay a corrected sensor origin for its
  free-space raycasting.
- ✅ **Map-quality metrics** (Phase 17): synthetic grid-alignment/IoU/coverage and
  chamfer/coverage checks; live 60s runs on both backends show sane, time-varying values
  (octomap: coverage ~0.98, IoU ~0.5; tsdf: coverage 0.89→0.82, chamfer 0.3m→0.72m).
- ✅ **Batch orchestrator** (Phase 18): `--dry-run` on the 35-run full matrix preset resolves
  correctly and rejects invalid configs (`motion:=wallfollow` without `mapper:=tsdf`); a real
  2-run/120s mini-batch produced complete per-run + batch-level artifacts including a clean
  back-to-back Stonefish restart.
- ✅ The 35-run full matrix (`matrix_full.yaml`, 7 configs × 5 seeds) has been run;
  results directly motivated one follow-on fix: loop closure barely moved ATE at
  this course/duration (the revisit planner addresses this). The matrix's
  `tsdf`/`tsdf_rebuild` rows were configured as `mode:frontier mapper:tsdf`,
  which had no map source for frontier detection (`frontier_extractor`
  subscribes to `/projected_map`, published only by `octomap_server`, which
  wasn't included under `mapper:=tsdf`) — the robot spun in place for the
  full run instead of exploring. The map-quality-degradation finding this
  produced is an artifact of that bug, not a rebuild-fidelity result, and is
  invalidated (Phase 21). Fix landed: `tsdf_mapper` derives `/projected_map`
  from its own grid (a Z band around the cruise depth,
  `publish_projected_map:=true`), so the planning map and the belief map are
  the same map. This superseded Phase 21's dual-map stopgap, which ran a
  second `octomap_server` purely as a planning-map source — no
  `octomap_server` runs under `mapper:=tsdf` any more.
  The `tsdf`/`tsdf_rebuild` rows still need re-specifying and
  re-running on a moving robot before any rebuild-fidelity claim can be made.
  A 10+ minute single session is also now done (the 600 s baseline above).
  Still open: `evo_ape`/`evo_rpe` cross-check against the written TUM files.
- ✅ **Teardown hardening**: every node now tolerates a second SIGINT during
  shutdown without printing a traceback (previously a stray `KeyboardInterrupt`
  inside `destroy_node()`'s `finally` block would escape uncaught), and
  `run_matrix.py`'s validity check classifies a traceback ending in
  `KeyboardInterrupt` as benign rather than `crashed_soft`. This means the prior
  full-matrix batch's `crashed_soft` statuses were an artifact of teardown noise,
  not real failures — the fresh 600 s baseline confirms zero harmful tracebacks
  under the same shutdown sequence. `run_matrix` statuses are meaningful as
  structural-validity labels only; they do not impose an accuracy threshold.
  Old manifests are left as historical record, not retroactively fixed.
- ⚠️ The synthetic square-loop test's "70% ATE reduction" (Progress.md Phase 6) is superseded
  by the real benchmark above — its dense, easily-overlapping synthetic point clouds make loop
  closure fire far more readily than real depth-camera data does. Both it and the old
  0.58 m keyframe-sampled figure are historical; rerun with Phase 36's continuous metrics.
- ✅ **Uncertainty-triggered revisit planner** (Phase 20, `revisit:=true`): forced-trigger
  live test (`ros2 param set /revisit_planner dopt_trigger 0.002`) went through the full
  cycle — suspend, drive to target, loop closure fires, dopt drops, resume. Natural-trigger
  A/B batch (`matrix_revisit.yaml`, 480 s × seeds 101/102) fired the mechanism naturally
  (2 revisits/run) but is a **mechanism demo, not an ATE-improvement claim** (n=2, mixed
  result — see Progress.md Phase 20 for the per-seed table). v1 scope cuts vs. `docs/ROADMAP.md`
  Week 3 (FPFH saliency, per-candidate covariance propagation) are future work.
- ✅ **Loop-closure edge dedup** (Phase 20, `pose_graph.py`): `/slam/loop_closure_count` was
  double-counting — a pair could be recorded once at initial detection (under the newer
  keyframe) and again under the older keyframe if it was later redetected after moving,
  inserting a genuine duplicate `BetweenFactorPose3` into iSAM2. Fixed with a run-lifetime
  closed-pairs set; live-verified 7 closure log lines = final lc_count 7 exactly on a 140 s
  run, including a 21-keyframe redetect cascade that correctly found no new pairs. lc_count
  figures from before this fix (the Phase 19 baseline's 35; the Phase 20 batch's 200–452) are
  inflated by an unknown amount and aren't directly comparable to future runs.

## Not yet implemented (Week 3 prep only — see docs/SLAM_PLAN.md Part 11)

- Mirror `NonlinearFactorGraph`/`Values` for virtual-factor uncertainty propagation along candidate revisit paths — not built; `revisit_planner.py` (Phase 20) reacts to the live `/slam/dopt` value, it does not project uncertainty forward per candidate
- Submap saliency (FPFH descriptors) — keyframe clouds are stored in body frame, ready to feed a descriptor pipeline, none built; `revisit_planner.py`'s v1 target scoring uses plain keyframe density instead
- Map rebuild after a large loop closure — **implemented for TSDF** (Phase 16, parameterized
  `map_rebuild:=true`); still not supported for OctoMap (see Verification status above)
