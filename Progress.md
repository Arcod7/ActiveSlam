# ActiveSlam SLAM Backend — Change Log

Each entry records a change, its objective, and the **observed impact** once tested.
Entries are ordered chronologically. Mark impact as ✅ positive, ⚠️ mixed/partial,
❌ negative (reverted), or 🔲 not yet tested.

Current parameters → `STATE.md`

---

## Phase 1 — Package scaffolding and dependencies

**Date**: 2026-07-03  
**Files**: `requirements.txt`, `slam_backend/`, `eval_tools/`

**Objective**: Set up the initial ROS 2 Python packages and update `requirements.txt` with `gtsam` and `small-gicp`.

**Observed impact**: ✅ Python packages installed and `colcon build` succeeded.

---

## Phase 2 — Noise Profiles

**Date**: 2026-07-03  
**Files**: `slam_backend/sensor_models/noise_profiles.py`, `slam_backend/config/noise_*.yaml`

**Objective**: Implemented the `NoiseProfile` dataclasses and the three YAML configurations (ideal, realistic, degraded).

**Observed impact**: ✅ Not yet fully tested with sensor models. Loader function tested via Python script successfully.

---

## Phase 3 & 4 — Sensor Simulators & sensors_only.launch.py

**Date**: 2026-07-03  
**Files**: `pressure_sim.py`, `imu_sim.py`, `dvl_sim.py`, `sensors_only.launch.py`

**Objective**: Implemented the standalone ROS 2 sensor simulator nodes. They subscribe to `/StoneFish/Odometry` and publish to `/slam/sensors/pressure_depth`, `/slam/sensors/imu_orientation`, and `/slam/sensors/dvl_odom` with their respective noise profiles applied. Built `sensors_only.launch.py` to launch all three with a configurable `noise_profile`.

**Observed impact**: 🔲 Compiled. Waiting for execution test.

---

## Phase 5 — Fix DVL heading-noise bug; scan matcher; dead-reckoning fusion node

**Date**: 2026-07-04
**Files**: `dvl_sim.py` (rewritten), `dead_reckoning.py` (new), `geometry_utils.py` (new), `scan_matcher.py` (new), `sensors_only.launch.py`, `setup.py`

**Objective**: Review found `dvl_sim.py` (Phase 3) integrated position using the
*exact ground-truth* rotation increment for its dead-reckoned pose, adding
noise only to translation. That leaves the orientation channel of the
between-factor odometry numerically perfect regardless of the selected noise
profile — the noise-profile comparison (ideal/realistic/degraded) would never
show the heading-driven drift that dominates real underwater dead reckoning.

Fixed by splitting responsibilities per explicit instruction ("a dead
reckoning that takes IMU, pressure and DVL all together... visual information
corrects the drift"): `dvl_sim.py` now publishes noisy body-frame **velocity**
only (`/slam/sensors/dvl_velocity`, `TwistStamped`) — matching what real DVL
hardware reports. A new `dead_reckoning.py` node fuses pressure (absolute
depth) + IMU (absolute attitude) + DVL (integrated velocity, the only relative/
drifting quantity) into one navigation estimate on
`/slam/sensors/dead_reckoned_odom`. Matrix/message conversion helpers
extracted to `geometry_utils.py` (previously duplicated per-sensor-node).
Added `scan_matcher.py`: a `small_gicp` GICP wrapper with gating on inlier
count/ratio *and* per-inlier error — `small_gicp.RegistrationResult` has no
`fitness` field, and a zero-overlap match reports `converged=True, error=0.0,
num_inliers=0`, which a bare error threshold would wrongly accept.

**Observed impact**: ✅ Verified with a standalone rclpy harness (synthetic
90°-turn trajectory): fused dead-reckoning position error 0.09–0.20 m over a
~10 m / 20 s run across all three noise profiles, confirmed profile-dependent
(vs. near-zero, profile-independent error before the fix). `ScanMatcher`
verified against a synthetic degenerate (non-overlapping) cloud pair: correctly
rejected via `is_acceptable()` despite `converged=True`.

---

## Phase 6 — GTSAM iSAM2 pose-graph SLAM node

**Date**: 2026-07-04
**Files**: `pose_graph.py` (new)

**Objective**: Core SLAM backend. Subscribes to `/slam/sensors/dead_reckoned_odom`
and `/cloud_in`; builds a single-level GTSAM iSAM2 factor graph following
roller's pattern (`graph.cpp`): one Huber-robust ICP noise model for all
Between factors (dead-reckoning odometry, sequential scan match, loop
closure), a per-node attitude+depth PriorFactor read directly off the fused
odometry (x/y unconstrained), geometric-proximity loop closure with
re-detection after graph updates move earlier keyframes. Publishes corrected
TF/odometry/path, typed graph-edge markers (odometry/loop-closure/rejected),
covariance ellipsoids, and a D-optimality scalar (`/slam/dopt`) — the last two
specifically to give the Week-3 uncertainty-driven revisit trigger a live
signal and a logged history before that policy exists.

Caught and fixed during implementation, before commit: (1) `ISAM2Params.relinearizeSkip`
is a plain property in gtsam 4.2.1, not a `setRelinearizeSkip()` method: the
threshold setter and the skip setter are inconsistent in this wrapper version.
(2) The loop-closure candidate search originally sliced
`self._keyframes[:current_idx - min_gap]`; reused for re-detection on an early
historical index this goes negative, and Python silently reinterprets a
negative slice bound as "count from the end" rather than clamping — replaced
with an explicit `abs(current_idx - kf.index) >= min_gap` filter, correct for
both the forward (newest-node) and re-detection call paths. (3) GTSAM's
marginal covariance is `[rot|trans]`-ordered; ROS's `PoseWithCovariance` is
`[trans|rot]`-ordered — publishing one as the other silently swaps position
and orientation uncertainty in RViz; `_gtsam_cov_to_ros()` permutes the blocks.

**Observed impact**: ✅ Verified with a synthetic square-loop trajectory
(deliberately drifted dead reckoning + a static point-cloud environment
sampled at the true pose): 25 keyframes, loop closures fired correctly on
revisiting the start, final position error dropped from 1.02 m (raw dead
reckoning) to 0.31 m (SLAM-corrected) — a 70% reduction, matching the
ROADMAP Week 2 done-criterion ("turning loop closure on measurably reduces
ATE"). Also verified live against the real Stonefish sim
(`demo.launch.py slam:=slam mode:=frontier`): loop closures fired correctly
on real depth-camera point clouds, frontier exploration drove successfully
from SLAM-corrected odometry, no exceptions over a 45 s run.

---

## Phase 7 — SLAM backend launch file

**Date**: 2026-07-04
**Files**: `slam.launch.py` (new)

**Objective**: Bundles the four sensor-layer nodes (via `sensors_only.launch.py`)
plus `pose_graph.py` behind a single `noise_profile` argument.

**Observed impact**: ✅ Verified via `demo.launch.py slam:=slam` (see Phase 11).

---

## Phase 8–10 — ATE/RPE benchmarking (eval_tools)

**Date**: 2026-07-04
**Files**: `tum_writer.py`, `benchmark.py`, `plot_results.py`, `eval.launch.py` (all new)

**Objective**: Real-time trajectory evaluation. `benchmark.py` pairs
`/StoneFish/Odometry` (ground truth), `/slam/pose`, and
`/slam/sensors/dead_reckoned_odom` by nearest timestamp, publishes running
absolute-position-error/ATE/RPE-translation/RPE-rotation/D-optimality scalars,
and writes TUM trajectory files (`gt_traj.tum`, `slam_traj.tum`,
`odom_traj.tum`) directly consumable by `evo_ape`/`evo_rpe`, plus a
`metrics.csv`. `plot_results.py` renders a 6-panel offline summary (XY
trajectory, absolute error, ATE, RPE translation, RPE rotation, D-optimality)
from a run directory. Runs are written to `eval/runs/<timestamp>/`
(gitignored — ephemeral per-session data, not tracked).

**Observed impact**: ✅ Verified live during Phase 11's demo run — TUM files
and `metrics.csv` were written correctly to a timestamped run directory.

---

## Phase 11 — Wire SLAM backend into the unified demo launch

**Date**: 2026-07-04
**Files**: `bringup/launch/demo.launch.py`, `slam/stonefish_groundtruth_mapping/launch/tf.launch.py`

**Objective**: Added `slam` (`none`/`slam`) and `noise_profile` arguments to
`demo.launch.py`. `tf.launch.py` gained a `use_gt_tf` argument (default
`true`) gating `odom_tf_sync`; `demo.launch.py` sets it to `false` before
including the mapper stack when `slam:=slam`, so `pose_graph.py` becomes the
sole broadcaster of `world_ned -> bluerov2/base_link` — without needing to
thread the argument through the three intermediate launch files
(`pointcloud.launch.py` -> `octomap.launch.py`/`tsdf.launch.py`) in between,
since `DeclareLaunchArgument` only applies its default when the configuration
isn't already set in the shared launch context. When `mode:=frontier`, the
frontier planner's existing `odom_topic` argument (already supported —
previously used for the disconnected `/StoneFish/Odometry/noisy` frame) is
pointed at `/slam/odometry` instead of ground truth.

**Observed impact**: ✅ Live-verified both branches. `slam:=none` (default):
identical to pre-existing behavior, `odom_tf_sync` starts, no SLAM nodes
present — no regression. `slam:=slam mode:=frontier noise_profile:=ideal`:
all 12 expected nodes start (sim, camera TF, depth pipeline, mapper, 4 sensor
sims, pose graph, benchmark, frontier planner), `odom_tf_sync` correctly
absent, frontier exploration drives the robot using SLAM-corrected odometry,
loop closures fire on real depth-camera clouds within the first ~10 s of
motion, zero exceptions over a 45 s run. One expected transient: `octomap_server`
drops a single early message during the TF-broadcaster handoff at startup
(normal message_filter behavior, not a bug).

---

## Phase 12 — Real 2-minute-class benchmark (correcting Phase 6's headline number)

**Date**: 2026-07-04
**Files**: none (verification only)

**Objective**: The Phase 6 "70% ATE reduction" figure came from a synthetic
square-loop trajectory with dense, easily-overlapping synthetic point clouds
and a single-endpoint distance comparison — not from `benchmark.py`'s actual
ATE computation, and not from a run long enough to be a meaningful trajectory
length (~5s of represented motion). Challenged on this; re-verified properly:
a 157s (~2.6 min) `slam:=slam mode:=frontier noise_profile:=realistic` session
against the real Stonefish sim, reading `benchmark.py`'s actual `metrics.csv`
output and `plot_results.py`'s rendered figure, not a manual proxy.

**Observed impact**: ✅ Zero exceptions, 116 keyframes, 12 loop closures.
Real numbers: final cumulative ATE **0.58 m**, peak instantaneous position
error 0.95 m (around t=90-110s), mean RPE (translation) 0.12 m. Key finding:
all 12 loop closures landed in two early clusters — **zero fired in the final
~63s** of the run, because frontier exploration pushed the robot into
genuinely new territory outside the 5 m loop-closure search radius. Loop
closure is opportunistic only: it corrects drift when the robot happens to
pass near an old keyframe, it has no mechanism to make that happen. This is
the concrete, measured version of the gap Week 3 (active decision: explore vs
revisit) is meant to close — see `docs/ROADMAP.md`. One run, one unseeded
noise draw; not yet a statistically defensible ideal/realistic/degraded
comparison (would need multiple seeded runs per profile).

---

## Phase 13 — Purpose-built RViz views per mapper/slam mode

**Date**: 2026-07-05
**Files**: `bringup/rviz/demo_tsdf.rviz` (new), `bringup/rviz/demo_slam.rviz` (new),
`bringup/launch/demo.launch.py`, `bringup/setup.py`, `eval/eval_tools/eval_tools/benchmark.py`,
`eval/eval_tools/package.xml`

**Objective**: The single `demo.rviz` used for every combination showed OctoMap
displays with nothing behind them under `mapper:=tsdf` (`octomap_server` isn't
even launched in that mode), and gave no way to see SLAM drift/noise beyond
eyeballing gaps between overlapping Odometry arrows. Added two new configs and
switched between all three automatically from `demo.launch.py`'s `mapper`/`slam`
arguments (`slam:=slam` takes priority since it's the more specific need):
`demo.rviz` stays byte-for-byte the original base view; `demo_tsdf.rviz` swaps
the OctoMap displays for TSDF surface/voxels; `demo_slam.rviz` adds ground-truth
(green)/SLAM (blue)/dead-reckoning (red) `Path` overlays, the existing
`/slam/graph_edges` and `/slam/covariance` `MarkerArray`s, and a new
`/eval/markers` `MarkerArray` published by `benchmark.py` — a drift arrow from
GT to the SLAM estimate plus a `TEXT_VIEW_FACING` HUD showing live
err/ATE/RPE/keyframe-count/loop-closure-count/D-optimality, computed from data
`benchmark.py`'s `_slam_cb` already had on hand (no new subscriptions needed
beyond caching `/slam/keyframe_count` and `/slam/loop_closure_count` for the
HUD text). Config selection itself is a `PythonExpression` computing the RViz
`-d` path, verified in isolation (all 4 mapper×slam combinations resolve to the
correct file) since `ros2 launch --print-description` doesn't evaluate
per-invocation substitutions.

**Observed impact**: ✅ All 4 `mapper`×`slam` combinations resolve to the
intended config file (checked directly via `PythonExpression.perform()`, not
just read from the launch file). Live-verified `slam:=slam mode:=frontier
noise_profile:=realistic` for 45 s against the real Stonefish sim: zero
exceptions, loop closures firing as before, and `metrics.csv` shows 28
successful `_slam_cb` calls — meaning `_publish_eval_markers` (and therefore
the new drift-arrow/HUD code path) ran cleanly on every SLAM update.

## Phase 14 — Ground-truth reference map (belief vs. reality comparison)

**Date**: 2026-07-05
**Files**: `slam/stonefish_groundtruth_mapping/stonefish_groundtruth_mapping/odom_tf_sync.py`,
`slam/stonefish_groundtruth_mapping/stonefish_groundtruth_mapping/cloud_relabel.py` (new),
`slam/stonefish_groundtruth_mapping/launch/gt_map.launch.py` (new),
`slam/stonefish_groundtruth_mapping/{setup.py,package.xml}`,
`bringup/launch/demo.launch.py`, `bringup/rviz/demo_slam.rviz`

**Objective**: Under `slam:=slam`, `odom_tf_sync` (ground truth) is fully
suppressed so `pose_graph.py` can be the sole broadcaster of
`world_ned -> bluerov2/base_link` — meaning no ground-truth map was ever
built during a SLAM run, only the drifting/estimated one. There was no way
to see "what the robot believes" next to "what's actually there."

**What changed**: TF can only hold one transform per frame at a time, so
the fix isn't to re-enable `odom_tf_sync` onto `bluerov2/base_link` (that's
`pose_graph.py`'s frame now) — it needs its own parallel chain:
`world_ned -> bluerov2/base_link_gt -> bluerov2/Dcam_gt`, always fed from
`/StoneFish/Odometry` regardless of the SLAM estimate. `odom_tf_sync.py`
gained a `target_frame` parameter (default unchanged: `bluerov2/base_link`)
so a second instance can broadcast onto `base_link_gt` instead of touching
the node itself. A static `Dcam_gt` transform mirrors the existing camera
mount offset.

The depth camera's raw point cloud is identical either way (same simulated
sensor, body-frame data) — only the pose used to place it in `world_ned`
differs — so a new `cloud_relabel` node just republishes `/cloud_in` as
`/gt/cloud_in` with `header.frame_id` swapped to `bluerov2/Dcam_gt`, no
recomputation needed. `gt_map.launch.py` (included only when `slam:=slam`,
since that's the only mode where belief and truth can actually diverge)
then runs a second `octomap_server`/`tsdf_mapper` instance off that
relabeled cloud, matching whichever backend `mapper:=` selected, with
outputs under `/gt/...` (`octomap_server` via `namespace='gt'`; `tsdf_mapper`
via explicit remaps since it hardcodes absolute topic names). `demo_slam.rviz`
gained a `GroundTruthMap` `OccupancyGrid` display (enabled by default,
opaque) plus disabled-by-default `TSDFSurface_GroundTruth`/
`TSDFVoxels_GroundTruth` displays for the `mapper:=tsdf` case, overlaid
against the existing belief-map displays.

**Observed impact**: ✅ Builds clean. Live-verified both backends against
the real Stonefish sim (`slam:=slam mode:=frontier noise_profile:=realistic`):
`mapper:=octomap` (45 s) — `/gt/octomap_binary` publishing at ~24–28 Hz,
`/gt/cloud_in` at ~25 Hz matching the raw depth-camera rate, zero exceptions.
`mapper:=tsdf` (45 s) — `tsdf_mapper_gt` integrating and growing independently
of the belief instance (voxels 2130 → 4243 → 6651 → 6896, surface points
climbing to 19598), including staying healthy through an unrelated `imu_sim`
crash that degraded the belief map's TF lookups — the ground-truth chain
doesn't depend on any SLAM-side node, so it's unaffected by belief-side
failures, which is exactly the point. Not yet visually confirmed in RViz on
this machine (headless verification only, no GUI available here).

⚠️ **Found, not fixed** (pre-existing, unrelated to this change):
`slam_backend/sensor_models/imu_sim.py` crashed once with
`ValueError: scale < 0` from `np.random.normal(0, gyro_bias_drift_rad_s * dt)`
— only possible if `dt` goes negative (out-of-order/backward timestamps).
Flaky, not reproduced on every run. Worth a `dt = max(dt, 0.0)` guard at
some point, but out of scope here.
