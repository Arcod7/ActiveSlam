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

## Phase 15 — Sonar noise model (datasheet-grounded)

**Date**: 2026-07-06
**Files**: `slam/slam_backend/slam_backend/sensor_models/sonar_noise.py` (new),
`slam/slam_backend/slam_backend/sensor_models/noise_profiles.py`,
`slam/slam_backend/config/noise_{ideal,realistic,degraded}.yaml`,
`slam/stonefish_groundtruth_mapping/launch/{pointcloud,gt_map}.launch.py`,
`bringup/launch/demo.launch.py`

**Objective**: The depth-camera cloud standing in for a WaterLinked Sonar
3D-15 (`sim/world/data/robot/simple_rov.scn`) was completely noise-free —
every point exactly where the sensor model says, no matter the range. Real
imaging sonar is nothing like that: range precision is roughly constant but
lateral/cross-range uncertainty grows with range (wider beam footprint the
further out you look), plus dropouts and occasional multipath outliers. None
of that was modeled, so "SLAM under realistic sonar noise" wasn't actually
being tested by anything.

**What changed**: A new `sonar_noise` node sits between `depth_image_proc`
and every consumer (`/cloud_in_raw -> sonar_noise -> /cloud_in`), only when
`slam:=slam`. Per finite point (excluding already-invalid and "no-return"
max-range pixels, so max-range filters downstream keep working): along-ray
range noise (`sigma0 + k*range`), 1.5mm range-bin quantization (the
datasheet's stated range resolution), beam-spreading lateral jitter
(`sigma_h/v * range`, derived from the datasheet's 0.35°/0.60° H/V beam
separation), range-growing dropout probability (NaN, same convention the
rest of the pipeline already handles), and sparse multipath outliers (a
positive range excursion, since multipath is always a late arrival). All
parameters live in a new `sonar:` YAML section per noise profile — zeroed
for `ideal` (exact passthrough), datasheet-derived for `realistic`, worse
for `degraded`. The ground-truth reference map switches its own cloud
source to `/cloud_in_raw` so it stays clean regardless.

**Observed impact**: ✅ Synthetic checks confirm exact range-noise std vs.
range, dropout fraction vs. range, exact quantization lattice, NaN/no-return
pixels left untouched, and bytes-identical passthrough for the `ideal`
profile. Live: `/cloud_in_raw`, `/cloud_in`, `/gt/cloud_in` all publish at
~5 Hz with correct frame IDs; `slam:=none` regression-checked unchanged (no
`sonar_noise` node, no `/cloud_in_raw` topic).

## Phase 16 — Benchmarking switches: loop closure, noise seed, map rebuild

**Date**: 2026-07-06
**Files**: `slam/slam_backend/slam_backend/pose_graph.py`,
`planner/frontier_slam/frontier_slam/tsdf_mapper.py`,
`slam/slam_backend/slam_backend/sensor_models/{imu,dvl,pressure,sonar_noise}*.py`,
`slam/slam_backend/launch/{slam,sensors_only}.launch.py`,
`slam/stonefish_groundtruth_mapping/launch/tsdf.launch.py`,
`bringup/launch/demo.launch.py`

**Objective**: A multi-run evaluation framework needs A/B levers to actually
sweep — none existed. Also, the map was never re-aligned after a loop
closure moved keyframe poses (integrated scans stayed wherever they were
first placed), a known limitation from Phase 6 that was worth turning into
a benchmarkable option rather than leaving it a fixed limitation.

**What changed**: (1) `loop_closure_enabled` param (default true) on
`pose_graph.py` — false skips loop-closure detection and re-detection
entirely while keyframe pose refresh keeps running, so paths/viz stay
correct either way; plumbed as `demo.launch.py`'s `loop_closure:=`. (2) A
`noise_seed` override (default -1 = use the profile's baked-in seed) on all
four noise sources, each offset by a fixed per-sensor constant so a shared
seed no longer gives every sensor an identical RNG stream — needed for
seeded, reproducible, decorrelated multi-run comparisons. (3) `map_rebuild`
(default false, TSDF-only — octomap_server has no clean way to replay a
corrected sensor origin for its free-space raycasting): when a loop closure
moves a keyframe beyond a threshold, `pose_graph.py` streams every
keyframe's cloud back out at its corrected pose on
`/slam/rebuild/{begin,scan,origin}` (throttled, paced by a drain timer, not
one burst), and `tsdf_mapper.py` resets its VDBFusion volume and
re-integrates from scratch — parameterized specifically so "does rebuilding
actually help" is itself something the benchmark matrix can answer, per
this session's design decision, rather than assumed.

**Observed impact**: ✅ Live-verified each switch independently:
`loop_closure:=false` keeps `/slam/loop_closure_count` at 0 with the default
still closing loops normally; `noise_seed:=7` correctly reaches all four
sensor nodes; a forced-threshold live run fired 4 rebuild cycles cleanly
(49/49, 54/54 scans re-integrated) with the ground-truth map instance
showing zero rebuild activity and the belief surface cloud still publishing
afterward. Default settings regression-checked unaffected.

## Phase 17 — Map-quality metrics (belief vs. ground truth)

**Date**: 2026-07-06
**Files**: `eval/eval_tools/eval_tools/map_metrics.py` (new),
`eval/eval_tools/eval_tools/{benchmark,plot_results}.py`,
`eval/eval_tools/launch/eval.launch.py`, `bringup/launch/demo.launch.py`

**Objective**: `benchmark.py` is pose-only — nothing scored the belief map
itself against the ground-truth reference map (Phase 14), even though both
run side by side under `slam:=slam` (`docs/ROADMAP.md` flags this as an
open gap).

**What changed**: New `map_metrics` node. For `mapper:=octomap`: aligns
`/projected_map` against `/gt/projected_map` by their `MapMetaData` origin
offset (python `octomap` bindings aren't importable in this environment, and
the vis-marker topic is too fragile to couple metrics to) and reports
occupied-cell IoU plus known-cell coverage. For `mapper:=tsdf`: subsampled
KD-tree nearest-neighbor chamfer distance and coverage between
`/tsdf/surface_cloud` and `/gt/tsdf/surface_cloud`. Writes
`map_metrics.csv` alongside `metrics.csv` (which also gained
`lc_count`/`rebuild_count` columns fed from Phase 16's rebuild counter);
`plot_results.py` gained a coverage/IoU-or-chamfer panel.
`eval.launch.py` now resolves `output_dir` once via an `OpaqueFunction`
instead of letting `benchmark.py` and `map_metrics.py` each independently
default to their own timestamp (which could land them in two different
directories a few hundred ms apart).

**Observed impact**: ✅ Synthetic checks of the grid-alignment/IoU/coverage
math and the chamfer/coverage math (identical clouds -> ~0 chamfer, coverage
1.0; shifted clouds -> chamfer >> 0, coverage -> 0). Live 60 s runs on both
backends produced sane, time-varying values (octomap: coverage ~0.98,
IoU ~0.5; tsdf: coverage 0.89 -> 0.82, chamfer 0.3 m -> 0.72 m as drift
accumulates) with zero non-shutdown exceptions.

## Phase 18 — Batch evaluation framework

**Date**: 2026-07-06
**Files**: `eval/eval_tools/scripts/run_matrix.py` (new),
`eval/eval_tools/config/matrix_{full,smoke}.yaml` (new)

**Objective**: Comparing approaches (mapper, next-pose policy, loop closure
on/off, noise conditions) meant launching and eyeballing runs by hand, one
at a time — no way to queue a long multi-run comparison unattended and come
back to a result.

**What changed**: `run_matrix.py`, a plain script (not a ROS node) reading
a YAML matrix (configs x seeds), launches `bringup/demo.launch.py` headless
one at a time — sequential, since Stonefish's GL context and the global
topic names rule out running two sims at once. There is no
run-completion signal anywhere in the stack (frontier exploration just logs
"no frontiers found" and keeps spinning), so a wall-clock `duration_s` is
the termination mechanism, escalating SIGINT -> SIGTERM -> SIGKILL on the
whole process group if the sim doesn't stop cleanly; `benchmark.py` and
`map_metrics.py` flush every row as it's written, so a killed run still
leaves valid partial data. Each run gets a `manifest.json` (resolved args,
seed, git SHA, timing, a validity verdict) and an automatic `plot_results`
call; one retry only fires when a run produced literally zero data (a
failed start, not a real result). At the batch level: `summary.csv` (one
row per run — final ATE/RPE/coverage/IoU/chamfer/loop-closure count/rebuild
count/traceback count) plus per-axis comparison boxplots.
`matrix_full.yaml` bundles all four requested axes into one 35-run
preset (~5.3 h); `matrix_smoke.yaml` is a 2-run/120 s pre-flight check.

**Observed impact**: ✅ `--dry-run` on the full matrix preset prints exactly
35 resolved commands and rejects a `motion:=wallfollow` + `mapper:=octomap`
config at load time; caught and fixed a bug where dry-run was still
creating empty run directories (verified fixed: identical directory count
before/after). A real mini-batch (2 runs x 120 s) produced every expected
per-run artifact plus a valid summary/comparison at the batch level,
including a clean back-to-back Stonefish restart; `--aggregate-only`
re-runs cleanly against existing manifests.

## Phase 19 — Teardown hardening + refreshed 600 s baseline

**Date**: 2026-07-09
**Files**: all 17 node `main()` functions (`slam/slam_backend/slam_backend/`,
`planner/frontier_slam/frontier_slam/`, `eval/eval_tools/eval_tools/`,
`slam/stonefish_groundtruth_mapping/stonefish_groundtruth_mapping/`),
`eval/eval_tools/scripts/run_matrix.py`, root `STATE.md`

**Objective**: The full 35-run matrix from Phase 18 came back with every single
run marked `crashed_soft`. Every node in the stack printed a teardown traceback
on `ros2 launch`'s SIGINT — `ExternalShutdownException` escaping an unguarded
`rclpy.spin()`, then a second, unconditional `rclpy.shutdown()` call raising
`RCLError: rcl_shutdown already called`. None of it was a real fault, but
`run_matrix.py` couldn't tell the difference, so `crashed_soft` was
meaningless noise on every batch run to date.

**What changed**: All 17 node `main()`s were rewritten to a uniform pattern —
`except (KeyboardInterrupt, ExternalShutdownException): pass` around `spin()`,
then `finally: node.destroy_node(); rclpy.try_shutdown()` — which fixed the
common case. Live testing then surfaced a second, rarer race: a *second* SIGINT
landing while a node was still inside `destroy_node()` in that `finally` block
raised `KeyboardInterrupt` out of `main()` uncaught. Fixed by wrapping the
`finally` body in a nested `try/except KeyboardInterrupt: pass` (also covering
the three nodes — `frontier_extractor`, `wall_follower`, `waypoint_controller`
— that close a CSV session log there too).

Separately, `run_matrix.py`'s validity check counted *any* line containing
`Traceback` as a crash. New `_count_tracebacks()` groups log lines by their
`[proc-name-N]` prefix and walks each traceback block to its actual exception
line: a block ending in `KeyboardInterrupt` is now benign (a second SIGINT
during shutdown, not a fault), anything else — or a block that never resolves
before EOF — stays harmful, and `status=crashed_soft` keys on harmful count
only. Building the classifier caught a real parsing bug of its own: a blank
line can appear mid-traceback (e.g. a C-implemented property frame with no
source line), and the first version misread it as the exception summary,
mislabeling several genuinely-benign `KeyboardInterrupt` cases as harmful.

Finally, reran the seeded baseline properly: 600 s, `noise_seed:=42`,
`realistic` profile (including the sonar noise model), on the now-clean
teardown code — replacing the 157 s/unseeded/pre-sonar-noise figure from
Phase 12.

**Observed impact**: ✅ A 60 s `slam:=slam mode:=frontier mapper:=tsdf
map_rebuild:=true` launch torn down with a single SIGINT now logs zero
`Traceback` lines and zero `process has died` entries. The new classifier
verified against real logs: the fresh 600 s baseline reports `(0 harmful, 1
benign)`; an old pre-fix batch log correctly still reports harmful tracebacks
(7, from the now-impossible double-shutdown `RCLError` chain) rather than
being retroactively whitewashed — old manifests stay as an accurate historical
record. Synthetic checks cover a genuine crash, an unterminated traceback at
EOF, and interleaved concurrent-process log output. Refreshed baseline: final
ATE **0.5356 m**, final instantaneous error 0.7908 m, mean RPE (translation)
0.1137 m, coverage 0.9651, IoU 0.5194, 35 loop closures over 94 metrics rows —
all 35 loop closures were, again, opportunistic (no autonomous revisit
behavior exists yet).

## Phase 20 — Uncertainty-triggered revisit planner (Week 3 v1)

**Date**: 2026-07-09
**Files**: `planner/frontier_slam/frontier_slam/revisit_planner.py` (new),
`frontier_extractor.py` (suspend switch), `bringup/demo.launch.py`,
`planner/frontier_slam/launch/frontier_slam.launch.py`,
`eval/eval_tools/eval_tools/benchmark.py`, `eval/eval_tools/scripts/run_matrix.py`,
`eval/eval_tools/config/matrix_revisit.yaml` (new), `slam/slam_backend/slam_backend/pose_graph.py`

**Objective**: Close the Week 3 gap identified in Phase 19 — loop closure was
opportunistic-only, the robot never deliberately revisits anything. New node
`revisit_planner.py` watches `/slam/dopt` (D-optimality of the latest keyframe's
marginal covariance — the only live-correct uncertainty signal available) and,
when it crosses a threshold, suspends frontier exploration and drives to a
scored candidate among past keyframes (density of nearby old keyframes minus
travel cost), preferring dense, distant, unrevisited clusters. Explicitly out of
v1, deferred to future work per `docs/ROADMAP.md` Week 3's open bullets: FPFH
submap saliency (still just geometric density, no descriptor-based
distinctiveness) and mirror-graph covariance propagation *along candidate
paths* (v1 reacts to the live D-optimality value rather than projecting it
forward for each candidate).

**What changed**: `frontier_extractor` gained a `/frontier_slam/suspend` (Bool)
switch — while suspended it stops publishing its own goal (it republishes every
2 s, so a race rather than a suspend would just have the two goal sources
fight) but keeps replanning its A* path toward whatever goal is currently
adopted, including externally-published ones on `/frontier_slam/goal`. The new
`revisit_planner` node is a plain state machine (`RevisitStateMachine`,
rclpy-free, unit-tested) — EXPLORING → REVISITING on trigger (suspend, pick
target, republish goal), → COOLDOWN on any of {loop closure count rose, dopt
dropped back below resume threshold, timeout, arrived-and-dwelled with no
closure}, → EXPLORING after a cooldown period. Wired into `demo.launch.py`
(`revisit:=true`, requires `slam:=slam`) and the eval stack (`revisit_count`
column, `matrix_revisit.yaml` A/B preset).

While reading the closure counter to interpret the batch results below, found
and fixed a real bug in `pose_graph.py`: `/slam/loop_closure_count` summed each
keyframe's own `loop_closures` list, but a closure was only ever recorded under
the *newer* keyframe at initial detection. When the *older* keyframe of the
pair later moved (any optimizer update shifting it >0.1 m/0.05 rad) and
triggered `_redetect_and_apply`, it re-discovered and re-added the same
physical pair under itself — a genuine duplicate `BetweenFactorPose3` edge into
iSAM2, not just a display artifact. Fixed with a run-lifetime `_closed_pairs`
set (keyed on the unordered keyframe-index pair) gating both the initial-
detection and redetect call sites. **Definition to quote alongside any
lc_count figure**: *lc_count = number of accepted loop-closure edges
(BetweenFactorPose3) added to the pose graph, one per unique keyframe pair.*
Figures from before this fix (the Phase 19 baseline's 35, and the batch below)
predate the dedup and should be read as edge counts inflated by an unknown
amount from the redetect-duplicate bug, not exact closure-event tallies —
not directly comparable to future runs.

Also classified post-SIGINT tracebacks that land during launch teardown but
raise something other than `KeyboardInterrupt` (e.g. an rclpy pybind11
teardown race) as benign — phase (before/after the SIGINT marker in the log),
not exception type, is the principled signal; see `run_matrix.py`.

**Observed impact**: ✅ Forced-trigger live test (`ros2 param set
/revisit_planner dopt_trigger 0.002`): state → revisiting, suspend → true,
frontier_extractor went silent, robot drove to the selected target, lc_count
rose, dopt dropped, state → cooldown → exploring. ✅ Loop-closure dedup fix
live-verified on a 140 s run: 7 `Loop closure: node X <-> [...]` log lines,
final lc_count exactly 7 — including a redetect cascade over 21 moved
keyframes that correctly found zero new pairs.

⚠️ Natural-trigger A/B batch (`matrix_revisit.yaml`, 480 s, seeds 101–102, no
forced params) — a **mechanism demonstration, not an ATE-improvement claim**
(n=2, and end-of-run ATE on trajectories that have diverged differently is not
apples-to-apples):

| run | status | final ATE | abs err | coverage | IoU | lc_count | revisits |
|---|---|---|---|---|---|---|---|
| baseline s101 | ok | 0.450 | 0.253 | 0.959 | 0.547 | 433 | 0 |
| baseline s102 | ok | 0.672 | 1.068 | 0.955 | 0.534 | 200 | 0 |
| revisit s101 | ok | 1.190 | 1.103 | 0.923 | 0.502 | 308 | 2 |
| revisit s102 | crashed_soft\* | 0.590 | 0.083 | 0.973 | 0.507 | 452 | 2 |

The mechanism fired naturally in both revisit runs (2 revisits each, full 480 s
completed). Mixed at n=2: s102 favours revisit on every pose metric (ATE 0.590
vs 0.672, abs err 0.083 vs 1.068, coverage up, more closures); s101 favours
baseline (ATE 1.190 vs 0.450, fewer closures). All `lc_count` values in this
table predate the dedup fix above — treat as inflated, not exact. \*The s102
`crashed_soft` label is teardown noise, not a run failure: its traceback (an
rclpy pybind11 `RuntimeError` during shutdown) landed ~25 log lines after
launch's SIGINT marker; all 480 s of metrics were written and are trustworthy.
A dedicated multi-seed batch (needed for a defensible ATE statement) and a
combined revisit+rebuild eval column are open, user-scheduled follow-ups, not
done here.

## Phase 21 — Frontier+TSDF spinning-robot discovery; dual-map fix; Phase 18 rebuild finding invalidated

**Date**: 2026-07-10
**Files**: `bringup/launch/demo.launch.py`, `bringup/package.xml`

**Discovery**: `frontier_extractor.py` subscribes to `/projected_map`, which
only `octomap_server` publishes. `demo.launch.py` previously included the
octomap stack only under `mapper:=octomap`, so `mode:=frontier mapper:=tsdf`
had no map source for frontier detection: goal selection silently returned
every tick (no goal, no error, no log line), and the robot rotated in place
for the entire run instead of exploring. Diagnostic signature for spotting
this in any past or future run: `lc_count` in the thousands (co-located
keyframes from pure rotation closing loops with each other), suspiciously low
final ATE (no translation ⇒ no drift), coverage stuck at whatever is visible
from spawn, and `waypoint_controller`/`frontier_extractor` silent after
startup in the launch log.

**Consequence — Phase 18 finding invalidated**: The Phase 18 result quoted
above and in `STATE.md` ("`map_rebuild` measurably hurt TSDF map quality —
coverage 0.77→0.5, chamfer roughly doubled") was measured with
`matrix_full.yaml`'s `tsdf`/`tsdf_rebuild` rows configured as
`mode:frontier mapper:tsdf` — i.e. on a spinning robot. The result is an
artifact of the missing map source, not a rebuild-fidelity measurement.
Whether `map_rebuild` helps or hurts map quality on a robot that actually
explores is unmeasured; the `tsdf`/`tsdf_rebuild` matrix rows need
re-specifying and re-running before any conclusion can be drawn. The rebuild
plumbing and correction math (Phase 20-era `pose_graph.py`/`tsdf_mapper.py`
work) were still exercised live and are mechanism-verified — only the
map-fidelity result on a moving robot is in question.

**Deleted data (user instruction, 2026-07-09)**:
`eval/runs/full_20260706_1406/tsdf_s101..105` and `tsdf_rebuild_s101..105`
(10 dirs; `summary.csv` re-aggregated to the remaining 25 rows);
`eval/runs/rebuild_20260709_1603/` (partial re-run batch); earlier same-day
`rebuild_20260709_1500` and `rebuild_20260709_1503`. Deleted numbers
preserved for the record (all from spinning robots — not directly comparable
to future runs): `tsdf` s101–105 ATE 0.10–0.34, coverage 0.74–0.80, chamfer
0.89–1.02, lc 4135–5718; `tsdf_rebuild` s101–105 ATE 0.09–0.36, coverage
0.26–0.58, chamfer 1.64–2.17, lc 4324–5172, rebuilds 1–5.

**Fix**: `demo.launch.py` now also launches a bare `octomap_server` node
(not `octomap.launch.py`'s include, which would double-launch
`stonefish_simulator`/`tf`/pointcloud) when `mode:=frontier mapper:=tsdf`,
feeding `/projected_map` to `frontier_extractor` for planning while
`tsdf_mapper` stays the map product — a dual-map setup, announced with a
`LogInfo`. `octomap_server` added to `bringup/package.xml` exec_depend (was
only reached transitively before).

**Observed impact**: ✅ Watched run `mode:=frontier mapper:=tsdf slam:=none`:
robot translated (5.5,-8.3)→(17.0,-0.6) over the log window instead of
spinning in place, `frontier_extractor`/A* operating on real mapped cells,
`ros2 node list | sort | uniq -d` empty, exactly one `stonefish_simulator`
process, clean SIGINT teardown with no orphans. RViz confirmed showing the
TSDF surface (octomap has no display in `demo_tsdf.rviz` by design — a
TSDF-focused view). `matrix_full.yaml`'s `tsdf`/`tsdf_rebuild` rows and
`matrix_smoke.yaml`'s `tsdf_rebuild` row remain invalid as currently
configured pending re-specification against the dual-map setup.

## Phase 22 — `sonar_only` noise profile

**Date**: 2026-07-10
**Files**: `slam/slam_backend/config/noise_sonar_only.yaml` (new),
`slam/slam_backend/launch/slam.launch.py`,
`slam/slam_backend/launch/sensors_only.launch.py`,
`slam/stonefish_groundtruth_mapping/launch/pointcloud.launch.py`

New `sonar_only` noise profile: sonar block copied from `realistic`
(WaterLinked 3D-15 datasheet values), pressure/IMU/DVL blocks from `ideal`,
`seed: 42`. Isolates sonar noise from dead-reckoning drift — fills the gap in
the difficulty ladder between all-ideal and all-realistic (noisy map input
against near-ground-truth odometry, with and without loop closure). The
three launch files only needed their description strings updated (they
build the config path from the name, unconstrained).

Verified: clean rebuild of `slam_backend` (new file in the `config/*.yaml`
`data_files` glob), `sensors_only.launch.py noise_profile:=sonar_only` starts
all four sensor sims with ideal-grade nav values (pressure sigma 0.001 m, DVL
scale err 0.000%).

## Phase 23 — Trackball defaults to following `bluerov2`

**Date**: 2026-07-13
**Files**: `sim/stonefish_ros2/src/stonefish_ros2/ROS2GraphicalSimulationApp.cpp`

Every launch previously opened Stonefish's free-floating default view; the
GUI has a "Trackball center" dropdown to glue the camera to a robot, but it
had to be set by hand each run. `Startup()` now looks up the `bluerov2`
robot right after `Init()` (scenario is fully built by then — robots and the
trackball are both constructed inside `SimulationManager::RestartScenario()`,
which `Init()` calls synchronously) and calls
`getTrackball()->GlueToMoving(rob->getBaseLink())` before
`StartSimulation()`. No stonefish core (patched fork) changes needed —
`GlueToMoving` was already public API, just never invoked outside the manual
GUI path.

Verified: `colcon build --symlink-install --packages-select stonefish_ros2`
succeeds cleanly. Not yet confirmed against a live sim run (no GPU/RViz
session in this pass) — flagged for the next visual check.

## Phase 24 — Wall-guided motion executors: orientation blend + forward controller

**Date**: 2026-07-14 to 2026-07-15
**Files**: `planner/frontier_slam/frontier_slam/wall_follower.py`,
`planner/frontier_slam/frontier_slam/wall_oriented_controller.py` (new),
`planner/frontier_slam/launch/frontier_slam.launch.py`,
`planner/frontier_slam/launch/wall_follow.launch.py`,
`planner/frontier_slam/setup.py`,
`planner/frontier_slam/test/test_wall_heading.py` (new),
`planner/frontier_slam/test/test_wall_oriented.py` (new), `README.md`

Two path executors now sit on top of the BLOCKED motion-status protocol
(Phase prior): `motion:=walllooking` (`wall_follower`) and
`motion:=walloriented` (the new `wall_oriented_controller`).

`wall_follower` separates translation and viewing controls. `path_influence`
blends wall-tangent and planner-path travel; viewing builds a path-derived
heading (`path_look_offset_deg`, toward the wall) and a wall-derived heading
(`wall_normal_offset_deg`, toward the route), then blends them over the
shortest circular arc with `path_heading_weight` (0 = wall-derived, 1 =
path-derived). A new SWITCH state turns to observe the goal side and selects
an alternative wall when no path progress is made within
`progress_timeout_s`; both states now report `BLOCKED` to the planner instead
of stalling silently.

`wall_oriented_controller` is a simpler sibling: it follows the same
goal/path as the ordinary waypoint controller, holonomically projecting route
velocity onto surge/sway, while yawing `look_offset_deg` toward whichever
side the nearest mapped surface point falls on. It does not estimate wall
normals, regulate standoff, or alter the route.

Verified in the ROS Jazzy container: `colcon test --packages-select
frontier_slam` passes all 46 tests (incl. `test_wall_heading.py` and
`test_wall_oriented.py`). A watched live simulation is still required to
tune/confirm the default offsets and blend weights.

## Phase 25 — Compass-corrected attitude, and two odometry noise profiles

**Date**: 2026-07-15
**Files**: `slam/slam_backend/slam_backend/attitude_filter.py` (new),
`slam/slam_backend/slam_backend/sensor_models/compass_sim.py` (new),
`slam/slam_backend/slam_backend/dead_reckoning.py`,
`slam/slam_backend/slam_backend/sensor_models/noise_profiles.py`,
`slam/slam_backend/config/noise_{ideal,realistic,degraded,sonar_only}.yaml`,
`slam/slam_backend/config/noise_odom_pos_only.yaml` (new),
`slam/slam_backend/config/noise_odom_only.yaml` (new),
`slam/slam_backend/launch/sensors_only.launch.py`,
`slam/slam_backend/package.xml`, `slam/slam_backend/setup.py`,
`slam/slam_backend/test/test_attitude_filter.py` (new),
`slam/slam_backend/test/test_noise_profiles.py` (new),
`README.md`, `STATE.md`

Attitude fusion no longer treats IMU orientation as absolute. Roll/pitch
still come straight from the IMU, but yaw is now propagated from IMU
increments and corrected by a separate, lower-rate absolute compass heading
through a wrapped-angle Kalman filter (`YawKalmanFilter` in the new
`attitude_filter.py`). A new `compass_sim` node publishes that absolute
heading from ground truth with its own noise/bias model
(`CompassNoise` in `noise_profiles.py`); `dead_reckoning.py` and
`sensors_only.launch.py` wire it in alongside pressure/IMU/DVL. Every
existing profile gained a `compass:` block, and `noise_realistic.yaml` /
`noise_degraded.yaml` reinterpret `imu.sigma_yaw_rad` as short-term
gyro noise (with heading error now carried by the compass) rather than
the old absolute-heading value; seed offsetting picks up a `compass +5`
slot alongside the existing per-sensor offsets.

Two new noise profiles isolate odometry from sonar error, the position
counterpart to Phase 22's `sonar_only`: `odom_pos_only` keeps realistic
DVL/pressure position noise while passing Stonefish attitude (IMU + compass)
and sonar through exactly, isolating position drift from attitude and map
error; `odom_only` keeps the full realistic nav stack (position and
attitude) with an exact sonar passthrough, isolating all odometry error
from sonar noise. `demo.launch.py`'s `noise_profile` choices and the three
noise-profile-consuming launch files gain both names.

Verified in the ROS Jazzy container: `colcon build --symlink-install
--packages-select slam_backend` succeeds cleanly; `colcon test
--packages-select slam_backend` passes all 5 tests, including
`test_attitude_filter.py`'s Kalman-filter propagation/correction/wraparound
cases and `test_noise_profiles.py`'s check that `odom_pos_only` matches
`realistic` position noise with zero-sigma attitude and exact sonar.

## Phase 26 — Launcher, restartable launch layers, and install friction

**Date**: 2026-07-22
**Files**: `launcher.py` (new), `launcher_core.py` (new),
`launcher_model.py` (new), `bootstrap.sh` (new),
`slam/stonefish_groundtruth_mapping/launch/core.launch.py` (new),
`slam/stonefish_groundtruth_mapping/launch/tf_only.launch.py` (new),
`slam/stonefish_groundtruth_mapping/launch/pointcloud_only.launch.py` (new),
`slam/stonefish_groundtruth_mapping/launch/mapper_only.launch.py` (new),
`slam/stonefish_groundtruth_mapping/launch/{tf,pointcloud,octomap,tsdf}.launch.py`,
`slam/stonefish_groundtruth_mapping/setup.py`,
`bringup/launch/demo.launch.py`, `sim/world/scenario/` (renamed from
`scnenario/`), `sim/world/setup.py`, `sim/world/run.sh`,
`sim/world/data/obj.sha256` (new), `sim/world/data/README.md`,
`.gitmodules` (new), `external/` (new),
`requirements.txt`, `docs/INSTALL.md`, `README.md`,
`{eval/eval_tools,slam/slam_backend,slam/stonefish_groundtruth_mapping}/package.xml`,
`tools/launch_tools/{package.xml,setup.py}`,
`planner/frontier_slam/test/test_bluerov2_model.py`

The mapping stack was a strict include cascade with the simulator at the
bottom, so changing any option restarted Stonefish. Each layer is now
launchable on its own (`core` / `tf_only` / `pointcloud_only` /
`mapper_only`) and the existing entry points are thin compositions of
those, leaving `demo.launch.py` and the node sets it produces unchanged.

On top of that, `launcher.py` runs the stack as independently restartable
groups: swapping the mapper, mode or pose source bounces only the layers
that depend on it (verified by PID — the simulator survives). It also arms
the fail-closed motion gate, which previously had no control outside RViz's
panel and silently left the vehicle immobile; resets a run by respawning
the vehicle and clearing map/pose-graph/eval state; drives by keyboard
without a second terminal; and retunes the wall parameters live via
`ros2 param set`. Every group runs in its own process group and is torn
down with an escalating SIGINT/SIGTERM/SIGKILL applied to the *group* —
`ros2 launch` exiting does not mean its nodes have.

Install friction: `bootstrap.sh` collapses the setup into one idempotent
command; Stonefish and vdbfusion are pinned as submodules under `external/`
rather than cloned and patched by hand (vdbfusion matters most on aarch64,
where PyPI ships no wheel); numpy/scipy/matplotlib moved to rosdep keys in
the packages that import them, leaving `requirements.txt` to what rosdep
genuinely cannot provide; the mesh set gained a tracked `obj.sha256`
manifest that `bootstrap.sh` verifies. The `scnenario/` misspelling is
fixed, and `demo.launch.py` derives the octomap `LD_PRELOAD` multiarch
triplet instead of hardcoding `aarch64-linux-gnu`, which had made the demo
x86_64-only.

Note: rosdep's `gtsam` key resolves to `ros-jazzy-gtsam`, which ships the
C++ libraries and no Python module, so it cannot replace the PyPI wheel
that `slam_backend` imports. Left on pip deliberately.

## Phase 27 — Fail-closed motion gate, RViz arming panel, and ArduSub adapter

**Date**: 2026-07-17 (acceptance evidence 2026-07-21)
**Files**: `planner/frontier_slam/frontier_slam/safety_gate.py` (new),
`planner/frontier_slam/frontier_slam/safety_logic.py` (new),
`planner/frontier_slam/frontier_slam/ardusub_adapter.py` (new),
`planner/frontier_slam/frontier_slam/ardusub_control.py` (new),
`planner/frontier_slam/config/ardusub.yaml` (new),
`planner/frontier_slam/launch/ardusub_adapter.launch.py` (new),
`planner/frontier_slam/frontier_slam/{control_utils,heavy_sim_mixer,wall_follower,wall_oriented_controller,waypoint_controller}.py`,
`planner/frontier_slam/launch/{frontier_slam,wall_follow}.launch.py`,
`planner/frontier_slam/test/{test_safety_logic,test_heavy_mixer}.py`,
`tools/motion_safety_rviz/` (new package),
`bringup/launch/demo.launch.py`, `bringup/rviz/*.rviz`,
`sim/world/data/robot/bluerov2_unphy.scn`, `IRL_TEST.md` (new),
`docs/INSTALL.md`, `planner/frontier_slam/README.md`

Every executor now publishes body commands through a single gate node
(`safety_gate.py`, pure decision logic in `safety_logic.py`) instead of
driving the vehicle directly. The gate is **fail-closed**: `start_enabled`
defaults to false, so nothing moves until `/motion/enable` is published, and
it zeroes output on a stale command, a non-finite or out-of-range value, or
a missing odometry heartbeat. `/motion/safety_status` reports the reason.
An RViz panel (`tools/motion_safety_rviz`) arms and disarms it without a
terminal, and is loaded by all four RViz configs.

`ardusub_adapter.py` translates the same gated body command into MAVLink
`MANUAL_CONTROL` for an ArduSub vehicle, with its own arming, mode and
failsafe handling in `ardusub_control.py` and per-vehicle scaling in
`config/ardusub.yaml`. `IRL_TEST.md` is the staged bring-up procedure for
taking the stack to a physical BlueROV2.

**Caveat: the adapter has not been accepted against a physical vehicle.**
Phase 0 Part 1 of `IRL_TEST.md` (bench acceptance of the gate itself, no
vehicle) passed 18/18 on 2026-07-21 — evidence in
`eval/runs/irl_phase0_20260721/` (`gate.log` plus two recorded bags), which
shows the gate rejecting malformed commands, holding DISABLED until armed,
and dropping to STALE_COMMAND when the command stream stops. Part 2 needs an
operator at the RViz panel and is not yet run.

Sim consequence, only understood later: because `frontier_slam.launch.py`
instantiates the gate for *every* `mode:=frontier` launch, headless
evaluation batches also started fail-closed — see Phase 28.

## Phase 28 — Cable-safe sweep scanning, thruster calibration, and armed eval batches

**Date**: 2026-07-21 / 2026-07-22
**Files**: `planner/frontier_slam/frontier_slam/scan_sweep.py` (new),
`planner/frontier_slam/frontier_slam/waypoint_controller.py`,
`planner/frontier_slam/test/test_scan_sweep.py` (new),
`planner/frontier_slam/launch/frontier_slam.launch.py`,
`bringup/launch/demo.launch.py`, `sim/world/data/robot/bluerov2_unphy.scn`,
`eval/eval_tools/scripts/run_matrix.py`,
`eval/eval_tools/config/matrix_*.yaml`

Scanning in place no longer spins a full revolution. `scan_sweep.py` drives
an odometry-confirmed right-half → left-full → return-to-start cycle whose
net cumulative yaw is zero, so a tethered vehicle cannot wind its cable up;
progress is measured from actual yaw rather than elapsed time, and a
per-cycle timeout (including the return phase) bounds the worst case at
twice the configured deadline. It replaces the spin at all three sites —
initial scan, no-goal `SCAN`, and `GOAL_REACHED` — leaving the CSV state
labels untouched for the eval scripts. `scan_style` (`sweep` default,
`spin` = previous behaviour) and `scan_sweep_deg` are launch arguments on
both launch files.

Two calibration errors surfaced while validating it. The BlueROV2's
thruster `max_setpoint` had been 340 RPM since the simulator was first
imported, roughly 11x below a real T200's ~3800-4000, suppressing achievable
thrust by close to two orders of magnitude and making `CTRL_STUCK_ESCAPE`
fire during ordinary drives; it is now 3800 on all eight thrusters, with
rotor dynamics switched from `zero_order` to `first_order` (0.4 s) so
setpoints ramp instead of stepping. `SCAN_YAW` was then recalibrated 0.08 →
0.026 (~15°/s), which gives the 5 Hz sonar proper frame overlap.
**Velocity and timing figures recorded before this fix are not comparable
with anything after it.**

The headless evaluation path needed one further change: the Phase 27 gate
is instantiated for every `mode:=frontier` launch, and `run_matrix.py` had
no way to arm it, so batches were silently benchmarking a motionless
vehicle (yaw pinned within ±3.5°, under 1 cm of travel over 90 s).
`safety_start_enabled` is now a valid matrix key and is set true in the sim
configs, leaving the gate's fail-closed default intact for real hardware.

**Measured counter-result.** The sweep was expected to *lower* the loop
closure count relative to a spin, on the theory that spinning piles up
keyframes. It does the opposite, and the reasoning behind the expectation
was wrong: loop-closure candidacy in `pose_graph.py` is positional
(`loop_closure_radius_m`, `loop_closure_min_gap`) and never looks at
heading. Over a 78 s seed-42 A/B, the sweep produced fewer keyframes (84 vs
107) but more accepted closure edges (101 vs 47, i.e. 3.6 vs 2.0 per closing
node) and better localisation on every metric — ATE 0.122 vs 0.192, mean
absolute error 0.102 vs 0.170, D-optimality 0.0036 vs 0.0062. The likely
mechanism is that re-traversing headings gives a new keyframe better cloud
overlap with an in-radius earlier one, so ICP accepts where a monotonic spin
is rejected. Sweep therefore stays the default on its own merits; the
tether-safety argument was never contingent on the closure count.

Tests: 117 pass in `planner/frontier_slam/test/`, including
`test_scan_sweep.py`'s phase-sequence, net-zero-yaw (with wraparound),
timeout-guard, return-phase-deadline and `spin`-bypass cases.

## Phase 29 — Batch-validity guards, rebuild A/B, free-space carving, uv + Open3D

**Date**: 2026-07-22
**Files**: `eval/eval_tools/scripts/run_matrix.py`,
`eval/eval_tools/test/test_run_validity.py` (new),
`eval/eval_tools/config/matrix_rebuild.yaml`,
`eval/eval_tools/config/matrix_carve.yaml` (new),
`planner/frontier_slam/frontier_slam/tsdf_mapper.py`,
`planner/frontier_slam/test/test_carve_no_return.py` (new),
`slam/stonefish_groundtruth_mapping/launch/{mapper_only,tsdf}.launch.py`,
`bringup/launch/demo.launch.py`, `bootstrap.sh`, `.gitmodules`,
`external/open3d` (new submodule), `docs/INSTALL.md`, `README.md`,
`requirements.txt`, `.gitignore`

**Batch-validity guards.** A run whose vehicle never moves still writes a full
`metrics.csv`, `map_metrics.csv` and TUM trajectory, so the failure is
invisible in the outputs. A six-run batch was lost to it: orphaned node stacks
from earlier killed launches were still alive, and their duplicate motion gates
and TF broadcasters pinned the vehicle at 0.05 m of travel and 1.7° of yaw over
480 s. `run_matrix.py` now refuses to start while such nodes are alive (listing
their PIDs), kills any run that has neither travelled 1 m nor swept 20° after
90 s, and aborts the whole batch on the first motionless run, since the cause
is environmental and applies to every run that follows. Ground-truth path
length and yaw range are recorded per run and in `summary.csv`. The floors are
low enough that a run still doing its initial in-place scan passes on yaw
alone.

**Rebuild A/B (`matrix_rebuild.yaml`, 3 seeds × 480 s, post-calibration).**
All six runs clean; `eval/runs/rebuild_20260722_0157/`.

| run | seed | ATE | coverage | chamfer | rebuilds |
|---|---|---|---|---|---|
| tsdf | 101 / 102 / 103 | 0.158 / 0.188 / 0.151 | 0.966 / 0.955 / 0.960 | 0.361 / 0.351 / 0.340 | 0 |
| tsdf_rebuild | 101 / 102 / 103 | 0.182 / 0.156 / 0.569 | 0.958 / 0.964 / 0.742 | 0.366 / 0.372 / 0.640 | 0 / 0 / 1 |

The rebuild fired in one seed of three, so five of six runs compare the
mechanism against itself; excluding that seed the arms are indistinguishable
(ATE 0.166 vs 0.169, coverage 0.961 vs 0.961, chamfer 0.351 vs 0.369). Where it
did fire everything was worse, but n=1 and confounded — a rebuild only triggers
on a large correction, i.e. on a run that had already drifted. Two arms are
also not identical when no rebuild fires: `map_rebuild:=true` switches on
per-scan caching regardless, and those runs show ~3.5x the final absolute error
at matched seeds. **No rebuild-fidelity claim is made from this**; measuring it
needs a forced trigger or a policy that reliably produces large closures.

**Free-space carving (`carve_no_return`, default false).** No-return pixels
carry information — the ray reached maximum range without hitting anything —
but were dropped with the other NaNs, leaving open water unknown. The
parameter synthesizes a pseudo-point along each such pixel's ray so VDBFusion's
space carving frees the voxels it traverses, with ray directions recovered from
intrinsics fitted to the cloud itself rather than restating the sensor's FoV.
vdbfusion has no carve-only ray API, so each pseudo-point's endpoint also
writes a surface; `carve_range_m` (16 m) was meant to keep that artefact past
the 15 m sensor maximum and outside the mapped envelope.

**It does not, and the reasoning was wrong.** One-seed A/B
(`eval/runs/carve_20260722_1207/`, 300 s): coverage fell 0.952 → 0.446, chamfer
rose 0.349 → 8.33 m, and belief-to-ground-truth RMSE 0.30 → 9.25 m. On a
vehicle that travels ~80 m in a run, 16 m from one pose is well inside the
volume already mapped from another, so no fixed carve range can park the
artefact outside the map. The parameter stays off and is kept only as the
record of a measured negative result; making this work needs carve-only rays,
which is a vdbfusion change, not a tuning one.

**Python dependencies.** `bootstrap.sh`'s `pip install` failed under PEP 668 on
Ubuntu 24.04 — ROS 2 Jazzy's own target — so the documented one-command setup
did not work on the platform it documents. Dependencies now install into a
uv-managed virtualenv (`--python /usr/bin/python3` so Debian's dist-packages
stay visible to `rclpy`, `--allow-existing` so re-runs stay idempotent). Open3D
joins Stonefish and vdbfusion as a pinned submodule (v0.19.0) built by
`--with-open3d`, since PyPI publishes no aarch64 wheel for it; the build needs
CMake pointed at the venv interpreter, and pulls VTK from source because no
aarch64 binary is published for that either. `docs/INSTALL.md` records the two
aarch64 pitfalls: the static-TLS import failure and its glibc tunable, and
VTK's download needing a host-side fetch behind a container that cannot
complete the TLS handshake.

Tests: 127 in `planner/frontier_slam/` and `eval/eval_tools/`, including 10 new
run-validity cases and 8 for the carving geometry.

## Phase 30 — Geometry-aware sonar noise: grazing dropout, correlated speckle, sound-speed scale

**Date**: 2026-07-22
**Files**: `slam/slam_backend/slam_backend/sensor_models/sonar_noise.py`,
`slam/slam_backend/slam_backend/sensor_models/noise_profiles.py`,
`slam/slam_backend/config/noise_{ideal,realistic,degraded,sonar_only,odom_only,odom_pos_only}.yaml`,
`slam/slam_backend/test/test_sonar_noise.py` (new)

**Objective**: Every term in the Phase 15 sonar noise model was drawn
independently per point, and none of them looked at the geometry being
measured. Two consequences. First, dropouts fell as uniform per-beam confetti
at a rate set only by range, whereas the dominant real no-return mechanism is
specular: a smooth surface at oblique incidence reflects energy away from the
transducer and an edge-on wall returns almost nothing. Second, independent
noise is precisely the kind a pose graph averages out, so the model flattered
the pipeline — real speckle is partly frozen between pings and real dropouts
arrive in patches.

**What changed**: Three additions, all inside `sonar_noise`.

*Grazing-incidence dropout.* The node now estimates a per-pixel surface normal
from the organized cloud and adds `dropout_p_grazing * (1 - cos θ)^n` to the
dropout probability. Degenerate normals (borders, NaN neighbours) fall back to
normal incidence; depth discontinuities yield edge-on normals and so a grazing
penalty, which matches real sonar dropping returns at object edges. The
unorganized-cloud path falls back to the previous range-only behaviour.

*Correlated noise fields.* Range error and dropout are drawn from smooth
Gaussian random fields (white noise filtered at `corr_length_px`, rescaled by
the analytic `2s*sqrt(pi)` so variance stays unit) rather than per point, with
AR(1) correlation `corr_rho_time` between consecutive pings. `range_corr_frac`
splits the range error between an independent and a correlated draw with
weights that keep total variance at sigma^2, so enabling correlation changes
the structure of the error without changing its magnitude. Dropout thresholds
the field at its own quantile, so the dropout *rate* is unchanged and only its
spatial arrangement becomes patchy.

*Speed-of-sound scale error.* One `sos_scale_error_pct` draw per run
multiplies every range. Unlike every other term it is systematic, so no amount
of averaging removes it; it is the mechanism behind map-scale drift.

**Observed impact**: 19 tests in `slam/slam_backend/`, and a live node probe
against synthetic walls at controlled incidence. Grazing dropout on the
`realistic` profile rises 0.024 (head-on) to 0.151 (75°), and 0.245 on
`degraded`. With outliers rejected, speckle std at 3 m is 0.00859 m against a
predicted `sigma0 + k*r` of 0.008; ping-to-ping correlation is +0.426 against
~0.35 predicted by `range_corr_frac * corr_rho_time`; spatial lag-1 correlation
is +0.559 where the old model gave ~0. The `ideal` profile remains an exact
passthrough (0.000 dropout, 0.0000 m error) end to end.

**Worth recording for anyone reading the raw statistics**: the multipath
outlier term dominates them. At `outlier_p` 0.002 with 0.3-3.0 m excursions it
contributes ~0.077 m of range-error std against ~0.008 m of speckle, so
uncorrected frame-to-frame correlation reads +0.002 and hides the effect
entirely. Any future check of the correlation terms has to reject outliers
first.

**Not validated against hardware.** No real Sonar 3D-15 range images have been
recorded, so these terms are geometrically motivated, not fitted. The numbers
in the profiles are argued from the datasheet and from the physics, and should
be treated as a stress test rather than a calibrated sensor model. Fitting
them needs a tank session: a flat wall swept through incidence angles gives
the dropout-vs-incidence curve directly, and a 90° corner exposes multipath.

Tests: 19 in `slam/slam_backend/`, 12 of them new.

## Phase 31 — Strongest-return ranging, projection-aware jitter, reverberation, geometric multipath

**Date**: 2026-07-22
**Files**: `slam/slam_backend/slam_backend/sensor_models/sonar_noise.py`,
`slam/slam_backend/slam_backend/sensor_models/noise_profiles.py`,
`slam/slam_backend/config/noise_{ideal,realistic,degraded,sonar_only,odom_only,odom_pos_only}.yaml`,
`slam/slam_backend/test/test_sonar_noise.py`

**Objective**: The Phase 30 model was still geometry-blind in four ways that
separate a z-buffer from an imaging sonar. The depth camera reports the nearest
surface per pixel; the real device (`RangeImage` in the WaterLinked 3D-15
protocol) reports the strongest echo per beam. Its beams are one contiguous
concave structure away from the sim's independent pixels, and there was no
mechanism for volume backscatter or for geometry-dependent multipath.

**What changed**: An organized-image stage now runs ahead of the per-point
noise, skipped for unorganized clouds and for the `ideal` profile.

*Strongest-return ranging (`argmax_window_px`).* A beam is wider than a pixel,
so within a small window each candidate surface is scored by a beam-pattern
weight times cos(incidence)/range^2, candidates are grouped by range, and the
range of the strongest group wins. A structure thin in both image dimensions
fills few window pixels and is outvoted by the broad surface behind it (it
fades); a bright near surface pulls neighbouring beams toward its range
(edges bleed). A one-pixel-wide vertical wire does **not** fade — the beam
integrates its whole column — which is physically correct and worth
remembering when reading results.

*Projection-aware lateral jitter (`lat_sigma_beam_frac`).* The scenario's depth
camera is a 90-degree pinhole, so the per-pixel angular step runs from
~0.45 deg at the image centre to ~0.22 deg at the 45-degree edge (cos^2 law),
not the uniform separation the old constant `lat_sigma_*_per_m` assumed. Each
pixel is one beam in this approximation, so the cross-range jitter is now the
measured local beam width times range times the fraction, and it varies across
the image as the real per-pixel footprint does. **This changes the lateral
error magnitude across the image and so breaks strict comparability with
pre-Phase-31 benchmark runs**; `realistic` keeps the image-centre jitter near
the old value so the change is a redistribution, not a global inflation. The
per-metre constants remain as the fallback when the fraction is zero.

*Volume reverberation (`reverb_p`, `reverb_max_m`, `reverb_weak_boost`).*
Particles, bubbles and the tether backscatter sound; because the device reports
the strongest echo, a near-field volume return can beat a weak surface return.
Fired on a spatially correlated field so returns clump, at a rate elevated
where the surface echo is weak (its strength falls as cos(incidence)/range^2),
with the replaced range drawn from a 1/r^2 backscatter density over
[0.2, reverb_max_m]. Off in clear-water `ideal`, small in `realistic`, a
defining feature of `degraded`.

*Geometric multipath (`multipath_p`).* Screen-space second bounce: reflect each
beam about its surface normal and march the depth buffer (intrinsics recovered
from the cloud's own pinhole layout); a re-intersection reports the late path
length r + t_hit, fired with probability rising as the direct return weakens.
A flat wall reflects away from all geometry and produces no phantom; a concave
corner does. Replaces the geometry-blind `outlier_p` term for the corner case
(that term is kept, additive, and unchanged). **Limitation**: a single
rasterized view cannot see a bounce off geometry outside the frustum (notably
the water surface behind the sensor), so only in-frustum multipath is modelled.

**Observed impact**: 29 tests (10 new), all passing, plus a live node probe
against a synthetic corner with a floated blob. `ideal` remains a byte-exact
passthrough end to end (0 dropout, 0 multipath, 0 reverb, scale 1.0). On
`realistic`: the blob fades to the background, multipath fires on ~0.4% of the
image concentrated at the concave seam and nowhere on the open faces (every
phantom a late arrival), reverberation injects ~250 near-field returns per
ping, and degraded turbidity roughly triples the reverberation.

**Still not fitted to hardware.** As with Phase 30, every value is argued from
the datasheet and acoustics, not measured — no real 3D-15 range images exist on
this project. A tank session (flat wall swept through incidence, a 90-degree
corner, still vs. stirred water, a static scene held 30 s) would constrain
nearly every parameter added across Phases 30 and 31.

Tests: 29 in `slam/slam_backend/`, 22 of them for the sonar model.

## Phase 32 — Near-field gate: a launch-tunable knob to clear the reverberation spray

**Date**: 2026-07-23
**Files**: `slam/slam_backend/slam_backend/sensor_models/sonar_noise.py`,
`slam/slam_backend/slam_backend/sensor_models/noise_profiles.py`,
`slam/slam_backend/config/noise_*.yaml`,
`slam/slam_backend/test/test_sonar_noise.py`,
`slam/stonefish_groundtruth_mapping/launch/pointcloud_only.launch.py`,
`bringup/launch/demo.launch.py`

**Objective**: In RViz the Phase 31 volume reverberation showed up as a fan of
returns spraying off the vehicle into open water — ~430 near-field points per
ping accumulating into a persistent junk cloud around every robot pose. That is
the term behaving as designed (particles/bubbles/tether backscatter, and the
device reports the strongest echo), but at `reverb_p=0.01` for clear water it
dominates the near field and does not match the clean ground truth. The user
wanted a single knob, settable per run over any noise profile, to pull the
noised cloud back toward ground truth.

**What changed**: A near-field gate. `min_range_m` drops any noised return
nearer than the threshold (NaN, the existing dropout convention), applied after
all other terms so it also removes reverberation, near multipath, and any other
near-field return. It is exposed as a launch argument `near_cutoff` on
`demo.launch.py`, threaded to the `sonar_noise` node the same way `noise_seed`
is (declared at the top level, inherited by the nested pointcloud include). The
node reads a `min_range_m` parameter that overrides the profile's own value when
non-negative (-1 = use the profile, which ships at 0 = gate off), so the knob
works over every profile without editing YAML:

```
ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic near_cutoff:=1.6
```

**Observed impact**: 31 tests (2 new), all passing. Live A/B on the `realistic`
profile against a 6 m wall: `near_cutoff:=-1` (off) leaves 429 near-field
returns per ping in open water; `near_cutoff:=1.6` leaves 0, while the 6 m wall
is untouched. The gate is off by default, so existing runs are unchanged.

**Note on the underlying value.** This gate hides the reverberation rather than
retuning it. `reverb_p=0.01` may itself be high for a clear-water "realistic"
profile — real clear water has little volume reverberation — so lowering
`reverb_p` is the alternative to gating. The gate is the blunt, per-run knob;
tuning `reverb_p` is the modelling fix. Neither is fitted to hardware.

Tests: 31 in `slam/slam_backend/`, 24 of them for the sonar model.

## Phase 33 — Expose the near-field gate in the launcher TUI ("Noise Attenuation")

**Date**: 2026-07-23
**Files**: `launcher_model.py`, `launcher_core.py`

**Objective**: Phase 32 added the `near_cutoff` launch argument, but the TUI
launcher (`launcher.py`) composes the individual launch files itself and never
passed it, so the knob was invisible to the interface actually used to start
runs.

**What changed**: A `noise_attenuation` parameter (label **"Noise Attenuation"**,
enum `none` / `cut_close`) in the SLAM section of the launcher model, visible
under `slam:=slam` alongside the noise profile. The `cloud` group now maps it to
the launch argument — `cut_close` -> `near_cutoff:=1.6`, `none` ->
`near_cutoff:=-1.0` (profile default) — and lists `noise_attenuation` in its
`depends`, so changing it restarts only the point-cloud layer.

**Observed impact**: Smoke-tested — the parameter registers with the expected
label and choices, and the emitted `pointcloud_only.launch.py` command carries
`near_cutoff:=1.6` for `cut_close` and `near_cutoff:=-1.0` for `none`. The
existing 31 sonar tests are unaffected.

## Phase 34 — Range-image view of the SLAM input cloud in RViz

**Date**: 2026-07-23
**Files**: `slam/slam_backend/slam_backend/sensor_models/range_image.py` (new),
`slam/slam_backend/setup.py`,
`slam/stonefish_groundtruth_mapping/launch/pointcloud_only.launch.py`,
`bringup/rviz/demo.rviz`, `bringup/rviz/demo_slam.rviz`, `bringup/rviz/demo_tsdf.rviz`

**Objective**: The noised sonar data only existed as a PointCloud2 (/cloud_in),
so there was no depth-camera-style 2D image of what SLAM actually consumes — no
way to look at the noise profile and the near-field gate the way the clean depth
camera (/sensor_msgs/image_depth) can be viewed.

**What changed**: A `range_image` node renders any organized cloud back into a
`sensor_msgs/Image` (32FC1, per-pixel euclidean range in metres, dropouts and
no-returns = 0). `pointcloud_only.launch.py` runs two instances:
`/cloud_in -> /cloud_in/range_image` (what SLAM receives: noise profile plus
whatever Noise Attenuation is active) and `/cloud_in_raw -> /cloud_in_raw/range_image`
(the clean reference). All three RViz layouts (`demo`, `demo_slam`, `demo_tsdf`)
gain an Image display **"SLAM input (noised range)"** (enabled) and
**"Clean cloud (range)"** (off by default), beside the existing DepthCamera
display. The launcher TUI already loads these layouts and launches
`pointcloud_only.launch.py`, so no launcher change was needed.

**Observed impact**: Verified the node publishes a valid 32FC1 257x67 image —
a 6 m wall renders as 6.00-8.61 m (euclidean range grows off-axis), frame_id
preserved. Toggling Noise Attenuation none/cut_close changes the
`SLAM input (noised range)` image live; the clean image stays put for
comparison. 9 packages build, 31 sonar tests unaffected.

## Phase 35 — Tunable cut distance for Noise Attenuation (advanced TUI param)

**Date**: 2026-07-23
**Files**: `launcher_model.py`, `launcher_core.py`

**Objective**: The Phase 33 "Noise Attenuation: cut_close" hardcoded a 1.6 m
gate, too short for profiles whose reverberation reaches further (degraded goes
to ~2.5 m) and not adjustable from the interface.

**What changed**: An advanced float parameter **"Cut distance (m)"**
(`near_cutoff_m`, default 1.6, range 0.2-15.0) in the SLAM section, visible only
in advanced mode when `noise_attenuation == cut_close`. The `cloud` group passes
its value as `near_cutoff:=<m>` and lists it in `depends`, so editing it
restarts only the point-cloud layer.

**Observed impact**: Smoke-tested — the parameter is advanced, bounded, and
visible only under slam + cut_close; the emitted command carries the chosen
distance (`cut_close` at 2.5 -> `near_cutoff:=2.5`, `none` -> `-1.0`).

## Phase 36 — Evaluation audit: time-uniform metrics and trustworthy structural validity

**Date**: 2026-07-23
**Files**: `eval/eval_tools/eval_tools/benchmark.py`,
`eval/eval_tools/launch/eval.launch.py`,
`eval/eval_tools/scripts/run_matrix.py`,
`eval/eval_tools/test/{test_benchmark_metrics,test_run_validity}.py`,
`slam/slam_backend/slam_backend/sensor_models/sonar_noise.py`,
`slam/slam_backend/test/test_sonar_noise.py`, `STATE.md`, `docs/RUN.md`

**Objective**: A live audit found that the benchmark scored only irregular
`/slam/pose` keyframes. Initial turning generated many low-error samples while
straight travel generated few, biasing cumulative ATE; RPE's sample-count delta
also represented a different time span on every interval. The audit additionally
reproduced a yaw-wrap false negative in the frozen-run guard, rejection of the
current `near_cutoff` launch argument, cross-domain false positives in orphan
detection, and expected all-NaN image borders warning inside the sonar model.

**What changed**: `benchmark.py` now writes and scores continuously corrected
`/slam/odometry` at the dead-reckoning rate, and `rpe_delta` now means a fixed
number of seconds (default 1.0). `run_matrix.py` unwraps yaw before measuring its
excursion, matches actual node executables instead of arbitrary command-line
substrings, compares each candidate process's `ROS_DOMAIN_ID`, accepts
`near_cutoff`, counts pre-shutdown child deaths and non-zero launch exits, and
records `status_scope=structural_validity_only`. The pinhole fit skips fully
invalid rows/columns before taking medians, eliminating normal all-NaN warnings.

**Observed impact**: Two identical pre-fix 120 s runs on commit `4011d3b`, seed
1 produced ATE 1.255 vs 0.709 m, endpoint error 2.258 vs 1.259 m, and 45 vs 25
loop closures. This establishes that `noise_seed` repeats sensor RNG streams but
does not make the asynchronous end-to-end experiment deterministic. After the
fix, a 60 s isolated-domain live run completed `ok` with 520 continuous metric
rows/520 SLAM poses (ATE 0.0736 m, endpoint error 0.175 m), zero tracebacks,
zero child deaths, and no all-NaN warnings. The two modified packages build,
the evaluation launch description parses, and all 173 focused project tests pass.

## Phase 37 — Post-audit simulation matrix, rebuild-path exercise, and TF timing diagnosis

**Date**: 2026-07-23
**Files**: `STATE.md`, `Progress.md` (simulation artifacts under `eval/runs/`)

**Objective**: Exercise the repaired continuous evaluator back-to-back, verify
that TSDF reset/replay survives real loop-closure corrections, and investigate
the mapper's recurring `TF unavailable` warnings.

**Observed impact**: The isolated-domain 2x120 s smoke batch
`eval/runs/smoke_20260723_0150/` completed both cells with structural status
`ok`, dense metrics, no pre-shutdown process death, and no harmful traceback.
Baseline finished at ATE 0.202 m / endpoint error 0.270 m; TSDF finished at
ATE 0.346 m / endpoint error 0.472 m. These are single asynchronous trajectories,
not a mapper-quality ranking. The normal rebuild-enabled smoke cell fired zero
rebuilds, confirming that the preset does not cover the mechanism reliably.

A 180 s seed-103 mechanism run in
`eval/runs/forced_rebuild_20260723_0156/` lowered only the live trigger thresholds.
It fired four genuine pose-graph rebuilds; all four TSDF cache replays completed
(104, 107, 114, and 181 cached scans), exploration continued, the continuous
metric stream's maximum gap was 0.384 s, and the manifest recorded zero harmful
tracebacks/process deaths. This verifies execution and continuity, not quality
benefit: the threshold intervention and trajectory make it unsuitable as an A/B.

Finally, a read-only timing probe alongside the normal 70 s run
`eval/runs/tf_probe_20260723_0202/` received 302 clouds: 151 had TF immediately,
150 more obtained the exact capture-time TF within 0.5 s, and one expired. The
mapper currently drops on the immediate lookup failure; its 0.1 s wait occurs
inside the same single-threaded executor that must receive TF, so it cannot cure
the arrival-order race. This is a real TSDF input-loss defect. A TF message
filter or bounded deferred-cloud queue is the appropriate follow-up.

## Phase 38 — Exact-time asynchronous TF deferral for TSDF input

**Date**: 2026-07-23
**Files**: `planner/frontier_slam/frontier_slam/tsdf_mapper.py`,
`planner/frontier_slam/test/test_tsdf_tf_queue.py`, `STATE.md`, `docs/RUN.md`

**Objective**: Fix the Phase 37 arrival-order defect without using latest-TF
fallbacks (which would permanently misplace points during rotation), unbounded
memory, or a home-grown transform-availability poller.

**What changed**: The Python Jazzy binding has no `tf2_ros.MessageFilter`, so
the mapper now uses the maintained `Buffer.wait_for_transform_async()` API.
An immediate exact-time lookup remains the fast path. A miss returns control to
the single-threaded executor and enters a capture-ordered FIFO backed by async
TF futures. The queue is bounded to 10 clouds and 0.5 s; expiry and overflow
cancel their futures explicitly. One recovered cloud is integrated per 20 ms
tick to avoid starving TF reception, and periodic counters make received,
integrated, deferred, recovered, expired, failed, overflow, and queued totals
observable. Both belief and ground-truth TSDF instances use the same path.

**Observed impact**: In the matched 70 s post-fix run
`eval/runs/tf_probe_20260723_0215/`, the belief mapper had received 290 clouds
at the final 60 s report, deferred 141, recovered 140, expired one, and
overflowed none. The ground-truth instance recovered 85/90 deferred clouds;
its five expiries were startup-only. This replaces Phase 37's behavior where
151/302 transforms were unavailable at callback time and those usable-later
clouds were discarded.

The 180 s stress run `eval/runs/forced_rebuild_20260723_0217/` reached 843
clouds by 170 s: belief deferred/recovered/expired = 417/408/9 and ground truth
= 291/283/8, with zero overflow and zero future failures. Expiry totals stopped
changing after startup. A natural 547-scan rebuild began 1.7 s before the run's
fixed shutdown, so a deterministic early correction was tested separately in
`eval/runs/rebuild_replay_20260723_0221/`: all 258 retained scans replayed in
1.32 s, cloud processing continued for another 30 s, maximum metric gap was
0.121 s, and no traceback/process death occurred. The full focused project
suite passes (179 tests); `colcon test --packages-select frontier_slam` passes
all 123 collected package tests, and `frontier_slam` builds cleanly.

## Phase 39 — Three-seed TSDF matrix and rebuild/error attribution

**Date**: 2026-07-23
**Files**: `STATE.md`, `docs/RUN.md`, `Progress.md` (simulation artifacts under
`eval/runs/`)

**Objective**: Run a three-seed TSDF/rebuild matrix after the TF deferral fix,
then reproduce the reported positional-error increase with SLAM + TSDF +
frontier + wall-oriented motion + realistic noise and determine whether TSDF
replay causes it.

**Observed impact**: All six 180 s cells in
`eval/runs/tf_fix_3seed_20260723_0233/` completed with structural status `ok`,
zero harmful traceback/process death, zero TF-queue overflow, and zero async-TF
future failure. Rebuild-minus-baseline final ATE deltas for seeds 101/102/103
were -0.158, -1.906, and +1.650 m respectively. The sign reversal and large
differences in path/yaw/loop-closure count confirm that seeded sensor noise does
not make asynchronous closed-loop trajectories paired; final arm means are not
causal rebuild estimates. Event alignment is more informative: rebuild seed
101 improved over the following 10 s, seed 102 began replay at shutdown and is
inconclusive, and seed 103's error increase began with the triggering pose-graph
update, before TSDF replay.

The targeted 2x240 s wall-oriented seed-103 batch is
`eval/runs/walloriented_rebuild_ab_20260723_0254/`. The no-rebuild arm finished
at ATE 0.119 m / endpoint error 0.490 m with 22 closures. In the rebuild arm,
instantaneous error was stable near 0.35 m immediately before node 84 closed to
node 51. It jumped 0.346520 -> 0.861994 m in one 109 ms metric step, 36 ms after
the closure log and 139 ms **before** the mapper announced rebuild start. The
graph update moved 40/85 existing keyframes (maximum 0.99 m), and re-detection
raised the closure count 69 -> 72. TSDF then replayed all 606 cached scans in
2.04 s. Mean absolute error was 0.323 m in the preceding 10 s and 0.795 m in the
following 10 s; during replay it was 0.875 m and remained 0.752 m over the 10 s
after replay. Map coverage was 0.891 before the event and 0.915 at the first
post-replay sample, so replay did not create the pose jump and did not visibly
corrupt the map at that instant.

**Conclusion**: TSDF rebuild is temporally correlated because it is triggered
by the same large graph correction, but it does not write pose-graph state and
is downstream of the measured positional jump. The likely failure mode is an
ambiguous/inconsistent loop closure: `pose_graph.py` accepts every locally good
ICP candidate within 5 m, may add several correlated factors at once (including
unlogged re-detected edges), and has no transform-innovation or cross-closure
consistency gate. No behavioral threshold was changed: a simple maximum
correction limit could also reject the legitimate large-drift closures rebuild
exists to handle. The reliable next step is to log each candidate's inlier
ratio/error and ICP-vs-prediction translation/yaw innovation, then validate a
consistency policy across more true and false closure events.

## Phase 40 — Split odometry/scan noise model; drift attributed to over-trusted scan-matching

**Date**: 2026-07-23
**Files**: `slam/slam_backend/slam_backend/pose_graph.py`, `STATE.md`, `Progress.md`
(simulation artifacts under `eval/runs/`)

**Objective**: Determine what drives the large positional error in the
wall-oriented / realistic-noise runs and fix it.

**Investigation**: Scored `/slam/odometry` (the pose-graph estimate) against the
raw `/slam/sensors/dead_reckoned_odom` input, both versus `/StoneFish/Odometry`
in `world_ned` with no SE3 alignment. Raw dead-reckoning stayed at ~0.1–0.3 m
final error across all runs, with vertical error ~0.01 m: depth (pressure) and
attitude (IMU+compass) are absolute, and only X/Y integrates DVL body velocity
(`dead_reckoning.py`: `delta_world = R @ v_body·dt`; no accelerometer
double-integration). The DVL error is realistically modelled — 0.2% per-run
scale, 0.001 m/s bias, 1% white — producing that 0.1–0.3 m drift. The pose graph
was often worse than this input. Two mechanisms, both from one cause: a single
noise model (σ=0.05 m/0.05 rad) shared by the DVL BetweenFactor and all sonar
scan-match/loop BetweenFactors. Because that σ was tighter than the DVL is
accurate and looser than sonar GICP is noisy, GTSAM over-trusted registration —
(1) sequential scan-match factors alone dragged the estimate ~2.6× worse than
raw odometry (visible with loop closure off), and (2) an over-trusted loop
closure occasionally warped the graph in one step (Phase 39's 0.35→0.86 m; other
seeds to 2.0–2.4 m). An index-free ground-truth-proximity test showed the Phase
39 closure fired at a genuine revisit (0.45 m true approach), so the defect is
metric over-weighting of an inaccurate relative transform, not perceptual
aliasing. This is dominated by run-to-run stochasticity — the same config/seed
ranged 0.08–0.65 m — so the degradation is a tail risk, not the typical case.

**What changed**: Replaced the shared `icp_sigma_*` model with three: a tight
`odom_sigma` (0.02/0.02) applied non-robustly to the dead-reckoning BetweenFactor
and the origin prior (so a bad closure cannot down-weight the reliable DVL), and
a looser Huber-robust `scan_sigma` (0.08/0.12) for sequential scan-match,
loop-closure, and re-detected-closure factors. All four sigmas are ROS
parameters and are logged at startup. The 0.02 m odometry σ accumulates to
√80·0.02 ≈ 0.18 m over ~80 keyframes, matching the observed DVL drift.

**Observed impact**: A wall-oriented / frontier / realistic-noise, 4-arm × 3-seed
batch was run before (`eval/runs/walloriented_cutclose_20260723_1057`) and after
(`eval/runs/walloriented_cutclose_20260723_1201`) the change (mapper=tsdf,
map_rebuild=false, 240 s). Mean final error, before→after: loop-closure-off
0.454→0.150 m, clean-sonar (odom_only) 0.677→0.104 m, near_cutoff 0.265→0.137 m,
baseline (already healthy) 0.176→0.180 m. Worst per-run peak error 1.30→0.30 m;
largest single closure-induced step 1.23→0.11 m; every one of the 12 runs now
degrades gradually with no jump. With loop closure off the estimate now ties raw
odometry (0.154 vs 0.157 m) instead of drifting 2.6× worse, and clean-sonar SLAM
beats raw dead-reckoning on all three seeds despite firing 111–456 closures.
`colcon build --packages-select slam_backend` is clean and the gtsam model
construction was smoke-tested.

**Caveat**: n=3 per arm and asynchronous Stonefish/ROS/planner execution
dominate run-to-run variance, so the arm means are directional, not precise —
the tail elimination (peak 1.30→0.30 m, zero closure jumps) is the robust result.
The sigmas are physically motivated but not tuned; a consistency/innovation gate
on loop closures (Phase 39) remains available as a complementary safeguard.

## Phase 41 — Launcher driving rework: always-live keys, AZERTY, follow-the-point

**Date**: 2026-07-24
**Files**: `launcher.py`, `launcher_core.py`, `launcher_model.py`,
`bringup/rviz/demo.rviz`, `bringup/rviz/demo_slam.rviz`,
`bringup/rviz/demo_tsdf.rviz`, `docs/RUN.md`, `STATE.md`, `README.md`,
`Progress.md`

**Objective**: Remove the `t` teleop mode key from the control screen, support
AZERTY keyboards, and add a target-point follow mode (`y`) where the drive
keys move a goal point the vehicle swims to autonomously.

**What changed**: Driving is no longer a sub-mode. In teleop mode the launcher
becomes the `/motion/body_command` source as soon as the gate/mixer group is
up, and the QWEASD cluster (plus Space/X/F) drives immediately; in frontier
mode the first drive keypress performs the old `t` flow — suspend the planner,
bring up the teleop gate/mixer, take over — and `Esc` releases back to
autonomy (first `Esc` drops the target point if one is active). Displaced
option-screen keys moved: edit `e`→`i`, advanced `a`→`o`, stop stack `s`→`k`,
quit is `Esc` only, and Space no longer cycles values (it is drive-up). The
key table lives in `launcher_core.DRIVE_KEYS` per layout; AZERTY maps the same
physical cluster (Z fwd, A strafe-left, Q yaw-left, W kept as a forward
alias), selected by a new persisted `keyboard` option, applied live with no
restart. The `y` toggle places a target point at the vehicle's current pose
(so engaging never commands a jump), moves it in the vehicle's yaw frame with
the same cluster (0.5 m/keypress × speed factor; yaw keys inert, `F` recalls
the point to the vehicle), and runs a P-controller (surge ∝ distance ×
cos heading error, yaw ∝ heading error with a 0.4 m dead zone, heave ∝
vertical error, saturated at the gate's ±1.0) at the UI's 5 Hz tick so the
fail-closed gate always sees a fresh command. The controller works in the
odometry the operator sees: `/slam/odometry` under slam:=slam, ground truth
otherwise. The point is drawn as a green sphere on
`/activeslam/target_point`, with a Marker display added to all three RViz
configs (fixed frame world_ned in each). Key translation, point motion and
the controller are pure-stdlib functions in `launcher_core.py`.

**Verification**: 60 headless checks pass (both layouts' key maps, command
signs against `keyboard_control.py`, point translation in the yaw frame,
surface clamping, controller tolerance/saturation/angle-wrap). RViz configs
re-parsed as YAML with the new display present. Not yet exercised against the
running simulator — the takeover/release and reset/apply interactions with
the planner gate deserve a manual smoke run (drive in teleop, `y` follow,
drive-key takeover and `Esc` release in frontier).

## Phase 42 — TSDF as an OcTree, and an rqt layer in the launcher

**Date**: 2026-07-25
**Files**: `slam/tsdf_octomap/` (new package),
`planner/frontier_slam/frontier_slam/tsdf_mapper.py`,
`slam/stonefish_groundtruth_mapping/launch/mapper_only.launch.py`,
`slam/stonefish_groundtruth_mapping/launch/tsdf.launch.py`,
`slam/stonefish_groundtruth_mapping/package.xml`,
`bringup/launch/demo.launch.py`, `bringup/rviz/demo_tsdf.rviz`,
`launcher_core.py`, `launcher_model.py`,
`eval/eval_tools/scripts/run_matrix.py`, `STATE.md`, `Progress.md`

**Discovery**: Under `mapper:=tsdf` nothing publishes `/octomap_binary`,
`/occupied_cells_vis_array` or `/octomap_point_cloud_centers` — Phase 21's
dual-map `octomap_server` was later replaced by `tsdf_mapper` deriving
`/projected_map` itself, and no octree source took its place. All three
OctoMap displays in `demo_tsdf.rviz` were therefore bound to topics with no
publisher (two also disabled), so the TSDF belief map had no occupancy view in
RViz beyond the marker-based `TSDFVoxels`, which was off. `STATE.md` still
described the dual-map arrangement and has been corrected.

**TSDF → OcTree**: new `tsdf_octomap` package (ament_cmake) with
`tsdf_to_octomap`, which rebuilds an `octomap::OcTree` from two clouds and
publishes it on `/octomap_binary` (latched, `octomap_msgs/Octomap`, binary).
`tsdf_mapper` gained `publish_free_voxels` → `/tsdf/free_voxels`, the
observed-empty (d > 0) half that `/tsdf/occupied_voxels` cannot express;
without it every unoccupied cell would be unknown and 3-D frontier detection
impossible. Both come from the walk that already feeds `/projected_map`, so
the extra cost is one cloud publish. The tree is rebuilt from scratch each
cycle rather than accumulated: the TSDF is reset+re-integrated after a large
loop closure (`map_rebuild:=true`), and an incrementally-updated octree would
retain cells the TSDF has already corrected away. Free cells are written
first so an occupied cell wins any coordinate claimed by both. Requires the
pyopenvdb grid path; the surface-vertex fallback has no free space and the
node logs an error and stays quiet.

Wired as `tsdf_octomap:=true` (default) through `demo.launch.py` →
`tsdf.launch.py` → `mapper_only.launch.py`, as a launcher option
(`TSDF → OcTree`, visible under `mapper:=tsdf`, in the mapper group's
`depends` so toggling restarts just that layer), and as a `run_matrix.py`
matrix key so batches can turn it off. `demo_tsdf.rviz`'s two OctoMap
displays are renamed `OcTree (occupied)` (now enabled, Z-axis coloured) and
`OcTree (free)`, both switched to transient-local QoS to match the latched
publisher.

**Verification**: ✅ Live headless run `mode:=frontier mapper:=tsdf`:
`/octomap_binary` publishes at 0.5 Hz (the cloud rate) with `id=OcTree`,
`resolution=0.2`, `binary=true`, growing 4.8k→5.7k occupied and 46k→57k free
cells into ~25k octree nodes; `tsdf_to_octomap` present exactly once in
`ros2 node list`. Not yet confirmed in the RViz GUI — headless only.
`/static_camera_tf` appears twice in `ros2 node list` on a single clean stack;
pre-existing and unrelated to this change, but worth a look.

**Launcher rqt layer**: new `rqt` bool option (default off) and a matching
group after `rviz` in `GROUP_ORDER`, launching plain `rqt` — node graph, topic
monitor, plots, parameter reconfigure. Like the RViz group it is killed
outright rather than SIGINT-laddered (a viewer has nothing to flush) and has no
`depends`, so it never restarts on a config change. `rqt` needs no install:
`ros-jazzy-rqt-common-plugins` is already present in the container.

## Phase 43 — One camera/sonar view per RViz config, and a docked eval metrics panel

**Date**: 2026-07-25
**Files**: `bringup/rviz/demo.rviz`, `bringup/rviz/demo_tsdf.rviz`,
`bringup/rviz/demo_slam.rviz`, `bringup/package.xml`,
`tools/eval_hud_rviz/` (new package), `STATE.md`, `Progress.md`

**Discovery**: all three RViz configs enabled two or three `Image` displays
at once (`DepthCamera`, `SLAM input (noised range)`, `Clean cloud (range)` in
`demo_slam.rviz`), each opening its own dock window. Multiple `Image` dock
windows tab together, and only the front tab renders — which one that is on
launch is controlled entirely by the saved `QMainWindow State` blob (an
opaque, hand-uneditable hex-encoded `QMainWindow::saveState()` binary), not by
anything the launch args select. That is why the visible camera/sonar view
appeared to change unpredictably between runs.

**Fix**: each config now enables exactly one `Image` display —
`DepthCamera` (ground truth depth camera) in `demo.rviz`/`demo_tsdf.rviz`
(no SLAM, so there is no noise model to contrast against), `SLAM input
(noised range)` in `demo_slam.rviz` (what the pose graph actually consumes).
The other Image displays are left in the config, disabled, rather than
deleted, so they stay one click away if needed.

**Eval HUD panel**: the ATE/RPE/D-opt live text (`benchmark.py`'s `eval_hud`
marker namespace on `/eval/markers`) was a `TEXT_VIEW_FACING` marker floating
above the robot in world space — legible only when the camera happened to be
pointed at it. New `tools/eval_hud_rviz` package (mirrors the structure of
`tools/motion_safety_rviz`: a `pluginlib`-registered `rviz_common::Panel`)
subscribes to `/eval/markers`, extracts the `eval_hud` namespace marker's
text, and renders it in a panel docked at the bottom of the window — the slot
the `Time` panel previously occupied in `demo_slam.rviz`. The `Time` panel
entry is removed from that config (not from `demo.rviz`/`demo_tsdf.rviz`,
which have no `ErrorHUD` display and so no metrics to show). The `eval_hud`
marker namespace itself is disabled in the `ErrorHUD` `MarkerArray` display
to avoid showing the same numbers twice; the `eval_drift` arrow in the same
display is untouched.

**Verification**: ✅ `eval_hud_rviz` builds cleanly against the existing
`ros_ws` install as an overlay (`colcon build --packages-select
eval_hud_rviz bringup`); its plugin is discoverable via the ament pluginlib
index (`rviz_common__pluginlib__plugin` resource resolves to the package);
`test_eval_hud_panel` (gtest, `QT_QPA_PLATFORM=offscreen`) passes — initial
placeholder text, and the queued-signal update path from a marker callback to
the label both verified. All three edited `.rviz` files parse as valid YAML.
**Not yet verified**: live GUI confirmation that the panel actually docks
where `Time` was — the saved `QMainWindow State` blob has no entry for the
new panel name (it never existed when that state was saved), so Qt's
`restoreState` will fall back to a default placement for it; may need
dragging into place once, after which RViz will save the new geometry.

## Phase 44 — One RViz config for every mode; the eval panel actually docks

**Date**: 2026-07-25
**Files**: `bringup/rviz/demo.rviz` (now the only config, renamed from
`demo_slam.rviz`), `bringup/rviz/demo_slam.rviz` + `bringup/rviz/demo_tsdf.rviz`
(deleted), `bringup/launch/demo.launch.py`, `bringup/setup.py`,
`bringup/package.xml`, `launcher_core.py`, `launcher_model.py`,
`docs/TROUBLESHOOTING.md`,
`slam/stonefish_groundtruth_mapping/launch/gt_map.launch.py`, `STATE.md`,
`Progress.md`

**Report**: after Phase 43, RViz still showed the ROS-time bar, the eval
metrics were nowhere on screen, and the ground-truth TSDF was missing.

**Discovery**: Phase 43 edited `demo_slam.rviz` only, but the config in use
was one of the other two — the picker in `demo.launch.py` (mirrored, a second
time, in `launcher_core.py`) selects `demo_slam.rviz` only under `slam:=slam`.
`demo.rviz` and `demo_tsdf.rviz` therefore still carried the `rviz_common/Time`
panel, had no `eval_hud_rviz/EvalHudPanel`, and — since the ground-truth
displays were only ever added to `demo_slam.rviz` — no GT map at all. Three
hand-maintained ~700-line configs meant every RViz change had to be applied
three times, and this one was applied once.

**Second defect, in `demo_slam.rviz` itself**: Phase 43 flagged that the saved
`QMainWindow State` blob had no entry for the new panel and might need
dragging into place. It is worse than that — the blob still named `Time` in
the bottom dock slot. Qt's `restoreState` turns a name it cannot match into a
`QPlaceHolderItem` and leaves the real, unnamed dock wherever `addPane` put
it, so the panel would not have appeared at the bottom even under
`slam:=slam`. The blob is hex-encoded `QMainWindow::saveState()` output and
the dock name inside it is a length-prefixed UTF-16BE string, so it is
patchable by hand after all: `fb 00000008 "Time"` became
`fb 00000010 "Eval HUD"`, in place, keeping the surrounding geometry ints.

**Open, and not caused by this change — `Image` displays ignore `Enabled` on a
fresh launch**: live runs showed `SLAM input (noised range)` coming up
unchecked with no image dock, despite `Enabled: true`. Bisected against
unmodified configs from `HEAD`: Phase 43's `demo_slam.rviz` does the same, and
so does the *old* `demo.rviz` with its `DepthCamera` — which nonetheless shows
as enabled in a long-running session (that window's title carries RViz's `*`
modified marker, i.e. it was switched on by hand). Removing the
`QMainWindow State` blob entirely does not change it either, so the saved
layout is not the cause. RViz ties an `Image` display's enabled state to its
dock widget's visibility (`Display::associatedPanelVisibilityChange` calls
`setEnabled`), and on this machine the dock is created while the main window
is still hidden, so the display disables itself before the config's value can
take effect. Consequence: which `Image` display a config enables cannot be
honoured at startup here — it takes one click in the Displays tree after
launch, which then shows the dock. No regression either way: the previous
default (`DepthCamera`) came up disabled too.

**Fix**: the three configs are collapsed into a single `bringup/rviz/demo.rviz`
(git-renamed from `demo_slam.rviz`, the superset), used unconditionally by
both `demo.launch.py` and the launcher — the `PythonExpression` picker and
`launcher_core._rviz_config`'s branch are gone. A display whose topic has no
publisher in the current mode draws nothing, which is what the per-mode
configs were working around. The two `OccupancyGrid` displays keep
`demo_tsdf.rviz`'s clearer `OcTree (free)` / `OcTree (occupied)` names, with
occupied enabled.

**Ground-truth TSDF comparison**: the belief and truth surfaces were being
shown in different representations — `TSDFSurface_GroundTruth` (points) on,
`TSDFSurface` (points) off since Phase 42, with only the marker-based
`TSDFVoxels` on for the belief side. Both surface clouds are now enabled in
contrasting flat colours (belief orange `255;140;30`, truth green
`40;220;40`); both voxel views stay present and off, one click away. The GT
stack only publishes under `slam:=slam`, so `slam:=slam mapper:=tsdf` remains
the combination that shows both.

**Verification** (live GUI, `QT_QPA_PLATFORM=xcb`, screenshot of the running
window): ✅ the `Eval HUD` panel docks along the bottom, full width, showing
its `no eval data yet (requires slam:=slam)` placeholder — the thing Phase 43
could not confirm; ✅ no ROS-time bar anywhere; ✅ `TSDFSurface` renders
orange against the OcTree, `TSDFVoxels` off, GT counterparts present;
✅ `Displays` and `Motion Safety` keep their left-dock placement; ✅ RViz
loads the config with no plugin load failures and subscribes to both
`/tsdf/surface_cloud` and `/gt/tsdf/surface_cloud`; ✅ `colcon build --paths
bringup` succeeds and installs exactly one `rviz/demo.rviz`; ✅ the config
parses as valid YAML with no `Time` entry in `Panels:`; ✅ no
`demo_slam`/`demo_tsdf` references remain in code, launch files or docs.
⚠️ The image dock still has to be enabled by hand after launch — see the
`Image` display note above; unchanged from before this phase.

The blob was rebuilt from the one config whose layout is known to work rather
than hand-repaired: the old `demo.rviz` blob, with its `DepthCamera` slot
renamed to `SLAM input (noised range)` and its `Time` slot to `Eval HUD`.
Docks absent from a blob are appended and shown normally (that is how
`Motion Safety` has always been placed), so dropping the stale entries costs
nothing.


## Phase 45 — /projected_map Z-band, configurable A* inflation zones, dashboard rename

**Date**: 2026-07-25
**Files**: `slam/stonefish_groundtruth_mapping/stonefish_groundtruth_mapping/z_band.py`
(new), `slam/stonefish_groundtruth_mapping/launch/mapper_only.launch.py`,
`slam/stonefish_groundtruth_mapping/launch/octomap.launch.py`,
`bringup/launch/demo.launch.py`, `planner/frontier_slam/frontier_slam/path_planner.py`,
`planner/frontier_slam/frontier_slam/frontier_extractor.py`,
`planner/frontier_slam/frontier_slam/visualizer.py`,
`planner/frontier_slam/launch/frontier_slam.launch.py`,
`planner/frontier_slam/README.md`, `launcher_model.py`, `launcher_core.py`

`/projected_map` was a flat "Z-sheet": `octomap_server`'s stock
`occupancy_min_z`/`occupancy_max_z` were never set, so the whole water
column collapsed into one 2D cell per XY position (`FutureWork.md` item 5's
documented limitation). A single-Z assumption at the robot's exact depth is
unsafe anyway — depth-hold isn't precise and the hull has vertical extent —
so `z_band.py` centres the projection on a band of `2 x ROBOT_HEIGHT_M`
(0.25 m, BlueROV2 Heavy datasheet height) around the commanded cruise
depth, applied to every `octomap_server` instance that feeds
`/projected_map` (`mapper_only.launch.py`'s primary instance and
`demo.launch.py`'s dual-map planning-only instance under
`mode:=frontier mapper:=tsdf`). When `depth` is auto-locked (`-1`, the
default — unknown at launch time), the band is skipped and the map stays
full-column, same as before.

`path_planner.py`'s three-zone A* inflation radii (`HARD_INFLATION_M`,
`INFLATION_M`, `PLAN_INFLATION_M`) were Python constants with no ROS
parameter, launch argument, or TUI control anywhere — `build_cost_grid()`
now takes `hard_m`/`soft_m`/`plan_m` overrides, `frontier_extractor.py`
exposes them as `hard_inflation_m`/`inflation_m`/`plan_inflation_m` ROS
parameters (module constants remain the defaults), and both new depth/zone
parameters are threaded through `frontier_slam.launch.py`,
`demo.launch.py`, and `launcher.py` (as `depth_m` + the three zone params,
advanced/frontier section).

`/frontier_slam/debug_image` is renamed to `/frontier_slam/planning_dashboard`
(`publish_debug_image` -> `publish_planning_dashboard`) — it was never a raw
debug passthrough, always the composite map+zones+path+robot-state overhead
view, so the old name undersold what it shows.

Verified in the ROS Jazzy container: `colcon build` clean on `bringup`,
`stonefish_groundtruth_mapping`, `frontier_slam` and their dependents;
`ros2 launch <pkg> <file> --show-args` confirms the new arguments resolve
on `mapper_only.launch.py`, `octomap.launch.py`, `frontier_slam.launch.py`
and `demo.launch.py`; `octomap_z_band_params(1.0)` /
`octomap_z_band_params(-1.0)` return the expected band / empty dict;
`colcon test --packages-select frontier_slam` passes all 109 existing
tests unchanged. Not yet live-run in simulation — the Z-band's effect on
frontier/A* behaviour with a real moving robot is still unconfirmed.

**Merged after Phase 42/44 (this entry was written against a pre-Phase-28
base; renumbered from 27 on merge).** Two adjustments were needed where this
work overlapped what landed on `dev` in the meantime:

- Phase 42 removed `demo.launch.py`'s dual-map planning-only `octomap_server`
  instance — under `mode:=frontier mapper:=tsdf` the TSDF mapper now derives
  `/projected_map` from its own grid, banded around `target_depth_m`. The
  Z-band wiring for that now-deleted node is dropped; the band still applies
  to `mapper_only.launch.py`'s primary `octomap_server`, i.e. to
  `mapper:=octomap`, which nothing on `dev` covers.
- Both branches independently added a cruise-depth knob: `depth`/`depth_m`
  here, `depth`/`robot_depth_target` on `dev`. Unified onto `dev`'s (launch
  arg `depth`, default 8.0, launcher param `robot_depth_target`), which now
  feeds both bands — `octomap_server`'s through `z_band.py` and the TSDF
  mapper's through `target_depth_m`. The duplicate `depth_m` launcher param
  and the second `depth:=` the auto-merge left in the planner group are gone.

## Phase 46 — `goto` as a mode, live telemetry on the side panel, one-row safety gate

**Date**: 2026-07-25
**Files**: `launcher.py`, `launcher_core.py`, `launcher_model.py`,
`planner/frontier_slam/frontier_slam/waypoint_controller.py`,
`planner/frontier_slam/frontier_slam/wall_oriented_controller.py`,
`planner/frontier_slam/frontier_slam/wall_looking.py`,
`eval/eval_tools/eval_tools/map_metrics.py`,
`tools/motion_safety_rviz/src/motion_safety_panel.cpp`, `docs/RUN.md`,
`bootstrap.sh`, `STATE.md`

Phase 41's follow-the-point was a `y` toggle layered on top of whatever mode
was already selected — a second, hidden piece of mode state that the Mode
option did not describe and the planner group's visibility did not account
for. It becomes `goto`, a third value of `mode` alongside `teleop` and
`frontier`, so who picks the goal is one decision read from one place. The
drive keys stay always-live in every mode; what they move is what changes.

`goto` is not a new controller. The launcher publishes the operator's point
on `/frontier_slam/goal` with frontier goal picking suspended, so A* and the
path executor reach it exactly as they would reach a frontier — the mode is
a goal *source*, which is why the planner layer and every wall-following
parameter are now shared by both planner modes (`_frontier` becomes
`_planner`, true for frontier or goto; the `frontier` section title drops to
`Planner`). It runs without `revisit` so nothing preempts the operator's
goal. `demo.launch.py` has no `goto` argument and does not need one: the mode
is the launcher publishing a goal, not a different launch graph.

The side panel showed configuration but not consequence — the run's own
numbers were only on disk. It now reads three topics, none of which required
new computation, only publishing values the nodes already had in hand for
their CSV logs: `/frontier_slam/activity` (the `event` label, from all three
executors) and `/eval/map_coverage` + `/eval/map_accuracy` (from
`map_metrics.py`, already computed per tick). Map accuracy has no
backend-independent definition — IoU under octomap, belief->GT RMSE under
TSDF — so the panel labels it from the selected mapper instead of showing a
bare number that means different things in different runs.

The motion safety gate panel was a tall vertical stack whose height pushed
the eval metrics panel docked in Phase 44 out of the bottom strip. Status and
both buttons now sit on one 28 px row; the ROS-gate-not-ArduSub warning and
the topic names move to tooltips, and the warning also stays in the enable
confirmation dialog where it is read at the moment it matters.

Also: `POINT_KEYS` gains an AZERTY variant (it was one QWERTY string, so
AZERTY users saw wrong key hints while moving the point), and `bootstrap.sh`
stops printing the two `source` lines it told the user to run — `launcher.py`
resolves the virtualenv and workspace itself.

Verified: `launcher_model.py` and `launcher_core.py` import clean, every
`Group.depends` key resolves to a real `Param`, and the planner group's
visibility evaluates to true under `goto`/`frontier` and false under
`teleop`. Not yet live-run in simulation — the side panel's telemetry has
not been observed against a moving robot, so the activity labels and the
coverage/accuracy feeds are unconfirmed end-to-end.

---

## Phase 47 — One octree topic per backend, and an RViz view that reads at a glance

**Date**: 2026-07-25
**Files**: `bringup/rviz/demo.rviz`,
`slam/stonefish_groundtruth_mapping/launch/mapper_only.launch.py`,
`slam/stonefish_groundtruth_mapping/launch/tsdf.launch.py`,
`slam/tsdf_octomap/src/tsdf_to_octomap.cpp`, `bringup/launch/demo.launch.py`,
`eval/eval_tools/eval_tools/benchmark.py`, `launcher_model.py`, `docs/RUN.md`,
`planner/frontier_slam/FutureWork.md`, `STATE.md`

Phase 42 gave the TSDF backend an octree by publishing it on `/octomap_binary`
— the topic `octomap_server` owns and the two RViz `OcTree` displays are bound
to. Under `mapper:=tsdf` that put octree voxels on top of `/tsdf/voxels`: two
renderings of the same cells, from two different reconstructions, fighting for
the same pixels. The octree is a planning interface (3-D frontier detection,
3-D A*), not a view, and nothing subscribes to it yet, so it moves to
`/tsdf/octomap_binary` via a node remap. `mapper:=octomap` is untouched, the
`OcTree` displays simply draw nothing under TSDF, and `eval_tools/map_saver`
already blanked its octomap service under TSDF so no consumer changes.

Ground truth is now always shown in a representation that does not occlude the
belief map it is being compared against: `Octopoints_GroundTruth` (green
points, `/gt/octomap_point_cloud_centers`) against the belief `Octomap`
voxels, alongside the existing `TSDFSurface_GroundTruth` against `TSDFVoxels`.
A solid GT voxel map hid exactly the divergence it was there to show.

`Sonar DepthMap` is enabled and its dock marked visible in the saved
`QMainWindow State` — the display was on a topic published in every mode but
was saved disabled, so it "disappeared" depending on which session last wrote
the config. `ProjectedMapSlice` (`/projected_map`) is added, off by default:
the same 2-D band rqt shows, available without a second tool.

The live drift marker becomes a `LINE_LIST` labelled `error x.xx m` at its
midpoint. As an `ARROW` the head and shaft are scaled independently of the
length, so at the sub-metre drift that matters it rendered as a blob rather
than a distance, and the number was only readable in the HUD panel.

**Verified**: `ros2 launch stonefish_groundtruth_mapping mapper_only.launch.py
mapper:=tsdf tsdf_octomap:=true` lists `/tsdf/octomap_binary` and no
`/octomap_binary`, with `tsdf_to_octomap` logging the resolved topic name.
`demo.rviz` parses as YAML with every display's topic as listed above; the
four touched packages build clean. Not yet live-run: the drift line label and
the ground-truth point overlay have not been watched against a moving robot
under `slam:=slam`.

## Phase 48 — D-optimality on the drifting DoF, an allowable-covariance trigger, and consistency metrics

The revisit trigger was reading a signal that could not mean what it was being
asked to mean. `/slam/dopt` was `det(Σ)^(1/3)` over the translation block
`[x, y, z]`, but Suresh et al. (2020) eq. 4 scores **XYH** — x, y and heading —
because depth, pitch and roll are directly observed by the pressure sensor and
IMU and do not accumulate drift. Folding the centimetre-scale depth variance
into a geometric mean deflates the result, which is the low-variance-term
failure the criterion is documented to have (Placed et al. 2023, §V-C).
`pose_graph.dopt_xyh()` now scores rows/cols `[3, 4, 2]` of the GTSAM
`[rot|trans]` marginal.

The threshold was a bare determinant, `dopt_trigger: 0.02`, with no
interpretable scale — and sat roughly 6x above anything the estimator ever
reported, which is why revisit never fired in the 2026-07-24 ablation. It is
replaced by eq. 5's ratio, `U_r = D(Σ)/D(Σ_allow)`, with `Σ_allow` stated as
per-axis sigmas (`sigma_allow_xy_m`, `sigma_allow_yaw_rad`) and the trigger at
`U_r > 1` — "revisit exactly when the estimate is less certain than the mission
allows". Equal sigmas make `D(Σ_allow)` exactly `σ²`, which is what the unit
tests use to state thresholds directly. Published on
`/frontier_slam/uncertainty_ratio`.

Neither of those says whether the covariance is *right*, so the run now
measures it. `eval_tools/consistency.py` scores NEES over the same XYH DoF the
trigger consumes, and `benchmark.py` accumulates ANEES with a chi-square
acceptance region and logs a verdict at shutdown. NEES needs ground truth, so
it can never be an online trigger input; the GT-free counterparts are
`/slam/nis` (per-closure normalised innovation against the scan-matching noise
model, chi-square 6 DoF) and `/slam/chi2_normalized` (whole-graph chi-square
per DoF, ~1.0 when the assumed sigmas match the residuals). All four land in
`metrics.csv` alongside the existing columns. `session_log` gained
per-column precision, because its 2-decimal default had been rounding
every logged D-optimality to `0.00` — the column was empty in every
revisit log written to date, including the 2026-07-24 ablation's.

NIS omits the estimate's own covariance from its denominator, so a single high
reading is not evidence against the noise model — a correct closure after real
drift carries that drift in its innovation. Only the distribution over a run is
diagnostic. A 300 s realistic run gives median NIS 1.4 against an expectation of
6 and a normalised graph chi-square of 0.17 against 1.0: the *relative* sigmas
are looser than the residuals need. ANEES over the same run is 17 against an
expectation of 3, i.e. the *absolute* marginal is far too tight. Internally
consistent and globally wrong at once is the signature of unmodelled bias, not
of noise.

Launcher: the option list gained `Space` as a Right-arrow synonym, so ascend
moved off Space to the key left of X (`Z` on QWERTY, `W` on AZERTY, the same
physical key). The value list is pinned to the bottom of the side panel under a
rule instead of sliding with the content above it, and is capped at half the
panel so a long enum cannot push the pose table off. The pose table gained a
fourth column, the per-axis GT-minus-belief gap, grey below 0.25 m and
coloured past 0.25 / 1.0 m. The `ros2 launch` preview row is a
`copy launch command` button (click, or `C`) rather than a line far too long to
read, copying via `wl-copy`/`xclip`/`xsel` with an OSC 52 fallback that works
over SSH.

**Verified**: NEES validated by Monte Carlo against a known covariance —
2.98 for a calibrated estimator (95% band [2.93, 3.08]), 26.6 when sigma is 3x
too small, 0.34 when 3x too large. NIS validated against chi-square 6 DoF —
exactly 1.0 at a one-sigma offset, 9.0 at three sigma, mean 5.98 over 20k
model-matched draws, 24.0 when sigma is assumed 2x too tight. 25 revisit-planner
unit tests pass; the 3 pre-existing `test_tsdf_tf_queue` failures are unrelated
(confirmed against a clean tree). Side-panel layout rendered headlessly at 24
and 45 rows, including the empty-selection and short-panel cases. A 300 s
`slam:=slam mode:=frontier revisit:=true` run publishes all four signals into
`metrics.csv`; the `sigma_allow` defaults were then set from that run's measured
XYH D-opt range (0.0002-0.0022, median 0.0010) rather than guessed, and a 240 s
confirmation run fired 2 revisits with `U_r` median 1.02 / max 1.52 — the trigger
that never fired in the 2026-07-24 ablation now engages. That run also reports
ANEES 19.7 against an expectation of 3, so the overconfidence is now a measured
number rather than an inference.

## Phase 49 — The yaw prior was labelled with the wrong sensor's sigma

Phase 48 ended with a measured ANEES of 19.7 against an expectation of 3 — the
marginal covariance was ~6.6x too tight and the cause was open. It is the
attitude+depth prior's yaw entry.

The prior asserts an absolute world-frame pose at every keyframe, and its yaw
sigma was `profile.imu.sigma_yaw_rad`. The value it asserts, though, is not the
IMU's yaw — it is `T_odom`'s, the *fused* output of `YawKalmanFilter`. Those are
not the same quantity. Replaying the exact sim sensor chain against a known
ground truth measures the gap:

```
                          true RMS err   claimed sigma    ratio
raw IMU message                0.00999         0.01000     1.00
fused filter output            0.02922         0.03931     0.74
pose graph prior               0.02922         0.01000     2.92
```

The IMU message really is absolute to its stated 0.01 rad — `imu_sim.py` builds
it from ground-truth attitude plus white noise. But `predict_imu` treats each
reading as an *increment* (`variance += 2σ²`), so the filter random-walks away
from a 0.01 rad input and is pulled back only by the 0.05 rad compass, landing
at a true error of 0.0292 rad. The graph then labelled that degraded value with
the undegraded sensor's spec: 2.9x too tight. Heading error is what converts
into cross-track position error over a path, so an over-tight yaw marginal is
exactly what lets D-optimality stay flat while ATE climbs.

The filter already computed the right number and nothing consumed it.
`dead_reckoning.py` now publishes `YawKalmanFilter.variance` in the fused
odometry's `pose.covariance[35]`, and `pose_graph.prior_sigmas_with_yaw()`
substitutes it into the prior per keyframe. Roll/pitch/depth keep their profile
sigmas — those are read straight off their sensors, so the spec is the right
figure. Missing covariance falls back to the profile default, so the ideal
profile and replayed bags are unchanged.

Same scene, same seed, same 240 s as the Phase 48 confirmation run:

| | before | after | target |
|---|---|---|---|
| ANEES (final) | 19.69 | 2.37 | 3.0 |
| NEES (median) | 11.00 | 1.45 | 3.0 |
| ATE (final) | 0.248 m | 0.103 m | — |
| abs_error (max) | 0.693 m | 0.177 m | — |
| D-opt XYH (median) | 0.0021 | 0.0048 | — |

The estimator is consistent now, marginally on the conservative side. ATE fell
58% as well: the over-tight prior was not only mislabelling the uncertainty, it
was over-constraining the solve and fighting the loop closures. D-opt roughly
doubled, which is the honest value — any `sigma_allow_*` tuned before this
change is now calibrated against a signal half the size and must be re-measured.

**Left open.** Every noise profile labels its IMU block "gyro-integrated
attitude" and Phase 34 recorded that the profiles "reinterpret
`imu.sigma_yaw_rad` as short-term gyro noise (with heading error now carried by
the compass)". `imu_sim.py` never followed: it still adds noise to ground-truth
yaw, so the topic is an absolute heading, which is what `STATE.md` documents it
as. The simulator and the stated design disagree, and no real IMU knows absolute
heading to 0.01 rad underwater. Resolving it means changing what the simulated
sensor is, which moves every recorded run number, so it is left as a decision
rather than folded into this change.

## Phase 50 — A depth limit cycle, and a planner that could not see what it hit

Two vehicle-behaviour faults, found by watching a run rather than reading a
metric.

**The vehicle was bouncing.** Depth hold was `heave = -KP * (pose_z - setpoint)`
with no rate term. The vehicle is effectively a double integrator and the loop
runs at 10 Hz, so once Phase 39's thruster-calibration fix restored the real
actuator authority there was no phase margin left. Ground truth over a 900 s
run:

```
z: mean 6.023 m (setpoint 6.0)  std 0.371  peak-to-peak 1.265 m
237 zero crossings / 730 s -> sustained 6.14 s period (FFT peak 0.163 Hz)
```

A clean limit cycle, not noise. `control_utils` gains `depth_hold_effort()` and
`LowPassRate`, a filtered finite difference; `wall_oriented_controller`,
`waypoint_controller` and `wall_looking` all feed a damped depth rate into the
law. Same scene afterwards: std **0.371 -> 0.038 m**, peak-to-peak **1.265 ->
0.230 m**, and no periodic component left at all.

**The vehicle was colliding.** The controller's own logs had been recording it:
233 `EMERG_STOP` events in a 10 min run with clearance pinned at the depth
sensor's 0.2 m floor. Two independent causes. The A* hard-wall radius was
0.20 m, so a planned path was permitted to graze structure. More seriously,
`/projected_map` projected only a +/-1.0 m Z-band around the cruise depth into
the 2-D planning grid, so wreck geometry above and below the cruise plane was
invisible to the planner — it was not avoiding those obstacles because it never
knew they existed.

```
hard_inflation_m      0.20 -> 1.00
inflation_m (soft)    0.75 -> 1.50
plan_inflation_m      1.50 -> 3.00
projected_map_band_m  1.0  -> 3.0     (the collision-relevant one)
wall_z_band_m         1.5  -> 3.0
```

Measured over the first ~550 s of a run under each setting:

| | EMERG_STOP | ticks < 0.4 m | median clearance | travel |
|---|---|---|---|---|
| hard 0.20, band +/-1.0 | 233 | 230 | 0.48 m | 136 m |
| hard 1.00, band +/-3.0 | 17 | 16 | 2.32 m | 322 m |

Travel distance is in that table deliberately: a robot that stops moving also
stops colliding, and the wider margins had to be shown not to strand it. They
do not — it covers more ground than before, because it is no longer spending
its time in emergency stop.

One coupling this introduces: a 1.0 m hard radius means a frontier goal placed
1.0 m off a surface lands exactly on the blocked boundary and A* can never
reach it. `tsdf_frontier_standoff_m` must now be >= 2.0; batches that pin it
lower will stall.
