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
