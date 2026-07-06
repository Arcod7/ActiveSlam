# ActiveSlam SLAM Backend — Current State

Mutable snapshot. Overwrite, never append. Last updated: 2026-07-06.

Change log → `Progress.md`. Detailed design + as-built deltas → `docs/SLAM_PLAN.md`.

## RViz views

`bringup/demo.launch.py` picks one of three configs automatically (`slam:=slam`
takes priority over `mapper`):
- `rviz/demo.rviz` — unchanged base view (`mapper:=octomap slam:=none`).
- `rviz/demo_tsdf.rviz` — TSDF surface/voxel displays instead of OctoMap's
  (OctoMap topics aren't published when `mapper:=tsdf`, so its displays would
  just be empty).
- `rviz/demo_slam.rviz` — the error/noise view: ground truth (green) vs SLAM
  (blue) vs raw dead-reckoning (red) paths, pose-graph edges, covariance
  ellipsoids, and a drift arrow + live text HUD sourced from
  `eval_tools/benchmark.py`'s `/eval/markers` (`MarkerArray`) topic —
  `err`/`ATE`/`RPE` translation+rotation/keyframe count/loop-closure
  count/D-optimality, refreshed on every `/slam/pose` update. Also overlays
  a second, ground-truth-only map (`GroundTruthMap` / `/gt/octomap_binary`,
  enabled by default; `TSDFSurface_GroundTruth`/`TSDFVoxels_GroundTruth` for
  `mapper:=tsdf`, disabled by default) against the belief map — see
  "Ground-truth reference map" below.

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

## Packages

- `slam/slam_backend/` — sensor sims (incl. `sonar_noise`), dead-reckoning fusion, pose graph, scan matcher
- `eval/eval_tools/` — benchmark node, map_metrics node, TUM writer, offline plotting, batch orchestrator (`scripts/run_matrix.py`)

## Parameters

### Noise Profiles (`slam/slam_backend/config/`)
- `noise_ideal.yaml`: near-perfect sensors (sanity checks); sonar section all-zero (exact passthrough)
- `noise_realistic.yaml`: matches Bar30 pressure + Pathfinder DVL + typical MEMS IMU; sonar section
  derived from the WaterLinked Sonar 3D-15 datasheet (`ActiveSlam-Resources/3d-sonar`)
- `noise_degraded.yaml`: turbid water / magnetic interference / degraded bottom-lock; sonar section
  worse-than-datasheet (full beam-separation lateral jitter, higher dropout/outlier rates)

Each profile's `seed:` (42 for `ideal`, -1/random for `realistic`/`degraded`) is combined with a
per-sensor offset (imu +1, dvl +2, pressure +3, sonar +4) before seeding, so co-launched sims no
longer draw identical RNG streams off one shared seed — this changed `ideal`'s exact per-sensor
draws vs. pre-Phase-16 runs (same seed, different effective value per node); nothing previously
published used `ideal`, so no quoted numbers are affected. Override with `noise_seed:=<int>`
(`-1` = use the profile's own seed).

### `pose_graph.py` key parameters (defaults)
| Parameter | Value | Purpose |
|---|---|---|
| `keyframe_dist_m` | 1.0 | Min travel to trigger a new keyframe |
| `keyframe_angle_rad` | 0.3 | Min rotation (~17°) to trigger a new keyframe |
| `loop_closure_enabled` | true | Master on/off switch (A/B benchmarking) |
| `loop_closure_radius_m` | 5.0 | Proximity search radius for loop closure |
| `loop_closure_min_gap` | 10 | Min keyframe-index gap for a valid closure |
| `min_inlier_ratio` | 0.3 | Registration gate: fraction of source points matched |
| `min_inlier_count` | 50 | Registration gate: absolute inlier floor |
| `max_error_per_inlier` | 0.05 | Registration gate: GICP error normalized per inlier |
| `icp_sigma_rot` / `icp_sigma_trans` | 0.05 / 0.05 | Shared noise model for ALL Between factors |
| `scan_voxel_size` | 0.1 | small_gicp downsampling resolution |
| `map_rebuild_enabled` | false | Rebuild the belief TSDF map after a big loop closure (TSDF only) |
| `rebuild_min_move_m` / `rebuild_min_move_rad` | 0.3 / 0.15 | Min keyframe shift to trigger a rebuild |
| `rebuild_min_interval_s` | 30.0 | Throttle between rebuild triggers |

### Launch usage
```bash
ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic mode:=frontier
ros2 launch bringup demo.launch.py                                       # unchanged default (slam:=none, ground truth)
ros2 launch slam_backend sensors_only.launch.py noise_profile:=degraded  # sensor layer only

# Benchmarking switches (all slam:=slam only, all default to current behavior):
ros2 launch bringup demo.launch.py slam:=slam loop_closure:=false                    # A/B: no loop closure
ros2 launch bringup demo.launch.py slam:=slam mapper:=tsdf map_rebuild:=true         # rebuild belief TSDF after big closures
ros2 launch bringup demo.launch.py slam:=slam noise_seed:=7                          # reproducible, decorrelated noise draws
ros2 launch bringup demo.launch.py slam:=slam output_dir:=/path/to/run              # label eval output instead of a timestamp

# Overnight batch evaluation (plain script, not a console_script -- needs
# `source install/setup.bash` first so eval_tools.plot_results is importable):
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_smoke.yaml       # ~5min pre-flight check
python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_overnight.yaml   # 35 runs, ~5.3h
python3 eval/eval_tools/scripts/run_matrix.py --aggregate-only eval/runs/<batch_dir>          # re-aggregate only
```

## Verification status

- ✅ Sensor fusion chain (pressure+IMU+DVL→dead_reckoning): profile-dependent drift confirmed via standalone rclpy harness
- ✅ `ScanMatcher` gating: correctly rejects a synthetic zero-overlap match despite `converged=True`
- ✅ Pose graph + loop closure logic: synthetic square-loop test (wiring/logic sanity check only, not representative of real numbers — see caveat below)
- ✅ **Real benchmark** (`demo.launch.py slam:=slam mode:=frontier noise_profile:=realistic`,
  real Stonefish sim, 157s / 116 keyframes / 12 loop closures, zero exceptions): final
  cumulative ATE **0.58 m**, peak instantaneous error 0.95 m, mean RPE (translation) 0.12 m.
  All 12 closures fired in two early clusters; zero in the final ~63s (frontier exploration
  moved past the 5 m loop-closure radius) — loop closure is opportunistic-only, it never makes
  the robot revisit anything. One run, one unseeded noise draw, predates both the re-detect fix
  (Phase 6/audit) and the Phase 15 sonar noise model — due for a rerun with both in place.
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
- ✅ **Batch orchestrator** (Phase 18): `--dry-run` on the 35-run overnight preset resolves
  correctly and rejects invalid configs (`motion:=wallfollow` without `mapper:=tsdf`); a real
  2-run/120s mini-batch produced complete per-run + batch-level artifacts including a clean
  back-to-back Stonefish restart.
- 🔲 Not yet run: the actual 35-run overnight matrix (`matrix_overnight.yaml`); long-duration
  (10+ min) single session; `evo_ape`/`evo_rpe` cross-check against the written TUM files.
- ⚠️ The synthetic square-loop test's "70% ATE reduction" (Progress.md Phase 6) is superseded
  by the real benchmark above — its dense, easily-overlapping synthetic point clouds make loop
  closure fire far more readily than real depth-camera data does. Use the 0.58 m figure, not 70%.

## Not yet implemented (Week 3 prep only — see docs/SLAM_PLAN.md Part 11)

- Mirror `NonlinearFactorGraph`/`Values` for virtual-factor uncertainty propagation along candidate revisit paths — not built; `pose_graph.py` only maintains the live iSAM2 graph
- Explore-vs-revisit decision rule (D-optimality threshold trigger) — `/slam/dopt` is published and logged, nothing consumes it yet
- Submap saliency (FPFH descriptors) — keyframe clouds are stored in body frame, ready to feed a descriptor pipeline, none built
- Map rebuild after a large loop closure — **implemented for TSDF** (Phase 16, parameterized
  `map_rebuild:=true`); still not supported for OctoMap (see Verification status above)
