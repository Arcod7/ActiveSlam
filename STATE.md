# ActiveSlam SLAM Backend — Current State

Mutable snapshot. Overwrite, never append. Last updated: 2026-07-05.

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
/cloud_in → cloud_relabel → /gt/cloud_in                  (same sensor data,
                                                             frame_id swapped)
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
    /cloud_in (5Hz) ────────────────┤
                                    ▼
                              pose_graph (GTSAM iSAM2)
                                    │
                    ┌───────────────┼────────────────┐
                    ▼               ▼                ▼
              TF broadcast    /slam/pose       /slam/path_slam,
        (world_ned →         (+ covariance)     graph_edges, dopt, ...
         base_link)               │
                                  ▼
                            benchmark (eval_tools)
                     ATE / RPE / TUM export / /eval/markers
```

## Packages

- `slam/slam_backend/` — sensor sims, dead-reckoning fusion, pose graph, scan matcher
- `eval/eval_tools/` — benchmark node, TUM writer, offline plotting

## Parameters

### Noise Profiles (`slam/slam_backend/config/`)
- `noise_ideal.yaml`: near-perfect sensors (sanity checks)
- `noise_realistic.yaml`: matches Bar30 pressure + Pathfinder DVL + typical MEMS IMU
- `noise_degraded.yaml`: turbid water / magnetic interference / degraded bottom-lock

### `pose_graph.py` key parameters (defaults)
| Parameter | Value | Purpose |
|---|---|---|
| `keyframe_dist_m` | 1.0 | Min travel to trigger a new keyframe |
| `keyframe_angle_rad` | 0.3 | Min rotation (~17°) to trigger a new keyframe |
| `loop_closure_radius_m` | 5.0 | Proximity search radius for loop closure |
| `loop_closure_min_gap` | 10 | Min keyframe-index gap for a valid closure |
| `min_inlier_ratio` | 0.3 | Registration gate: fraction of source points matched |
| `min_inlier_count` | 50 | Registration gate: absolute inlier floor |
| `max_error_per_inlier` | 0.05 | Registration gate: GICP error normalized per inlier |
| `icp_sigma_rot` / `icp_sigma_trans` | 0.05 / 0.05 | Shared noise model for ALL Between factors |
| `scan_voxel_size` | 0.1 | small_gicp downsampling resolution |

### Launch usage
```bash
ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic mode:=frontier
ros2 launch bringup demo.launch.py                                       # unchanged default (slam:=none, ground truth)
ros2 launch slam_backend sensors_only.launch.py noise_profile:=degraded  # sensor layer only
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
  the robot revisit anything. One run, one unseeded noise draw (`realistic` uses `seed: -1`).
- ✅ Regression check: `slam:=none` (default) unchanged, no SLAM nodes started, `odom_tf_sync` still the TF source
- ✅ Ground-truth reference map (`gt_map.launch.py`): live-verified both `mapper:=octomap`
  (`/gt/octomap_binary` ~24–28 Hz, `/gt/cloud_in` ~25 Hz) and `mapper:=tsdf`
  (`tsdf_mapper_gt` voxel/surface counts growing steadily, unaffected by a
  belief-side `imu_sim` crash that has since been fixed — see below). Not yet
  visually confirmed in RViz (headless verification only, no GUI here).
- ✅ **Fixed**: `pose_graph.py`'s re-detect-loop-closures-after-move step was dead
  code — every keyframe's pose was overwritten with the post-optimization estimate
  *before* the moved-keyframe check ran, so it always compared a value against
  itself and never found anything to re-check. Fixed by letting
  `_find_moved_keyframes` do the refresh itself (it already did) instead of a
  separate premature overwrite. Verified two ways: a synthetic call with a
  keyframe shifted 0.5 m confirms the method detects and refreshes it; a live
  100s/60s run shows the log line `Re-detecting loop closures for moved
  keyframes [...]` actually firing after a real loop closure. The 0.58 m ATE
  figure above predates this fix and may improve on a rerun.
- ✅ **Fixed**: `imu_sim.py` crashed (`ValueError: scale < 0`) when a non-monotonic
  odom timestamp made `dt` negative, feeding straight into a noise draw's scale
  parameter — only possible with `gyro_bias_drift_rad_s > 0` (realistic/degraded
  profiles), matching the crash as observed. `pressure_sim.py`/`dvl_sim.py` had the
  same root cause but stalled silently instead of crashing. All three now guard
  against non-positive `dt`. Verified with a synthetic reproduction of the exact
  crash input and a 60s live run on `noise_profile:=degraded`.
- 🔲 Not yet run: long-duration (10+ min) session; `evo_ape`/`evo_rpe` cross-check against the written TUM files; noise-profile comparison at statistical significance (multiple seeded runs per profile)
- ⚠️ The synthetic square-loop test's "70% ATE reduction" (Progress.md Phase 6) is superseded
  by the real benchmark above — its dense, easily-overlapping synthetic point clouds make loop
  closure fire far more readily than real depth-camera data does. Use the 0.58 m figure, not 70%.

## Not yet implemented (Week 3 prep only — see docs/SLAM_PLAN.md Part 11)

- Mirror `NonlinearFactorGraph`/`Values` for virtual-factor uncertainty propagation along candidate revisit paths — not built; `pose_graph.py` only maintains the live iSAM2 graph
- Explore-vs-revisit decision rule (D-optimality threshold trigger) — `/slam/dopt` is published and logged, nothing consumes it yet
- Submap saliency (FPFH descriptors) — keyframe clouds are stored in body frame, ready to feed a descriptor pipeline, none built
- `/slam/rebuild_map` service for TSDF/OctoMap re-integration after a large loop closure — map inconsistency after closure is accepted as a known limitation, not mitigated
