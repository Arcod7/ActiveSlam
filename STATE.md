# ActiveSlam SLAM Backend — Current State

Mutable snapshot. Overwrite, never append. Last updated: 2026-07-04.

Change log → `Progress.md`. Detailed design + as-built deltas → `docs/SLAM_PLAN.md`.

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
                          ATE / RPE / TUM export
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
- ✅ Pose graph + loop closure: synthetic square-loop test, 70% ATE reduction after loop closure
- ✅ Full live run (`demo.launch.py slam:=slam mode:=frontier`, real Stonefish sim): all nodes start, loop closures fire on real depth-camera clouds, frontier exploration drives from SLAM-corrected odometry, zero exceptions over 45s
- ✅ Regression check: `slam:=none` (default) unchanged, no SLAM nodes started, `odom_tf_sync` still the TF source
- 🔲 Not yet run: long-duration (10+ min) session; `evo_ape`/`evo_rpe` cross-check against the written TUM files; noise-profile comparison at statistical significance (multiple seeded runs)

## Not yet implemented (Week 3 prep only — see docs/SLAM_PLAN.md Part 11)

- Mirror `NonlinearFactorGraph`/`Values` for virtual-factor uncertainty propagation along candidate revisit paths — not built; `pose_graph.py` only maintains the live iSAM2 graph
- Explore-vs-revisit decision rule (D-optimality threshold trigger) — `/slam/dopt` is published and logged, nothing consumes it yet
- Submap saliency (FPFH descriptors) — keyframe clouds are stored in body frame, ready to feed a descriptor pipeline, none built
- `/slam/rebuild_map` service for TSDF/OctoMap re-integration after a large loop closure — map inconsistency after closure is accepted as a known limitation, not mitigated
