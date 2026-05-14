# OctoMap Misalignment — Research Progress

## Problem statement

When the robot moves (especially fast yaw), the OctoMap builds walls where there
are only pillars. Static: map is perfect. Moving: voxels accumulate at wrong
world positions and are never cleared.

Key diagnostic: `/cloud_in` is **perfectly aligned in `bluerov2/base_link`** but
**misaligned in `world_ned`** during movement.

---

## Root cause — CONFIRMED

**OctoMap ray-cast accumulation from multiple viewing angles.**

When a ray hits a surface, it marks only the endpoint as OCCUPIED and everything
before it as FREE. It does **not** cast through the surface. Voxels on the far
side of an object are therefore never cleared by rays from this angle.

When the robot rotates and views the same pillar from a different angle:
- New rays mark the near surface OCCUPIED from the new angle
- Old voxels from the previous angle are now off-axis — no new ray passes through
  them to clear them
- Both sets of voxels stay OCCUPIED

Repeating from many angles builds up a cluster wider than the actual pillar → wall.

This is **inherent to OctoMap's binary ray-cast model** and is independent of
timestamp accuracy.

---

## Timestamp hypothesis — DISPROVEN

**Hypothesis:** Stonefish stamps depth images after GPU readback, causing a large
offset (Δ_render) between image content time and image timestamp. When moving,
this offset would cause the TF lookup to use the wrong robot pose.

**Measured data (`timestamp_debug`):**

| State  | Frames | Mean  | Std   | Min   | Max    |
|--------|--------|-------|-------|-------|--------|
| static | 39     | 5.67 ms | 2.78 ms | 0.11 ms | 10.25 ms |
| moving | 321    | 5.24 ms | 3.07 ms | 0.05 ms | 12.57 ms |

**Conclusion:** The offset is ~5 ms regardless of movement. This is normal
sampling jitter from where in the 10 ms odometry period the depth frame fires.
At 5 ms and 1 m/s speed, the resulting position error is 5 mm — far below
OctoMap's 10 cm resolution. **Timestamps are not the problem.**

The timestamp correction in `depth_fix.py` (replacing image stamp with nearest
odometry stamp) is still correct and eliminates even this small jitter, but it
does not affect the wall problem.

---

## Paths explored / ruled out

| Approach | Outcome |
|---|---|
| `odom_tf_sync` — interpolate TF to exact image timestamp | Not the root cause; timing error is negligible (~5 ms) |
| GPU render latency hypothesis | **Disproven** by measurement: offset is identical static vs moving |
| `depth_fix` NaN replacement | Fixed the ghost sphere at camera origin ✓ |
| `depth_fix` timestamp correction | Correct implementation, eliminates ~5 ms jitter, but not the wall cause |
| Reduced yaw speed (`step/4`) | Reduces symptom (fewer viewing angles per second) but does not cure |
| OctoMap probabilistic decay | Rejected: masks the problem instead of solving it |

---

## Remaining ideas to explore

1. **Switch to TSDF-based mapper (voxblox / nvblox)**
   TSDF (Truncated Signed Distance Function) stores a signed distance field
   instead of binary occupancy. New observations can actively correct previous
   ones — a voxel marked occupied from the front gets un-marked when new evidence
   from the back contradicts it. This is the architecturally correct solution.

2. **OctoMap hit/miss probability tuning**
   Lower the `sensor_model/hit` probability and raise `sensor_model/miss` so
   occupied voxels require more consistent evidence and fade faster under
   contradicting observations. Weaker than TSDF but worth measuring.

3. **Increase OctoMap resolution**
   Smaller voxels (e.g., 0.05 m) mean each viewing angle marks fewer incorrect
   voxels. Reduces smear area but does not eliminate it. High memory cost.

4. **Limit OctoMap integration rate / keyframe selection**
   Only integrate frames when the robot has moved significantly (distance or
   angle threshold). Reduces redundant observations from nearby poses. Does not
   fix the fundamental issue but reduces noise.

---

## Key invariant confirmed

> The timestamps are accurate. The point cloud is correctly placed in the world
> for each individual frame. The wall problem is caused by OctoMap's inability
> to clear occupied voxels that are off the ray path of subsequent observations.
> No timing fix can solve this.

