# OctoMap Misalignment — Research Progress

## Problem statement

When the robot moves (especially fast yaw), the OctoMap builds walls where there
are only pillars. Static: map is perfect. Moving: voxels accumulate at wrong
world positions and are never cleared.

Key diagnostic: `/cloud_in` is **perfectly aligned in `bluerov2/base_link`** but
**misaligned in `world_ned`** during movement.

---

## Root cause — TWO contributing factors

### 1. GPU render latency (Δrender) — CONFIRMED

Stonefish timestamps depth images with `nh_->get_clock()->now()` **at publish
time**, which is AFTER the async PBO GPU readback completes (one render frame
delay). Odometry is also stamped with `get_clock()->now()` but at near-zero
latency (CPU computation only).

**Stonefish source evidence** (`stonefish_ros2/src/ROS2SimulationManager.cpp`):
```
line 851: img->header.stamp = nh_->get_clock()->now();  // after PBO readback
line 840: info->header.stamp = img->header.stamp;
ROS2Interface.cpp line 273: msg.header.stamp = nh_->get_clock()->now();  // odom
```

**Measured delta (`timestamp_debug`):**

| State  | Frames | Mean  | Std   | Min   | Max    |
|--------|--------|-------|-------|-------|--------|
| static | 39     | 5.67 ms | 2.78 ms | 0.11 ms | 10.25 ms |
| moving | 321    | 5.24 ms | 3.07 ms | 0.05 ms | 12.57 ms |

**Correct interpretation:** The ~5 ms mean IS Δrender (GPU readback delay), not
sampling jitter. It is consistent between static and moving because GPU render
time does not depend on robot velocity.

**Failure mode in depth_fix:** `_best_odom_stamp()` picks the most recent odom
stamp ≤ T_img. For ~85% of frames (Δrender < 10 ms) it correctly selects T_N.
For ~15% of outlier frames (Δrender > 10 ms) it accidentally selects T_N+10ms
(the NEXT odometry step), associating the image with a future robot pose.

**Impact at long range / fast yaw:**
- Error = Δrender_outlier × angular_velocity × distance
- Example: 12 ms × 1 rad/s × 10 m = **12 cm ≈ 1 voxel** (OctoMap res 10 cm)
- At short range this is sub-voxel. At 10 m+ with fast rotation, it places
  voxels in the wrong cell → visible wall thickening around pillars.

### 2. OctoMap ray-cast back-face limitation

When a ray hits a surface, it marks only the endpoint as OCCUPIED and everything
before it as FREE. Voxels on the far side are never cleared.

When the robot rotates and views the same pillar from a new angle, old voxels
from the previous angle are off-axis — no new ray passes through them. Both sets
stay OCCUPIED → cluster wider than the actual pillar → wall.

This is **inherent to OctoMap's binary ray-cast model**. It worsens with fast
yaw because more angles accumulate per second.

---

## Timestamp hypothesis — REVISED

**Original (wrong) conclusion:** "Offset is identical static vs moving →
GPU latency disproven."

**Correct conclusion:** The offset *is* Δrender. Being identical static vs moving
is expected — GPU render time is independent of robot velocity. The script
measured the right quantity but the conclusion was wrong.

---

## Paths explored / ruled out

| Approach | Outcome |
|---|---|
| `odom_tf_sync` — interpolate TF to exact image timestamp | Addresses symptom correctly but root cause is wrong stamp, not TF interpolation |
| GPU render latency hypothesis | **CONFIRMED** by Stonefish source: `get_clock()->now()` at publish, after PBO readback |
| `depth_fix` NaN replacement | Fixed the ghost sphere at camera origin ✓ |
| `depth_fix` timestamp correction | Correct approach; works for ~85% of frames; fails for ~15% outlier frames (picks wrong odom step) |
| Reduced yaw speed (`step/4`) | Reduces symptom (fewer viewing angles per second) but does not cure |
| OctoMap probabilistic decay | Rejected: masks the problem instead of solving it |

---

## Remaining ideas to explore

1. **Fix depth_fix outlier handling** (highest priority)
   Current `_best_odom_stamp` picks nearest odom ≤ T_img. For outlier frames
   (Δrender > 10 ms), this picks T_N+10ms instead of T_N. Options:
   - Subtract a fixed margin (~7 ms) from T_img before lookup so the search
     window centres on the correct odom step
   - Use closest-to-expected-Δrender (pick odom stamp ~5 ms before T_img)
   - Increase odometry publish rate (100 Hz → 500+ Hz) so worst-case error < 2 ms

2. **Switch to TSDF-based mapper (voxblox / nvblox)**
   TSDF stores a signed distance field instead of binary occupancy. New
   observations actively correct previous ones. Solves both the OctoMap
   back-face problem and tolerates small timestamp errors.

3. **OctoMap hit/miss probability tuning**
   Lower `sensor_model/hit`, raise `sensor_model/miss` so occupied voxels fade
   faster under contradicting observations. Weaker than TSDF but cheap to try.

4. **Increase OctoMap resolution**
   Smaller voxels (0.05 m) reduce smear area but do not eliminate it. High
   memory cost.

5. **Keyframe selection / rate limiting**
   Only integrate frames when the robot has moved significantly. Reduces
   redundant observations from nearby poses but does not fix the fundamental
   issue.

---

## Key invariant (updated)

> The point cloud is correctly placed in `bluerov2/base_link` every frame.
> The world-frame misalignment during movement has two causes:
> (1) ~15% of frames receive a wrong TF lookup because depth_fix picks the next
>     odometry step instead of the current one on outlier GPU readback frames;
> (2) OctoMap cannot clear voxels that are off the ray path of subsequent
>     observations.
> Improving depth_fix to correctly handle outlier frames will reduce (1).
> Solving (2) requires TSDF or accepting the back-face smear.
