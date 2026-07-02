# OctoMap Misalignment — Research Progress

## Problem statement

When the robot moves (especially fast yaw), the OctoMap builds walls where there
are only pillars. Static: map is perfect. Moving: voxels accumulate at wrong
world positions and are never cleared.

Key diagnostic: `/cloud_in` is **perfectly aligned in `bluerov2/base_link`** but
**misaligned in `world_ned`** during movement.

---

## Root cause — THREE contributing factors

### 1. GPU render latency (Δrender) — FIXED in Stonefish (fix3 + addendum)

**Original bug**: Stonefish stamped depth images with `nh_->get_clock()->now()` at
publish time (after async PBO GPU readback). Odometry is stamped at near-zero
latency. Δrender ≈ 5 ms mean, up to 12 ms peak.

**Fix3 (phase 1)**: Added `captureTime_` double-buffer in `OpenGLDepthCamera`.
`pendingCaptureTime_` was captured in `UpdateTransform()` on the **GL thread**,
frozen to `captureTime_` in `DrawLDR()`, exposed via `getCaptureTime()` /
`getLastCaptureTime()`.

**Fix3 addendum (phase 2 — this session)**: Found a remaining race: the GL thread
called `getSimulationTime(true)` in `UpdateTransform()`, which could return a
*later* physics step's time while the pose still belonged to step N. At 100 Hz
physics / ~60 Hz render, this introduced up to 10–20 ms timestamp error — enough
for 1–2 voxels at 10 m range, 0.8 rad/s.

**Final fix**: `DepthCamera::SetupCamera()` on the **physics thread** now snapshots
`getSimulationTime(true)` and writes it to `glCamera->tempCaptureTime_` via
`SetPendingCaptureTime()` — same double-buffer pattern as `tempEye/dir/up`. The GL
thread then copies `tempCaptureTime_ → pendingCaptureTime_` without querying the
clock itself. Timestamp and pose are always from the same physics step.

**Also fixed**: `stonefish_ros2/ROS2SimulationManager.cpp` was calling
`cam->getLastCaptureTimeNs()` (fix1 wall-clock API) — an undefined symbol against
our installed fix3 library. Updated to:
```cpp
double captureTimeS = (double)cam->getLastCaptureTime();
img->header.stamp = captureTimeS > 0.0
    ? rclcpp::Time(static_cast<int64_t>(captureTimeS * 1e9))
    : nh_->get_clock()->now();
```

**Data flow after all fixes:**
```
physics thread:
  SetupCamera(P_N) + SetPendingCaptureTime(T_N)  ← tempEye/dir/up + tempCaptureTime_

GL thread UpdateTransform():
  pendingCaptureTime_ = tempCaptureTime_           ← no clock query, no race
  eye/dir/up = tempEye/dir/up

GL thread DrawLDR() — reads P_N pixels to PBO:
  captureTime_ = pendingCaptureTime_               ← T_N frozen with P_N pixels

GL thread UpdateTransform() next cycle — NewDataReady:
  lastCaptureTime_ = getCaptureTime() = T_N        ← delivered with P_N frame ✓
```

---

### 2. OctoMap ray-cast back-face limitation

When a ray hits a surface, it marks only the endpoint as OCCUPIED and everything
before it as FREE. Voxels on the far side are never cleared.

When the robot rotates and views the same pillar from a new angle, old voxels
from the previous angle are off-axis — no new ray passes through them. Both sets
stay OCCUPIED → cluster wider than the actual pillar → wall.

This is **inherent to OctoMap's binary ray-cast model**. It worsens with fast
yaw because more angles accumulate per second.

At 0.8 rad/s with 10 cm voxels, each degree of rotation accumulates independent
voxels from the new viewing angle that old rays never clear.

### 3. Simulation speed drift (minor)

`getSimulationTime(true)` = `simTime + epochOffset` where `epochOffset` is set at
simulation START using wall clock. If Stonefish runs faster or slower than 1× real
time, `getSimulationTime(true)` drifts from `nh_->get_clock()->now()`. This means
the depth image stamp and odometry stamp are in slightly different time domains.

**Impact**: Small (typically < 1 ms at 1× speed). `depth_fix._best_odom_stamp()`
tolerates this since it picks the nearest odom stamp ≤ img_stamp.

---

## Timestamp flow summary (current state)

| Message | Stamp source | After fix |
|---|---|---|
| Odometry | `get_clock()->now()` at publish | Unchanged (wall-clock accurate) |
| TF (world_ned→base_link) | `get_clock()->now()` at physics step | Unchanged |
| Depth image | `getLastCaptureTime()` → `getSimulationTime(true)` at **physics-thread SetupCamera** | ✓ Fixed — same step as odom |
| CameraInfo | Copied from depth image stamp | ✓ Fixed transitively |

---

## depth_fix.py status

- **NaN replacement**: Still needed — Stonefish writes 0 for no-return pixels.
- **Timestamp correction**: Still useful as a safety net. With correct Stonefish
  stamps, `_best_odom_stamp()` should now pick the exact same-step odom (Δ < 1 ms)
  rather than correcting a ~5 ms error. Leaving it in place adds no harm and
  protects against edge cases.

---

## Paths explored / ruled out

| Approach | Outcome |
|---|---|
| `odom_tf_sync` — interpolate TF to exact image timestamp | Addresses symptom correctly but root cause is wrong stamp, not TF interpolation |
| GPU render latency hypothesis | **CONFIRMED** by Stonefish source: `get_clock()->now()` at publish, after PBO readback |
| `depth_fix` NaN replacement | Fixed the ghost sphere at camera origin ✓ |
| `depth_fix` timestamp correction | Correct approach; safety net after Stonefish fix |
| Reduced yaw speed (`step/4`) | Reduces symptom (fewer viewing angles per second) but does not cure |
| OctoMap probabilistic decay | Rejected: masks the problem instead of solving it |
| GL-thread `getSimulationTime()` | **FOUND & FIXED**: race introduced 1–2 step timing error |
| stonefish_ros2 undefined `getLastCaptureTimeNs` | **FOUND & FIXED**: was causing silent runtime crash in depth callback |

---

## Remaining ideas to explore

1. **Verify residual after Stonefish fix3 addendum** (highest priority)
   After rebuilding with the physics-thread snapshot fix, re-run the 0.8 rad/s
   test. If displacement drops to 0–1 voxel, the fix is effective. If 1 voxel
   remains, it is likely the OctoMap back-face limitation (not a timing problem).

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
> After the Stonefish fix3 + addendum, depth image timestamps should match
> their physics-step odometry to within ~1 ms.
> Any remaining world-frame misalignment during fast yaw is attributable to
> OctoMap's ray-cast back-face limitation — a fundamental model limitation,
> not a timestamp issue.
