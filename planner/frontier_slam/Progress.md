# frontier_slam — Change Log

Each entry records a change, its objective, and the **observed impact** once tested.
Entries are ordered chronologically. Mark impact as ✅ positive, ⚠️ mixed/partial,
❌ negative (reverted), or 🔲 not yet tested.

Current parameters → `STATE.md` | Session findings → `Sessions.md`

---

## Change 1 — Initial package creation

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`, `waypoint_controller.py`, `setup.py`, `setup.cfg`,
`package.xml`, `launch/frontier_slam.launch.py`

**Objective**: Create the baseline frontier-based exploration stack.
- `frontier_extractor`: reads `/projected_map`, detects free↔unknown boundaries,
  clusters with `scipy.ndimage.label`, publishes nearest centroid as `/frontier_slam/goal`.
- `waypoint_controller`: P-controller (surge + yaw + heave) toward the goal, publishes
  thruster setpoints to `/bluerov2/controller/thruster_setpoints_sim`.

**Key constants at creation**:
```
KP_YAW=0.4  KP_SURGE=0.25  KP_HEAVE=0.3
MAX_SURGE=0.40  GOAL_RADIUS=2.0  SCAN_YAW=0.08
MIN_CLUSTER_CELLS=5  UPDATE_HZ=0.5
```

**Observed impact**: 🔲 Launch failed — see Change 2.

---

## Change 2 — Fix missing `setup.cfg` (libexec directory)

**Date**: 2026-05-19  
**Files**: `setup.cfg` (created)

**Objective**: Fix launch error `"libexec directory does not exist"`. Without `setup.cfg`,
ament_python does not install console_scripts into `lib/<package_name>/`.

**Fix**:
```ini
[develop]
script_dir=$base/lib/frontier_slam
[install]
install_scripts=$base/lib/frontier_slam
```

**Observed impact**: ✅ Launch succeeded. Both nodes started.

---

## Change 3 — Add `MIN_EXPLORE_DIST` to skip nearby frontiers

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: Robot was immediately entering "Goal reached — hovering" because the only
visible frontier was within `GOAL_RADIUS` of the start position.

**Fix**: Added `MIN_EXPLORE_DIST = 3.0 m` filter — any frontier closer than 3 m is
discarded before goal selection.

**Observed impact**: ✅ Robot started moving toward distant frontiers instead of hovering.

---

## Change 4 — Fix sinking during hover (depth hold)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Robot sank from depth 10 m to 18 m+ while hovering at "goal reached"
state. Zero thrust on vertical thrusters causes free-fall in NED frame (Z+ = down).

**Fix**: Always apply `heave_cmd = KP_HEAVE * (-dz)` — even when goal is reached.
Also apply a slow `SCAN_YAW` rotation to survey the area rather than sitting still.

Before:
```python
self._mix_and_send(0.0, 0.0, 0.0)
```
After:
```python
heave_cmd = float(np.clip(self.KP_HEAVE * (-dz), -1.0, 1.0))
self._mix_and_send(0.0, self.SCAN_YAW, heave_cmd)
```

**Observed impact**: ✅ Robot held depth. Sinking eliminated.

---

## Change 5 — Replace hard heading threshold with cosine scaling

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Hard threshold at `HEADING_THRESH = 0.3 rad` caused bang-bang oscillation:
robot alternated between "spinning in place" and "surging forward", leading to a jerky
zigzag path and a wall collision.

**Fix**: Removed `HEADING_THRESH` entirely. Replaced with smooth cosine scaling:
```python
surge_raw = KP_SURGE * dist_xy * max(0.0, math.cos(heading_err))
```
Surge is maximum when perfectly aligned (cos=1), drops to zero at 90° misalignment,
and naturally produces a curved approach path.

**Observed impact**: ✅ Smooth curved approach. No more oscillation. Wall collision eliminated.

---

## Change 6 — Add goal hysteresis (`_committed` + `SWITCH_HYSTERESIS`)

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: Robot oscillated between two equidistant frontier centroids every 2 s,
flipping the goal on each update cycle. Wasted time, caused erratic heading.

**Fix**: Added `_committed` goal state. A new candidate only replaces the current goal
if it is more than `SWITCH_HYSTERESIS = 3.0 m` closer than the committed one. If the
committed goal's cluster has drifted (map update), track the drift by updating
`_committed` without resetting the timer.

**Observed impact**: ✅ Goal oscillation eliminated. Robot commits to one direction.

---

## Change 7 — Slow down yaw and scan (10×)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Heading changes were abrupt and visually jarring. User requested 10× slower
turning for smoother behaviour.

**Fix**:
```python
KP_YAW   = 0.4  → 0.04
SCAN_YAW = 0.08 → 0.008
```

**Observed impact**: ✅ Smooth, gradual heading changes. User confirmed improvement.

---

## Change 8 — Forward obstacle avoidance (depth camera proximity)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Robot collided with a wall while surging toward a frontier behind it.
Add reactive braking using the forward depth camera.

**Fix**: Subscribe to `/sensor_msgs/image_depth` (32FC1). Parse the central 40% horizontal
strip, take the minimum finite range, and scale surge linearly:
- `obs_factor = 1.0` at ≥ 2.5 m → full surge allowed
- `obs_factor = 0.0` at ≤ 0.8 m → surge cut to zero

```python
obs_factor = min(1.0, max(0.0,
    (self._min_front_dist - OBSTACLE_STOP_DIST) /
    (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)
))
surge_cmd = clip(surge_raw * obs_factor, 0.0, MAX_SURGE)
```

**Observed impact** (session 1): ⚠️ No wall collisions observed — avoidance works.
However, the robot repeatedly gets [BLOCKED] for 20-30 s when turning away from structure
corner near (14.8, 2.5): heading error starts at 120-150°, forward camera sees wall at
0.2-0.7 m during the turn, so surge stays zero while yaw slowly clears the obstacle.
Not dangerous but wastes significant exploration time each visit.

**Known limitation**: Only the forward camera is used. The robot has no lateral or rear
awareness — wall strikes from the side remain possible. See FutureWork §3.

---

## Change 9 — Stuck detection + goal blacklisting

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: Robot was committed to goal (12.0, −7.4, 8.5) for 20+ seconds with no
progress — likely blocked by a wall. No mechanism existed to abandon unreachable goals.

**Fix**: Track distance to committed goal at commit time (`_committed_dist`). On each
update cycle check:
```
elapsed > STUCK_TIMEOUT (15 s)  AND  progress < STUCK_MIN_PROGRESS (0.5 m)
```
If stuck: WARN, add goal to `_blacklist` for `BLACKLIST_DURATION = 30 s`, clear
`_committed` so the next best candidate is selected.

**New constants**:
```
STUCK_TIMEOUT      = 15.0 s
STUCK_MIN_PROGRESS = 0.5 m
BLACKLIST_DURATION = 30.0 s
GOAL_VANISH_DIST   = 3.0 m   (radius for blacklist proximity check)
```

**Observed impact** (session 1): ✅ Triggered correctly 3 times:
- `(12.0,-7.4)` after 16s, 0.00m progress
- `(9.3,-9.1)` after 16s, 0.15m progress
- `(11.9,-11.1)` after 16s, 0.06m progress

Side effect: all 3 blacklisted simultaneously → "All candidates blacklisted" for ~14s
(waiting for first 30s timer to expire). Robot holds position during this window.

---

## Change 10 — Rich diagnostic logging

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Previous logs gave no visibility into why the robot was stuck. Added
structured per-cycle logs to allow post-hoc diagnosis from log output alone.

**frontier_extractor** (every update cycle, ~2 s):
```
robot=(x,y,z)  goal=(gx,gy)  dist=X.Xm  clusters=N  stuck=X%  blacklist=N
```
`stuck%` = percentage of `STUCK_TIMEOUT` elapsed on current goal.

**waypoint_controller** (throttled 2 s):
```
pos=(x,y,z) goal=(gx,gy,gz) dist=X.Xm  hdg_err=±Xdeg  surge=X.XX  yaw=±X.XXX  heave=±X.XX  obs=X.Xm  [BLOCKED]
```
`[BLOCKED]` tag appears when `obs_factor < 0.05`.

**Observed impact** (session 1): ✅ Logs were essential for diagnosing all issues in session 1.
Identified: initial stuck (0.00m progress), BLOCKED pattern at (14.8, 2.5) corner,
back-and-forth loop between two frontiers, Z drift.

---

## Change 11 — Automatic CSV session logging

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Manual log pasting is tedious and lossy. Both nodes now write structured
CSV files automatically to `logs/` at startup, enabling session-to-session comparison.

**Format**: Two files per session, named by wall-clock start time:
- `YYYY-MM-DD_HH-MM-SS_extractor.csv` — written at 0.5 Hz (every update cycle)
- `YYYY-MM-DD_HH-MM-SS_controller.csv` — written at 1 Hz (every 10th control tick)

**extractor columns**: `t_ros, rx, ry, rz, gx, gy, dist_m, clusters, stuck_pct, blacklist_n, event`  
**controller columns**: `t_ros, rx, ry, rz, gx, gy, gz, dist_m, hdg_err_deg, surge, yaw_cmd, heave, obs_m, blocked, event`

**event values**:
- extractor: `""` (normal), `STUCK_BLACKLIST`, `ALL_BLACKLISTED`
- controller: `""` (normal), `BLOCKED`, `GOAL_REACHED`, `SCAN`

Log path is derived from `__file__` via `os.path.realpath()` so it always resolves
to the source `logs/` directory regardless of how the package is installed.

**Observed impact**: ✅ Session 2 (`2026-05-19_14-51-54`): both CSV files created at startup,
both nodes wrote correct data for 82s. Float formatting, column names, and flush-on-write
all confirmed. Controller CSV `depth_err_m` column confirms the refactored schema was live.

---

## Change 12 — Depth-hold uses a fixed setpoint (Z drift fix)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`, `frontier_extractor.py`

**Objective**: Session 1 logs showed the robot sinking from 8.5 m to 13.8 m over 4 minutes
even though every heave command was small (+0.00 to +0.06). The cause was a feedback
loop in the goal-Z plumbing:

```
extractor: gz = robot.z       ← uses CURRENT robot z each cycle
controller: heave = KP * (-(gz - pose.z))
```

When the robot drifted down between updates (2 s), the next frontier message brought a
new, deeper `gz`. The depth error never grew large enough to trigger meaningful heave,
so each cycle the controller accepted the drifted depth as the new target.

**Fix**:
- `WaypointController` now owns the depth setpoint. It is captured once on the first
  odometry callback (`self._depth_setpoint = pose.z`) and held for the lifetime of the
  node. The control law became `heave = KP_HEAVE * (pose.z - setpoint)`.
- The goal's Z field is now ignored by the controller. The extractor still publishes
  `gz = robot.z` but only so the RViz marker sits at the right height — it does not
  influence the depth controller.
- `KP_HEAVE` bumped from 0.30 → 0.40 to give a bit more authority against any
  persistent buoyancy bias.

**Observed impact**: ❌ Sign inverted — robot sank rapidly. `KP_HEAVE * (pose.z - setpoint)`
gives **positive** heave when robot is too deep, but positive = DOWN in Stonefish.
The robot fell faster than without any depth control. Fixed in Change 14.

---

## Change 13 — Refactor: split into focused modules

**Date**: 2026-05-19  
**Files**: new `session_log.py`, `frontier_detection.py`, `goal_manager.py`,
`control_utils.py`; rewritten `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Both node files had grown to mix several unrelated concerns (frontier
detection + goal state + viz + CSV logging in one; thruster mixing + control law +
CSV logging in the other). The single update/loop methods were 80+ lines.

**Structure**:
```
frontier_slam/
├── frontier_extractor.py     # node: orchestration, ROS I/O, marker viz
├── waypoint_controller.py    # node: orchestration, ROS I/O, control loop
├── frontier_detection.py     # pure: OccupancyGrid → list[Cluster]
├── goal_manager.py           # state: hysteresis, stuck detection, blacklist
├── control_utils.py          # pure: yaw_from_quat, wrap_angle, mix_thrusters
└── session_log.py            # shared: CSV opener + float-formatting writer
```

**Behavioural guarantees**: refactor only — no algorithmic changes alongside Change 12.
Same constants, same control law, same selection rules. Identical CSV schemas
(controller gained one column: `depth_err_m`).

**Observed impact**: ✅ Session 2 (`2026-05-19_14-51-54`): all modules imported correctly,
both nodes started, frontier detection ran (7 clusters at t=0), GoalManager selected and
switched goals, stuck detection was not triggered (no regression). No behavioural
differences from pre-refactor sessions observed.

---

## Change 14 — Fix heave sign (Stonefish convention: negative = UP)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Change 12 introduced `heave = KP_HEAVE * (pose.z - setpoint)`. In NED,
`pose.z - setpoint > 0` when the robot is too deep — the correct response is upward
thrust. In the Stonefish simulation **negative heave drives upward thrust**, so the
formula produced strong downward thrust, accelerating the sink.

**Fix**: Negate the error term:
```python
# Before (wrong — positive when deep = pushes DOWN):
return float(np.clip(self.KP_HEAVE * depth_err, -1.0, 1.0))

# After (correct — negative when deep = pushes UP in Stonefish):
return float(np.clip(-self.KP_HEAVE * depth_err, -1.0, 1.0))
```

The docstring is updated to state "Negative output = upward thrust in Stonefish" to
prevent future sign confusion.

**Observed impact**: ✅ Session 2 (`2026-05-19_14-51-54`): depth setpoint locked at 8.14m
on first odom. Z stayed in [8.12, 8.25]m over 82s (max error 0.11m). Compare to Session 1
where z drifted 5.3m in 4 minutes with no depth control. `depth_err_m` column confirms
the controller was actively correcting (heave between −0.01 and −0.06 throughout).

---

## Change 15 — Goal timeout: clear stale goal and enter scan mode

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 2 showed the robot frozen in GOAL_REACHED for 63s after frontiers
ran dry. The controller held `_goal` forever; SCAN mode (`_goal is None`) was never entered.

**Fix**: Track when GOAL_REACHED state was first entered. If no new goal arrives within
`GOAL_REACHED_TIMEOUT = 10.0 s`, clear `_goal` and log. Next tick enters SCAN mode
(slow yaw rotation) which may help the robot find new frontiers by rotating into new
areas. Timer resets whenever a fresh goal arrives.

```python
if now - self._goal_reached_at > self.GOAL_REACHED_TIMEOUT:
    self._goal = None          # → next tick: SCAN mode
    self._goal_reached_at = None
```

**Observed impact**: ✅ Session 3 (`2026-05-19_15-05-00`): coverage columns present in
extractor CSV. Confirmed growth from 8514 → 9756 mapped cells (+14.6%) in 12s of
exploration. Largest gain (+556 cells) when robot was closest to wall (obs=0.23m).
Coverage stalled once robot moved into open water — useful signal for future sessions.

---

## Change 17 — Restore SCAN_YAW to 0.08 (split scan speed from nav yaw)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 3 showed the robot frozen at (14.44, −4.06) in SCAN mode for 59s
without finding any new frontiers. `SCAN_YAW=0.008` was reduced 10× in Change 7 alongside
`KP_YAW` to smooth navigation turns. That reduction was appropriate for navigation but
made scanning nearly useless: ~13 minutes for a full rotation.

**Fix**: Restore `SCAN_YAW = 0.08` (its pre-Change-7 value). `KP_YAW` stays at 0.04 —
the two constants now serve different purposes and should be tuned independently.

At 0.08 the robot completes a meaningful scan arc in a few seconds, giving the extractor
a chance to see new frontier cells as the camera sweeps past unexplored structure.

**Observed impact**: ✅ Session 3 (`2026-05-19_15-05-00`): GOAL_REACHED entered at t=519,
goal cleared and SCAN entered at t=530 (11s later). Controller CSV shows clean transition:
rows switch from `GOAL_REACHED` with valid goal columns to `SCAN` with empty goal columns.
Timer reset on new goal confirmed by clean re-acquisition at start of session.

---

## Change 19 — Frontier scoring: prefer large clusters over nearest cluster

**Date**: 2026-05-19  
**Files**: `goal_manager.py`

**Objective**: Session 7 showed the robot looping for 231s between two equidistant frontier
clusters (A and B at opposite corridor ends), gaining only 74 new cells. The root cause:
`candidates.sort(key=lambda c: c.distance)` gives equal weight to any two clusters at the
same distance regardless of size. When the large unexplored installation is 18m away and a
small 4-cell boundary patch is 15m away, nearest-first picks the patch every time.

**Fix**: Change sort key to `c.distance / c.size` (ascending = best first). A cluster of
50 cells at 18m scores 0.36; a cluster of 4 cells at 15m scores 3.75. The robot now
consistently heads toward the information-rich region.

Side effect: noise immunity. OccupancyGrid noise creates phantom clusters of 5–8 cells
in already-explored areas. These score poorly vs genuine large frontiers — the robot
ignores them without any additional filtering.

The mid-journey hysteresis (`switch_hysteresis = 3.0m`) is intentionally kept distance-based:
it prevents abandoning a large distant goal for a small nearby distraction mid-journey.

**Observed impact**: ✅ Session 8 (`2026-05-19_16-15-40`): robot traveled south to y=−14
(session 7 reached only y=−4). 16,100 new cells in 311s vs 11,803 in 360s — +58% cells/minute.
Odometer dropped from 262m to 149m — robot is exploring smarter. The large unexplored
installation was reached and partially mapped. New failure mode exposed: after 3 STUCK events
in the obstacle-dense south zone, only 4 frontier clusters remained and exploration stalled.

---

## Change 18 — Obstacle escape: forced yaw when BLOCKED + position-stuck detection

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Sessions 4–6 exposed two distinct failure modes that the previous obstacle
avoidance could not handle:

1. **Aligned BLOCKED** (`obs < STOP_DIST`, `heading_err ≈ 0`): robot is aimed directly at
   a pillar; both surge and yaw drop to ~0. No escape maneuver.
2. **Off-axis physical stuck** (`obs ≈ 2.4m`, `blocked=False`): depth camera sees 2.4m of
   clear water straight ahead, but the robot's body is pressing against a wall or pillar
   that is slightly off-axis from the central 40% strip. Controller applies full `surge=0.40`
   indefinitely; robot barely moves (position oscillates ±0.03m for 35+ seconds).

**Fix — two mechanisms added:**

*A. Escape yaw on BLOCKED*: When `obs < STOP_DIST` (sensor-blocked), override the
goal-tracking `yaw_cmd` with `±ESCAPE_YAW = 0.40` in the direction of the heading error.
This ensures the robot always pivots away from a head-on obstacle regardless of heading_err magnitude.

*B. Position-stuck detection*: If `surge ≥ STUCK_SURGE_MIN = 0.15` continuously for
`STUCK_WINDOW = 5.0s` without moving more than `STUCK_MOVE_MIN = 0.25m`, the controller
logs `CTRL_STUCK`, sets `_escape_until = now + ESCAPE_DURATION`, and spins at `ESCAPE_YAW`
for `ESCAPE_DURATION = 4.0s` before resuming normal drive. This catches off-axis physical
blockages that the depth camera cannot see. The stuck reference is also reset whenever a
new goal arrives.

New CSV `event` values: `STUCK_ESCAPE` (spinning during escape), `CTRL_STUCK` (trigger row).

New constants: `ESCAPE_YAW=0.40`, `ESCAPE_DURATION=4.0s`, `STUCK_SURGE_MIN=0.15`,
`STUCK_WINDOW=5.0s`, `STUCK_MOVE_MIN=0.25m`.

**Observed impact**: ✅ Session 8 (`2026-05-19_16-15-40`): all BLOCKED events resolve in
1–3 ticks (max consecutive run 15 ticks at one south-zone pillar). No frozen BLOCKED events
like sessions 4–5 (80+ seconds). CTRL_STUCK did not fire — a gap was found: position-stuck
requires `surge≥0.15` which is never true when BLOCKED (obstacle factor kills surge). Robot
can spin indefinitely in BLOCKED state without triggering the escape timer. Fix tracked as
future work (add BLOCKED-duration counter separate from surge-based detection).

---

## Change 16 — Map coverage metrics in extractor CSV

**Date**: 2026-05-19  
**Files**: `frontier_extractor.py`

**Objective**: The extractor CSV gave no visibility into how much of the map was being
built over time. Without coverage numbers it is impossible to compare exploration
efficiency across sessions or algorithm variants.

**Fix**: Added `_map_stats()` which counts OccupancyGrid cell states on each update
cycle. Three new columns added to `extractor.csv`:

| Column | Meaning |
|---|---|
| `free_cells` | Cells with value 0 (navigable space confirmed) |
| `occ_cells` | Cells with value 100 (obstacle confirmed) |
| `mapped_cells` | `free + occ` — total cells with known state |

`mapped_cells` is the primary coverage metric: it grows as the robot explores and
plateaus when exploration stalls. Also printed in the extractor's per-cycle ROS log line.

**Observed impact**: ✅ Session 7 (`2026-05-19_15-55-06`): `mapped_cells` tracked continuously
from 15,348 at session start to 27,151 at plateau. Growth rate visible in CSV (fast early,
then near-zero after t≈135s). Used to confirm the back-and-forth loop in phase 4.

---

## Change 20 — Lateral obstacle readings (left / right strips)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`, `tools/analyze_session.py` (new)

**Objective**: The forward depth camera previously used only the central 40% column strip
(30%–70%) to measure obstacle distance. This left the 30% on each side unseen — a robot
pressing sideways against a lateral wall could still show `obs_m = inf` and full surge.
Session 8 showed multiple south-zone BLOCKED chains that the forward camera alone couldn't
fully characterise.

Additionally: created `tools/analyze_session.py` as a standalone analysis script intended
to evolve with the system. Produces per-session reports and two-session comparisons from
controller + extractor CSV files.

**Fix — lateral strips**:
The single `_depth_cb` extraction is split into three non-overlapping strips:

| Strip | Columns | State variable |
|---|---|---|
| Left  | 0%–30%  | `_min_left_dist` |
| Centre| 30%–70% | `_min_front_dist` (unchanged) |
| Right | 70%–100%| `_min_right_dist` |

Two new CSV columns: `obs_left_m`, `obs_right_m` inserted between `obs_m` and `blocked`.
The obstacle factor and BLOCKED decision still use only `_min_front_dist` — lateral values
are logged for analysis. Using them for active steering is future work (e.g. soft wall-following
or lateral escape direction preference).

**New CSV columns (controller)**:
```
… obs_m, obs_left_m, obs_right_m, blocked, event
```

**tools/analyze_session.py**:
- Auto-detects latest session or accepts a prefix argument
- `--compare A B`: side-by-side metric table for two sessions
- `--list`: enumerate all sessions with duration
- Sections: Overview, Coverage (with per-quarter bar chart), Navigation,
  Goal behaviour, Obstacle/BLOCKED, Depth control
- Reads `obs_left_m` / `obs_right_m` if present in log (graceful fallback for older logs)

**Observed impact**: 🔲 Not yet tested.

---

## Change 21 — BLOCKED direction hysteresis + timeout escape

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 8 showed two BLOCKED-state problems:

1. **Direction jerk**: when `heading_err` alternates sign on consecutive ticks (robot spinning
   near a wall at a shallow angle), `yaw_cmd` flipped between +0.40 and −0.40 every second.
   This looks like rapid jerking to the observer and produces no net rotation.
2. **Indefinite BLOCKED stall**: position-stuck detection requires `surge≥0.15`, which is
   always false when BLOCKED (obstacle factor kills surge). The robot could spin in BLOCKED
   state for 15+ seconds with no escape trigger.

**Fix — direction hysteresis**: Track `_escape_yaw_dir` (+1 or -1) and `_escape_dir_flip_t`.
Only flip spin direction after `BLOCKED_DIR_MIN_DURATION = 2.0s` in the current direction.
This commits the robot to one rotation sense long enough to actually clear the obstacle.

**Fix — BLOCKED timeout**: Track `_blocked_since`. If continuously BLOCKED for
`BLOCKED_ESCAPE_TIMEOUT = 5.0s`, trigger the same committed 4s escape spin as `CTRL_STUCK`.
New CSV event: `BLOCKED_TIMEOUT`.

Both `_blocked_since` and `_escape_yaw_dir` reset on new goal arrival (may be in clear water).
`_blocked_since` also resets after each committed escape spin completes.

New constants: `BLOCKED_ESCAPE_TIMEOUT=5.0s`, `BLOCKED_DIR_MIN_DURATION=2.0s`.

**Observed impact**: 🔲 Not yet tested.

---

## Change 22 — Goal manager: minimum commit duration + longer blacklist

**Date**: 2026-05-19  
**Files**: `goal_manager.py`

**Objective**: Session 8 exposed two related failure modes in `_pick_with_hysteresis`:

1. **17m U-turn oscillation**: after a STUCK_BLACKLIST, the only candidate was sometimes
   (12.90, 3.00) at 17m north. Within seconds of heading that way, a southern cluster at
   ~3m re-appeared with a competitive score, causing an immediate reversal. The 3m
   `switch_hysteresis` is too small when the committed goal is 17m away.
2. **30s blacklist too short**: goal (6.6, −12.5) was blacklisted at t=140s and t=214s —
   the 30s expired and the robot immediately retried the same unreachable area.

**Fix — `min_commit_duration = 10.0s`**: Score-based switches are blocked for the first
10s after any new commitment. The goal-vanished path (committed cluster disappears from the
map) is still allowed to switch immediately — that reflects a genuine map change.

**Fix — `blacklist_duration` default: 30s → 60s**: Doubles the cooling-off period.
A goal that couldn't be reached in 15s of trying is unlikely to become reachable in the
next 30s; 60s gives the map more time to update or the robot's approach angle to change.

Both parameters are constructor arguments with sensible defaults — no call-site changes needed.

**Observed impact**: 🔲 Not yet tested.

---

## Change 23 — Fix CTRL_STUCK timer reset + reduce min_commit_duration

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`, `goal_manager.py`

**Objective**: Session 9 analysis revealed two bugs that together prevented any recovery
from a 111-second freeze near (7.6, −4.1):

1. **CTRL_STUCK timer reset bug** (`waypoint_controller.py`): `_goal_cb` unconditionally
   reset `_stuck_ref_pos = None` on every call. The frontier extractor republishes at
   0.5 Hz (every 2s), so the stuck timer was reset every 2s. With `STUCK_WINDOW = 5.0s`,
   the 5s accumulation window could never be reached — the timer was always cleared before
   it matured. The GoalManager STUCK mechanism also failed: the robot had made 7.4m of
   progress on first approach (enough to clear the 0.5m threshold), so the
   `committed_dist - cur_dist < STUCK_MIN_PROGRESS` check permanently returned "not stuck"
   even after the robot froze.

2. **min_commit_duration = 10s too conservative** (`goal_manager.py`): Session 9 produced
   only 7 goal commits vs 35 in session 8. The robot stayed locked on distant goals even
   when large newly-discovered frontiers appeared closer. The robot never explored the
   "other side of the boat" or the left building's far side — likely because min_commit_duration
   suppressed the switches that would have redirected it.

**Fix 1 — `_goal_cb` goal-change threshold**:
```python
goal_changed = (self._goal is None or
                np.hypot(new_goal[0] - self._goal[0],
                         new_goal[1] - self._goal[1]) > 1.0)
self._goal = new_goal
self._goal_reached_at = None
if goal_changed:
    self._stuck_ref_pos = None
    self._stuck_ref_t   = None
    self._blocked_since = None
```
The extractor republishing the same frontier (±0.1m drift) no longer resets the stuck
window. `_goal_reached_at` is still reset unconditionally — always correct on any goal update.

**Fix 2 — `min_commit_duration` 10.0s → 5.0s**:
Halving the minimum commit time lets the robot respond to better frontiers discovered en
route (e.g. a large cluster that becomes visible as the map grows) while still blocking
the rapid flip-flopping that triggered the session 8 U-turn oscillation.
The 5s window keeps the anti-oscillation benefit for transitions under ~5m radius.

**Observed impact**: 🔲 Not yet tested.

---

## Change 24 — Vanish-guard: hold committed goal when robot is close

**Date**: 2026-05-19  
**Files**: `goal_manager.py`

**Objective**: Session 10 exposed a figure-8 oscillation in the (12.5, −8) area. The robot
alternated between goals (12.86, −5.06) and (12.40, −11.20) every ~10s for 163 seconds with
zero new cells mapped. Root cause: frontier cluster at the committed goal disappears from the
map right as the robot approaches it (the sensor maps those cells as free/occupied), triggering
an immediate vanish-based switch in `_pick_with_hysteresis`. The controller never fires
`GOAL_REACHED` because `_goal_cb` replaces the goal before the 2m radius check. `min_commit_duration`
cannot help because this uses the vanish path, not the score path.

**Fix**: Before doing a vanish-based switch, check whether the robot is already within
`goal_vanish_dist` (3m) of the committed position. If so, the cluster was mapped by the
robot's own sensor during approach — return the committed coordinates unchanged so the
controller can declare `GOAL_REACHED` naturally. Only switch immediately if the robot is
far from the vanished goal (genuine map change, not just self-mapping).

```python
if not near_old:
    cur_dist = np.hypot(robot_xy[0] - self._committed[0],
                        robot_xy[1] - self._committed[1])
    if cur_dist <= self.goal_vanish_dist:
        return float(self._committed[0]), float(self._committed[1])
    self._commit(best.wx, best.wy, robot_xy, now)
    return best.wx, best.wy
```

Safety: if the robot is stuck near the held goal (wall), `_check_and_blacklist_if_stuck`
fires after 15s with <0.5m progress and blacklists it, breaking the hold.

**Observed impact**: 🔲 Not yet tested.

---

## Change 25 — Reduce yaw gains (turning too fast)

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Robot visually turns too fast. With `KP_YAW=0.10`, a 180° goal reversal
produces `yaw_cmd ≈ 0.31`. `ESCAPE_YAW=0.40` (BLOCKED spin) compounds this. Session 10
figure-8 made it especially visible — continuous high-speed reversals.

**Fix**:
- `KP_YAW`: 0.10 → 0.07 — reduces max navigation yaw from 0.31 to 0.22 (a 180° turn)
- `ESCAPE_YAW`: 0.40 → 0.20 — reduces BLOCKED/escape spin rate

New relationship: `ESCAPE_YAW (0.20) ≈ SCAN_YAW * 2.5 ≈ max nav yaw * 0.9`. The escape
spin is now a committed timed maneuver at a moderate rate rather than a high-speed spin.

**Observed impact**: 🔲 Not yet tested.

---

## Change 26 — A* path planning (replaces reactive BLOCKED machinery)

**Date**: 2026-05-19  
**Files**: `path_planner.py` (new), `frontier_extractor.py`, `waypoint_controller.py`

**Objective**: Replace the sensor-reactive BLOCKED state machine with proactive A* path
planning on the OccupancyGrid.  Reactive avoidance had three failure modes that persisted
across sessions 4–10: (1) robot gets trapped in BLOCKED spin loops at complex corners,
(2) figure-8 oscillations caused by approaching frontiers through mapped walls, (3) escape
spins are blind — they often spin into the same obstacle from a different angle.

A* solves all three: it plans around mapped walls before committing to a direction.

**What was removed** (from `waypoint_controller.py`):
- Left/right camera strip processing (`_min_left_dist`, `_min_right_dist`)
- `_obstacle_factor()` surge scaling
- `BLOCKED` state machine and all related state/constants:
  `_blocked_since`, `_escape_yaw_dir`, `_escape_dir_flip_t`,
  `BLOCKED_ESCAPE_TIMEOUT`, `BLOCKED_DIR_MIN_DURATION`,
  `OBSTACLE_SLOW_DIST`, `OBSTACLE_STOP_DIST`
- CSV columns `obs_left_m`, `obs_right_m`, `blocked`

**What was added**:

*`path_planner.py` (pure functions)*:
- `inflate_occupied(grid, radius_cells)` — binary dilation of occupied (100) cells only;
  unknown (−1) cells are never inflated into obstacles (optimistic for exploration)
- `_astar(blocked, start, goal)` — standard A* with 8-connectivity, min-heap, visited set;
  unknown and free cells both traversable; returns list of (row, col) or None
- `plan_path(grid_msg, robot_xy, goal_xy, inflation_m=0.75)` — full pipeline:
  inflate → A* → thin waypoints (every 10th cell); snaps start/goal out of inflated walls
  by finding nearest free cell; returns world-frame `[(x, y), ...]` or `[]` on failure

*`frontier_extractor.py`*:
- `_current_goal_xy` state variable; set in `_publish_goal`
- `_path_pub` on `/frontier_slam/path` (nav_msgs/Path)
- `_replan()` timer at 1 Hz: runs A*, publishes path; warns if A* returns empty

*`waypoint_controller.py`*:
- Subscribes to `/frontier_slam/path`; drives toward `_path[_wp_idx]` instead of raw goal
- Waypoint advance: while closest waypoint is within `WAYPOINT_ADVANCE_DIST=1.5m`, advance index
- On new path arrival: reset `_wp_idx` to the nearest waypoint to current position (handles 1Hz replanning smoothly)
- On new goal (>1m change): clear path and reset index so stale waypoints aren't followed
- **Initial 360° scan**: spin for `INIT_SCAN_DURATION=20.0s` on first odom before navigating — gives A* a populated local map before the first path is planned
- Emergency stop: if `obs_m < EMERGENCY_STOP_DIST=0.4m` → zero surge (single line, no state machine); last resort for dynamic obstacles not in the map
- `CTRL_STUCK` position-stuck detection unchanged (safety net; should rarely fire now)

**CSV schema change** (controller):
```
Before: … obs_m, obs_left_m, obs_right_m, blocked, event
After:  … obs_m, path_len, wp_idx, event
```

**Why inflation_m=0.75?**  
BlueROV2 body radius ≈ 0.3m. 0.75m gives ≈ 0.45m clearance margin beyond the body,
keeping the planned path well away from walls and preventing the robot from brushing
inflated-but-passable gaps it would physically block.

**Why unknown = free?**  
Treating unknown cells as obstacles would prevent planning into unexplored space — the robot
could never reach frontier goals that are surrounded by unmapped area.  Optimistic planning
is standard practice in frontier exploration.

**Observed impact**: 🔲 Not yet tested.

---

## Change 28 — GoalManager vanish-guard: release committed goal when robot has arrived

**Date**: 2026-05-19  
**Files**: `goal_manager.py`, `frontier_extractor.py`

**Objective**: Session 12 showed GOAL_REACHED still looping indefinitely every ~12s, even
after Ch27's fix to `_goal_reached_at`.

**Root cause**: The vanish-guard (Ch24) held the committed position whenever the cluster
disappeared and `cur_dist ≤ goal_vanish_dist=3.0m`. This was correct for the approaching
(2–3m) case. But when the robot was already inside `GOAL_RADIUS=2.0m`, the cluster was
gone, the vanish-guard held (6.13, −5.22), and the extractor kept republishing it. The
controller cleared `_goal=None` after GOAL_REACHED_TIMEOUT=10s, entered SCAN for 1 tick,
then received (6.13, −5.22) again as a "new" goal (was None → something = `goal_changed`),
resetting `_goal_reached_at` and restarting the cycle.

**Fix**: Split the vanish-guard into two sub-cases based on `goal_radius`:
```python
if cur_dist <= self.goal_vanish_dist:
    if cur_dist > self.goal_radius:
        return committed   # still approaching — hold so controller reaches it (Ch24)
    # inside goal_radius: robot has arrived — switch to next frontier immediately
    self._commit(best.wx, best.wy, robot_xy, now)
    return best.wx, best.wy
```
The `goal_radius=2.0` parameter is added to GoalManager and passed from the extractor.
Result: once the robot is within 2.0m of the committed position, GoalManager proactively
switches to the next frontier — the extractor starts publishing a different goal, and the
GOAL_REACHED loop cannot form.

**Why the figure-8 does not return**: The original figure-8 (session 10) involved the
vanish-guard being bypassed at ~1.4m distance (inside goal_radius). The Ch24 fix held the
goal down to 0m. The new fix holds from `goal_vanish_dist` down to `goal_radius` (3.0m →
2.0m), then switches. At 1.4m, the switch fires — but the robot has already mapped the
frontier, so the switched-to goal is a genuinely new candidate, not the previous one in
a loop.

**Observed impact**: 🔲 Not yet tested.

---

## Change 27 — Fix GOAL_REACHED reset on republish + shorten initial scan

**Date**: 2026-05-19  
**Files**: `waypoint_controller.py`

**Objective**: Session 11 exposed two regressions from the Ch26 rewrite:

1. **GOAL_REACHED stuck forever**: Robot reached goal (7.60, −7.50) at t≈42s and spun in
   GOAL_REACHED for 82s with no exit. Root cause: `_goal_cb` reset `_goal_reached_at = None`
   unconditionally on every call. The extractor republishes the same goal at 0.5 Hz; every
   2s the timer was cleared, preventing the 10s `GOAL_REACHED_TIMEOUT` from accumulating.
   Identical pattern to the Ch23 `_stuck_ref_pos` bug — same extractor republish, same fix.

2. **INIT_SCAN too long (20s)**: Robot completed 3–4 full rotations instead of ~1. Also,
   the GoalManager commits to the first goal at t=0 and its 15s stuck timer fires at t≈17s
   (while the robot is still in INIT_SCAN and stationary), blacklisting the first goal
   before the robot even moves (`STUCK_BLACKLIST` for (12.50, 0.00) visible in session 11
   extractor CSV at t≈17s).

**Fix 1** — move `_goal_reached_at = None` inside `if goal_changed:`:
```python
self._goal = new_goal
if goal_changed:            # ← was outside this block
    self._goal_reached_at = None
    self._path     = []
    ...
```

**Fix 2** — `INIT_SCAN_DURATION`: 20.0s → 5.0s (~1 rotation, finishes before GoalManager
stuck_timeout=15s fires).

**Observed impact**: 🔲 Not yet tested.

---

## Change 29 — Rich debug visualisation + INFLATION_M single source of truth

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`, `path_planner.py`

**Objective**: Add live visualisation so wall crashes and stuck behaviour can be diagnosed
in RViz2 / rqt_image_view without relying solely on CSV post-mortem analysis.
Also eliminate the duplicated `INFLATION_M` constant.

**What was added** (`frontier_extractor.py`):

- `/frontier_slam/inflated_map` (`OccupancyGrid`): same grid as `/projected_map` but with
  inflation zone encoded as value 50 (walls stay 100, free stays 0). Display alongside the
  raw map in RViz2 and toggle independently to see exactly what A* treats as blocked.

- `/frontier_slam/debug_image` (`sensor_msgs/Image`, rgb8): top-down composite image updated
  at 1 Hz alongside `_replan`. Colour key:
  - Dark gray (80,80,80) = unknown
  - Light gray (210,210,210) = free
  - Near-black (20,20,20) = occupied wall
  - **Orange (200,100,50) = inflation zone** — the safety margin A* avoids
  - **Green (0,220,0) = current A\* path waypoints**
  - **Red cross = goal position**
  - **Blue cross = robot position**
  View with `ros2 run rqt_image_view rqt_image_view /frontier_slam/debug_image`

- Goal arrow (`MarkerArray` ns=`goal_arrow`): orange `Marker.ARROW` with tail 2 m behind
  the goal along the robot→goal vector, head at the goal. Shows required approach heading
  at a glance; complements the existing red sphere.

**What was changed** (`path_planner.py`):
- `INFLATION_M = 0.75` promoted to module-level constant (was an inline default argument).
- `plan_path` default argument now references `INFLATION_M` directly.

**What was removed** (`frontier_extractor.py`):
- `INFLATION_M = 0.75` class constant (was a duplicate); now imported from `path_planner`.

**New state** (`frontier_extractor.py`):
- `_current_path: list` — last A\* waypoints, stored in `_replan` before publishing the Path
- `_inflated_grid: np.ndarray | None` — inflated grid recomputed each `_replan` tick

**Observed impact**: 🔲 Not yet tested.

---

## Change 30 — Fix crash when STUCK_BLACKLIST exhausts all candidates

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`

**Objective**: Fix `ValueError: cannot convert float NaN to integer` crash in `_replan`.

**Root cause**: When `STUCK_BLACKLIST` fires and every remaining cluster happens to be within
`goal_vanish_dist=3.0m` of the just-blacklisted goal, `GoalManager.select()` returns
`GoalSelection(nan, nan, 0, 'STUCK_BLACKLIST', ...)`. The extractor's early-return only
checked for `event == 'ALL_BLACKLISTED'`, so the NaN goal fell through to `_publish_goal(nan, nan)`,
which set `_current_goal_xy = [nan, nan]`. On the next 1Hz `_replan` tick,
`_current_goal_xy is not None` passed (NaN array is not None), and `plan_path` crashed on
`int(float('nan'))`.

**Fix 1** — merge the two "no valid goal" cases in `_update`:
```python
if selection.event == 'ALL_BLACKLISTED' or math.isnan(selection.gx):
    self._current_goal_xy = None
    self._current_path    = []
    ...
    return
```
Clearing `_current_goal_xy` prevents `_replan` from planning to a stale position while
waiting for the blacklist to expire.

**Fix 2** — defensive NaN guard in `_replan`:
```python
if self._current_goal_xy is not None and not np.any(np.isnan(self._current_goal_xy)):
    waypoints = plan_path(...)
```

**Observed impact**: 🔲 Not yet tested.

---

## Change 31 — Debug image: heading arrow, path lines, state colour bar

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`

**Objective**: Make the debug image actually useful for diagnosing stuck behaviour and
wall crashes.

**Robot marker** (was: blue cross):
- Now: **arrow** whose direction matches the robot's heading (yaw from odometry quaternion)
  and whose length scales with ground speed (`arm = 4 + speed_m_s * 20 px`).
  At `MAX_SURGE=0.4 m/s` the arrow is ~12 px; stationary → 4 px minimum.
- Colour encodes extractor state:
  - Blue  = navigating normally
  - Yellow = stuck_pct ≥ 50 % (approaching timeout)
  - Orange = stuck_pct = 100 % (maxed, blacklist imminent)
  - Cyan  = no valid goal (scanning / all blacklisted)

**Path** (was: isolated 3×3 dots per waypoint):
- Now: connected **line segments** between consecutive waypoints via `_draw_line`
  (numpy linspace interpolation, no OpenCV required).

**Status bar**: 4-pixel coloured strip at the top of the image, same colour key as the
robot arrow.

**New helpers added**:
- `_draw_line(img, r0, c0, r1, c1, color)` — static, numpy-only line drawing
- `_robot_arrow_color()` — maps stuck_pct / goal state to a display colour

**New `_odom_cb` fields**:
- `_robot_yaw` — extracted via `yaw_from_quat` (imported from `control_utils`)
- `_robot_speed` — `hypot(twist.linear.x, twist.linear.y)` (body-frame horizontal speed)

**Coordinate note**: in the pre-flip OccupancyGrid image, col↔X(North) and row↔Y(East).
Arrow direction `(dc, dr) = (cos(yaw), sin(yaw))` maps correctly after `np.flipud`:
North = right, East = up in the displayed image.

**Observed impact**: 🔲 Not yet tested.

---

## Change 32 — Progressive obstacle slow-down + reduce MAX_SURGE

**Date**: 2026-05-20  
**Files**: `waypoint_controller.py`

**Objective**: Session 13 showed the robot arriving at walls at full speed (0.40 m/s) with
only a binary hard-stop at 0.4m as protection. The EMERG_STOP freeze is invisible to
CTRL_STUCK, leaving the robot permanently stuck.

**Fix — surge ramp**:
Replace the single-threshold hard-stop with a continuous linear ramp:
```
obs ≥ OBS_SLOW_DIST (2.0m) → surge unchanged (full speed)
obs in [0.4m, 2.0m]        → surge *= (obs − 0.4) / (2.0 − 0.4)
obs < EMERGENCY_STOP_DIST (0.4m) → surge = 0, event = 'EMERG_STOP'
```
At 1.2m the robot is moving at half speed; at 0.8m, quarter speed. This gives the
controller time to steer away before actually stopping.

**Fix — lower MAX_SURGE**:
`MAX_SURGE`: 0.40 → 0.25 m/s. Gives A* replanning (1Hz) enough time to react to
newly-mapped obstacles before the robot reaches them at full speed.

**Observed impact**: ✅ Session 14 (`2026-05-20_13-47-57`): robot slowed from 0.25→0 over
last 1.6m before wall (obs 2.0→0.37m). No hard crash. Still frozen at wall — back-surge
fix needed (Ch33).

---

## Change 33 — Back-surge escape + GoalManager sliding-window stuck detection

**Date**: 2026-05-20  
**Files**: `waypoint_controller.py`, `goal_manager.py`

**Objective**: Two independent failure modes both prevent recovery from a wall encounter:
the controller freezes in place and the GoalManager never detects it as stuck.

**Fix 1 — back-surge** (`waypoint_controller.py`):
When `obs < EMERGENCY_STOP_DIST (0.4m)`, apply `surge = -BACK_SURGE_SPEED (0.12 m/s)`
instead of zeroing surge. The robot actively reverses away from the wall, moving back into
the free zone where A* can replan from a valid starting cell.

Sequence: approach wall → ramp reduces surge [2.0m→0.4m] → back-surge kicks in [<0.4m]
→ robot backs up → obs rises → ramp re-engages → A* has a clear start position and replans
→ path found → follows route around wall.

**Fix 2 — sliding-window stuck detection** (`goal_manager.py`):
Replace the one-shot `committed_dist - cur_dist` check with a progress clock:
- Track `_closest_dist` (minimum distance to goal ever seen) and `_closest_t` (when it was set).
- Clock resets whenever the robot closes the gap by > `stuck_min_progress (0.5m)`.
- STUCK fires when `now - _closest_t >= stuck_timeout (15s)`.

Before: "robot made 5m total progress → permanently not stuck."
After: "robot hasn't gotten any closer in 15s → stuck."

`stuck_pct` now shows % of stuck_timeout elapsed since last progress (not since commitment),
which is the genuinely meaningful quantity.

**Observed impact**: ✅ Session 14 (2026-05-20): back-surge prevented the wall-freeze completely. Robot still ends up stuck inside the 0.75m soft inflation zone where A* cannot find a path. Fixed by Ch34.

---

## Change 34 — Dual-inflation A* (soft zone + hard wall)

**Date**: 2026-05-20  
**Files**: `path_planner.py`, `frontier_extractor.py`

**Problem**: When a frontier goal lands inside the 0.75m inflation zone, A* treats every
cell there as hard-blocked, so `nearest_free` snaps the goal to the inflation boundary.
If the corridor is also sealed by inflation, no path is ever found, and the robot freezes.

**Fix — dual inflation layers**:
- `HARD_INFLATION_M = 0.20 m` — true hard wall for A*. Only cells within 0.20m of an
  obstacle are `inf` in the cost grid. Narrow corridors remain passable.
- `INFLATION_M = 0.75 m` (soft) — cells between 0.20m and 0.75m are penalised by
  `SOFT_COST = 8.0`, so A* naturally routes around them, but can pass through when there
  is no alternative (e.g. the goal is inside the soft zone).
- `nearest_free` now snaps only against the hard boundary — a goal sitting in the soft
  zone is valid and reachable.

Changes in `path_planner.py`:
- Added `HARD_INFLATION_M = 0.20` and `SOFT_COST = 8.0` constants.
- `_astar` signature changed from `blocked: np.ndarray` (bool) to `cost_grid: np.ndarray`
  (float). `inf` = wall, `SOFT_COST` = avoidance zone, `1.0` = free. Step cost is
  `(1.414 if diagonal else 1.0) * cell_cost`.
- `plan_path` renamed parameter `inflation_m` → `soft_inflation_m`, added `hard_inflation_m`.
  Builds `cost_grid` from two `inflate_occupied` calls.

Changes in `frontier_extractor.py`:
- Imports `HARD_INFLATION_M` from `path_planner`.
- Added `_hard_inflated_grid` state, computed alongside `_inflated_grid` in `_replan`.
- Debug image: hard zone shown as dark red `(150, 30, 30)`, soft zone stays orange `(200, 100, 50)`.
- Inflated map: hard zone → 75, soft zone → 50.

**Observed impact**: ✅ Session 15 (2026-05-20): A* finds paths and robot reaches multiple goals. Two remaining issues fixed by Ch35: (1) back-and-forth oscillation between visited goals, (2) surge ramp engaging too early.

---

## Change 35 — Arrival blacklist + tighter obstacle slow-down

**Date**: 2026-05-20  
**Files**: `goal_manager.py`, `waypoint_controller.py`

**Problem 1 — back-and-forth oscillation** (session 15): After reaching a frontier, GoalManager
immediately commits to the next best candidate, which is often the other side of the same pillar.
The robot then reaches that goal and commits back. Neither goal was blacklisted because only
STUCK goals went on the blacklist.

**Fix**: Add `arrival_blacklist_duration = 20 s`. When the robot arrives (cur_dist ≤ goal_radius)
in `_pick_with_hysteresis`, append the committed goal to the blacklist with a 20s expiry before
committing to the next one. This prevents immediately re-picking a recently visited frontier.
Parameter sits alongside `blacklist_duration` (60 s, used for STUCK goals).

**Problem 2 — surge ramp starting too early**: `OBS_SLOW_DIST = 2.0 m` caused surge to begin
ramping down during normal circumnavigation (obs ~1.4–1.8 m while routing around the pillar).
The A* soft zone already steers the path away from walls; the controller ramp is last-resort
safety, not primary path shaping.

**Fix**: `OBS_SLOW_DIST` 2.0 m → 1.5 m. Ramp now only engages when genuinely close to an
obstacle, not during planned detours through tight corridors.

**Observed impact**: ✅ Session 16 (2026-05-20): arrival blacklist confirmed working (no back-and-forth). But STUCK fires 4× in 2.5 min because A* routes through soft zone → surge drops to 0.06 m/s → underwater drift → robot moves sideways rather than toward goal → GoalManager fires after 15s. Fixed by Ch37.

---

## Change 36 — Grid padding so A* can route near map boundaries

**Date**: 2026-05-20  
**Files**: `path_planner.py`

**Problem**: When a wall or obstacle touches the edge of the OccupancyGrid, A* cannot route
around it because no cells exist outside the grid bounds. The robot gets stuck with no path.

**Fix**: In `plan_path`, pad the raw grid with `PAD_CELLS = 5` layers of unknown cells (`-1`)
on all four sides before planning. Unknown cells have cost 1.0 (same as free space), so A*
can route through the padded border to go around boundary obstacles. The origin `ox/oy` is
shifted by `PAD_CELLS × resolution` to keep world coordinates consistent. The padding only
affects planning — visualization still uses the original unpadded map.

**Observed impact**: 🔲 Not yet tested.

---

## Change 37 — Three-zone A* cost grid + longer stuck timeout

**Date**: 2026-05-20  
**Files**: `path_planner.py`, `frontier_extractor.py`

**Problem** (session 16): A* routed paths through the soft zone (0.75m from walls). Underwater
drift has no friction correction, so any deviation put the robot inside the soft zone where surge
dropped to 0.06–0.12 m/s. The robot moved sideways rather than toward the goal, distance
increased, and GoalManager fired STUCK four times in 2.5 min — goal abandoned before any area
was properly explored.

**Fix 1 — Planning zone** (`path_planner.py`):
Add a third zone between the soft zone and free space:
- Free space (d > 1.5m): cost = 1.0 — A*'s first preference
- Planning zone (0.75m < d ≤ 1.5m): cost = PLAN_COST = 3.0 — moderate penalty
- Soft zone (0.20m < d ≤ 0.75m): cost = SOFT_COST = 8.0 — only if forced
- Hard zone (d ≤ 0.20m): cost = inf — never

A* now routes paths through corridors ≥1.5m from walls when available. The planning zone acts
as a buffer between the desired path and the sensor slow-down zone — even with drift, the robot
stays out of the orange soft zone on most paths.

**Fix 2 — Longer stuck timeout** (`frontier_extractor.py`):
`stuck_timeout` 15s → 30s. When a valid A* path arcs around an obstacle, the robot's distance
to the final goal temporarily increases. 15s was too short to tolerate this; 30s gives the robot
time to clear an obstacle arc before declaring it stuck.

Visualisation updated: planning zone shown as olive/khaki `(180, 160, 60)` in debug image,
value 35 in the inflated OccupancyGrid.

**Observed impact**: ✅ Session 17 (2026-05-20): three-zone planning confirmed wider paths. But rapid silent goal switches (score-based) still firing — root cause identified in Ch38.

---

## Change 38 — Remove score-based goal switching: commit-until-done model

**Date**: 2026-05-20  
**Files**: `goal_manager.py`, `frontier_extractor.py`

**Problem** (session 17): A cluster appeared 3.5m from the robot while it was heading toward a
10m goal. The silent score-based switch fired because `3.5 < 8.25 − 3.0 = 5.25m` — correct
by the old logic but wrong behaviour. A robot that has already traveled 2m toward a goal should
not abandon it because a nearby cluster appeared. The current-distance comparison made the robot
progressively *less* committed the more progress it made.

**Fix**: Remove score-based switching entirely.  A committed goal is held until:
  1. **Arrived** — robot within `goal_radius`, cluster has vanished.
     Brief arrival blacklist (20s) prevents immediate re-pick.
  2. **Stuck** — no progress toward goal for `stuck_timeout` (30s).
     Stuck blacklist (30s) prevents immediate re-pick.

`_pick_with_hysteresis` → renamed `_pick_committed`. Logic:
- Cluster still in frontier list → track map-drift of committed position.
- Cluster vanished + robot arrived → arrival blacklist + pick best.
- Cluster vanished + robot not arrived → keep heading to last known position
  (STUCK will fire if progress genuinely stalls).
- No score comparison, no hysteresis, no min_commit_duration.

Removed parameters: `switch_hysteresis`, `min_commit_duration` (dead code).

**Observed impact**: ⚠️ Session 18 (`2026-05-20_15-16-03`): goal commitment held correctly — no silent score-based switches. Robot committed to (10.40, 0.00) and stayed on it throughout. However, `STUCK_BLACKLIST` fired at t≈90s despite the robot actively following a valid A* path at full surge (0.25 m/s). Root cause: STUCK measures straight-line distance to goal; A* routed a large southward arc around a structure, temporarily increasing goal-distance from 8.19m to ~9.6m. The stuck timer ran 30s with no improvement in goal-distance → false positive. Fixed by Ch39.

---

## Change 39 — Fix false STUCK during A* detour arcs (displacement-based reset)

**Date**: 2026-05-20  
**Files**: `goal_manager.py`

**Problem** (session 18): STUCK_BLACKLIST fired while the robot was correctly following an A* path. The robot committed to goal (10.40, 0.00), made its closest straight-line approach of 8.19m, then A* routed it south along a 5-6m arc to go around a structure. During that arc goal-distance increased from 8.19m to ~9.6m over ~28s. The stuck clock (`now - _closest_t`) accumulated 30s without a reset because the stuck detector only resets on `cur_dist < _closest_dist - 0.5m`. The robot was never physically stuck — it was moving at full surge (0.25 m/s) with obs=2-4m and a valid path (path_len=22-33).

**Root cause**: STUCK was measuring progress toward the final goal (straight-line), not progress along the planned path. For any significant A* detour, those diverge — the robot gets further from the goal while actively navigating around an obstacle.

**Fix**: Add a displacement-based reset to the stuck sliding window. In addition to resetting when `cur_dist` improves by `stuck_min_progress (0.5m)`, also reset when the robot has physically moved `stuck_min_progress` from its position at the last reset. This catches detour arcs (robot moves several meters → constant resets) without false-passing a genuinely frozen robot (moves <0.3m from drift → no reset).

New state: `_closest_ref_pos` — robot position at the time the stuck reference was last set.

```python
# Goal-distance progress (existing):
if cur_dist < self._closest_dist - self.stuck_min_progress:
    self._closest_dist    = cur_dist
    self._closest_t       = now
    self._closest_ref_pos = robot_xy.copy()
# Displacement progress (new — catches A* arcs):
elif (self._closest_ref_pos is not None and
      np.hypot(robot_xy[0] - self._closest_ref_pos[0],
               robot_xy[1] - self._closest_ref_pos[1])
      >= self.stuck_min_progress):
    self._closest_t       = now
    self._closest_ref_pos = robot_xy.copy()
```

`_closest_dist` is not updated on displacement reset — it continues to track the best
goal approach ever, so a robot that drifts sideways without approaching doesn't get credit.

**Session 18 replay**: at t≈1779286591 (closest_dist=8.19m, ref_pos=(3.20,-3.89)), the robot
moved to (3.68,-4.70) — displacement 0.93m > 0.5m → timer resets. Then to (4.02,-5.55) —
another 0.72m → resets again. The arc covers ~6m total, resetting the timer ~12 times.
STUCK would never fire during active navigation.

**Observed impact**: ✅ Sessions 20+21 (`2026-05-20_15-26-54`, `2026-05-20_15-39-59`): `stuck_pct` stayed ≤40% throughout both sessions — even during the same type of southward arc (12.39m) that caused the session 18 false positive. No STUCK_BLACKLIST fired in either session (vs session 18's false positive at t≈90s). Session 21 also set the best-ever coverage record (+26,208 cells).

---

## Change 40 — Blacklist A*-unreachable goals after repeated planning failures

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`, `goal_manager.py`

**Problem**: when A* returns no path (e.g. goal is surrounded by inflated walls, or in an isolated pocket), the controller falls back to direct heading and crashes, or the robot spins in SCAN. GoalManager never learns the goal is unreachable — it keeps re-selecting the same blacklisted-on-stuck goal after the blacklist expires, or a newly spawned equivalent cluster.

**Fix**: count consecutive A* failures per goal in the extractor. After `REPLAN_FAIL_MAX` failures (= 2 s at REPLAN_HZ=3), call `GoalManager.mark_unreachable()` which:
- Adds the position to the blacklist with `blacklist_duration` (30 s)
- Clears `_committed` so `select()` immediately picks a different frontier

The fail counter resets when the goal changes by >1m or when A* finds a path. This tolerates 1–2 transient failures (back-surge temporarily puts the start inside the inflated zone, sparse map at startup).

**New state** (`frontier_extractor.py`): `_astar_fail_count`, `_astar_fail_goal`  
**New method** (`goal_manager.py`): `mark_unreachable(goal_xy, now)`

**Observed impact**: 🔲 Not yet tested.

---

## Change 41 — REPLAN_HZ: 1 Hz → 3 Hz + expose as class constant

**Date**: 2026-05-20  
**Files**: `frontier_extractor.py`

**Objective**: the previous hardcoded `create_timer(1.0, self._replan)` gave A* replanning a 1-second cadence — slow compared to the controller's 10 Hz loop. At 0.25 m/s the robot travels 0.25m per replan cycle; at 3 Hz it travels ≈0.08m, giving the path much tighter tracking. Also exposes `REPLAN_HZ = 3.0` as a class constant (like `UPDATE_HZ`) so it is easy to find and change. `REPLAN_FAIL_MAX` adjusted from 5 to 6 (still ≈2 s at the new rate).

**Observed impact**: 🔲 Not yet tested.

---

## Change 42 — Code cleanup: redundant inflations, docstring fixes, side-effect removal

**Date**: 2026-05-21  
**Files**: `path_planner.py`, `frontier_extractor.py`, `frontier_detection.py`,
`goal_manager.py`, `control_utils.py`

**Objective**: No behavioural changes — quality pass to remove accumulated technical debt.

**`path_planner.py`** — refactored to expose `CostGrid` dataclass + `build_cost_grid()` +
`find_path()`. The three inflation passes and cost grid are now computed once per replan tick
(inside `build_cost_grid`) and the result is reused by both the planner and the visualisation
code. Eliminated the redundant triple inflation that was happening separately in `_replan` and
`_publish_inflated_map`. Updated module docstring to "Three-zone inflation strategy".

**`frontier_extractor.py`** — replaced `_inflated_grid`, `_hard_inflated_grid`,
`_plan_inflated_grid` state attributes (and the inline inflation in `_replan`) with a single
`self._cg: CostGrid | None`. `_publish_inflated_map` and `_publish_debug_image` now read from
`self._cg.{soft_zone,hard_blocked,plan_zone}[PAD_CELLS:-PAD_CELLS, PAD_CELLS:-PAD_CELLS]` and
`self._cg.raw` instead of re-parsing `self._map.data` each tick.

**`frontier_detection.py`** — removed redundant `iterations=1` keyword from
`binary_dilation(unknown, iterations=1)` (default is already 1; passing it explicitly was
misleading noise).

**`goal_manager.py`** — moved blacklist expiry pruning out of `is_blacklisted` (a query
method that had a hidden side-effect) into `select()` (called once per cycle). `is_blacklisted`
signature simplified to `(wx, wy)` — no longer needs `now`.

**`control_utils.py`** — corrected module docstring: was "positive heave = upward in NED"
which contradicts the actual controller formula `heave = -KP * depth_err` (negative = upward
thrust in Stonefish). Now states "negative = upward thrust in Stonefish" consistently with
`waypoint_controller.py`'s `_heave_cmd` docstring.

**Observed impact**: 🔲 No sessions run yet (pure refactor — no algorithmic changes).

---

## Change 43 — Frontier definition: occupied↔unknown instead of free↔unknown + MIN_CLUSTER_CELLS=1

**Date**: 2026-05-21  
**Files**: `frontier_detection.py`, `frontier_extractor.py`

**Objective**: Free↔unknown frontiers are unstable targets in a sonar-based system: the boundary
recedes as the robot approaches because the sonar fills in cells ahead as free. The robot commits
to a goal that no longer exists by the time it arrives.

Occupied↔unknown frontiers are stable: the wall surface is fixed, and navigating near it lets
the sonar illuminate the unknown space on the other side from a new angle (around corners, behind
pillars, etc.). These are genuine "something to investigate" targets rather than "edge of what
was scanned so far" targets.

**Changes in `frontier_detection.py`**:
- `frontier = free & binary_dilation(unknown)` → `frontier = occupied & binary_dilation(unknown)`
- Module docstring updated to reflect the new definition.

**Changes in `frontier_extractor.py`**:
- `MIN_CLUSTER_CELLS = 5` → `MIN_CLUSTER_CELLS = 1`. Wall-surface clusters tend to be small
  (a single isolated pillar corner is 1–3 cells), so requiring 5 would filter most of them out.

**Navigation behaviour**: Cluster centroids now fall on occupied cells. A* snaps goal cells to
the nearest free cell via `nearest_free`, so the robot navigates to the closest accessible point
near the wall surface rather than into it. The existing inflation zones keep the planned path at
a safe distance.

**Observed impact**: 🔲 Not yet tested.

---

## Change 44 — FutureWork.md: fill gaps identified vs literature

**Date**: 2026-05-21  
**Files**: `FutureWork.md`

**Objective**: Three gaps relative to the active-SLAM literature were not captured:

1. **§5 (3-D frontiers)**: Added note on TSDF/ESDF (voxblox/nvblox) as an alternative
   volumetric representation — provides surface gradients and signed distance as first-class
   fields, which removes the three-zone inflation pass and makes standoff selection trivial.

2. **§6 (Active SLAM)**: Expanded the NextBestView description to name the
   unmapped-volume-in-frustum formulation (ray-cast candidate viewpoints, count unseen
   voxels, divide by path cost) and note that it removes the need for the oscillation
   band-aids (stuck detection, blacklisting).

3. **§7 (Evaluation) — new section**: Added four concrete metrics for comparing baseline
   vs improved policy: coverage % against a reference OctoMap, ATE (requires §4 first),
   path efficiency (distance / coverage), and stuck-event count (already logged).

**Observed impact**: 🔲 Documentation only.

---

## Change 45 — Wire vertical_fov through stonefish DepthCamera; update sensor config to WaterLinked Sonar 3D-15 specs

**Date**: 2026-05-22  
**Files**:
- `stonefish/Library/include/sensors/vision/DepthCamera.h`
- `stonefish/Library/src/sensors/vision/DepthCamera.cpp`
- `stonefish/Library/src/core/ScenarioParser.cpp`
- `slam_ws/src/stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp`
- `slam_ws/src/world/data/robot/simple_rov.scn`

**Objective**: `vertical_fov` in `.scn` files was silently ignored for `depthcamera` sensors
(ScenarioParser never read it, DepthCamera constructor never accepted it). The actual vFOV
was derived from the resolution aspect ratio, so the config value was misleading. Additionally,
the published `CameraInfo` intrinsics (`fy`) were also computed from aspect ratio, not the
true vFOV.

**Fix**:
1. `ScenarioParser`: reads optional `vertical_fov` attribute for `depthcamera` and passes it to the constructor.
2. `DepthCamera`: added `vFOVDeg` parameter (default -1 = aspect-ratio fallback), stores as `fovV` (`Scalar`), passes through to `OpenGLDepthCamera`.
3. `ROS2Interface::GenerateCameraMsgPrototypes`: when cam is a `DepthCamera` with explicit vFOV, uses `tan(vfov/2)` directly for `fy` instead of the aspect-ratio approximation.
4. `simple_rov.scn`: updated sensor specs to match WaterLinked Sonar 3D-15 datasheet (low-frequency mode) — `resolution_x` 128→257 (90°/0.35°), `resolution_y` 32→67 (40°/0.60°), `vertical_fov="40.0"` now takes effect.

**Observed impact**: 🔲 Requires stonefish rebuild + colcon rebuild to take effect.

---

## Change 46 — Extract visualisation into FrontierVisualizer

**Date**: 2026-05-22  
**Files**:
- `slam_ws/src/frontier_slam/frontier_slam/visualizer.py` (new)
- `slam_ws/src/frontier_slam/frontier_slam/frontier_extractor.py`

**Objective**: `frontier_extractor.py` was doing six distinct jobs. Extract the three
visualisation concerns (RViz markers, inflated-map overlay, debug image) into a dedicated
`FrontierVisualizer` class.

**Fix**:
- New `visualizer.py`: `FrontierVisualizer` class owns the three publishers and exposes
  `publish_markers()`, `publish_inflated_map()`, `publish_debug_image()`. Module-level
  helpers `_sphere`, `_draw_line`, `_arrow_color` moved here.
- `frontier_extractor.py`: instantiates `FrontierVisualizer(self)`, delegates all
  rendering calls to it. File shrinks from 445 → 240 lines. Removed imports:
  `Duration`, `Point`, `Image`, `ColorRGBA`, `Marker`, `MarkerArray`, `PAD_CELLS`.

**Observed impact**: 🔲 Not yet run in a session.

---

## Change 47 — Align RViz/debug colours with the poster legend

**Date**: 2026-05-28  
**Files**:
- `slam_ws/src/frontier_slam/frontier_slam/visualizer.py`
- `slam_ws/src/frontier_slam/frontier.rviz`

**Objective**: make the live RViz view and the debug image use the same colour palette
as the IEEE-OES poster's shared legend, so the poster schematic and the screenshot read
as one visual language.

**Palette** (single source of truth, defined as `C_*` constants at the top of `visualizer.py`):
- Occupied `16,65,158` blue · Frontier `0,168,193` teal-cyan · Path `38,174,96` green ·
  Goal `245,194,40` amber · Robot `240,124,32` orange · Free near-white · Unknown light blue.

**Changed** (`visualizer.py`):
- Frontier spheres `(0,1,1)` → `C_FRONTIER` (teal-cyan); goal sphere `(1,0,0)` red → `C_GOAL` (amber).
- Debug image: occupied → blue, free → near-white, unknown → light blue, path → green, goal → amber.

**Removed** (`visualizer.py`):
- The `goal_arrow` `Marker.ARROW` block (the stray orange arrow) and its now-unused
  `geometry_msgs/Point` import. Frontier/goal direction is conveyed by the markers alone.

**Changed** (`frontier.rviz`):
- Odometry (robot, `/StoneFish/Odometry`) arrow `46,194,126` → `240,124,32` orange.
- Pose (`/goal_pose`) `255,25,0` red → `245,194,40` amber. Path `25,255,0` → `38,174,96` green.
- Dropped the dead `goal_arrow` namespace from the Frontiers MarkerArray display.
- **Flat-blue occupied voxels**: the `OctomapPoints` PointCloud2 (`/octomap_point_cloud_centers`)
  switched to `Color Transformer: FlatColor`, `Color 16,65,158`, `Style: Boxes`, `Size 0.1 m`
  (= octomap resolution). The two `octomap_rviz_plugins/OccupancyGrid` displays are **disabled**:
  a plain `OcTree` has no per-voxel colour, so their "Cell Color" mode errors with
  "can't extract cell color". Re-enable only with a ColorOcTree.

**Note**: Background left at the RViz default dark `48,48,48`.

**Observed impact**: 🔲 Requires `colcon build` + reloading `frontier.rviz` to take effect.

---

## Change 48 — Frontier markers: cyan arrows pointing into the unknown

**Date**: 2026-05-28  
**Files**:
- `slam_ws/src/frontier_slam/frontier_slam/frontier_detection.py`
- `slam_ws/src/frontier_slam/frontier_slam/visualizer.py`

**Objective**: replace the frontier **spheres** in RViz with **cyan arrows** that point toward
the unknown space each frontier borders (matches the poster legend/schematic).

**Changed** (`frontier_detection.py`):
- `Cluster` gains `dx, dy` — a unit vector toward the unknown, computed per cluster as
  (centroid of bordering unknown cells − cluster centroid), normalised. `col→world x`,
  `row→world y` (same mapping as `wx/wy`).

**Changed** (`visualizer.py`):
- New `_arrow(...)` helper (ARROW marker from a point along `(dx,dy)` for `length` m).
- `publish_markers` now emits frontier arrows (`C_FRONTIER` cyan, 0.8 m) instead of spheres;
  goal stays an amber sphere. Re-added the `geometry_msgs/Point` import.

**Observed impact**: 🔲 Not yet run; needs the node relaunched (symlink-install, no rebuild).

---

## Change 49 — Repo migration: slam_ws → ActiveSlam monorepo

**Date**: 2026-07-02
**Files**: repo-wide (path changes only, no logic changes)

**Objective**: consolidate five independent git repos
(`frontier_slam`, `stonefish_groundtruth_mapping`, `stonefish_ros2`, `launch_tools`, `world`,
previously under `slam_ws/src/`) into one public monorepo, `ActiveSlam`, ahead of the
dissertation deadline. `ActiveSlam` is now the main repository; the colcon workspace lives
separately at `~/delivery/ros_ws` (standard ROS2 convention — repo holds packages, workspace
is a disposable local wrapper, never committed).

**Changed**:
- `frontier_slam` → `ActiveSlam/planner/frontier_slam/` (git history preserved, manual
  merge-subtree recipe — `git subtree` porcelain isn't installed on this machine, so used
  `git fetch` + `git merge -s ours --no-commit --allow-unrelated-histories` +
  `git read-tree --prefix=... -u` + `git commit` instead).
- `stonefish_groundtruth_mapping` → `ActiveSlam/slam/stonefish_groundtruth_mapping/` (history
  preserved, same recipe). `STONEFISH_WORLD_DIR` default in `step1_tf.launch.py` updated to
  the new `sim/world` path.
- `stonefish_ros2` (patched fork, `dev` branch) → `ActiveSlam/sim/stonefish_ros2/` (history
  preserved).
- `launch_tools` → `ActiveSlam/tools/launch_tools/` (history preserved). Untracked a
  previously-committed `__pycache__/keyboard_control.cpython-312.pyc`; added `.gitignore`.
- `world` → `ActiveSlam/sim/world/` — **copied as plain files, history NOT preserved**. The
  source repo's `.git` is 85 MB packed / ~313 MB of mesh data on disk (`shipwreck.obj` alone
  is 289 MB); importing its full history would permanently bake those blobs into
  `ActiveSlam`'s `.git`, well past GitHub's 100 MB single-file limit, even after gitignoring
  the directory going forward. Only the 6 small tracked files (`.scn` configs, one texture,
  `run.sh`) were copied and committed; `data/obj/*.obj|.mtl` are gitignored but physically
  present on disk (needed to run the sim). See `sim/world/data/README.md`.
- Stonefish's 3 patch commits (vertical-FOV depth camera, physics-thread capture timestamps)
  exported as `.patch` files in `sim/stonefish_patches/`, with upstream URL + base SHA
  (`09208f9`, `patrykcieslak/stonefish`) documented — the patched fork itself stays outside
  the repo (~250 MB C++ checkout).
- `ROADMAP.md` copied (not moved) from `ActiveSlam-Resources` into `docs/ROADMAP.md` — the
  repo's own living copy going forward; the source is untouched.
- New root `README.md`: project framing, package table, install, current manual 3-step run
  sequence (no unified demo launch file yet — that's Sprint 1).
- Fixed stale content while migrating: `frontier_slam/README.md`'s sensor table still
  described the pre-Ch45 fallback (128×32, ~22.5° vFOV from aspect ratio) instead of the
  wired-through WaterLinked 3D-15 spec (257×67, 40° vFOV); its build instructions pointed at
  `slam_ws`; its citation had wrong author names/venue detail for the Suresh et al. paper
  (corrected against the actual PDF: Suresh, Sodhi, Mangelson, Wettergreen, Kaess, ICRA 2020,
  Paris).
- Root private directory-map/index file written at `~/delivery/` — build/run quickstart +
  deadlines + doc conventions (kept outside the public repo).

**Not yet done** (deferred — see the private planning notes):
- Deleting `stonefish_original` (257 MB redundant pristine copy) and the old `slam_ws`
  workspace (kept until the new `ros_ws` build + a full sim run are verified).
- Removing the 4 placeholder `content.txt` files left over from the original `ActiveSlam`
  skeleton (`eval/`, `planner/`, `sim/`, `slam/`).
- `colcon build` verification from `ros_ws` and the fresh-clone install test (Gates 1–2).

**Observed impact**: 🔲 Not yet built or run from the new workspace layout — Gate 1 (colcon
build + manual 3-step launch + RViz map) is the next step before anything here is considered
verified.

---

## Change 50 — Gate 1: build + end-to-end pipeline verification from ros_ws

**Date**: 2026-07-02
**Files**: none (verification only, one doc fix below)

**Objective**: confirm the Change 49 migration didn't break anything before deleting
`slam_ws`/`stonefish_original` — `colcon build` from the new workspace, then exercise the
actual sense→map loop rather than just checking the code compiles.

**Verified** (inside the `ros2-jazzy` distrobox container):
- `colcon build --symlink-install` from `~/delivery/ros_ws`: 4/4 packages, clean (only
  benign C++17/C++14 ABI-change compiler notes from `stonefish_ros2`, no errors).
- `stonefish_simulator` loads all meshes from the new `sim/world/data/obj/` path
  successfully (confirms the plain-copy, non-history-preserved import for `world` didn't
  lose or corrupt anything).
- `/cloud_in` publishes real depth data at ~5 Hz (matches sensor spec).
- Direct thruster commands (`ros2 topic pub` to `/bluerov2/controller/thruster_setpoints_sim`,
  bypassing the teleop node to test the sim interface directly) produce real robot motion —
  confirmed via a proper `rclpy` subscriber sampling `/StoneFish/Odometry` over 3 s (position
  changed measurably), not a single `ros2 topic echo` snapshot, which turned out to be
  unreliable here (dropped/stale messages, see below).
- `/projected_map` grows with movement: 2368/5698 known cells before, 7031/15748 after ~15 s
  of forward motion.
- `ros2 launch frontier_slam frontier_slam.launch.py` produces valid, sane session CSVs
  (position, cluster counts climbing 132→133→137) in the new `planner/frontier_slam/logs/`
  location.

**Found and fixed**: the `step1_tf` → `step2_pointcloud` → `step3_octomap` launch files each
`IncludeLaunchDescription` the previous one (step3 = step1 + step2 + octomap_server), so
they **cascade** — running all three in separate terminals (as the README instructed) spawns
three independent `stonefish_simulator` processes all publishing on the same topics. Docs
fixed to say "run only the one level you need" (README.md, the private root doc index).

**Not verified**: actual RViz rendering — no visual/display channel available in this
session to confirm the map/frontier markers render correctly, only that the underlying
topics carry correct data. Antoine should eyeball RViz once before trusting this for the
demo.

**Observed impact**: ✅ Gate 1 passed. Cleared to proceed with Gate 2 (fresh-clone install
test) and, once that passes, delete `slam_ws` / `stonefish_original`.

---

## Change 51 — bringup package: unified demo.launch.py

**Date**: 2026-07-02
**Files**: new `bringup/` package; renamed `stonefish_groundtruth_mapping/launch/step{1,2,3}_*`
→ `tf`/`pointcloud`/`octomap.launch.py`; new `tsdf.launch.py`; `frontier_slam.launch.py`
docstring path fix.

**Objective**: one command to bring up the flagship demo — Stonefish + TF + point cloud +
mapper + operator mode + RViz — with `mode:=teleop|frontier` and `mapper:=octomap|tsdf` as
independent switches, per the Sprint 1 plan.

**Changed**:
- Flattened `step1_tf`/`step2_pointcloud`/`step3_octomap` → `tf`/`pointcloud`/`octomap.launch.py`
  (same cascade structure, just named for what they do instead of their position in a
  numbered sequence). Added a parallel `tsdf.launch.py` (pointcloud + `tsdf_mapper` instead of
  `octomap_server`).
- New `bringup` package: `demo.launch.py` (`mode`, `mapper`, `rviz` launch arguments) +
  `demo.rviz` (extends `frontier_slam/frontier.rviz` with two TSDF displays — PointCloud2 on
  `/tsdf/surface_cloud`, MarkerArray on `/tsdf/voxels` — both disabled by default since
  `octomap` is the default mapper).
- **`mode:=teleop` cannot bundle `keyboard_control` as a launch `Node` action** —
  `termios.tcgetattr(sys.stdin)` needs a real TTY, which `ros2 launch` doesn't give child
  processes (confirmed: `termios.error: (25, 'Inappropriate ioctl for device')`, process dies
  immediately). Standard ROS2 limitation, not fixable from this side. `mode:=teleop` instead
  logs a reminder to run `ros2 run launch_tools my_keyboard` in another terminal.

**Verified** (each run cleanly, all expected nodes up, no errors beyond the known benign
Stonefish shader-link warnings):
- `demo.launch.py` (defaults: teleop + octomap) — sim + mapper come up, teleop reminder logs.
- `demo.launch.py mode:=frontier` — `frontier_extractor` + `waypoint_controller` join cleanly.
- `demo.launch.py mapper:=tsdf` — **first ever end-to-end run of `tsdf_mapper.py`** (untracked
  WIP until Change 49): integrates real point clouds (12–13k pts/scan), publishes a surface
  (3743 pts) and voxel view (6909 markers). One transient `TF unavailable
  (ExtrapolationException)` warning at cold start, then normal — not a bug, just the TF buffer
  not having data yet on the very first callback.

**Found and fixed along the way**: forgot `static_transform_publisher` and
`component_container` (depth_proc) processes in the Gate 1 cleanup — only killed
`stonefish_simulator`/`odom_tf_sync`/`octomap_server`, leaving 3 orphaned processes each from
the earlier step1/2/3 testing. Cleaned up; `ros2 daemon stop && ros2 daemon start` was needed
once to clear stale discovery-graph entries for already-dead nodes.

**Observed impact**: ✅ `demo.launch.py` is the new documented entry point (README and the
private root doc index updated). Not yet verified visually: `rviz:=true` confirmed to start without logging errors and
the underlying topics carry correct data, but not yet eyeballed on screen.

---

## Change 52 — Fix world-path resolution; diagnose RViz crash on Apple Silicon

**Date**: 2026-07-02
**Files**: new `sim/world/{package.xml,setup.py,resource/world}`; `slam/stonefish_groundtruth_mapping/launch/tf.launch.py`; `slam/stonefish_groundtruth_mapping/package.xml`.

**Objective**: Antoine ran `colcon build` (no `--symlink-install` mentioned) + `ros2 launch
bringup demo.launch.py` in his own terminal and hit two real bugs neither B0a's testing nor
Gate 1 caught, since both always used `rviz:=false` / a workspace built once, freshly, in one
shot.

**Bug 1 — Stonefish "structures didn't load" (real, now fixed):**
```
Scenario parser: Loading scenario from '.../install/stonefish_groundtruth_mapping/sim/world/scnenario/waterlinked.scn'.
Scenario parser: File not found!
```
Root cause: Change 51's `tf.launch.py` computed its `STONEFISH_WORLD_DIR` default by walking
up from `os.path.realpath(__file__)`, assuming colcon's `--symlink-install` always makes the
installed launch file a symlink back to source. Checking his (shared) install directory
showed why that's false: `step1_tf.launch.py` etc. survived as **stale orphaned symlinks**
pointing at `build/` paths from *before* the Ch51 rename (colcon doesn't prune removed
`data_files` entries — an incremental rebuild after renaming files just adds the new ones
alongside the old), while the *renamed* files (`tf.launch.py` etc.) were installed as **plain
copies**, not symlinks. `realpath()` on a plain copy resolves to itself, at the install path —
computing a `sim/world` location that only exists relative to the *source* tree.

Fixed by not depending on symlink-vs-copy behavior at all: `sim/world` is now a minimal
`ament_python` package (`package.xml` + `setup.py` with glob-based `data_files`, since the
~313 MB mesh data is gitignored and may or may not be present at build time) so `tf.launch.py`
resolves it via `get_package_share_directory('world')` instead — the mechanism
`ament_index` exists for, reliable regardless of how colcon happens to install a given file.

**Verified**: fresh local clone (`git clone` of the local repo, not yet pushed) into a
scratch workspace, mesh data copied in per the README's documented manual step, clean
`colcon build --symlink-install` — 6/6 packages (previously 5, `world` wasn't discovered
before it had a `package.xml`). Scenario loads and meshes load correctly, no more
"File not found!".

**Operational gotcha this surfaces**: renaming files in a `data_files` list needs a **clean**
rebuild (`rm -rf build install`, not just `colcon build` again) to avoid stale orphaned
installs. Worth remembering for any future launch-file reorganisation.

**Bug 2 — RViz crash (FIXED: `QT_QPA_PLATFORM=xcb` not actually set in Antoine's shell):**
```
rviz::RenderSystem: error creating render window: RenderingAPIException: Invalid parentWindowHandle
Unable to create the rendering window after 100 tries
terminate called after throwing an instance of 'std::runtime_error'
```
Renderer is zink Vulkan over Apple's M1 Max GPU via the Asahi "Honeykrisp" driver (no native
Mesa OpenGL on Apple Silicon) — initial diagnostic inside a `distrobox enter` session showed
`QT_QPA_PLATFORM=xcb` already set and assumed it was correctly forcing X11 in the real launch
too, but that was the distrobox environment, not the interactive shell — it wasn't set there. **Fix confirmed working**: `export QT_QPA_PLATFORM=xcb` added to
`~/.zshrc`. With it set, `demo.launch.py` (teleop + octomap, `rviz:=true`) runs correctly —
Stonefish, RViz, and the octomap build all confirmed working by Antoine directly.

**Observed impact**: ✅ Both bugs fixed and confirmed working end to end.

## Change 53 — Fix rosdep resolution: drop bogus `ament_python` buildtool dep, correct `pcl` key

**Objective**: Make `rosdep install --from-paths src -i -y` exit cleanly instead of erroring on unresolvable keys.

**What changed**:
- Removed `<buildtool_depend>ament_python</buildtool_depend>` from `launch_tools`, `frontier_slam`, `stonefish_groundtruth_mapping`, and `world` package.xml files. The build type is already declared in the `<export>` block (which is what colcon reads); the `<buildtool_depend>` was dead metadata with no rosdep key. `bringup` already omitted it.
- Changed `<depend>pcl</depend>` → `<depend>libpcl-all-dev</depend>` in `stonefish_ros2/package.xml` — `pcl` is not a valid rosdep key.
- Removed the `pip install -r requirements.txt` step from README.md and INSTALL.md — `python3-numpy` and `python3-scipy` are proper rosdep keys already declared in `frontier_slam/package.xml`, and the system packages (numpy 1.26.4, scipy 1.11.4) satisfy the version constraints. The pip step was redundant.
- Removed the "expect rosdep to error" section and PEP 668 workaround from INSTALL.md since rosdep now resolves cleanly.

**Observed impact**: ✅ `rosdep install --from-paths src -i -y` completes with "All required rosdeps installed successfully".

## Change 54 — Suppress build noise: CMake dev warnings and aarch64 psABI notes

**Objective**: Get a clean `colcon build` with no stderr, after Change 53 fixed rosdep resolution.

**What changed**:
- `stonefish_ros2/CMakeLists.txt`: added `-Wno-psabi` alongside the existing `-Wall -Wextra -Wpedantic` for GCC. Suppresses GCC 10+'s aarch64-specific ABI notes for `std::pair<float,float>`/`std::pair<double,double>` parameter passing, triggered by Stonefish's own headers (`SimulationManager.h`, `ActuatorDynamics.h`, `Robot.h`). Purely informational (`note:`, not `warning:`) — no behavior change since all consumers share this toolchain.
- README.md / INSTALL.md: `colcon build --symlink-install` → `colcon build --symlink-install --cmake-args -Wno-dev`, silencing CMake dev-mode warnings (`CMP0144`/`CMP0074`) that PCL's own cmake modules trigger — noise from upstream PCL, not this project.

**Observed impact**: ✅ `colcon build --symlink-install --cmake-args -Wno-dev` now produces zero stderr across all 6 packages (verified via isolated rebuild of `stonefish_ros2`, the only package that previously had output).

## Change 55 — Implement Wall-normal tracking motion controller

**Date**: 2026-07-03
**Files**: `frontier_slam/control_utils.py`, `frontier_slam/wall_follower.py`, `launch/wall_follow.launch.py`, `bringup/launch/demo.launch.py`, `setup.py`

**Objective**: Enable the ROV to lock onto the nearest TSDF surface and strafe along it at a fixed standoff distance.

**What changed**:
- Added `sway` command support to `control_utils.py` and modified `mix_thrusters` to map it to the 6-thruster layout (strafing sideway using the 4 horizontal thrusters).
- Created `wall_follower.py`, a new node that listens to `/tsdf/surface_normals_cloud` (published by `tsdf_mapper.py`), computes distance to the nearest wall, and aligns the ROV heading perpendicular to the wall normal while strafing.
- Created `wall_follow.launch.py` to start the wall follower node.
- Updated `demo.launch.py` to introduce the `motion` launch argument. Users can run `ros2 launch bringup demo.launch.py motion:=wallfollow mapper:=tsdf` to engage the wall follower. (Using `motion` distinct from the operator `mode` argument).
- Updated `setup.py` to register the new node and launch file.
- Added closed-loop kinematic tests verifying the controller's convergence.

**Observed impact**: ✅ The WallFollower successfully tracks the wall at a set standoff distance while orienting towards it and maintaining continuous movement along the tangent.

## Change 56 — Filter /tsdf/voxels to confidently-solid, well-observed voxels only

**Date**: 2026-07-05
**Files**: `tsdf_mapper.py`

**Objective**: The `/tsdf/voxels` `MarkerArray` scaled each cube's *size* by its
observation-count weight (log-scale, 10 buckets, 0.1x-1.0x voxel size) to convey
confidence. Differently-sized cubes at adjacent grid cells don't tile cleanly,
which is what made the map look like everything was overlapping. Also, the
only filter on what got shown was `d <= surface_thresh_m` (0.15 m) with
`min_weight` hardcoded to 1.0 — practically no filtering at all, so
low-confidence and single-observation voxels cluttered the view alongside
real structure.

**What changed**: Cube size is now fixed at the true grid resolution
(`voxel_size`) for every voxel; the size channel is retired. Weight/confidence
now drives *color* instead (log-scaled, orange = just past the observation
floor, green = heavily observed) via a new `_confidence_colormap()`, replacing
the old occupied/surface/free `_tsdf_colormap()` (which is dropped — with the
new solid-only filter below, every remaining voxel is already known-occupied,
so an occupied/free color axis has nothing left to distinguish). Two new
parameters gate what's shown at all:
- `voxel_min_weight` (default 10.0): hides voxels observed fewer times.
- `voxel_min_solid_confidence` (default 0.95): using VDBFusion's TSDF
  convention (`d=0` surface = maximally ambiguous, `d=-trunc` = fully
  saturated occupied), `solid_confidence = (trunc - d) / (2*trunc)` — a
  voxel must clear 0.95 (equivalently `d <= -0.9*trunc`) to be shown at all.
  Both are real `declare_parameter`s, tunable from a launch file without a
  rebuild if the default turns out too strict/loose for a given voxel/trunc
  size pairing.

**Observed impact**: ✅ Builds clean, live-verified against the real Stonefish
sim (`demo.launch.py mode:=frontier mapper:=tsdf`, 40 s, zero exceptions):
voxel count grows steadily as more of the scene gets confidently observed
(307 → 741 → 909 → 1010 over ~24 s), confirming the stricter filter still
passes real voxels rather than filtering the map down to nothing.

## Change 57 — Fix /tsdf/voxels draw-order artifact (render opaque)

**Date**: 2026-07-05
**Files**: `tsdf_mapper.py`

**Objective**: Change 56 fixed the size-mismatch overlap but voxels still
rendered in visibly wrong front/back order. Root cause: `_confidence_colormap`
published alpha=0.85 per cube; any marker alpha < 1 puts the whole CUBE_LIST
into OGRE's transparent render queue, which draws in insertion order (i.e.
VDB grid iteration order) rather than depth order. With thousands of cubes in
one marker, overlapping ones then render in whatever order they happened to
be added, not the order the camera would actually see them in.

**What changed**: `_confidence_colormap` now returns alpha=1.0 (fully
opaque) for every voxel. Opaque geometry is resolved per-pixel by the GPU's
depth buffer regardless of draw order, so occlusion is always correct.
`octomap_rviz_plugins` already does exactly this: its "Occupied Voxels" mode
uses `Voxel Alpha: 1`, while its "Free Voxels" haze mode uses `~0.01` (low
enough that the same sorting glitch is imperceptible) — confirmed by
inspecting `bringup/rviz/demo.rviz`'s two `OccupancyGrid` display configs.
Since this view is already gated to confidently-solid, well-observed voxels
(Change 56), there was no real transparency information left to lose.

**Observed impact**: ✅ Builds clean. Not yet re-verified visually in RViz on
this machine (headless verification only, per Change 56) — worth eyeballing
once RViz is run interactively.

## Change 58 — Retune voxel_min_solid_confidence 0.95 → 0.80

**Date**: 2026-07-05
**Files**: `tsdf_mapper.py`

**Objective**: Change 56's 0.95 default (`d <= -0.9*trunc`) is very strict at
the default `trunc=0.6`, showing only voxels extremely close to fully
saturated — after eyeballing the result, loosen the threshold to show more
of the confidently-observed surface.

**What changed**: `voxel_min_solid_confidence` default 0.95 → 0.80, which
loosens the effective cutoff to `d <= -0.6*trunc` (was `-0.9*trunc`) — more
near-surface voxels now clear the bar and get displayed. `_voxel_max_d`
(derived from this parameter) updates automatically; no other logic changed.
Supersedes the 0.95/-0.9·trunc numbers quoted in Change 56.

**Observed impact**: ✅ Builds clean, live-verified against the real sim
(`mapper:=tsdf`, 45 s): voxel/surface counts grow steadily as before
(voxels 2130 → 4243 → 6651 → 6896, surface points climbing to 19598),
confirming the looser threshold still passes real, growing voxel counts
rather than flooding the map with noise.
