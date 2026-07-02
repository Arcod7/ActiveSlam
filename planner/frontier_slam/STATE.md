# frontier_slam — Current State

Mutable snapshot. Overwrite, never append. Last updated: 2026-05-22.

Change log → `Progress.md` | Session findings → `Sessions.md`

---

## Parameters

### frontier_extractor.py / goal_manager.py
| Parameter | Value | Purpose |
|---|---|---|
| `MIN_CLUSTER_CELLS` | 1 | Discard tiny frontier clusters (noise); lowered for small wall-surface clusters (Ch43) |
| `MIN_EXPLORE_DIST` | 3.0 m | Skip frontiers the robot is already at |
| `GOAL_VANISH_DIST` | 3.0 m | Committed goal "gone" if no cluster within this radius |
| `GOAL_RADIUS` | 2.0 m | Robot "arrived" threshold — GoalManager switches when cur_dist ≤ this (Ch28) |
| `STUCK_TIMEOUT` | 30.0 s | Abandon goal after this long with no progress (Ch37, was 15s) |
| `STUCK_MIN_PROGRESS` | 0.5 m | Minimum distance closed to not be "stuck" |
| `BLACKLIST_DURATION` | 30.0 s | Blacklist duration after STUCK or A*-unreachable |
| `ARRIVAL_BLACKLIST_DURATION` | 20.0 s | Blacklist duration after goal reached — prevents re-picking (Ch35) |
| `UPDATE_HZ` | 0.5 Hz | Frontier re-evaluation rate |
| `REPLAN_HZ` | 3.0 Hz | A* path replanning rate (Ch41, was 1 Hz) |
| `REPLAN_FAIL_MAX` | 6 | Consecutive A* failures before blacklisting goal (Ch40; ≈2 s at 3 Hz) |

### waypoint_controller.py
| Parameter | Value | Purpose |
|---|---|---|
| `KP_YAW` | 0.07 | Heading P-gain |
| `KP_SURGE` | 0.35 | Forward speed P-gain |
| `KP_HEAVE` | 0.40 | Depth-hold P-gain |
| `MAX_SURGE` | 0.35 | Surge clamp (Ch32) |
| `GOAL_RADIUS` | 2.0 m | "Goal reached" threshold |
| `GOAL_REACHED_TIMEOUT` | 10.0 s | Clear stale goal and enter scan if no new goal arrives |
| `SCAN_YAW` | 0.08 | Rotation speed during scan / initial scan |
| `INIT_SCAN_DURATION` | 10.0 s | Startup spin duration before navigating |
| `WAYPOINT_ADVANCE_DIST` | 1.5 m | Advance to next path waypoint when this close (Ch26) |
| `OBS_SLOW_DIST` | 1.5 m | Begin linear surge ramp-down at this distance (Ch35, was 2.0m) |
| `EMERGENCY_STOP_DIST` | 0.4 m | Below this: switch from ramp to back-surge (Ch33) |
| `BACK_SURGE_SPEED` | 0.12 m/s | Backward speed when obstacle inside EMERGENCY_STOP_DIST (Ch33) |
| `ESCAPE_YAW` | 0.20 | Spin rate for CTRL_STUCK escape |
| `ESCAPE_DURATION` | 4.0 s | How long to spin after CTRL_STUCK trigger |
| `STUCK_SURGE_MIN` | 0.15 | Min surge to activate position-stuck detection |
| `STUCK_WINDOW` | 5.0 s | Position-stuck detection window |
| `STUCK_MOVE_MIN` | 0.25 m | Min movement expected in STUCK_WINDOW |
| `CTRL_HZ` | 10.0 Hz | Control loop rate |

### path_planner.py
| Parameter | Value | Purpose |
|---|---|---|
| `HARD_INFLATION_M` | 0.20 m | Hard A* wall — never traversed (Ch34) |
| `INFLATION_M` | 0.75 m | Soft zone — last-resort passage, cost×8 (Ch34) |
| `PLAN_INFLATION_M` | 1.50 m | Planning margin — A* prefers staying outside, cost×3 (Ch37) |
| `SOFT_COST` | 8.0 | Cost multiplier inside soft zone (Ch34) |
| `PLAN_COST` | 3.0 | Cost multiplier inside planning zone (Ch37) |
| `PAD_CELLS` | 5 cells | Unknown-cell border added around the map before A* (Ch36) |
| `stride` | 10 cells | Waypoint thinning — keep every 10th path cell |

---

## Sensor config (WaterLinked Sonar 3D-15, 1.2 MHz mode)

| Parameter | Value | Source |
|---|---|---|
| `resolution_x` | 257 | 90° / 0.35° beam separation |
| `resolution_y` | 67 | 40° / 0.60° beam separation |
| `horizontal_fov` | 90° | Datasheet |
| `vertical_fov` | 40° | Datasheet — now wired through (Ch45) |
| `depth_min` | 0.2 m | Datasheet |
| `depth_max` | 15 m | Datasheet |

---

## Pending validation

| Change | What needs testing |
|---|---|
| Ch43 | Frontier definition: occ↔unknown (was free↔unknown). Not yet run in a session. |
| Ch45 | Stonefish vFOV fix — requires stonefish rebuild + colcon rebuild before it takes effect. |
