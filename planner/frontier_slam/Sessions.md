# frontier_slam — Session Log

Full per-session findings. Cross-referenced with change entries in `Progress.md`.
Current parameter values are in `STATE.md`.

---

## Run index

| File | Date | Summary |
|---|---|---|
| `2026-05-19_13-46-59_session1_raw.log` | 2026-05-19 | Pre-CSV manual log. 3 stuck detections. Loop established between (14.7,2.2) and (10,9). Z drift 8.5→13.8m. |
| `2026-05-19_14-51-54_*.csv` | 2026-05-19 | First CSV session. Depth hold confirmed working (z drift 0.13m over 82s). Frontiers depleted after 12s, robot froze at GOAL_REACHED for 63s. |
| `2026-05-19_15-05-00_*.csv` | 2026-05-19 | Goal timeout (Ch15) fired correctly. Coverage metrics (Ch16) working. Robot reached (14.44,−4.06), SCAN mode entered but SCAN_YAW=0.008 too slow to reorient. 9756 mapped cells. |
| `2026-05-19_15-22-50_*.csv` | 2026-05-19 | Session 4. BLOCKED at (6.04,−0.05) with obs=0.20m from t=+6s to end (80+ seconds frozen). yaw_cmd=0.00 throughout — aligned BLOCKED with no escape. |
| `2026-05-19_15-40-14_*.csv` | 2026-05-19 | Session 5. BLOCKED at (5.70, 0.56) obs=0.25m for 60+ seconds, then BLOCKED again at (13.88,−0.31). Same aligned-BLOCKED pattern; change 18 not yet applied. |
| `2026-05-19_15-43-04_*.csv` | 2026-05-19 | Session 6. Two failure modes: (1) brief BLOCKED at (5.7,0) cleared by goal change, (2) physical stuck at (9.88,−1.71) for 35+ seconds with obs=2.4m — off-axis wall not in camera FOV. Root cause of Ch18. |
| `2026-05-19_15-55-06_*.csv` | 2026-05-19 | Session 7. 366s. Ch18 confirmed: all BLOCKED events clear in 1–2 ticks. 27,151 mapped cells (2.2× session 6). 89% of coverage in first 55s. Final 231s stuck in y≈9.9 loop — nearest-first oscillation. Slow depth drift (err 0→0.55m). |
| `2026-05-19_16-15-40_*.csv` | 2026-05-19 | Session 8. 311s. Ch19 (frontier scoring) confirmed: robot went south to y=−14, reached installation area. 28,139 mapped cells, 16,100 new (+58% cells/min vs session 7, −43% odometer). 3 STUCK_BLACKLIST events. BLOCKED 32.4% of session (south zone obstacle-dense). Phase 4 (t>214s) stalled: only 4 clusters remain, all unreachable. |
| `2026-05-19_16-50-54_*.csv` | 2026-05-19 | Session 9. 182s. Ch21+22 applied. CTRL_STUCK timer bug exposed: robot frozen 62s at (7.6,−4.1) with no escape trigger. 7 goal commits only (min_commit=10s too conservative). 23,127 mapped cells. Fixed by Ch23. |
| `2026-05-19_17-17-59_*.csv` | 2026-05-19 | Session 10. 283s. Best coverage: +16,543 cells. Excellent phase 1 (0–120s). Phase 2: 163s figure-8 loop between (12.86,−5.06) and (12.40,−11.20) — frontier vanish-switch triggered as robot arrives, controller never fires GOAL_REACHED. Fixed by Ch24. Yaw too fast: Ch25. |
| `2026-05-19_17-54-27_*.csv` | 2026-05-19 | Session 11. Ch26 (A* path planning). No obstacle hits. Clean nav 0→(13.10,−6.90)→(7.60,−7.50). GOAL_REACHED frozen 82s — extractor republish reset `_goal_reached_at` every 2s. Fixed by Ch27. |
| `2026-05-20_13-41-51_*.csv` | 2026-05-20 | Session 13. Wall crash immediately after INIT_SCAN (path_len=0, A* fail → direct heading). EMERG_STOP freeze at obs=0.20m with no escape. Fixed by Ch32 (ramp) and Ch33 (back-surge). |
| `2026-05-20_13-47-57_*.csv` | 2026-05-20 | Session 14. Ramp working (no hard crash). But still frozen at wall with path_len=0 all session — A* not failing gracefully. Back-surge (Ch33) confirmed working. |
| `2026-05-20_14-10-52_*.csv` | 2026-05-20 | Session 15. Ch33+34 applied. Back-surge working. Goal oscillation: robot arrived at frontier, A* planned back to origin, arrival at origin relaunched back to original goal → loop. Added arrival blacklist (Ch35). |
| `2026-05-20_14-32-18_*.csv` | 2026-05-20 | Session 16. STUCK_BLACKLIST fired 4× in ~2.5min. A* routing paths through soft zone → surge ramp → drift → goal-distance increase → stuck. Added 3rd planning zone (Ch37) + STUCK_TIMEOUT 15→30s. |
| `2026-05-20_15-01-26_*.csv` | 2026-05-20 | Session 17. Rapid silent goal switch from (10.10,0.00) to (0.00,−0.36) to (2.25,8.15) within 30s. Score-based hysteresis triggered on committed-dist (shrinks as robot progresses). Removed score switching entirely (Ch38). |
| `2026-05-20_15-16-03_*.csv` | 2026-05-20 | Session 18. Ch38 confirmed: goal held throughout. STUCK_BLACKLIST false positive at t≈90s — A* routed a southward arc, goal-distance increased 8.19→9.6m over 28s → stuck timer expired. Robot was moving at full surge with valid path. Fixed by Ch39 (displacement reset). |
| `2026-05-20_15-18-22_*.csv` | 2026-05-20 | Session 19. Ch39 first test. 338s, +14,057 mapped cells. One legitimate STUCK_BLACKLIST at (14.33,−5.96) — max stuck_pct=93, robot genuinely stalled. No false positives. |
| `2026-05-20_15-26-54_*.csv` | 2026-05-20 | Session 20. 222s, +20,853 mapped cells. STUCK_BLACKLIST=0, max stuck_pct=40. Ch39 confirmed: no false STUCK during southward arc (12.39m detour). |
| `2026-05-20_15-39-59_*.csv` | 2026-05-20 | Session 21. Best session ever: 336s, +26,208 mapped cells. STUCK_BLACKLIST=0, max stuck_pct=39. Explored new northwest territory. Ch39 fully confirmed. |

---

## Session 1 — Findings (2026-05-19)

**Duration**: ~4 min of captured log  
**File**: `logs/2026-05-19_13-46-59_session1_raw.log`

### What worked
- **Stuck detection** ✅: All 3 blacklisting events fired correctly and enabled goal switching
- **Obstacle avoidance** ✅: No wall collisions; [BLOCKED] correctly prevented surge near walls
- **Goal reached** ✅: Robot successfully navigated to and reached multiple frontiers after initial stucks

### Issues observed

**1. BLOCKED delay at structure corner (~14.8, 2.5)**
After reaching (14.7, 2.2), the robot needs to turn 120-150° to its next goal. During
this turn the forward camera faces the wall (obs=0.2-0.7m), blocking surge for ~20s
per visit. The robot eventually yaws clear and surges — no crash — but wastes time.
Root cause: slow KP_YAW=0.04 combined with OBSTACLE_STOP_DIST cutting surge during turn.
Potential fix: increase yaw rate when BLOCKED (e.g. `KP_YAW_BLOCKED = 0.15`).

**2. Back-and-forth loop between two frontiers**
After initial stucks cleared, robot locked into a ~60s cycle between (14.7, 2.2) and
(10.0-10.3, 9.x). Nearest-first selection always picks these same two clusters because
they remain visible frontier boundaries throughout the run.
Root cause: nearest-frontier scoring ignores cluster size.
Potential fix: score by `cluster_size / distance` (FutureWork §1) — larger distant
clusters beat small nearby ones, breaking the loop.

**3. Z drift**
Robot depth increases from 8.5m to 13.8m+ over ~4 minutes. Goal Z is set to robot's Z
at publish time (frontier_extractor), but robot drifts down between updates (2s cycle).
The small positive heave commands (+0.00 to +0.06) are insufficient to counteract
persistent downward drift — possibly buoyancy simulation or integration error.
Not critical for current testing but will become a problem in deeper environments.
**→ Fixed by Changes 12 + 14. Session 2 confirmed: drift reduced to 0.13m over 82s.**

---

## Session 2 — Findings (2026-05-19)

**Duration**: 82s  
**Files**: `logs/2026-05-19_14-51-54_*.csv`

### What worked
- **Depth hold** ✅: z stayed in [8.12, 8.25]m (±0.07m around setpoint 8.14m). Changes 12+14 confirmed.
- **Navigation** ✅: Robot reached (13.74, -3.58) from origin cleanly in ~18s, no collisions.
- **Obstacle avoidance** ✅: Brief 2s BLOCKED at wall, then cleared without collision.

### Issues observed

**1. Exploration terminated after 12 seconds (critical)**  
Frontier count collapsed 7→4→3→3→1→0 as the robot moved forward. The last cluster
(13.98,-4.20) disappeared once the robot navigated past the obstacle and the map updated.
After t=726 the extractor found zero clusters and returned early on every tick, publishing
no new goals. The controller reached the stale goal at t=734 and entered GOAL_REACHED.
The robot then hovered frozen for 63 seconds with no recovery.

Root cause: when frontiers run dry, the controller holds the last received goal forever.
GOAL_REACHED state spins at `SCAN_YAW=0.008` but never clears `_goal`, so it never
enters true SCAN mode. The extractor, finding no clusters, stays silent.

**→ Fixed by Change 15 (goal timeout).**

**2. Why did frontiers run dry so fast?**  
Clusters shrank 7→1 in 11s. Likely the 2D OctoMap only surfaces frontiers within
sensor range; as the robot mapped the near field, small residual patches dropped below
`MIN_CLUSTER_CELLS=5` and disappeared. In open water (obs=inf from t=747) there is
genuinely no structure ahead to generate new unknown cells.
The robot needs to rotate and move to discover new frontiers — which Change 15 enables.
If scanning also fails to find frontiers, `MIN_CLUSTER_CELLS` may need lowering.

---

## Session 7 — Findings (2026-05-19)

**Duration**: 366s (~6 minutes)  
**Files**: `logs/2026-05-19_15-55-06_*.csv`

### What worked

- **Obstacle escape (Ch18)** ✅: Every BLOCKED event resolved in 1–2 ticks. Sessions 4–5
  had BLOCKED lasting 80+ seconds. Here the worst case was 2 consecutive ticks (2s), then
  `obs` jumped from 0.30m to 1.95m. Zero time wasted frozen against a pillar.
- **Coverage** ✅: 27,151 `mapped_cells` — 2.2× the previous session (12,412). Robot
  explored corridors above (y≈9–10) and south (y≈−7) that had never been reached before.
- **Navigation** ✅: Robot moved cleanly at max surge across the full environment with no
  collisions. Heading error typically < 5° during straight runs.

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — Active exploration | 0–55s | 15,348 → 24,157 | Fast goal-switching across 4 corridors; 89% of total coverage |
| 2 — S/N oscillation | 55–90s | 24,157 → 24,739 | Loop between south goal (y≈−7) and north goal (y≈6.8) |
| 3 — New territory | 90–135s | 24,739 → 27,077 | Robot reaches y≈9–10 corridor for first time |
| 4 — Tight corridor loop | 135–366s | 27,077 → 27,151 | 231s in 5m strip, only 74 new cells |

### Issues observed

**1. Back-and-forth loop — nearest-first selection oscillates (critical)**  
In phase 4 the robot locks between goals A=(9.90,8.80) and B=(−2.00 to −3.40,9.80),
±100° heading changes every ~12 seconds. Nearest-first selection picks them alternately
because they are equidistant from the robot's midpoint. The GoalManager's
`SWITCH_HYSTERESIS=3.0m` does not help: it only applies while actively pursuing a goal,
but `GOAL_REACHED` fires and clears `_goal` before the 10s timeout, so each new dispatch
starts fresh. The extractor at 0.5 Hz republishes before the timeout can fire.  
`CTRL_STUCK` does not help either: the robot IS moving 0.25m+ every 5s — it physically
covers the corridor, just not new territory.

Root cause: nearest-first scoring has no preference for information gain. Two equidistant
clusters of very different sizes receive equal priority.

**Potential fix**: Score frontiers by `cluster_size / distance`. A cluster with 20 cells
at 8m beats a cluster with 3 cells at 7m. This breaks equidistance ties toward the more
informative direction and naturally deprioritises tiny residual boundary patches.
See FutureWork §1.

**2. Slow depth drift**  
`depth_err` grows from 0.01m at start to 0.55m by t=135s, then stays there.
Heave output reaches −0.20 (full upward thrust at 0.5m error) but cannot fully
counteract the drift. Likely cause: pitch coupling during aggressive surge — forward
thrust creates a slight nose-down moment, pushing the robot deeper. The setpoint is
locked correctly; the controller just can't overcome the physical effect.
Not critical at current scale but will worsen in longer sessions or deeper environments.

**3. Yaw speed during goal switches**  
On each goal switch (±100°), `yaw_cmd ≈ ±0.17–0.18`. This is fast but below the ESCAPE_YAW
cap. The user's "turns too fast" observation likely refers to ESCAPE_YAW=0.40 firing when
BLOCKED — which is correct behaviour and should stay. Navigation yaw at KP_YAW=0.10
produces smooth turns for small errors but snappy large-angle corrections.

### Key insight: when does exploration actually happen?

89% of session coverage (8,809 cells) was gained in the first 55 seconds. The rest of
the 5-minute session contributed only 2,994 cells. The bottleneck is not obstacle avoidance
or navigation — it is goal selection. Nearest-first with 23 equidistant clusters generates
a random walk among visited frontiers rather than directing the robot toward large unknown
regions. Frontier scoring is the next necessary change.

---

## Session 9 — Findings (2026-05-19)

**Duration**: 182s  
**Files**: `logs/2026-05-19_16-50-54_*.csv`

### Summary

Mixed result. The robot explored well in the first 100s, mapping 23,127 cells (session 8
reached 28,139 in 311s, but session 9 ended earlier). Coverage stalled completely from
t≈120s to end: robot frozen at (7.6, −4.1) for 62s with goal (6.78, −9.38) visible but
unreachable — neither CTRL_STUCK nor GoalManager STUCK fired.

Goal commits: only 7 vs 35 in session 8 — `min_commit_duration=10s` was too conservative.
The robot missed the "other side of the boat" and the far side of the left building.

### What worked

- **BLOCKED_TIMEOUT (Ch21)** ✅: no evidence of long BLOCKED stalls; wall contacts were
  brief. Direction hysteresis appears to have helped.
- **Frontier scoring (Ch19)** ✅: robot still reached interesting deep zones (south
  corridor, y≈−9).
- **Depth hold** ✅: z stayed near 8.2m throughout.

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — Active south exploration | 0–60s | 12,897 → 17,355 | Rapid exploration south, 4,458 new cells |
| 2 — North sweep attempt | 60–100s | 17,355 → 22,122 | Goal switched to (9.6, 3.85), 4,767 new cells |
| 3 — Approach & freeze | 100–120s | 22,122 → 23,127 | Goal (6.78, −9.38), robot approached to 5.3m then froze |
| 4 — Full freeze | 120–182s | 23,127 → 23,127 | Zero new cells, stuck at (7.6,−4.1), 0 CTRL_STUCK events |

### Root causes of session 9 freeze

1. **CTRL_STUCK timer reset**: `_goal_cb` resets `_stuck_ref_pos` every 2s from the
   extractor republish — the 5s CTRL_STUCK window can never accumulate. Robot frozen
   for 62s with surge≥0.15 but never triggering escape. (**Fixed in Ch23**)

2. **GoalManager STUCK silent**: goal was at 12.72m when committed; robot closed to 5.32m
   (7.4m progress, well above 0.5m threshold). `committed_dist - cur_dist = 7.4m` permanently
   above STUCK_MIN_PROGRESS. After the robot froze at 5.32m the check sees "already made
   great progress" and never re-arms. (**Partially mitigated by Ch23 fix 1**; a deeper fix
   would reset `committed_dist` after each successful progress interval.)

3. **min_commit_duration=10s over-restraint**: only 7 goal commits total, missing large
   unexplored zones that appeared en route. (**Fixed in Ch23**)

---

## Session 10 — Findings (2026-05-19)

**Duration**: 283s  
**Files**: `logs/2026-05-19_17-17-59_*.csv`  
**Code state**: Ch20–22 applied; Ch23 applied just before next session.

### Summary

Best coverage session so far: **+16,543 cells** (9,676 → 26,219) in 283s. Phase 1 (0–120s)
was excellent with rapid multi-directional exploration reaching y≈−13. Phase 2 (120–283s)
lost 163s to a figure-8 oscillation between two goals that the robot could never complete.

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — Active exploration | 0–120s | 9,676 → 26,019 | Many goal commits, south zone reached (y≈−13), +16,343 cells |
| 2 — Figure-8 loop | 120–283s | 26,019 → 26,219 | Oscillating between (12.86,−5.06) and (12.40,−11.20), only +200 cells |

### What worked

- Many goal commits in phase 1 (more spatial coverage than sessions 8–9)
- BLOCKED events brief (Ch21 working); no long BLOCKED stalls
- Depth hold stable throughout
- South zone (y≈−12 to −14) explored well again

### Root cause of figure-8 loop

Robot approaches (12.40, −11.20) → at 1.4m (inside `GOAL_RADIUS=2.0m`), the frontier
cluster is mapped by the sensor → `near_old` empty → GoalManager publishes (12.86, −5.06)
at 8m away → robot reverses north → approaches (12.86, −5.06) → cluster vanishes at ~2.1m
→ GoalManager publishes (12.40, −11.20) → repeat. `min_commit_duration` can't prevent
vanish-based switches. **Fixed by Change 24.**

Additionally, 180° goal reversals with `KP_YAW=0.10` produced visually fast spinning.
**Fixed by Change 25.**

---

## Session 11 — Findings (2026-05-19)

**Duration**: ~124s  
**Files**: `logs/2026-05-19_17-54-27_*.csv`  
**Code state**: Ch26 applied (A* path planning, INIT_SCAN, no BLOCKED machinery).

### Summary

No obstacle hits at all — A* routing is working. Navigation clean: robot went from start
to (13.10, −6.90) then to goal (7.60, −7.50) smoothly with correct path following.
Coverage: 10,581 → 22,307 cells (+11,726) in 124s.  Two bugs found and fixed (Ch27).

### Phase breakdown

| Phase | t range | mapped_cells | Robot behaviour |
|---|---|---|---|
| 1 — INIT_SCAN | 0–20s | 10,581 → 18,366 | 20s spin — 3–4 rotations (too long) |
| 2 — Navigation | 20–42s | 18,366 → 22,079 | Clean A* path follow, no obstacles hit |
| 3 — GOAL_REACHED loop | 42–124s | 22,079 → 22,307 | Stuck spinning at (7.82,−7.27) — GOAL_REACHED_TIMEOUT never fired |

### Root causes found

1. **GOAL_REACHED never times out**: extractor republish of same goal resets
   `_goal_reached_at` every 2s. 10s timeout cannot accumulate. **Fixed Ch27.**

2. **INIT_SCAN 20s causes GoalManager false STUCK**: first goal (12.50, 0.00) blacklisted
   at t≈17s because robot hadn't moved (still in INIT_SCAN). **Fixed Ch27 (5s scan).**

3. **Extra goal switch at t≈35s**: after reaching area near first goal, GoalManager's
   vanish-switch fired for (13.99,−2.49) within 2s of committing (robot far from it, map
   updated) — causing one extra turn. Not a bug; min_commit_duration correctly did not
   block the vanish path.

---

## Session 13 — Findings (2026-05-20)

**Duration**: ~95s  
**Files**: `logs/2026-05-20_13-41-51_*.csv`  
**Code state**: Ch26–31 applied (A* planning, debug viz, NaN crash fix).

### Summary

Session crashed into a wall immediately after INIT_SCAN, then recovered partially before freezing permanently at (7.07, 3.20) with obs=0.20m for the remaining ~40s.

### Phase breakdown

| Phase | t | Robot pos | obs_m | path_len | Behaviour |
|---|---|---|---|---|---|
| INIT_SCAN | 0–10s | (0,0) | 4–6m | 0 | Spinning, mapping |
| Direct-goal crash | 10–20s | →(5.75,0.35) | →0.26 | **0** | A* failed → MAX_SURGE straight into wall |
| EMERG_STOP freeze | 20–36s | (~5.9,−0.3) | 0.20–0.29 | 0 | Frozen at wall, GoalManager cycling |
| A* backward path | 36–46s | →(0.5,−0.2) | clears | 7 | Routes back to (−0.18,0.12) ✓ |
| Second direct-goal | 46–60s | →(1.9,6.9) | →0.21 | 0 | A* failed again → another wall hit |
| A* path + reach | 60–73s | →(6.3,3.4) | clears | 4 | Clean path following ✓ |
| Final permanent freeze | 73–end | (7.07,3.20) | **0.20** | 0 | EMERG_STOP from tick 1, no escape |

### Root causes

1. **A* consistently failing on initial goals**: `path_len=0` whenever goal is in the direction
   of a known wall. A* returns no path — robot falls back to direct heading at MAX_SURGE and
   crashes. Diagnosis requires seeing the debug image inflated map. Likely cause: inflation
   radius blocks the only corridor, or goal cell is inside inflated zone.

2. **No speed reduction near obstacles**: binary hard-stop at 0.4m; robot arrives at 0.40 m/s.
   **Fixed in Ch32.**

3. **CTRL_STUCK blind to EMERG_STOP freeze**: when EMERG_STOP keeps surge=0, the stuck
   detector never arms (`surge < STUCK_SURGE_MIN=0.15`). Robot can be frozen indefinitely.
   At t=73s, obs=0.20m from the first tick of a new goal — robot never moved before freezing.

---

## Session 14 — Findings (2026-05-20)

**Duration**: ~70s  
**Files**: `logs/2026-05-20_13-47-57_*.csv`  
**Code state**: Ch32 applied (progressive ramp, MAX_SURGE=0.25).

### Summary

Ramp working: robot slowed from 0.25→0 over the last 1.6m before the wall (obs 2.0→0.37m).
No hard crash. But frozen indefinitely once at wall — same root cause as session 13.

### Key findings

- **path_len=0 the entire session**: A* fails even from open water (robot at (0,0), goal at
  (9.0,0.0)). Robot falls back to direct goal heading and reaches the wall at x≈5.75m.
- **EMERG_STOP correct but no escape**: backs nothing, freezes at obs≈0.25m.
- **GoalManager STUCK silent again**: robot closed 8.60m→3.26m = 5.34m total progress.
  `committed_dist - cur_dist` permanently above `stuck_min_progress=0.5m` so the check
  reads "not stuck" even after the robot has been frozen for 40+ seconds. **Fixed in Ch33.**
- **CTRL_STUCK blind**: surge=0 from EMERG_STOP → stuck detector never arms.
