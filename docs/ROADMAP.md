# Active SLAM — 1-Month Roadmap

Working doc. Organises proposed improvements into a phased plan. Overwrite freely —
this is the *living plan*, not an append-only log (that's `frontier_slam/Progress.md`).

## North star (from the poster abstract)

Active SLAM that **explicitly trades exploration against revisitation**: push into unknown
space when safe, loop back to a geometrically distinctive area to close a loop and correct
drift when uncertainty grows. Baseline = Suresh et al. (ICRA 2020) submap-saliency on an
OctoMap pose-graph backend. Extensions: wide-FoV 3D sonar (90°×40°), FPFH submap
descriptors, TSDF map. Deliverable = reproducible benchmark reporting **localisation drift,
map coverage, mission time, pose uncertainty** across turbidity / current / obstacle density.

## Priority order (non-negotiable)

1. **Don't get lost** (localisation accuracy)
2. **Explore as much as possible** (coverage)
3. **Do it fast** (mission time)

Everything below is ordered to serve #1 first. The current code optimises #2 under a
*perfect-pose assumption* — so #1 is both the biggest gap and the critical path.

## Where we are now (Week 0 base + most of Weeks 1–2, 2026-07-05)

**Week 0 base (done):**
- Frontier exploration on the 2-D projected OctoMap, occupied↔unknown frontiers.
- Greedy `cluster_size / distance` (Euclidean) goal selection, commit-until-done.
- 2-D A* with 3-zone inflation; waypoint controller; depth hold; forward-only obstacle stop.
- => This is the *frontier-coverage comparison baseline*, not yet Active SLAM.

**Since then:** a real SLAM backend exists and is wired in behind
`slam:=slam` (see `slam/slam_backend`, `eval/eval_tools`, and
`docs/SLAM_PLAN.md` for the design). `slam:=none` (default) keeps the
Week-0 baseline above byte-for-byte. Detail under Weeks 1–2 below.

**Update, 2026-07-22.** Weeks 1–3 have landed: TSDF mapping, the
uncertainty-triggered revisit planner, map rebuild after large closures, the
batch orchestrator, and six noise profiles are all in and benchmarked
(`Progress.md` Phases 1–28). Two things since have changed how runs behave and
what the remaining schedule looks like:

- **A fail-closed motion gate** now sits between every executor and the
  thrusters, with an ArduSub MAVLink adapter behind it (Phase 27). This is the
  opportunistic real-robot path; bench acceptance passed, no physical vehicle
  yet.
- **Scanning sweeps instead of spinning** (Phase 28), which is tether-safe, and
  the sim's thruster calibration was corrected by ~11x. Any velocity or timing
  figure recorded before 2026-07-21 is not comparable with later runs.

**Remaining before the 2026-08-01 code freeze** is organised as a **4-day
parallel push (2026-07-23 → 26)** across seven file-disjoint tracks — full
spec in `docs/plans/plan.md`, one file per track in `docs/plans/tracks/`. In
priority order:

1. **Spine (priority #1):** re-run the benchmark matrix on the corrected
   baseline — every number the dissertation quotes must come from after the
   2026-07-21 calibration fix. Includes the loop-closure A/B (`matrix_lc_ab`)
   and a fresh-machine reproducibility install.
2. **Reference-trajectory mission** (Track 1) — follow a known loop / go to a
   point, break off to close a loop when uncertain, resume the path. Replaces
   the parked shipwreck demo as the presentation centrepiece.
3. **Demo scene** (Track 4) — TUI scene selection with per-object xyz / scale /
   roll-pitch-yaw at launch (not the 289 MB wreck, which stalls Stonefish in
   the sonar's view; not a live editor).
4. **Control-centre TUI + RViz** (Tracks 2, 3) — live metrics + robot-state
   dashboard, camera-follow, param exposure.
5. **Three algorithmic features in parallel** (Tracks 5, 6): 3-D frontier on
   the TSDF → info-gain exploration of low-information regions; and FPFH submap
   saliency for revisit target selection (now unblocked on aarch64).
6. **Validation** (Track 7) — rosbag vs noise-model, a localisation-uncertainty
   measure beyond D-optimality, an architecture-modularity review.

Out of this window: `motion:=walllooking` stays work-in-progress
(loop-closure-inversion finding, `docs/FUTURE_WORK.md`) and out of the evaluated
matrix; `motion:=walloriented` is supported. The `wall_looking` motion-only
reshape and the sonar/camera fusion build are deferred past the freeze.

---

## Week 1 — Make "lost" measurable + unify the cost layer

Goal: turn localisation error from "identically zero" into a measured quantity, and remove
the band-aids caused by using Euclidean distance as if it were travel cost.

- [x] **Consume the estimated/noisy pose.** Superseded the planned `base_link/noisy` frame
      (drift with no correction) with the full Week-2 SLAM backend: `slam:=slam` points the
      map + planner at `/slam/odometry` (simulated pressure/IMU/DVL, fused, SLAM-corrected)
      instead of ground truth. Drift is real and visible in RViz, and gets corrected by loop
      closure rather than growing unbounded. `odom_topic` on the frontier planner already
      supported this pattern; `slam_backend`/`eval_tools` supply the rest.
- [x] **Evaluation harness**, partially: `eval_tools/benchmark.py` logs ATE, RPE, and a
      D-optimality pose-uncertainty scalar (real covariance from iSAM2, not a placeholder) to
      CSV + TUM trajectory files (`evo`-compatible), with `plot_results.py` for offline plots.
      **Not done:** ground-truth coverage % and map error vs the GT OctoMap — the harness is
      pose-only so far, not map-quality.
- [ ] **Single Dijkstra/wavefront cost field** from the robot over the inflated cost grid,
      once per selection tick. Replaces, in one pass: (a) Euclidean selection score →
      path-length score, (b) Euclidean stuck metric → remaining-path-length progress,
      (c) A*-fail reachability heuristic (unreachable = ∞ cost), (d) `nearest_free` scan.
      Lets you delete the Ch39 displacement-reset and the A*-fail-count blacklist.
- [x] **Wall-normal + standoff targeting** (Ch55, `wall_looking.py`) — closed-loop kinematic
      test passes; still needs an end-to-end Stonefish session per `Sessions.md`.

Done when: a session reports coverage %, ATE, and map error; selection/stuck use path length;
robot approaches surfaces head-on at a set standoff.
— ATE done, coverage %/map error still open, cost field still open.

## Week 2 — Real SLAM backend (the core of "don't get lost")

Goal: estimate pose with a pose graph and correct drift with loop closure.

- [x] **GTSAM / iSAM2 pose graph** — `slam/slam_backend/pose_graph.py`. Odometry factors from
      fused pressure+IMU+DVL dead reckoning (`dead_reckoning.py`); registration factors from
      `small_gicp` GICP on the depth cloud (`scan_matcher.py`), gated on inlier count/ratio +
      per-inlier error, not just convergence.
- [x] **Build the OctoMap from the optimised pose, not the raw odom** — when `slam:=slam`,
      `pose_graph.py` is the sole broadcaster of `world_ned -> bluerov2/base_link` and
      `octomap_server` follows that TF. **Caveat:** no map re-integration after a loop closure
      moves earlier keyframes — pre-closure scans stay wherever they were first integrated;
      only new scans benefit from the correction. Accepted as a known limitation (see
      `docs/SLAM_PLAN.md` Part 10), not solved this week.
- [x] **Loop closure v0: geometric proximity** — fires and measurably pulls drift back (see
      below), with re-detection when a graph update moves earlier keyframes.
- [ ] Metric: ATE and map error **with vs without** loop closure on the same trajectory. Not
      done as a controlled A/B — see the real run below instead.

**Real benchmark, 2026-07-04** (`slam:=slam mode:=frontier noise_profile:=realistic`, real
Stonefish sim, 157s / 116 keyframes / 12 loop closures, zero exceptions):
final ATE (cumulative RMSE) **0.58 m**, peak instantaneous position error 0.95 m. The 12
closures all landed in two early clusters; **zero fired in the final ~63s** because frontier
exploration pushed into genuinely new territory outside the 5 m loop-closure search radius —
loop closure here is opportunistic only, it corrects drift when the robot happens to revisit
somewhere, it doesn't make the robot revisit. That's precisely the gap Week 3 exists to close.
One run, one unseeded noise draw — not yet a statistically defensible number, just a real one.

Done when: turning loop closure on measurably reduces ATE on a fixed path.
— Loop closure verified to reduce drift when it fires; the controlled with/without
comparison on one fixed path is still open.

## Week 3 — Active decision: explore vs revisit (this is the Active SLAM contribution)

Goal: choose actions using uncertainty, not just information gain.

- [x] **Scalar uncertainty (D-optimality)** published live from the pose graph and consumed
      by the planner. Per-candidate covariance *propagation* was cut — the planner reacts to
      the live value instead (Phase 20).
- [x] **Utility trade-off + decision rule**: `revisit:=true` breaks off exploration when
      D-optimality crosses a threshold, drives to a previously-seen target, and resumes.
- [~] **Submap saliency v0** (FPFH → k-means → idf rarity): **now in scope** for the
      4-day push (Track 6). Was thought platform-blocked; it is not — the
      `external/open3d` submodule builds on aarch64 and `compute_fpfh_feature` runs
      once `GLIBC_TUNABLES=glibc.rtld.optional_static_tls=2097152` is set. Target
      scoring still uses keyframe density until Track 6 lands.

Done when: the robot autonomously breaks off exploration to close a loop and the logged
uncertainty drops afterward. — **Met** (Phase 20): forced-trigger run completes the full
suspend → drive → closure → dopt-drop → resume cycle. The natural-trigger A/B is a mechanism
demo, not an ATE-improvement claim (n=2, mixed).

## Week 4 — 3D + benchmark + consolidation

Goal: one dimensional extension + the comparison the thesis needs.

- [x] **Picked TSDF** (`mapper:=tsdf`, VDBFusion): surface reconstruction, normals, solid-voxel
      goal safety, and rebuild-on-large-closure. OctoMap stays as the planning map source.
- [~] **Benchmark matrix**: the orchestrator, six noise profiles and the metric set are in and
      have been run; the numbers now need re-running on the post-calibration baseline.
- [ ] Poster/presentation figures + demo scene (see the code-freeze list above).

Done when: a table compares the policies on drift / coverage / time / uncertainty, from runs
made after the 2026-07-21 thruster calibration fix.

---

## Parking lot (explicitly out of scope for the month)

- Sampling-based NBV viewpoint planning (RRT-style frustum info-gain — the 2016 RH-NBV
  paper). Powerful but a project on its own; revisit only if Week 3 finishes early.
- Learned 3-D descriptors (FCGF, PPF-FoldNet) replacing FPFH.
- End-to-end action selection via deep RL.
- Omnidirectional reactive repulsion controller (the map-derived clearance field from
  Week 1 covers most of this need already).
- Real WaterLinked Sonar 3D-15 hardware integration (sim proxy only for now).

## Risks / honesty

- **Critical path = Week 2 SLAM backend — landed, 2026-07-04.** The pose graph exists,
  builds, and measurably reduces drift when loop closure fires (real run above). The
  remaining Week-2 gap isn't the backend itself, it's that loop closure is opportunistic —
  it never *makes* the robot revisit anything, which is exactly Week 3's job, not a Week 2
  shortfall. Weeks 3–4 are no longer gated on Week 2 existing at all.
- Weeks 3–4 are ambitious for one month; treat them as stretch and protect Weeks 1–2.
- Scan matching on wide-FoV sonar submaps is harder than on the depth-cam proxy — prototype
  on the depth cloud first, swap in sonar later.
- Don't let perfect saliency block the loop-closure win: a crude revisit rule that reduces
  ATE beats an unfinished GloSSy pipeline.
