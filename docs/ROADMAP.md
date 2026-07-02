# Active SLAM — 1-Month Roadmap

Working doc. Compacts the candidate improvements into a phased plan. Overwrite freely —
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

## Where we are now (the base — counts as Week 0, done)

- Frontier exploration on the 2-D projected OctoMap, occupied↔unknown frontiers.
- Greedy `cluster_size / distance` (Euclidean) goal selection, commit-until-done.
- 2-D A* with 3-zone inflation; waypoint controller; depth hold; forward-only obstacle stop.
- **Pose = ground truth** (`/StoneFish/Odometry`); **map built from ground truth**.
  A noisy-odom TF is simulated but nothing consumes it.
- => This is the *frontier-coverage comparison baseline*, not yet Active SLAM.

---

## Week 1 — Make "lost" measurable + unify the cost layer

Goal: turn localisation error from "identically zero" into a measured quantity, and remove
the band-aids caused by using Euclidean distance as if it were travel cost.

- [ ] **Consume the estimated/noisy pose.** Point the map + planner at the noisy frame
      (`base_link/noisy`) instead of ground truth. Drift becomes real and visible in RViz.
- [ ] **Evaluation harness** (blocking — without it nothing downstream is defensible):
      - ground-truth coverage % and map error vs the GT OctoMap you already have
      - ATE: estimated trajectory vs GT trajectory
      - pose-uncertainty logging (placeholder until Week 2 gives a real covariance)
      - one script, one CSV, reuse `analyze_session.py` style.
- [ ] **Single Dijkstra/wavefront cost field** from the robot over the inflated cost grid,
      once per selection tick. Replaces, in one pass: (a) Euclidean selection score →
      path-length score, (b) Euclidean stuck metric → remaining-path-length progress,
      (c) A*-fail reachability heuristic (unreachable = ∞ cost), (d) `nearest_free` scan.
      Lets you delete the Ch39 displacement-reset and the A*-fail-count blacklist.
- [ ] **Wall-normal + standoff targeting.** Goal = stand off a chosen distance along the
      surface normal, oriented orthogonal to the structure (your idea). Replaces "snap the
      occupied centroid to nearest free cell." Standoff distance = a parameter.

Done when: a session reports coverage %, ATE, and map error; selection/stuck use path length;
robot approaches surfaces head-on at a set standoff.

## Week 2 — Real SLAM backend (the core of "don't get lost")

Goal: estimate pose with a pose graph and correct drift with loop closure.

- [ ] **GTSAM / iSAM2 pose graph** (Simon + your iSAM2 paper). Odometry factors from the
      noisy DVL/IMU; registration factors from scan matching (GICP/ICP on the depth cloud,
      later sonar submaps).
- [ ] **Build the OctoMap from the optimised pose**, not the raw odom.
- [ ] **Loop closure v0: geometric proximity** (revisit when near a past keyframe), verify
      the graph actually pulls drift back.
- [ ] Metric: ATE and map error **with vs without** loop closure on the same trajectory.

Done when: turning loop closure on measurably reduces ATE on a fixed path.

## Week 3 — Active decision: explore vs revisit (this is the Active SLAM contribution)

Goal: choose actions using uncertainty, not just information gain.

- [ ] **Propagate pose covariance** along candidate paths; reduce to a scalar (D-optimality).
- [ ] **Utility = info-gain(frontier) traded against expected uncertainty.** Revisit
      candidates = previously-seen distinctive areas.
- [ ] **Submap saliency v0**: FPFH → k-means dictionary → idf rarity (GloSSy-lite). Start
      with plain geometric distinctiveness if the full pipeline runs long.
- [ ] Decision rule: explore while uncertainty is bounded; trigger a revisit when the
      propagated covariance crosses a threshold.

Done when: the robot autonomously breaks off exploration to close a loop and the logged
uncertainty drops afterward.

## Week 4 — 3D + benchmark + consolidation

Goal: one dimensional extension + the comparison the thesis needs.

- [ ] **Pick ONE** (time-boxed): 3-D frontier detection on octomap voxels, **or** TSDF map
      (the poster's map-representation axis; gives surface normals for free → helps Week 1
      wall-normal targeting and local nav).
- [ ] **Benchmark matrix**: open-loop vs frontier-coverage vs saliency-active, across
      turbidity / current / obstacle density. Report the 4 metrics.
- [ ] Buffer for slippage + poster figures.

Done when: a table compares the three policies on drift / coverage / time / uncertainty.

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

- **Critical path = Week 2 SLAM backend.** If it slips, Weeks 3–4 compress. The pose graph
  is the gate for everything labelled "Active".
- Weeks 3–4 are ambitious for one month; treat them as stretch and protect Weeks 1–2.
- Scan matching on wide-FoV sonar submaps is harder than on the depth-cam proxy — prototype
  on the depth cloud first, swap in sonar later.
- Don't let perfect saliency block the loop-closure win: a crude revisit rule that reduces
  ATE beats an unfinished GloSSy pipeline.
