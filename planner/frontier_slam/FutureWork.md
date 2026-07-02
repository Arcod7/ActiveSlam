# frontier_slam — Future Work

Items 1–4 are direct extensions of the current simulation stack.
Items 5–6 are the core academic contributions of the thesis.

---

## 1. Wall-normal targeting

The current frontier detector finds occupied↔unknown boundary clusters and sends the
robot toward the cluster centroid.  A better target is the point on the wall surface
that the robot should face to get the **maximum new information** from a single sonar
ping.

Approach:
1. For each occupied frontier cell, compute the local surface normal from the
   OccupancyGrid gradient.
2. Project a standoff point along the inward normal at ~1.5× `INFLATION_M` from the
   wall surface (just outside the hard inflation zone).
3. Use the normal direction as the desired robot heading at the standoff point, so the
   sonar's 90° cone is centred on the unexplored shadow.

This turns the robot's exploration path into a structured surface-inspection scan
rather than a random walk around wall centroids.

---

## 2. Realistic odometry: drift + noise

The simulation currently uses a noiseless odometry source
(`/StoneFish/Odometry`).  A realistic BlueROV2 odometry stack would include:

- **DVL slip**: velocity noise proportional to forward speed
- **IMU drift**: heading bias that grows with time (random walk on yaw)
- **Depth sensor noise**: small Gaussian perturbation on Z

Adding these exposes whether the A\* planner and GoalManager are robust to the
coordinate-frame drift that will occur in real deployments, and is a prerequisite
before any SLAM evaluation is meaningful.

---

## 3. Realistic sonar: noise + shadow artefacts

The Stonefish depth camera is a clean ray-cast — it returns the exact range with no
noise, no multi-path, and no shadow artefacts.  A more realistic sonar model would
add:

- **Range noise**: Gaussian with σ ≈ sonar range resolution (1.5 mm for the
  WaterLinked 3D-15)
- **Speckle / missing returns**: random dropout of a fraction of beams per ping
- **Angular jitter**: small perturbation on beam direction

These artefacts directly affect OctoMap quality and are the dominant source of
spurious frontier cells in real-world deployments.

---

## 4. Full SLAM pipeline

The current stack uses ground-truth odometry and treats mapping as a side-effect of
navigation.  A full SLAM system would:

1. Replace `/StoneFish/Odometry` with an estimated pose from scan-matching
   (e.g. ICP between consecutive sonar point clouds).
2. Build a pose graph where each node is a sonar scan and each edge is the
   scan-match transformation.
3. Detect loop closures (revisiting a previously mapped region) and apply graph
   optimisation (iSAM2 / g2o) to correct accumulated drift.

Prerequisite for the active SLAM work in item 6.

---

## 5. 3-D frontier detection

The current extractor works on the 2-D projected OctoMap (`/projected_map`).
This collapses the Z dimension: the robot only seeks frontiers in the XY plane
and maintains its current depth.

True 3-D frontier detection would subscribe to `/octomap_binary`, deserialize
with the Python `octomap` bindings, and find occupied voxels adjacent to unknown
voxels in all 6 directions.  This enables:
- Vertical exploration (above/below the structure)
- Detecting overhangs and cavities invisible to the 2-D projection

Dependency: `python3-octomap` (or equivalent bindings for ROS 2 Jazzy).

An alternative to OctoMap is a **TSDF/ESDF** representation (e.g. voxblox or nvblox).
ESDF gives continuous signed distance to the nearest surface as a first-class field,
which makes wall-following (§1) and standoff-point selection trivial: the standoff is
simply the point where `esdf_value == INFLATION_M` along the surface normal, with no
extra inflation pass required.  ESDF also removes the need for the three-zone A* cost
grid — gradient descent on the ESDF naturally encodes obstacle avoidance.

---

## 6. Active SLAM: saliency-driven revisits + information gain

The long-term goal of this project: replace the greedy frontier selector with the
dual-behaviour policy from *Active SLAM using 3D Submap Saliency for Underwater
Volumetric Exploration* (Suresh et al., ICRA 2020):

- Build a submap-based pose graph alongside the OctoMap.
- Score each submap by **GloSSy** (FPFH descriptors → k-means dictionary → idf
  rarity).  High-saliency submaps are good loop closure candidates.
- Replace the current `cluster_size / distance` score with a proper
  **information-gain utility**: for each candidate viewpoint, ray-cast the sensor
  frustum into the volumetric map and count the expected number of newly observed
  unknown voxels (unmapped-volume-in-frustum).  Divide by path cost (not Euclidean
  distance) to get an information-rate that accounts for detours around obstacles.
  This removes the oscillation band-aids (stuck detection, goal blacklisting) that
  compensate for the current greedy score.
- At each decision step, choose between **NextBestView** (frontier exploration
  weighted by information gain and map uncertainty) and **revisit** (loop closure)
  based on D-optimality of the propagated pose covariance.

This directly addresses the "don't get lost" priority from the project goal
hierarchy and is the core contribution of the thesis.

---

## 7. Evaluation against ground truth

The simulation provides ground-truth data that should be exploited to produce
defensible thesis metrics.

**Coverage %**
  At any point in a run, query the Stonefish scene for the full set of occupied
  voxels (or use a pre-built reference OctoMap from a scripted survey pass).
  Coverage = `|mapped_occupied ∩ reference_occupied| / |reference_occupied|`.
  Plot coverage vs. time and vs. total path length for each algorithm variant.

**Absolute Trajectory Error (ATE)**
  Once §4 (SLAM) is in place and pose is estimated rather than given, compute
  ATE = RMSE of `||p_est - p_gt||` over all timesteps.  Until then ATE is
  identically 0 and the "don't get lost" objective is untestable.

**Path efficiency**
  Total distance travelled divided by coverage gained — lower is better.
  Complements coverage % and exposes oscillation or redundant revisits.

**Stuck events**
  Already logged in the controller CSV.  Report count and total time lost to
  `CTRL_STUCK` and `EMERG_STOP` events per run.

These four metrics are the concrete comparison axes between the frontier baseline
(this package) and any improved policy (§6).
