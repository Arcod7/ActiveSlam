# Future work

Items investigated or prototyped during the project but deliberately left
outside the delivered scope. Each entry records what exists, why it was
parked, and what completing it would take.

## Wall-looking motion executor (work in progress)

`motion:=walllooking` (`wall_looking.py`) is implemented behind the
motion-executor interface and unit-tested, but is not part of the evaluated
system. An A/B benchmark under it (`matrix_lc_ab_wallfollow`, whose
`motion: wallfollow` is the `walllooking` alias) showed loop closure
*increasing* trajectory error (final ATE 0.405 m with closures vs 0.200 m
without, 437 closure edges on a small physical loop), the opposite of the
drift-and-return result. The leading explanation is perceptual aliasing: a
wall observed at a constant standoff is locally self-similar, so
proximity-gated scan matching can accept constraints between different
points along the surface. Confirming that hypothesis needs per-closure
inlier diagnostics; until then wall-looking is not a reliable evaluation
setting, and it stays selectable but marked WIP in the launcher.

`motion:=walloriented` (`wall_oriented_controller.py`) is a supported
executor and is *not* covered by that finding — the A/B above never ran
under it. It follows the planner path while holding a fixed yaw offset
toward the nearest mapped surface, which keeps the sonar on structure
without changing the travel policy.

## Submap saliency descriptors (FPFH)

The revisit planner scores candidate keyframes by local keyframe density.
The intended upgrade — FPFH descriptors clustered into a vocabulary with
rarity weighting, following Suresh et al. — is unimplemented. Keyframe
clouds are already stored in body frame ready for a descriptor pipeline.

**This is descoped for time, not blocked by the platform.** Open3D publishes
no aarch64 wheel, but the pinned `external/open3d` submodule builds and runs
here: with `GLIBC_TUNABLES=glibc.rtld.optional_static_tls=2097152` set (see
`docs/TROUBLESHOOTING.md`), `import open3d` succeeds and
`compute_fpfh_feature` returns a 33xN descriptor matrix. A node needing it
can set that variable through the launch file's environment. Earlier
revisions of this file and the roadmap called FPFH blocked on aarch64; that
was true before the submodule build landed and is no longer accurate.

## Per-candidate uncertainty propagation

Revisit triggering reacts to the live D-optimality scalar. Projecting
uncertainty forward along each candidate path (virtual factors on a mirrored
factor graph) would let the planner compare expected uncertainty reduction
against detour cost instead of using a plain threshold.

## 3-D frontier detection on the TSDF

Frontier extraction runs on the 2-D projected OctoMap; under `mapper:=tsdf`
a second, planning-only OctoMap instance provides that map. Detecting
frontiers directly on TSDF voxels would retire the dual-map workaround and
extend exploration to fully 3-D structure.

## Frontier route ordering

Goal selection is greedy per tick. A small TSP/2-opt ordering over the top-k
frontier clusters (as popularised by TARE) would reduce mission time without
changing the mapping or SLAM layers.

## OctoMap rebuild after loop closure

Map rebuild after a large loop closure is implemented for the TSDF backend
only. `octomap_server` has no clean way to replay a corrected sensor origin
for its free-space raycasting, so OctoMap belief maps retain pre-closure
integration error.

## Alternative SLAM baselines

GLIM and KISS-ICP were evaluated as candidate comparison baselines and set
aside for time. A rosbag-based comparison (same sensor stream through an
external SLAM system) remains the cleanest way to position the pose-graph
backend against established systems.

## Sonar multipath effects

The sonar noise model covers range noise, lateral jitter, dropout, and
outliers, but not multipath returns, which matter in confined structures
such as wrecks. A multipath term would strengthen the degraded-conditions
benchmark.

## Real-vehicle campaign

`IRL_TEST.md` defines a staged acceptance procedure from simulation to a
BlueROV2 Heavy (fail-closed gate, MAVLink adapter, phased go/no-go
checklists). Phases beyond software acceptance require vehicle, site, and
crew, and remain open.
