#!/usr/bin/env python3
"""
Parameter surface, presets and reference text for launcher.py.

Split from the UI so the option table can be checked against demo.launch.py's
declared arguments without a terminal. Pure standard library.
"""

# Live-tunable parameters are re-read by their node every control cycle, so a
# change can be pushed with `ros2 param set` instead of restarting the group.
# (node basename, node parameter name)
LIVE = {
    "sigma_allow_xy_m":           ("revisit_planner", "sigma_allow_xy_m"),
    "sigma_allow_yaw_rad":        ("revisit_planner", "sigma_allow_yaw_rad"),
    "ratio_trigger":              ("revisit_planner", "ratio_trigger"),
    "ratio_resume":               ("revisit_planner", "ratio_resume"),
    "revisit_min_closures":       ("revisit_planner", "revisit_min_closures"),
    "wall_standoff":             ("wall_looking", "standoff_m"),
    "wall_switch_goal_distance": ("wall_looking", "switch_goal_distance_m"),
    "wall_switch_scan_angle":    ("wall_looking", "switch_scan_angle_rad"),
    "wall_switch_scan_yaw":      ("wall_looking", "switch_scan_yaw"),
    "wall_path_influence":       ("wall_looking", "path_influence"),
    "wall_path_look_offset_deg": ("wall_looking", "path_look_offset_deg"),
    "wall_normal_offset_deg":    ("wall_looking", "wall_normal_offset_deg"),
    "wall_path_heading_weight":  ("wall_looking", "path_heading_weight"),
}

# These object-pose fields are applied through Stonefish's /set_entity_pose
# service rather than a ROS parameter.  They stay static bodies, preserving
# Stonefish's optimised collision/sensor path; scale remains launch-time only.
LIVE_SCENE_POSE = {
    "obj_x", "obj_y", "obj_z", "obj_roll", "obj_pitch", "obj_yaw",
}

# Robot pose is read back from Stonefish odometry while it moves and can be
# applied live through the existing respawn service.
LIVE_ROBOT_POSE = {
    "robot_x", "robot_y", "robot_z", "robot_roll", "robot_pitch", "robot_yaw",
}

# speed_factor/turn_factor apply in both teleop (this process, no ROS param
# involved) and frontier mode — but in frontier mode the live-tunable node
# depends on which motion executor is actually running, so this can't be a
# fixed (node, param) pair like LIVE. control_screen resolves the node name
# from this map before pushing the value with ros2 param set.
LIVE_SPEED_TURN = {"speed_factor", "turn_factor"}

# Map thresholds tsdf_mapper applies at publish time rather than building the
# grid from, so they can be pushed to a running mapper instead of restarting it
# and losing the map (see tsdf_mapper.LIVE_PARAMS). Launcher id -> node
# parameter. Nothing here works under octomap: octomap_server takes its Z band
# as occupancy_min_z/max_z at launch, and has no voxel thresholds at all.
LIVE_MAPPER = {
    "voxel_min_weight":           "voxel_min_weight",
    "voxel_min_solid_confidence": "voxel_min_solid_confidence",
    "robot_depth_target":         "target_depth_m",
}


def mapper_live_targets(pid, values):
    """(node basename, node parameter) pairs a mapper option pushes to.

    The ground-truth mapper is built from the same thresholds so the map
    metrics keep comparing like with like, and it only exists under slam:=slam.
    """
    if pid not in LIVE_MAPPER or not _tsdf(values):
        return []
    nodes = ["tsdf_mapper"]
    if values["slam"] == "slam":
        nodes.append("tsdf_mapper_gt")
    return [(node, LIVE_MAPPER[pid]) for node in nodes]


def is_live(p, values):
    """Whether a change to this option reaches the running stack without a
    restart. Values-dependent: the map thresholds are live under TSDF only."""
    return bool(p.live or mapper_live_targets(p.id, values))


MOTION_EXECUTOR_NODE = {
    "default":      "waypoint_controller",
    "walloriented": "wall_oriented_controller",
    "walllooking":  "wall_looking",
}


class Param:
    def __init__(self, id, label, kind, default, section, description,
                 choices=None, choice_help=None, advanced=False, step=None,
                 visible=lambda v: True, lo=None, hi=None, soon=(), wip=()):
        self.id = id
        self.label = label
        self.kind = kind                  # enum, bool, int, float, text
        self.default = default
        self.section = section
        self.description = description
        self.choices = choices or []
        self.choice_help = choice_help or {}
        self.advanced = advanced
        self.step = step if step is not None else (1 if kind == "int" else 0.05)
        self.visible = visible
        self.lo = lo
        self.hi = hi
        # Choices that name planned work rather than a runnable configuration.
        self.soon = frozenset(soon)
        # Choices that do run but are not validated against the ground-truth
        # map yet. Unlike soon, these apply — the tag says results are provisional.
        self.wip = frozenset(wip)

    def is_soon(self, value):
        return value in self.soon

    def is_wip(self, value):
        return value in self.wip

    @property
    def live(self):
        return (self.id in LIVE or self.id in LIVE_SCENE_POSE
                or self.id in LIVE_ROBOT_POSE or self.id in LIVE_SPEED_TURN)

    def clamp(self, value):
        if self.lo is not None:
            value = max(self.lo, value)
        if self.hi is not None:
            value = min(self.hi, value)
        return value


SECTION_TITLES = {
    "primary": "Primary",
    "mapping": "Map geometry + wall threshold",
    "scene": "Scene + object",
    "robot": "Robot pose",
    "frontier": "Planner",
    "slam": "SLAM / pose source",
    "advanced": "Other",
}
SECTION_ORDER = ["primary", "mapping", "scene", "robot", "frontier", "slam",
                 "advanced"]

# Which sections are folded away behind their < SHOW > heading. Persisted in
# config.yaml next to the options, so a layout survives a restart. Only the
# primary section starts open — the rest is opened when it is wanted.
HIDDEN_SECTIONS_KEY = "hidden_sections"
DEFAULT_HIDDEN_SECTIONS = [s for s in SECTION_ORDER if s != "primary"]

_frontier = lambda v: v["mode"] == "frontier"
# goto and frontier both run the planner layer (A* + path executor); they
# differ only in who picks the goal.
_planner = lambda v: v["mode"] in ("frontier", "goto")
_slam = lambda v: v["slam"] == "slam"
# Both TSDF choices run the same VDBFusion backend; they differ only in how
# many volumes it keeps.
_tsdf = lambda v: v["mapper"].startswith("tsdf")

PARAMS = [
    Param("mode", "Mode", "enum", "teleop", "primary",
          "Who picks where the vehicle goes. teleop hands control to the "
          "keyboard — the drive keys are always live on this screen, no mode "
          "key needed. goto lets you fly a target point around with the same "
          "keys and has the planner swim to it: A* replans to the point, the "
          "path executor follows. frontier runs autonomous exploration: "
          "frontier detection on the occupancy map, A* planning, then the "
          "same executor. Pressing a drive key in frontier mode suspends the "
          "planner and takes over (Esc hands control back).",
          ["teleop", "goto", "frontier"],
          {"teleop": "Manual keyboard control, driven from this launcher.",
           "goto": "Drive a target point; the planner swims the vehicle to it.",
           "frontier": "Autonomous frontier-based exploration."}),
    Param("mapper", "Mapper", "enum", "octomap", "primary",
          "Map backend. OctoMap is an octree occupancy grid (probabilistic, "
          "raycast free space, 20 cm voxels by default — see Voxel size). "
          "TSDF is VDBFusion: a truncated "
          "signed-distance field on OpenVDB, meshed with marching cubes — "
          "gives surfaces and normals rather than occupied cells. "
          "tsdf_directional is the same backend keeping one volume per "
          "view-direction bin (6, axis-aligned) instead of one shared field. A "
          "surface's sign depends on the side it was seen from, so both faces "
          "of a thin structure land in separate volumes and never average; "
          "reads merge by taking the most solid bin per voxel. This is the fix "
          "the truncation band can only trade against. Costs 6 grid walks per "
          "visualisation cycle, and free/solid disputes resolve toward solid. "
          "WIP: runs, but is not yet validated against the ground-truth map, "
          "so map metrics taken under it are provisional.",
          ["octomap", "tsdf", "tsdf_directional"],
          {"octomap": "octomap_server, /projected_map + occupancy voxels.",
           "tsdf": "VDBFusion/OpenVDB surface reconstruction with normals.",
           "tsdf_directional": "One TSDF volume per view direction; thin "
                               "structure survives (WIP)."},
          wip=["tsdf_directional"]),
    Param("tsdf_octomap", "TSDF → OcTree", "bool", True, "primary",
          "Rebuild the TSDF grid into an octomap::OcTree on /tsdf/octomap_binary "
          "(tsdf_to_octomap), from its occupied and free voxels. Gives the TSDF "
          "backend the octree interface 3-D frontier detection and 3-D A* "
          "expect. Its own topic, not /octomap_binary, so RViz's OcTree displays "
          "stay empty under TSDF and only TSDFVoxels draws the belief map.",
          visible=_tsdf),
    Param("slam", "Pose source", "enum", "none", "primary",
          "Where the vehicle pose comes from. none uses the simulator's exact "
          "pose. slam runs a GTSAM iSAM2 pose graph over simulated "
          "pressure/IMU/DVL dead reckoning with loop closure, so the estimate "
          "drifts and gets corrected like a real system.",
          ["none", "slam"],
          {"none": "Ground-truth TF straight from Stonefish.",
           "slam": "GTSAM iSAM2 pose graph + sonar/nav noise + ATE/RPE eval."}),
    Param("rviz", "RViz", "bool", True, "primary",
          "Start RViz on demo.rviz — one view for every mode (OctoMap and "
          "TSDF, belief and ground truth, drift arrow, error HUD, covariance "
          "ellipsoids). Displays with no publisher in this mode draw nothing."),
    Param("rqt", "Planning Dashboard (RQT)", "bool", False, "primary",
          "Start rqt on the planning dashboard — map, inflation zones, path "
          "and robot/goal state, top-down. Off by default: it is a debugging "
          "view, not part of the demo. Offered wherever the planner runs — "
          "frontier and goto.",
          visible=_planner),
    Param("rqt_depthmap", "Sonar DepthMap (RQT)", "bool", False, "primary",
          "Start rqt on the sonar range image (/cloud_in/range_image) — the "
          "2D depth-camera-style view of what SLAM consumes, noise and all. "
          "In rqt rather than RViz because an RViz Image display unticks "
          "itself whenever its dock is hidden, e.g. moving desktop."),
    Param("keyboard", "Keyboard layout", "enum", "qwerty", "primary",
          "Physical-key mapping for the drive cluster, so the same finger "
          "positions drive on either layout. qwerty uses W/S/Q/E/A/D; azerty "
          "uses the letters those physical keys produce (Z/S/A/E/Q/D), with W "
          "kept as a second forward key. Affects only this launcher's teleop "
          "and target-point driving, live — the stack is not restarted. Set "
          "from the welcome screen, not listed here.",
          ["qwerty", "azerty"],
          {"qwerty": "W fwd, S back, Q/E strafe, A/D yaw.",
           "azerty": "Z fwd, S back, A/E strafe, Q/D yaw (same finger positions)."},
          visible=lambda v: False),

    Param("voxel_size", "Voxel size (m)", "float", 0.2, "mapping",
          "Edge length of one map cell, for whichever backend is selected — "
          "octomap_server's resolution or the TSDF grid's voxel size. Halving "
          "it resolves finer geometry at roughly 8x the voxels, so integration "
          "and the voxel view both get slower. Under TSDF the truncation band "
          "follows at 3x this value unless Truncation band overrides it. The "
          "ground-truth reference map is built at the same size, so the map "
          "metrics keep comparing like with like.",
          step=0.05, lo=0.05, hi=1.0),
    Param("trunc_distance", "Truncation band (m)", "float", 0.0, "mapping",
          "How far either side of a return the signed distance is written, in "
          "metres. 0 follows the voxel size at 3x, the minimum VDBFusion needs "
          "for a usable gradient. This is the hard limit on thin structure: a "
          "wall thinner than twice this band, observed from both faces, has its "
          "two opposing fields averaged into one another and its zero crossing "
          "flattens away — the map goes speckled and holed exactly where it was "
          "seen best. Lower it to map thin plating, at the cost of a coarser "
          "gradient and a noisier surface. Directional TSDF removes the limit "
          "instead of trading against it.",
          step=0.05, lo=0.0, hi=3.0,
          visible=_tsdf),
    Param("space_carving", "Space carving", "bool", True, "mapping",
          "Whether a return frees the whole ray back to the sensor, or only the "
          "band just ahead of the surface. On, the map clears space it has flown "
          "through and unknown volume shrinks faster. Off, a ray that misses a "
          "thin target — past the bow, through a gap — can no longer carve away "
          "the far face that an earlier pass mapped, which is the usual reason a "
          "structure degrades as it is circled rather than improving.",
          visible=_tsdf),
    Param("voxel_min_solid_confidence", "Wall threshold", "float", 0.80, "mapping",
          "How deep behind the reconstructed surface a voxel has to sit before "
          "it counts as a wall, as solid confidence (trunc - d) / (2 * trunc): "
          "0.5 is exactly at the zero crossing, 1.0 is fully saturated solid. "
          "Lowering it calls thinner, less certain returns walls — more "
          "geometry survives, at the cost of noise being mapped as structure. "
          "Applies to the voxel view, the goal-safety cloud and the 2-D "
          "planning map alike, so it moves what A* refuses to route through.",
          step=0.05, lo=0.5, hi=1.0,
          visible=_tsdf),
    Param("voxel_min_weight", "Observations for a wall", "float", 10.0, "mapping",
          "How many times a voxel must be observed before it counts as a wall. "
          "Each integrated scan adds weight, so this is the persistence filter: "
          "raise it and single-scan sonar noise stops becoming structure, lower "
          "it and the map fills in sooner from thinner evidence. Same three "
          "consumers as the wall threshold.",
          step=1.0, lo=1.0, hi=200.0,
          visible=_tsdf),

    Param("scene", "Scene", "enum", "waterlinked", "scene",
          "Stonefish world to launch. waterlinked is the unchanged baseline; "
          "target contains one selectable static object that can be moved live.",
          ["waterlinked", "target"],
          {"waterlinked": "Baseline offshore-station scene.",
           "target": "One static mesh or built-in pipe for sonar demonstrations."}),
    Param("obj_mesh", "Object mesh", "enum", "pipe", "scene",
          "Object to place in the target scene. The launcher scans data/obj "
          "each time it starts, so newly added .obj and .stl files appear here. pipe is "
          "a lightweight built-in primitive.",
          ["pipe"], {"pipe": "Built-in 4 m pipe; fast to load and sonar-visible."},
          visible=lambda v: v["scene"] == "target"),
    Param("obj_x", "Object X (m)", "float", 8.0, "scene",
          "Static object north/X position in world_ned. Changes move the "
          "object live while Stonefish keeps running.",
          step=0.5, lo=-100.0, hi=100.0,
          visible=lambda v: v["scene"] == "target"),
    Param("obj_y", "Object Y (m)", "float", -2.0, "scene",
          "Static object east/Y position in world_ned. Live while running.",
          step=0.5, lo=-100.0, hi=100.0,
          visible=lambda v: v["scene"] == "target"),
    Param("obj_z", "Object Z (m)", "float", 8.0, "scene",
          "Static object down/Z position in world_ned. Live while running.",
          step=0.5, lo=-100.0, hi=100.0,
          visible=lambda v: v["scene"] == "target"),
    Param("obj_scale", "Object scale", "float", 1.0, "scene",
          "Uniform multiplier for the selected object. Applied when the target "
          "scene starts; changing it restarts only Stonefish.",
          step=0.03, lo=0.0001, hi=10.0,
          visible=lambda v: v["scene"] == "target"),
    Param("obj_roll", "Object roll (deg)", "float", 0.0, "scene",
          "Static object roll in degrees. Live while running.",
          step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: v["scene"] == "target"),
    Param("obj_pitch", "Object pitch (deg)", "float", 0.0, "scene",
          "Static object pitch in degrees. Live while running.",
          step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: v["scene"] == "target"),
    Param("obj_yaw", "Object yaw (deg)", "float", 0.0, "scene",
          "Static object yaw in degrees. Live while running.",
          step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: v["scene"] == "target"),

    Param("robot_x", "Robot X (m)", "float", 0.0, "robot",
          "Robot north/X position in world_ned. Live edits teleport the robot; "
          "teleop updates this value from Stonefish ground-truth odometry.",
          step=0.5, lo=-1000.0, hi=1000.0),
    Param("robot_y", "Robot Y (m)", "float", 0.0, "robot",
          "Robot east/Y position in world_ned. Live and read back during teleop.",
          step=0.5, lo=-1000.0, hi=1000.0),
    Param("robot_z", "Robot Z (m)", "float", 8.0, "robot",
          "Robot down/Z position in world_ned. Live and read back during teleop.",
          step=0.5, lo=-1000.0, hi=1000.0),
    Param("robot_roll", "Robot roll (deg)", "float", 0.0, "robot",
          "Robot roll. Live and read back during teleop.",
          step=5.0, lo=-180.0, hi=180.0),
    Param("robot_pitch", "Robot pitch (deg)", "float", 0.0, "robot",
          "Robot pitch. Live and read back during teleop.",
          step=5.0, lo=-180.0, hi=180.0),
    Param("robot_yaw", "Robot yaw (deg)", "float", 0.0, "robot",
          "Robot yaw. Live and read back during teleop.",
          step=5.0, lo=-180.0, hi=180.0),
    Param("robot_depth_target", "Depth target (m)", "float", 8.0, "robot",
          "Fixed NED Z/depth that the autonomous path controller holds while "
          "exploring (positive is below the surface). Also centres the "
          "/projected_map Z band used for frontier detection and A* — "
          "octomap_server's through z_band.py, the TSDF mapper's through "
          "target_depth_m — instead of a full-column projection. Under TSDF the "
          "band re-centres live and the map is kept; the planner reads its "
          "setpoint at startup, so it restarts either way. Teleop vertical "
          "motion remains manual.",
          step=0.5, lo=0.0, hi=1000.0),
    Param("speed_factor", "Speed factor", "float", 1.0, "robot",
          "Multiplies forward/strafe/vertical motion, in teleop (W/S/Q/E/Space/X) "
          "and in frontier mode (whichever motion executor is driving). Commands "
          "saturate at the safety gate's +-1.0 limit, so above roughly 1.1 this "
          "stops making the vehicle faster. Live, no restart.",
          step=0.20, lo=0.1, hi=5.0),
    Param("turn_factor", "Turn factor", "float", 1.0, "robot",
          "Multiplies yaw, in teleop (A/D) and in frontier mode (whichever motion "
          "executor is driving), independent of the speed factor. Also saturates "
          "at the safety gate's +-1.0 limit (roughly 6.7x). Live, no restart.",
          step=0.10, lo=0.1, hi=5.0),
    Param("robot_save_pose_on_exit", "Save robot pose on exit", "bool", False, "robot",
          "When enabled, preserve the robot's final live/teleop pose in config.yaml "
          "when leaving this control screen. When off, teleop motion is display-only "
          "and the configured launch pose is kept."),

    Param("frontier_space", "Frontier space", "enum", "2d", "frontier",
          "Where frontiers — the boundary between known-free and unknown — are "
          "detected. Today that is the 2-D occupancy grid projected around the "
          "cruise depth, so exploration reasons about one horizontal band even "
          "when the map underneath it is volumetric. Detecting them directly on "
          "the map's own voxels would extend exploration to full 3-D structure "
          "and retire the projection.",
          ["2d", "3d"],
          {"2d": "Frontier cells on /projected_map, the depth band around the "
                 "cruise altitude.",
           "3d": "Soon — frontier voxels straight off the TSDF, no projection, "
                 "goals anywhere in the volume."},
          soon=["3d"], visible=_frontier),
    Param("exploration", "Exploration policy", "enum", "frontier", "frontier",
          "What the planner drives toward. Frontier search stops having "
          "anything to say once the frontier set empties, even where the map is "
          "thin; scoring candidate views by the information they are expected "
          "to add keeps exploration going and biases it toward the parts of the "
          "map that are worst reconstructed.",
          ["frontier", "infogain", "nbv"],
          {"frontier": "Frontier clusters only — explore until none are left.",
           "infogain": "Soon — fall through to the lowest-information TSDF "
                       "regions once frontiers are exhausted.",
           "nbv": "Soon — sample viewpoints and score expected sonar coverage "
                  "(receding-horizon next-best-view)."},
          soon=["infogain", "nbv"], visible=_frontier),
    Param("goal_order", "Goal ordering", "enum", "greedy", "frontier",
          "How the next goal is picked out of the candidate set. Greedy takes "
          "the best-scoring cluster each tick and commits to it, which "
          "backtracks over ground it has already crossed. Ordering the top few "
          "into a short tour cuts mission time without touching the mapping or "
          "SLAM layers.",
          ["greedy", "route"],
          {"greedy": "Best cluster per tick (size/path-cost), commit until "
                     "reached.",
           "route": "Soon — order the top-k clusters into a short tour "
                    "(TARE-style 2-opt)."},
          soon=["route"], visible=_frontier),

    Param("motion", "Path executor", "enum", "default", "frontier",
          "How the planned path is followed. default drives straight down the "
          "path. walloriented follows it while yawing toward the nearest "
          "mapped surface. walllooking blends the wall tangent with the path "
          "and needs TSDF normals — it is work in progress: an A/B under it "
          "showed loop closure increasing trajectory error, attributed to "
          "perceptual aliasing along a self-similar wall.",
          ["default", "walloriented", "walllooking"],
          {"default": "Direct path following, heading along travel.",
           "walloriented": "Path following with a fixed yaw offset toward the wall.",
           "walllooking": "WIP — wall-tangent/path blend; consumes TSDF normals."},
          visible=_planner),
    Param("scan_style", "Scan style", "enum", "sweep", "frontier",
          "Rotation used when scanning at a waypoint. sweep is the cable-safe "
          "right-then-left motion (net yaw returns to zero). spin is the "
          "legacy full 360 rotation, which piles up keyframes and inflates "
          "loop-closure counts.",
          ["sweep", "spin"],
          {"sweep": "Right half, left full, return — tether safe.",
           "spin": "Legacy 360 degree rotation."},
          visible=_frontier),
    Param("scenario", "Scenario", "enum", "none", "frontier",
          "Scripted evaluation run. drift_return leaves the start position and "
          "comes back to it, so loop closure can be observed firing on demand "
          "instead of waiting for exploration to revisit somewhere.",
          ["none", "drift_return"],
          {"none": "Free exploration.",
           "drift_return": "Scripted outbound leg then return to start."},
          visible=_frontier),
    Param("hard_inflation_m", "Hard wall zone (m)", "float", 1.00, "frontier",
          "A* hard-wall radius around occupied cells: completely blocked "
          "(cost = inf). Small enough that paths can still pass through "
          "narrow corridors.",
          advanced=True, step=0.05, lo=0.0,
          visible=_frontier),
    Param("inflation_m", "Soft zone (m)", "float", 1.50, "frontier",
          "A* soft-zone radius around occupied cells: high but finite cost, "
          "so A* routes around when a free path exists but can pass through "
          "if forced.",
          advanced=True, step=0.05, lo=0.0,
          visible=_frontier),
    Param("plan_inflation_m", "Planning margin zone (m)", "float", 3.00, "frontier",
          "A* planning-margin radius around occupied cells: moderate cost "
          "to steer paths away from walls while keeping them usable.",
          advanced=True, step=0.05, lo=0.0,
          visible=_frontier),

    Param("noise_profile", "Noise profile (all sensors)", "enum", "realistic", "slam",
          "Which sensor error model feeds the pose graph and the sonar. Also "
          "drives the noise applied to /cloud_in.",
          ["realistic", "realistic_no_reverb", "ideal", "sonar_only",
           "odom_pos_only", "odom_only", "degraded", "degraded_no_reverb"],
          {"realistic": "Datasheet-grounded sonar + nav sensor noise.",
           "realistic_no_reverb": "Realistic with volume reverberation off — no "
                                  "near-field spray, close geometry still mapped.",
           "ideal": "No noise — upper bound / sanity check.",
           "sonar_only": "Realistic sonar, near-ideal nav sensors.",
           "odom_pos_only": "Realistic DVL/pressure, exact orientation and sonar.",
           "odom_only": "Ground-truth sonar, realistic nav sensors.",
           "degraded": "Worst case — stresses loop closure and revisit.",
           "degraded_no_reverb": "Degraded with volume reverberation off — "
                                 "worst-case nav/range error, no near-field spray."},
          visible=_slam),
    Param("noise_profile_sonar", "  Sonar profile", "enum", "inherit", "slam",
          "Override the noise profile for the sonar sensor alone — this is the "
          "map/range error knob. Each sim node reads only its own YAML section, so "
          "isolating one error source needs no merged profile. "
          "inherit = follow the master profile above.",
          ["inherit", "realistic", "realistic_no_reverb", "ideal", "sonar_only",
           "odom_pos_only", "odom_only", "degraded", "degraded_no_reverb"],
          {"inherit": "Follow the master Noise profile."},
          advanced=True, visible=_slam),
    Param("noise_profile_dvl", "  DVL profile", "enum", "inherit", "slam",
          "Override the noise profile for the dvl sensor alone — this is the "
          "position drift knob. Each sim node reads only its own YAML section, so "
          "isolating one error source needs no merged profile. "
          "inherit = follow the master profile above.",
          ["inherit", "realistic", "realistic_no_reverb", "ideal", "sonar_only",
           "odom_pos_only", "odom_only", "degraded", "degraded_no_reverb"],
          {"inherit": "Follow the master Noise profile."},
          advanced=True, visible=_slam),
    Param("noise_profile_imu", "  IMU profile", "enum", "inherit", "slam",
          "Override the noise profile for the imu sensor alone — this is the "
          "angular error (roll/pitch/yaw rate) knob. Each sim node reads only its own YAML section, so "
          "isolating one error source needs no merged profile. "
          "inherit = follow the master profile above.",
          ["inherit", "realistic", "realistic_no_reverb", "ideal", "sonar_only",
           "odom_pos_only", "odom_only", "degraded", "degraded_no_reverb"],
          {"inherit": "Follow the master Noise profile."},
          advanced=True, visible=_slam),
    Param("noise_profile_compass", "  Compass profile", "enum", "inherit", "slam",
          "Override the noise profile for the compass sensor alone — this is the "
          "angular error (absolute heading) knob. Each sim node reads only its own YAML section, so "
          "isolating one error source needs no merged profile. "
          "inherit = follow the master profile above.",
          ["inherit", "realistic", "realistic_no_reverb", "ideal", "sonar_only",
           "odom_pos_only", "odom_only", "degraded", "degraded_no_reverb"],
          {"inherit": "Follow the master Noise profile."},
          advanced=True, visible=_slam),
    Param("noise_profile_pressure", "  Pressure profile", "enum", "inherit", "slam",
          "Override the noise profile for the pressure sensor alone — this is the "
          "depth error knob. Each sim node reads only its own YAML section, so "
          "isolating one error source needs no merged profile. "
          "inherit = follow the master profile above.",
          ["inherit", "realistic", "realistic_no_reverb", "ideal", "sonar_only",
           "odom_pos_only", "odom_only", "degraded", "degraded_no_reverb"],
          {"inherit": "Follow the master Noise profile."},
          advanced=True, visible=_slam),
    Param("noise_attenuation", "Noise Attenuation", "enum", "none", "slam",
          "Post-filter applied over any noise profile, clearing the near-field "
          "volume-reverberation spray around the vehicle so the noised cloud "
          "sits closer to the clean ground truth. cut_close cuts the near field "
          "outright; fade_close thins it, which costs some spray but keeps an "
          "obstacle the vehicle drives up to.",
          ["none", "cut_close", "fade_close"],
          {"none": "Keep every return the profile produces.",
           "cut_close": "Gate out the near-field reverberation spray.",
           "fade_close": "Thin the near field, keeping close geometry visible."},
          visible=_slam),
    Param("near_cutoff_m", "Near-field distance (m)", "float", 1.6, "slam",
          "Extent of the near field the filter acts on: the range cut_close "
          "cuts at, or the range fade_close fades out to. Raise it to reach a "
          "larger spray (e.g. the degraded profile reaches ~2.5 m); lower it to "
          "leave more close geometry alone.",
          advanced=True, step=0.1, lo=0.2, hi=15.0,
          visible=lambda v: _slam(v) and v["noise_attenuation"] != "none"),
    Param("near_fade_p", "Fade strength", "float", 0.8, "slam",
          "fade_close only: probability of dropping a return at the sensor "
          "itself, falling to zero at the near-field distance. 0.8 discards "
          "four of five near returns — enough to clear a sparse spray, while a "
          "surface filling every beam still comes through at a fifth of its "
          "density and refills over successive pings.",
          advanced=True, step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _slam(v) and v["noise_attenuation"] == "fade_close"),
    Param("loop_closure", "Loop closure", "bool", True, "slam",
          "Detect revisited places and add graph constraints that correct "
          "accumulated drift. Turning it off is the A/B baseline: the pose "
          "graph becomes pure dead reckoning.",
          visible=_slam),
    Param("revisit", "Uncertainty revisit", "bool", True, "slam",
          "Suspend the current goal and drive back to mapped areas when the "
          "pose covariance (D-optimality) crosses a threshold, to force a loop "
          "closure. This is the active part of active SLAM. Under goto the "
          "revisit interrupts the run to the target point and the launcher "
          "resends the point once the detour finishes.",
          visible=lambda v: _slam(v) and _planner(v)),
    Param("revisit_scoring", "Revisit scoring", "enum", "keyframe_density", "slam",
          "How a revisit target is chosen once the trigger fires. Keyframe "
          "density is a proxy for 'somewhere the graph already knows well'; it "
          "says nothing about whether the geometry there is distinctive enough "
          "to register against. Suresh et al. score submap saliency instead, so "
          "the detour goes where a loop closure is likely to succeed rather "
          "than merely where the robot has been.",
          ["keyframe_density", "keypoint_density", "fpfh"],
          {"keyframe_density": "Count past keyframes within the candidate "
                               "radius, minus a travel penalty.",
           "keypoint_density": "Soon — score by geometric keypoint density: "
                               "featureful submaps register more reliably than "
                               "flat ones.",
           "fpfh": "Soon — FPFH descriptors into a k-means vocabulary with idf "
                   "rarity weighting, so the target is distinctive, not just "
                   "busy."},
          soon=["keypoint_density", "fpfh"],
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("revisit_trigger", "Revisit trigger", "enum", "live_dopt", "slam",
          "What decides that it is time to break off and close a loop. The "
          "live D-optimality scalar crossing a threshold is reactive: it fires "
          "on uncertainty already accumulated and cannot compare one candidate "
          "against another. Propagating covariance along each candidate path "
          "would turn the decision into a cost/benefit one — expected "
          "uncertainty reduction against the detour it costs.",
          ["live_dopt", "propagated"],
          {"live_dopt": "Live D(Sigma)/D(Sigma_allow) above the trigger ratio.",
           "propagated": "Soon — virtual factors on a mirrored graph project "
                         "uncertainty forward per candidate before choosing."},
          soon=["propagated"],
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("sigma_allow_xy_m", "Allowable XY sigma", "float", 0.045, "slam",
          "Largest horizontal position uncertainty the mission tolerates, in "
          "metres. With the yaw sigma it defines D(Sigma_allow), the level the "
          "live D-optimality is divided by to get the trigger ratio. Set it "
          "from what the mission actually needs, not from the raw D-opt scale. "
          "Live-tunable while running.",
          advanced=True, step=0.01, lo=0.001, hi=5.0,
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("sigma_allow_yaw_rad", "Allowable yaw sigma", "float", 0.045, "slam",
          "Largest heading uncertainty the mission tolerates, in radians. "
          "Live-tunable while running.",
          advanced=True, step=0.01, lo=0.001, hi=3.0,
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("ratio_trigger", "Revisit trigger ratio", "float", 1.0, "slam",
          "Uncertainty ratio U_r = D(Sigma)/D(Sigma_allow) that suspends the "
          "current goal and drives back to close a loop. 1.0 means revisit "
          "exactly when the estimate is less certain than the mission allows; "
          "lower is more cautious. Shown next to the live ratio on the right. "
          "Live-tunable while running.",
          advanced=True, step=0.1, lo=0.0, hi=20.0,
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("ratio_resume", "Revisit resume ratio", "float", 0.5, "slam",
          "Uncertainty ratio below which the run resumes after a revisit. "
          "Must stay below the trigger ratio or revisit will not exit. "
          "Live-tunable while running.",
          advanced=True, step=0.1, lo=0.0, hi=20.0,
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("revisit_min_closures", "Revisit min closures", "int", 0, "slam",
          "Loop closures required before a revisit ends on closure count "
          "alone. 0 (default) means the count is not taken into account, so "
          "the revisit ends only when the uncertainty ratio falls below the "
          "resume ratio (or on timeout). Ending on the first closure resumes "
          "exploring while sigma is still near the allowance, because one "
          "closure rarely restores the covariance. "
          "Live-tunable while running.",
          advanced=True, step=1, lo=0, hi=10,
          visible=lambda v: _slam(v) and _planner(v) and v["revisit"]),
    Param("map_rebuild", "Rebuild map on closure", "bool", False, "slam",
          "After a large loop closure, reset the TSDF and re-integrate every "
          "keyframe at its corrected pose, so the map geometry is fixed too "
          "rather than just the trajectory. TSDF only — OctoMap cannot "
          "reproduce its raycast free space this way.",
          visible=lambda v: _slam(v) and _tsdf(v)),
    Param("noise_seed", "Noise seed", "int", -1, "slam",
          "Seed for the noise draws. -1 uses the profile's own seed; set an "
          "explicit value for reproducible or decorrelated repeat runs.",
          advanced=True, visible=_slam),
    Param("output_dir", "Output dir", "text", "", "slam",
          "Where the eval stack writes TUM trajectories and CSV metrics. "
          "Empty means a timestamped directory under eval/runs/.",
          advanced=True, visible=_slam),

    Param("safety_start_enabled", "Safety gate starts enabled", "bool", False, "advanced",
          "Start the motion safety gate already enabled. Normally left off so "
          "motion is armed deliberately from the RViz panel after checking the "
          "scene. The gate is fail-closed: it blocks commands that are stale, "
          "oversized, or missing odometry.",
          advanced=True),
    Param("thrust_boost", "Thrust boost (2.5x ceiling)", "bool", False, "advanced",
          "Scale the simulated thruster RPM ceiling 2.5x for fast repositioning "
          "between runs. Body commands still normalize to the safety gate's "
          "+-1.0 range, so this never trips INVALID_COMMAND — it raises what "
          "100% effort means physically, not the command range. Not a "
          "physically realistic BlueROV2/T200 value; restarts the simulator.",
          advanced=True),

    Param("scan_sweep_deg", "Sweep width", "float", 180.0, "frontier",
          "Total sweep angle in degrees for scan_style:=sweep.",
          advanced=True, step=5.0, lo=10.0, hi=360.0,
          visible=lambda v: _frontier(v) and v["scan_style"] == "sweep"),
    Param("scenario_out_dx", "Outbound dx (m)", "float", 15.0, "frontier",
          "drift_return: outbound leg X offset from the captured start pose.",
          advanced=True, step=1.0,
          visible=lambda v: _frontier(v) and v["scenario"] == "drift_return"),
    Param("scenario_out_dy", "Outbound dy (m)", "float", 0.0, "frontier",
          "drift_return: outbound leg Y offset from the captured start pose.",
          advanced=True, step=1.0,
          visible=lambda v: _frontier(v) and v["scenario"] == "drift_return"),
    Param("wall_orientation_offset_deg", "Wall yaw offset", "float", 30.0, "frontier",
          "Degrees to yaw away from the travel bearing toward the nearest "
          "mapped surface, so the sonar keeps the wall in view while moving "
          "along the path.",
          advanced=True, step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: _planner(v) and v["motion"] == "walloriented"),
    Param("wall_orientation_lookahead_m", "Wall lookahead (m)", "float", 0.0, "frontier",
          "Lookahead radius along the A* path used to pick the heading. 0 uses "
          "the heading at the current position.",
          advanced=True, step=0.5, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walloriented"),
    Param("tsdf_frontier_standoff_m", "TSDF frontier standoff (m)", "float", 1.0, "frontier",
          "How far off the reconstructed surface, along its outward normal, a "
          "TSDF frontier goal is placed — keeps goals in free water.",
          advanced=True, step=0.1, lo=0.0,
          visible=lambda v: _frontier(v) and _tsdf(v)),
    Param("wall_standoff", "Wall standoff (m)", "float", 1.5, "frontier",
          "Target distance to hold from the wall while wall-looking. "
          "Live-tunable while running.",
          advanced=True, step=0.1, lo=0.1,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_switch_goal_distance", "Wall-switch goal dist (m)", "float", 6.0, "frontier",
          "When the planner goal is nearer than this, look for another wall "
          "instead of continuing along the current one. Live-tunable.",
          advanced=True, step=0.5, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_switch_scan_angle", "Wall-switch sweep (rad)", "float", 3.14159, "frontier",
          "Sweep angle used when searching for the next wall. Live-tunable.",
          advanced=True, step=0.1, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_switch_scan_yaw", "Wall-switch sweep yaw", "float", 0.08, "frontier",
          "Yaw rate command used during that sweep. Live-tunable.",
          advanced=True, step=0.01, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_path_influence", "Path influence", "float", 0.70, "frontier",
          "Travel-direction blend: 0 follows the wall tangent, 1 follows the "
          "planned path. Live-tunable.",
          advanced=True, step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_path_look_offset_deg", "Path look offset", "float", 30.0, "frontier",
          "Degrees to turn the path-derived look heading toward the wall. "
          "Live-tunable.",
          advanced=True, step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_normal_offset_deg", "Wall normal offset", "float", 0.0, "frontier",
          "Degrees to turn the wall-facing normal back toward the path "
          "bearing. Live-tunable.",
          advanced=True, step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_path_heading_weight", "Look blend", "float", 0.35, "frontier",
          "Orientation blend: 0 uses the wall-derived heading, 1 the "
          "path-derived heading. Live-tunable.",
          advanced=True, step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
]

PARAM_MAP = {p.id: p for p in PARAMS}
DEFAULTS = {p.id: p.default for p in PARAMS}
DEFAULTS[HIDDEN_SECTIONS_KEY] = list(DEFAULT_HIDDEN_SECTIONS)


# demo.launch.py's name for an option, where it differs from the launcher's id.
LAUNCH_ARG_ALIASES = {"robot_depth_target": "depth"}
# Options demo.launch.py has no argument for: launcher-only UI state, the
# noise attenuation switch (which it expresses as near_cutoff/near_fade), and
# the roadmap options below, whose implemented value is the only behaviour
# there is — nothing downstream reads them.
LAUNCH_ARG_SKIP = {"keyboard", "robot_save_pose_on_exit", "rqt", "rqt_depthmap",
                   "thrust_boost", "noise_attenuation", "near_cutoff_m",
                   "near_fade_p",
                   "frontier_space", "exploration", "goal_order",
                   "revisit_scoring", "revisit_trigger"}


def unimplemented(values):
    """Selected options that name planned work, as (param, value) pairs.

    These choices exist so the roadmap is visible where the system is actually
    driven, next to the option they will change. They are not runnable, so the
    launcher refuses to apply a configuration holding one rather than starting
    a stack that quietly does something else.
    """
    return [(p, values[p.id]) for p in PARAMS
            if p.soon and p.is_soon(values.get(p.id))]


def launch_command(values):
    """The `ros2 launch` line that reproduces this configuration.

    Only options that differ from their default are listed, so the line stays
    readable on one row. The launcher itself runs the per-group launch files
    rather than this command; it is the equivalent single-shot invocation.
    """
    parts = ["ros2 launch bringup demo.launch.py"]
    for p in PARAMS:
        if p.id in LAUNCH_ARG_SKIP or values.get(p.id, p.default) == p.default:
            continue
        value = values[p.id]
        if isinstance(value, bool):
            value = "true" if value else "false"
        if p.id == "mapper" and value == "tsdf_directional":
            # demo.launch.py has no directional backend: it is mapper:=tsdf
            # plus a flag on the volume it builds.
            parts.append("mapper:=tsdf directional_tsdf:=true")
            continue
        parts.append(f"{LAUNCH_ARG_ALIASES.get(p.id, p.id)}:={value}")
    if values.get("noise_attenuation") == "cut_close":
        parts.append(f"near_cutoff:={values['near_cutoff_m']}")
    elif values.get("noise_attenuation") == "fade_close":
        parts.append(f"near_fade:={values['near_fade_p']}")
        parts.append(f"near_fade_range:={values['near_cutoff_m']}")
    return " ".join(parts)


def configure_object_meshes(meshes):
    """Populate the TUI's object line from data/obj at launcher startup."""
    names = sorted({name for name in meshes if isinstance(name, str)
                    and name.lower().endswith((".obj", ".stl"))})
    param = PARAM_MAP["obj_mesh"]
    param.choices = ["pipe", *names]
    param.choice_help = {
        "pipe": "Built-in 4 m pipe; fast to load and sonar-visible.",
        **{name: "Static mesh from sim/world/data/obj." for name in names},
    }

PRESETS = [
    ("Teleop + OctoMap (default)", {}),
    ("Autonomous frontier (TSDF)", {"mode": "frontier", "mapper": "tsdf"}),
    ("Wall-oriented exploration",
     {"mode": "frontier", "mapper": "tsdf", "motion": "walloriented"}),
    ("Wall-looking exploration (WIP)",
     {"mode": "frontier", "mapper": "tsdf", "motion": "walllooking"}),
    ("SLAM benchmark (realistic)",
     {"slam": "slam", "mode": "frontier", "noise_profile": "realistic"}),
    ("Headless / CI", {"mode": "frontier", "rviz": False}),
]

TELEOP_KEYS = {
    "qwerty": "W/S fwd/back  Q/E strafe  A/D yaw  Z/X up/down  F stop",
    "azerty": "Z/S fwd/back  A/E strafe  Q/D yaw  W/X up/down  F stop",
}
# goto mode: the same cluster moves the target point instead of the vehicle,
# but along world axes (a point has no heading).
POINT_KEYS = {
    "qwerty": "W/S X  A/D Y  Q/E Z  F recalls it to the vehicle",
    "azerty": "Z/S X  Q/D Y  A/E Z  F recalls it to the vehicle",
}

INFOS = [
    ("What this is", [
        "Active SLAM for underwater volumetric exploration — an MSc",
        "dissertation extending Suresh et al. (IEEE ICRA 2020) with a",
        "wide-FoV 3D sonar, FPFH submap descriptors and a TSDF backend.",
        "Heriot-Watt University, Ocean Systems Lab.",
    ]),
    ("Simulation", [
        "Stonefish        underwater dynamics, sensors and rendering,",
        "                 run from a patched fork (depth-camera vertical",
        "                 FoV + physics-thread capture timestamps).",
        "BlueROV2         vehicle model, thrusters and 3D sonar proxy",
        "                 (depth camera at 90 x 40 degrees).",
        "ROS 2 Jazzy      middleware; the stack is one launch graph.",
    ]),
    ("Mapping", [
        "OctoMap          probabilistic octree occupancy grid, 20 cm",
        "                 voxels by default, raycast free space",
        "                 (default backend).",
        "VDBFusion        truncated signed-distance field on OpenVDB,",
        "                 meshed with marching cubes — gives surfaces",
        "                 and normals (mapper:=tsdf).",
        "depth_image_proc depth image to organised point cloud.",
        "",
        "A TSDF's sign says which side a surface was seen from, so",
        "structure thinner than twice the truncation band degrades as",
        "it is circled: the two faces average each other out and the",
        "zero crossing flattens. Truncation band trades against this;",
        "tsdf_directional (WIP) splits the bins so it cannot happen.",
    ]),
    ("SLAM", [
        "GTSAM iSAM2      incremental pose-graph optimisation.",
        "Dead reckoning   simulated pressure, IMU and DVL fused into an",
        "                 odometry estimate that drifts realistically.",
        "Loop closure     revisit detection adds constraints that pull",
        "                 the trajectory (and optionally the map) back.",
        "Sonar noise      WaterLinked Sonar 3D-15 datasheet model.",
    ]),
    ("Planning", [
        "Frontier search  boundary between known-free and unknown space",
        "                 on the occupancy map.",
        "A*               grid path planning to the selected frontier.",
        "Executors        direct, wall-oriented, or wall-looking (WIP;",
        "                 consumes TSDF surface normals).",
        "Safety gate      fail-closed: blocks stale, oversized or",
        "                 odometry-less commands.",
    ]),
    ("Evaluation", [
        "ATE / RPE        absolute and relative trajectory error against",
        "                 the simulator's exact pose.",
        "Map metrics      IoU and coverage (OctoMap), chamfer distance",
        "                 and coverage (TSDF), against a ground-truth map",
        "                 built from the exact pose in parallel.",
        "TUM export       trajectories written to eval/runs/<timestamp>/.",
    ]),
    ("Coming next", [
        "Marked (soon) in the option list, at the option each one",
        "changes. Selecting one describes it; applying is refused.",
        "",
        "3-D frontiers    detect frontiers on TSDF voxels instead of",
        "                 the projected 2-D band (Frontier space).",
        "Info-gain / NBV  keep exploring past the last frontier by",
        "                 expected information (Exploration policy).",
        "Route ordering   a short tour over the top-k clusters rather",
        "                 than greedy per tick (Goal ordering).",
        "Submap saliency  keypoint density, then FPFH descriptors with",
        "                 idf rarity, to pick revisit targets that will",
        "                 actually register (Revisit scoring).",
        "Propagated cov.  project uncertainty along each candidate path",
        "                 instead of thresholding the live scalar",
        "                 (Revisit trigger).",
    ]),
]
