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
    "dopt_trigger":               ("revisit_planner", "dopt_trigger"),
    "dopt_resume":                ("revisit_planner", "dopt_resume"),
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
MOTION_EXECUTOR_NODE = {
    "default":      "waypoint_controller",
    "walloriented": "wall_oriented_controller",
    "walllooking":  "wall_looking",
}


class Param:
    def __init__(self, id, label, kind, default, section, description,
                 choices=None, choice_help=None, advanced=False, step=None,
                 visible=lambda v: True, lo=None, hi=None):
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
    "scene": "Scene + object",
    "robot": "Robot pose",
    "frontier": "Planner",
    "slam": "SLAM / pose source",
    "advanced": "Other",
}
SECTION_ORDER = ["primary", "scene", "robot", "frontier", "slam", "advanced"]

_frontier = lambda v: v["mode"] == "frontier"
# goto and frontier both run the planner layer (A* + path executor); they
# differ only in who picks the goal.
_planner = lambda v: v["mode"] in ("frontier", "goto")
_slam = lambda v: v["slam"] == "slam"

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
          "raycast free space, 20 cm voxels). TSDF is VDBFusion: a truncated "
          "signed-distance field on OpenVDB, meshed with marching cubes — "
          "gives surfaces and normals rather than occupied cells.",
          ["octomap", "tsdf"],
          {"octomap": "octomap_server, /projected_map + occupancy voxels.",
           "tsdf": "VDBFusion/OpenVDB surface reconstruction with normals."}),
    Param("tsdf_octomap", "TSDF → OcTree", "bool", True, "primary",
          "Rebuild the TSDF grid into an octomap::OcTree on /octomap_binary "
          "(tsdf_to_octomap), from its occupied and free voxels. Gives the TSDF "
          "backend the octree interface 3-D frontier detection and 3-D A* "
          "expect, and makes the belief map visible under RViz's OctoMap "
          "displays.",
          visible=lambda v: v["mapper"] == "tsdf"),
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
    Param("rqt", "rqt", "bool", False, "primary",
          "Start rqt alongside RViz for introspection — node graph, topic "
          "monitor, live plots and parameter reconfigure. Off by default: it "
          "is a debugging tool, not part of the demo view."),
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
          "target_depth_m — instead of a full-column projection. Changing it "
          "restarts the mapper and the planner. Teleop vertical motion remains "
          "manual.",
          step=0.5, lo=0.0, hi=1000.0),
    Param("speed_factor", "Speed factor", "float", 1.0, "robot",
          "Multiplies forward/strafe/vertical motion, in teleop (W/S/Q/E/Space/X) "
          "and in frontier mode (whichever motion executor is driving). Commands "
          "saturate at the safety gate's +-1.0 limit, so above roughly 1.1 this "
          "stops making the vehicle faster. Live, no restart.",
          step=0.25, lo=0.1, hi=5.0),
    Param("turn_factor", "Turn factor", "float", 1.0, "robot",
          "Multiplies yaw, in teleop (A/D) and in frontier mode (whichever motion "
          "executor is driving), independent of the speed factor. Also saturates "
          "at the safety gate's +-1.0 limit (roughly 6.7x). Live, no restart.",
          step=0.25, lo=0.1, hi=5.0),
    Param("robot_save_pose_on_exit", "Save robot pose on exit", "bool", False, "robot",
          "When enabled, preserve the robot's final live/teleop pose in config.yaml "
          "when leaving this control screen. When off, teleop motion is display-only "
          "and the configured launch pose is kept."),

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
    Param("hard_inflation_m", "Hard wall zone (m)", "float", 0.20, "frontier",
          "A* hard-wall radius around occupied cells: completely blocked "
          "(cost = inf). Small enough that paths can still pass through "
          "narrow corridors.",
          advanced=True, step=0.05, lo=0.0,
          visible=_frontier),
    Param("inflation_m", "Soft zone (m)", "float", 0.75, "frontier",
          "A* soft-zone radius around occupied cells: high but finite cost, "
          "so A* routes around when a free path exists but can pass through "
          "if forced.",
          advanced=True, step=0.05, lo=0.0,
          visible=_frontier),
    Param("plan_inflation_m", "Planning margin zone (m)", "float", 1.50, "frontier",
          "A* planning-margin radius around occupied cells: moderate cost "
          "to steer paths away from walls while keeping them usable.",
          advanced=True, step=0.05, lo=0.0,
          visible=_frontier),

    Param("noise_profile", "Noise profile", "enum", "realistic", "slam",
          "Which sensor error model feeds the pose graph and the sonar. Also "
          "drives the noise applied to /cloud_in.",
          ["realistic", "ideal", "sonar_only", "odom_pos_only", "odom_only", "degraded"],
          {"realistic": "Datasheet-grounded sonar + nav sensor noise.",
           "ideal": "No noise — upper bound / sanity check.",
           "sonar_only": "Realistic sonar, near-ideal nav sensors.",
           "odom_pos_only": "Realistic DVL/pressure, exact orientation and sonar.",
           "odom_only": "Ground-truth sonar, realistic nav sensors.",
           "degraded": "Worst case — stresses loop closure and revisit."},
          visible=_slam),
    Param("noise_attenuation", "Noise Attenuation", "enum", "none", "slam",
          "Post-filter applied over any noise profile. cut_close drops sonar "
          "returns nearer than ~1.6 m, clearing the near-field volume-"
          "reverberation spray around the vehicle so the noised cloud sits "
          "closer to the clean ground truth.",
          ["none", "cut_close"],
          {"none": "Keep every return the profile produces.",
           "cut_close": "Gate out the near-field reverberation spray."},
          visible=_slam),
    Param("near_cutoff_m", "Cut distance (m)", "float", 1.6, "slam",
          "cut_close only: drop sonar returns nearer than this. Raise it to "
          "clear a larger near-field spray (e.g. the degraded profile reaches "
          "~2.5 m); lower it to keep more close geometry.",
          advanced=True, step=0.1, lo=0.2, hi=15.0,
          visible=lambda v: _slam(v) and v["noise_attenuation"] == "cut_close"),
    Param("loop_closure", "Loop closure", "bool", True, "slam",
          "Detect revisited places and add graph constraints that correct "
          "accumulated drift. Turning it off is the A/B baseline: the pose "
          "graph becomes pure dead reckoning.",
          visible=_slam),
    Param("revisit", "Uncertainty revisit", "bool", True, "slam",
          "Suspend exploration and drive back to mapped areas when the pose "
          "covariance (D-optimality) crosses a threshold, to force a loop "
          "closure. This is the active part of active SLAM.",
          visible=lambda v: _slam(v) and _frontier(v)),
    Param("dopt_trigger", "D-opt trigger threshold", "float", 0.02, "slam",
          "D-optimality [det(cov_pos)^(1/3)] level that suspends exploration and "
          "drives back to close a loop. Lower triggers revisit sooner (more "
          "cautious); higher lets more drift accumulate before correcting. "
          "Live-tunable while running.",
          advanced=True, step=0.005, lo=0.0, hi=1.0,
          visible=lambda v: _slam(v) and _frontier(v) and v["revisit"]),
    Param("dopt_resume", "D-opt resume threshold", "float", 0.01, "slam",
          "D-optimality level below which exploration resumes after a revisit. "
          "Must stay below the trigger threshold or revisit will not exit. "
          "Live-tunable while running.",
          advanced=True, step=0.005, lo=0.0, hi=1.0,
          visible=lambda v: _slam(v) and _frontier(v) and v["revisit"]),
    Param("map_rebuild", "Rebuild map on closure", "bool", False, "slam",
          "After a large loop closure, reset the TSDF and re-integrate every "
          "keyframe at its corrected pose, so the map geometry is fixed too "
          "rather than just the trajectory. TSDF only — OctoMap cannot "
          "reproduce its raycast free space this way.",
          visible=lambda v: _slam(v) and v["mapper"] == "tsdf"),
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
          visible=lambda v: _frontier(v) and v["mapper"] == "tsdf"),
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
    "qwerty": "W/S fwd/back  Q/E strafe  A/D yaw  Space/X up/down  F stop",
    "azerty": "Z/S fwd/back  A/E strafe  Q/D yaw  Space/X up/down  F stop",
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
        "                 voxels, raycast free space (default backend).",
        "VDBFusion        truncated signed-distance field on OpenVDB,",
        "                 meshed with marching cubes — gives surfaces",
        "                 and normals (mapper:=tsdf).",
        "depth_image_proc depth image to organised point cloud.",
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
]
