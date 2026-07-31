#!/usr/bin/env python3
"""
Parameter surface, presets and reference text for launcher.py.

Split from the UI so the option table can be checked against demo.launch.py's
declared arguments without a terminal. Pure standard library.

The option list is a two-level tree: the three parts of active SLAM
(localisation, mapping, path planning) plus the scene they run in and the
tools around them. Each category opens with the one option that says which
implementation is running — pose source, mapper, mode — and everything that
tunes that implementation sits in a subcategory under it.
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
    "wall_z_band_m":              ("wall_oriented_controller", "wall_z_band_m"),
    "arrival_dwell_s":            ("revisit_planner", "arrival_dwell_s"),
    "stall_exit_s":               ("revisit_planner", "stall_exit_s"),
    "wall_standoff":             ("wall_looking", "standoff_m"),
    "wall_max_surface_dist":     ("wall_looking", "max_surface_dist_m"),
    "wall_switch_goal_distance": ("wall_looking", "switch_goal_distance_m"),
    "wall_switch_scan_angle":    ("wall_looking", "switch_scan_angle_rad"),
    "wall_switch_scan_yaw":      ("wall_looking", "switch_scan_yaw"),
    "wall_path_influence":       ("wall_looking", "path_influence"),
    "wall_path_look_offset_deg": ("wall_looking", "path_look_offset_deg"),
    "wall_normal_offset_deg":    ("wall_looking", "wall_normal_offset_deg"),
    "wall_path_heading_weight":  ("wall_looking", "path_heading_weight"),
    "wall_yaw_only":             ("wall_looking", "yaw_only"),
}

# These object-pose fields are applied through Stonefish's /set_entity_pose
# service rather than a ROS parameter.  They stay static bodies, preserving
# Stonefish's optimised collision/sensor path; scale remains launch-time only.
LIVE_SCENE_POSE = {
    "obj_x", "obj_y", "obj_z", "obj_roll", "obj_pitch", "obj_yaw",
}

# Robot pose is read back from Stonefish odometry while it moves and can be
# applied live through the existing respawn service — but only when the
# operator has asked for that with robot_pose_live, since an edit then
# teleports a running vehicle. See is_live().
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


def is_live(p, values, applied=None):
    """Whether a change to this option reaches the running stack without a
    restart.

    Values-dependent: the map thresholds are live under TSDF only. A robot
    pose edit teleports the vehicle, so it is live only once the stack has
    actually been told to allow that — `applied` is the configuration the
    running stack was last started with, and ticking the switch does nothing
    until Enter commits it.
    """
    if p.id in LIVE_ROBOT_POSE:
        source = values if applied is None else applied
        return bool(source.get("robot_pose_live"))
    return bool(p.live or mapper_live_targets(p.id, values))


MOTION_EXECUTOR_NODE = {
    "default":      "waypoint_controller",
    "walloriented": "wall_oriented_controller",
    "walllooking":  "wall_looking",
}


class Param:
    def __init__(self, id, label, kind, default, section, description,
                 choices=None, choice_help=None, step=None,
                 visible=lambda v: True, lo=None, hi=None, soon=(), wip=(),
                 cycle_skip=()):
        self.id = id
        self.label = label
        self.kind = kind                  # enum, bool, int, float, text
        self.default = default
        self.section = section            # section id, dotted for a subsection
        self.description = description
        self.choices = choices or []
        self.choice_help = choice_help or {}
        self.step = step if step is not None else (1 if kind == "int" else 0.05)
        self.visible = visible
        self.lo = lo
        self.hi = hi
        # Choices that name planned work rather than a runnable configuration.
        self.soon = frozenset(soon)
        # Choices that do run but are not validated against the ground-truth
        # map yet. Unlike soon, these apply — the tag says results are provisional.
        self.wip = frozenset(wip)
        # Choices the value can hold but cycling steps over: a state the
        # configuration lands in rather than one worth selecting.
        self.cycle_skip = frozenset(cycle_skip)

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


class Section:
    """One heading in the option list.

    A subsection names its parent; the tree is two deep, which is as far as a
    heading can be indented before the option under it stops looking like an
    option. Declaration order is display order, and a category's own options
    are drawn before its subsections.
    """

    def __init__(self, id, title, description, parent=None):
        self.id = id
        self.title = title
        # What lives here and why it is one group — the heading's own line in
        # the description pane. Not how many options it holds or how to unfold
        # it: the row already shows both, and the key line says the rest.
        self.description = description
        self.parent = parent
        self.depth = 0 if parent is None else 1


SECTIONS = [
    Section("preset", "0. Platform & preset",
            "What the stack is driving — the simulator or a real vehicle — and "
            "which rung of the evaluation ladder it stands on. Each rung adds "
            "a single piece of the SLAM stack to the one before it, so the "
            "difference between two runs is that piece alone."),

    Section("localisation", "1. Localisation",
            "Where the vehicle thinks it is: the sensors dead reckoning is "
            "built from, the pose graph that fuses them, and the motion that "
            "decides what those sensors get to see in the first place."),
    Section("localisation.sensors", "Sensors",
            "What each simulated navigation sensor reports. They are separate "
            "error sources, so a run can degrade one of them and leave the "
            "rest alone to see what that costs.", "localisation"),
    Section("localisation.graph", "Pose graph",
            "How the graph is built: how far the vehicle travels before a "
            "keyframe is laid down, and which pairs of keyframes are allowed "
            "to close a loop between them.", "localisation"),
    Section("localisation.noise", "Factor noise",
            "What the graph assumes its measurements are worth. These weight "
            "every factor in the optimisation; the ANEES/NIS pair on the "
            "right is the test of whether the assumption matches what the "
            "sensors are actually doing.", "localisation"),
    Section("localisation.revisit", "Active revisit",
            "When to break off exploring and drive back somewhere the graph "
            "already knows, to force a loop closure before the estimate drifts "
            "past what the mission allows. The active half of active SLAM.",
            "localisation"),
    Section("localisation.motion", "Motion",
            "How the vehicle moves: speed, the depth it holds, how a planned "
            "path is followed and how it turns to scan at a waypoint. Motion "
            "is what a sonar gets to see, which is why it sits here.",
            "localisation"),
    Section("localisation.executor", "Executor tuning",
            "Tuning for the path executor selected above, and only that one — "
            "each executor steers on different geometry, so the options here "
            "change with it.", "localisation"),

    Section("mapping", "2. Mapping",
            "What the world looks like: the sonar the map is built from, the "
            "volume it is built into, and what counts as a surface once it is."),
    Section("mapping.sensors", "Sensors",
            "What the simulated sonar reports, and the filter applied to its "
            "returns before anything integrates them.", "mapping"),
    Section("mapping.grid", "Grid",
            "The volume itself: cell size, how far either side of a return the "
            "distance field is written, and whether a ray clears the water it "
            "passed through. The map is constructed from these, so changing "
            "one discards it.", "mapping"),
    Section("mapping.surface", "Surface thresholds",
            "Where the reconstructed surface becomes a wall. The same answer "
            "drives the voxel view, the goal-safety cloud and what A* refuses "
            "to route through.", "mapping"),
    Section("mapping.outputs", "Outputs",
            "What the mapper publishes besides the map itself, and what it "
            "does with the map after a loop closure moves the poses it was "
            "built from.", "mapping"),

    Section("planning", "3. Path planning",
            "Where to go next: who picks the goal, how it is chosen out of the "
            "map, and the 2-D slice the route to it is planned on."),
    Section("planning.goals", "Goal selection",
            "How the next goal comes out of the map, and how far exploration "
            "is allowed to range in search of one.", "planning"),
    Section("planning.map", "Planning map",
            "The 2-D slice A* and frontier detection both work on: how thick "
            "it is, and how close to a wall a route may pass.", "planning"),

    # Shares the number with Scene: both answer "what world does this run
    # happen in", and the platform switch means only one is ever on screen.
    Section("hardware", "4. Real vehicle",
            "The links and hardware geometry a real run needs — the two "
            "MAVLink endpoints, how much authority ArduSub is given, and where "
            "the sonar sits on the vehicle."),
    Section("hardware.link", "MAVLink",
            "The two BlueOS endpoints this stack opens: one to send body "
            "demands to ArduSub, one to read its navigation estimate back. "
            "pymavlink binds each UDP port, so they cannot share one.",
            "hardware"),
    Section("hardware.sonar", "Sonar mount",
            "Where the Sonar 3D-15 is on the vehicle. The mapper integrates "
            "clouds through this transform, so an error here is an error in "
            "the map that no amount of SLAM will remove.", "hardware"),

    Section("scene", "4. Scene",
            "The world the run happens in and where things start in it — the "
            "Stonefish world, the object under inspection, and the vehicle's "
            "spawn pose."),
    Section("scene.object", "Object",
            "The static mesh the target scene places. Position and orientation "
            "move it while the simulator runs; scale is set when the scene is "
            "built.", "scene"),
    Section("scene.robot", "Robot",
            "Where the vehicle is put: at start, at reset, and on demand while "
            "running. One-way — these place it, they never report where it has "
            "got to. The pose table on the right does that.", "scene"),

    Section("tools", "5. Tools",
            "Everything around the stack rather than in it: the viewers, and "
            "what a run writes down."),
    Section("tools.eval", "Evaluation",
            "What a run records and how repeatable it is — where trajectories "
            "and metrics are written, the seed the noise is drawn from, and "
            "the scripted runs.", "tools"),
]

SECTION_ORDER = [s.id for s in SECTIONS]
SECTION_TITLES = {s.id: s.title for s in SECTIONS}
SECTION_DESCRIPTIONS = {s.id: s.description for s in SECTIONS}


def section_pane_text(section_id):
    """The description pane for a heading: what is selected, then what it holds.

    The name leads, as it does on an option row — the pane is read after the
    eye has left the list, so it has to say what it is describing.
    """
    return f"{SECTION_TITLES[section_id]} — {SECTION_DESCRIPTIONS[section_id]}"
SECTION_PARENT = {s.id: s.parent for s in SECTIONS}
SECTION_DEPTH = {s.id: s.depth for s in SECTIONS}

# Which sections are folded away behind their < SHOW > heading. Persisted in
# config.yaml next to the options, so a layout survives a restart. Every
# subsection starts folded and every category open: what is left is one line
# per category saying which implementation it runs, which is the whole
# configuration at a glance.
HIDDEN_SECTIONS_KEY = "hidden_sections"
DEFAULT_HIDDEN_SECTIONS = [s.id for s in SECTIONS if s.parent is not None]

PLATFORM_SIM = "stonefish"
PLATFORM_REAL = "real_life"

# Everything downstream of the simulator is gated on this rather than on the
# option it would otherwise read: a hidden option keeps its last value (see
# matching_preset), so slam:=slam left over from a sim run must not be able to
# start a pose graph fed by topics no real vehicle publishes.
_sim = lambda v: v.get("platform", PLATFORM_SIM) == PLATFORM_SIM
_real = lambda v: v.get("platform", PLATFORM_SIM) == PLATFORM_REAL

_frontier = lambda v: _sim(v) and v["mode"] == "frontier"
# goto and frontier both run the planner layer (A* + path executor); they
# differ only in who picks the goal.
_planner = lambda v: _sim(v) and v["mode"] in ("frontier", "goto")
_slam = lambda v: v["slam"] == "slam"
# Injected sensor noise and every ground-truth-derived metric are simulator
# properties; the pose graph consuming them is not.
_sim_slam = lambda v: _sim(v) and _slam(v)
# Both TSDF choices run the same VDBFusion backend; they differ only in how
# many volumes it keeps.
_tsdf = lambda v: v["mapper"].startswith("tsdf")
# A revisit exists to force a loop closure, so closure off takes the switch and
# every option under it off the screen.
_revisit_armed = lambda v: _slam(v) and _planner(v) and v["loop_closure"]
_revisit = lambda v: _revisit_armed(v) and v["revisit"]
_target_scene = lambda v: _sim(v) and v["scene"] == "target"
# The sonar mount only matters where a physical sonar has to be located
# relative to the vehicle; in the sim the scenario file already places it.
_real_sonar = lambda v: _real(v) and v["real_sonar"]

# The preset line lands here when the options match no rung of the ladder.
CUSTOM_PRESET = "custom"

# Each sim node reads only its own section of noise_<profile>.yaml, so a sensor
# picks its profile independently and the old composite files (sonar_only,
# odom_only, odom_pos_only — hand-mixed ideal/realistic sections) are just
# combinations of these three. Volume reverberation is a sonar phenomenon: the
# no_reverb files are their base verbatim outside the sonar section, so they are
# offered on the sonar line alone.
NAV_NOISE = ["ideal", "realistic", "degraded"]
NAV_NOISE_HELP = {
    "ideal": "Ground truth — this sensor reports the simulator's exact value.",
    "realistic": "Datasheet-grounded error for this sensor.",
    "degraded": "Worst case — what this sensor does in poor conditions.",
}
SONAR_NOISE = ["ideal", "realistic", "realistic_no_reverb",
               "degraded", "degraded_no_reverb"]
SONAR_NOISE_HELP = {
    "ideal": "Ground-truth ranges — no noise, for an upper-bound map.",
    "realistic": "WaterLinked Sonar 3D-15 datasheet model, 1.2 MHz mode.",
    "realistic_no_reverb": "Realistic with volume reverberation off — no "
                           "near-field spray, close geometry still mapped.",
    "degraded": "Turbid water: worst-case range error and heavy reverberation.",
    "degraded_no_reverb": "Degraded range error with the reverberation spray off.",
}

PARAMS = [
    Param("platform", "Platform", "enum", PLATFORM_SIM, "preset",
          "What the stack drives. stonefish is the simulator: ground-truth "
          "pose, simulated sonar and nav sensors, the full SLAM and "
          "exploration stack. real_life drives a BlueROV2 Heavy through "
          "ArduSub over MAVLink and maps from a physical Sonar 3D-15 — no "
          "simulator, no ground truth, and so no ATE/RPE evaluation. "
          "Everything the simulator was the only source of disappears from "
          "this list when it is selected.",
          [PLATFORM_SIM, PLATFORM_REAL],
          {PLATFORM_SIM: "Simulator: ground truth, simulated sensors, full "
                         "SLAM and exploration stack.",
           PLATFORM_REAL: "Real BlueROV2 through ArduSub, real sonar, teleop "
                          "only. Read IRL_TEST.md before enabling motion."},
          wip=[PLATFORM_REAL]),

    Param("preset", "Preset", "enum", "custom", "preset",
          "Which rung of the evaluation ladder to run. Each one adds a single "
          "piece of the SLAM stack to the one before it, so the difference in "
          "trajectory error between two runs is the contribution of that piece "
          "and nothing else. Selecting one sets the options it names and leaves "
          "everything else alone; changing any of those options afterwards "
          "drops this line back to custom, so it never claims a condition the "
          "configuration is not in.",
          ["custom", "ground_truth", "no_loop_closure", "loop_closure",
           "loop_closure_revisit"],
          {"custom": "Whatever the options below say — no condition claimed.",
           "ground_truth": "Pose straight from the simulator: no SLAM, no "
                           "drift. The upper bound every other rung is read "
                           "against.",
           "no_loop_closure": "SLAM as pure dead reckoning — the pose graph "
                              "runs but never closes a loop. The baseline "
                              "drift.",
           "loop_closure": "Dead reckoning plus loop closure, corrected "
                           "wherever exploration happens to revisit somewhere.",
           "loop_closure_revisit": "Loop closure plus uncertainty-driven "
                                   "revisits — the active SLAM condition, "
                                   "where the planner spends travel to go and "
                                   "close a loop on purpose."},
          # Every rung is defined by a SLAM condition measured against ground
          # truth; a real run has neither.
          visible=_sim, cycle_skip=[CUSTOM_PRESET]),

    # ---------------------------------------------------------------- localisation
    Param("slam", "Pose source", "enum", "none", "localisation",
          "Where the vehicle pose comes from. none uses the simulator's exact "
          "pose. slam runs a GTSAM iSAM2 pose graph over simulated "
          "pressure/IMU/DVL dead reckoning with loop closure, so the estimate "
          "drifts and gets corrected like a real system.",
          ["none", "slam"],
          {"none": "Ground-truth TF straight from Stonefish; on hardware, "
                   "ArduSub's own estimate through mavlink_odometry.",
           "slam": "GTSAM iSAM2 pose graph + loop closure. In the simulator "
                   "it fuses simulated sensors and is scored by ATE/RPE; on "
                   "hardware it fuses the autopilot's, with no ground truth "
                   "to score against."}),

    Param("noise_profile_dvl", "DVL noise", "enum", "realistic",
          "localisation.sensors",
          "Error injected into the simulated DVL — the position drift knob. "
          "Dead reckoning integrates the DVL, so this is what decides how fast "
          "the estimate walks away from truth, and therefore how often a "
          "revisit has to be spent pulling it back.",
          NAV_NOISE, NAV_NOISE_HELP, visible=_sim_slam),
    Param("noise_profile_imu", "IMU noise", "enum", "realistic",
          "localisation.sensors",
          "Error injected into the simulated IMU — the angular rate knob "
          "(roll, pitch and yaw rate, integrated into attitude). Yaw rate "
          "integrates without bound, so this and the compass between them set "
          "the heading half of the pose uncertainty.",
          NAV_NOISE, NAV_NOISE_HELP, visible=_sim_slam),
    Param("noise_profile_compass", "Compass noise", "enum", "realistic",
          "localisation.sensors",
          "Error injected into the simulated compass — the absolute heading "
          "knob, and the only thing that stops integrated yaw drifting "
          "without bound.",
          NAV_NOISE, NAV_NOISE_HELP, visible=_sim_slam),
    Param("noise_profile_pressure", "Pressure noise", "enum", "realistic",
          "localisation.sensors",
          "Error injected into the simulated pressure sensor — the depth knob. "
          "Depth is directly observed rather than integrated, so this is the "
          "best-constrained axis of the pose and the reason planning happens "
          "in a horizontal band.",
          NAV_NOISE, NAV_NOISE_HELP, visible=_sim_slam),

    Param("loop_closure", "Loop closure", "bool", True, "localisation.graph",
          "Detect revisited places and add graph constraints that correct "
          "accumulated drift. Turning it off is the A/B baseline: the pose "
          "graph becomes pure dead reckoning.",
          visible=_slam),

    Param("keyframe_dist_m", "Keyframe spacing (m)", "float", 0.5,
          "localisation.graph",
          "How far the vehicle travels before the pose graph adds another "
          "keyframe. Smaller packs the graph denser — more scan-matching "
          "factors and more loop-closure candidates, at more optimisation per "
          "step; larger leaves longer dead-reckoned gaps between constraints.",
          step=0.1, lo=0.05, hi=10.0, visible=_slam),
    Param("keyframe_angle_rad", "Keyframe spacing (rad)", "float", 0.2,
          "localisation.graph",
          "The same threshold on rotation: a turn this large adds a keyframe "
          "even where the vehicle has barely moved. Scanning in place is all "
          "rotation, so this is what decides how many keyframes a scan lays "
          "down.",
          step=0.05, lo=0.01, hi=3.14, visible=_slam),
    Param("keyframe_max_per_cell", "Keyframes per cell", "int", 3,
          "localisation.graph",
          "Cap on keyframes kept in one spatial cell, so holding station does "
          "not grow the graph without adding information. This is the cap a "
          "parked vehicle saturates during a revisit dwell — past it the "
          "covariance cannot move, which is what Revisit stall exit leaves on.",
          step=1, lo=1, hi=50, visible=_slam),
    Param("loop_closure_radius_m", "Closure search radius (m)", "float", 5.0,
          "localisation.graph",
          "How far from the current pose past keyframes are considered as "
          "loop-closure candidates. It has to cover the drift accumulated "
          "since that place was last seen, or the true match sits outside the "
          "search; too wide and registration is attempted against geometry "
          "that was never the same place.",
          step=0.5, lo=0.5, hi=50.0,
          visible=lambda v: _slam(v) and v["loop_closure"]),
    Param("loop_closure_min_gap", "Closure min index gap", "int", 20,
          "localisation.graph",
          "Keyframes must be at least this many apart in the graph before "
          "they may close a loop. Without it the strongest match is always "
          "the keyframe just behind, which adds a constraint that carries no "
          "new information and corrects nothing.",
          step=5, lo=1, hi=500,
          visible=lambda v: _slam(v) and v["loop_closure"]),
    Param("min_inlier_ratio", "Registration inlier ratio", "float", 0.3,
          "localisation.graph",
          "Fraction of points that must register before a candidate closure "
          "is accepted as one. This is the guard against perceptual aliasing: "
          "a self-similar wall registers plausibly against the wrong stretch "
          "of itself, and the A/B where loop closure increased trajectory "
          "error under the wall-looking executor is what that looks like. "
          "Raise it to demand a more distinctive match.",
          step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _slam(v) and v["loop_closure"]),

    Param("noise_profile", "Assumed noise model", "enum", "realistic",
          "localisation.noise",
          "The profile the pose graph builds its factor noise from: what the "
          "estimator assumes the sensors do, as against what the Sensors "
          "options make them actually do. It reads the IMU, pressure and DVL "
          "sections — the attitude and depth priors, and the per-edge dead "
          "reckoning sigma that scales with distance travelled. Setting it to "
          "something other than the sensors are running is how the filter is "
          "made over- or under-confident on purpose, and ANEES on the right is "
          "the measurement of that. No sonar section is read here, so the "
          "no-reverb variants would mean nothing and are not offered.",
          NAV_NOISE,
          {"ideal": "Assume noiseless sensors — the graph trusts everything.",
           "realistic": "Assume the datasheet error model; matches sensors set "
                        "to realistic.",
           "degraded": "Assume poor conditions — wide sigmas, a cautious graph."},
          visible=_slam),
    Param("scan_sigma_trans", "Scan match sigma (m)", "float", 0.12,
          "localisation.noise",
          "Translational uncertainty the graph assigns to a scan-matching "
          "constraint. Together with the odometry sigmas it sets how much the "
          "optimiser trusts geometry against dead reckoning — and it is what "
          "NIS measures: NIS far from ~6 means this does not match the "
          "residuals the matcher is actually producing.",
          step=0.01, lo=0.001, hi=5.0, visible=_slam),
    Param("scan_sigma_rot", "Scan match sigma (rad)", "float", 0.08,
          "localisation.noise",
          "Rotational uncertainty on a scan-matching constraint. Same story "
          "as the translational one; heading is the axis a wide-FoV sonar "
          "constrains worst.",
          step=0.01, lo=0.001, hi=3.0, visible=_slam),
    Param("odom_sigma_trans", "Odometry sigma (m)", "float", 0.002,
          "localisation.noise",
          "Translational uncertainty on a dead-reckoning factor, per step. "
          "This is the growth rate of the covariance the revisit trigger "
          "watches: too small and the estimate claims a confidence the DVL "
          "cannot support (ANEES climbs, revisit never fires), too large and "
          "the graph discards good odometry in favour of noisy geometry.",
          step=0.001, lo=0.0001, hi=1.0, visible=_slam),
    Param("odom_sigma_rot", "Odometry sigma (rad)", "float", 0.02,
          "localisation.noise",
          "Rotational uncertainty on a dead-reckoning factor, per step. Drives "
          "the heading half of D-optimality, so it moves when a revisit fires "
          "for heading rather than position.",
          step=0.005, lo=0.0001, hi=1.0, visible=_slam),

    Param("revisit", "Uncertainty revisit", "bool", True, "localisation.revisit",
          "Suspend the current goal and drive back to mapped areas when the "
          "pose covariance (D-optimality) crosses a threshold, to force a loop "
          "closure. This is the active part of active SLAM. Under goto the "
          "revisit interrupts the run to the target point and the launcher "
          "resends the point once the detour finishes.",
          visible=_revisit_armed),
    Param("revisit_scoring", "Revisit scoring", "enum", "keyframe_density",
          "localisation.revisit",
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
          visible=_revisit),
    Param("revisit_trigger", "Revisit trigger", "enum", "live_dopt",
          "localisation.revisit",
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
          visible=_revisit),
    Param("sigma_allow_xy_m", "Allowable XY sigma", "float", 0.045,
          "localisation.revisit",
          "Largest horizontal position uncertainty the mission tolerates, in "
          "metres. With the yaw sigma it defines D(Sigma_allow), the level the "
          "live D-optimality is divided by to get the trigger ratio. Set it "
          "from what the mission actually needs, not from the raw D-opt scale. "
          "Live-tunable while running.",
          step=0.01, lo=0.001, hi=5.0,
          visible=_revisit),
    Param("sigma_allow_yaw_rad", "Allowable yaw sigma", "float", 0.045,
          "localisation.revisit",
          "Largest heading uncertainty the mission tolerates, in radians. "
          "Live-tunable while running.",
          step=0.01, lo=0.001, hi=3.0,
          visible=_revisit),
    Param("ratio_trigger", "Revisit trigger ratio", "float", 1.0,
          "localisation.revisit",
          "Uncertainty ratio U_r = D(Sigma)/D(Sigma_allow) that suspends the "
          "current goal and drives back to close a loop. 1.0 means revisit "
          "exactly when the estimate is less certain than the mission allows; "
          "lower is more cautious. Shown next to the live ratio on the right. "
          "Live-tunable while running.",
          step=0.1, lo=0.0, hi=20.0,
          visible=_revisit),
    Param("ratio_resume", "Revisit resume ratio", "float", 0.5,
          "localisation.revisit",
          "Uncertainty ratio below which the run resumes after a revisit. "
          "Must stay below the trigger ratio or revisit will not exit. "
          "Live-tunable while running.",
          step=0.1, lo=0.0, hi=20.0,
          visible=_revisit),
    Param("revisit_min_closures", "Revisit min closures", "int", 0,
          "localisation.revisit",
          "Loop closures required before a revisit ends on closure count "
          "alone. 0 (default) means the count is not taken into account, so "
          "the revisit ends only when the uncertainty ratio falls below the "
          "resume ratio (or on timeout). Ending on the first closure resumes "
          "exploring while sigma is still near the allowance, because one "
          "closure rarely restores the covariance. "
          "Live-tunable while running.",
          step=1, lo=0, hi=10,
          visible=_revisit),
    Param("arrival_dwell_s", "Revisit dwell (s)", "float", 30.0,
          "localisation.revisit",
          "How long the robot waits at the revisit target for the uncertainty "
          "to come back down before giving up on the detour. Counts from "
          "arrival, so a long drive out never shortens it. "
          "Live-tunable while running.",
          step=5.0, lo=1.0, hi=300.0,
          visible=_revisit),
    Param("stall_exit_s", "Revisit stall exit (s)", "float", 5.0,
          "localisation.revisit",
          "Leave the revisit target once neither the keyframe count nor the "
          "closure count has moved for this long. A parked vehicle saturates "
          "the pose graph's per-cell keyframe cap, after which the rest of the "
          "dwell cannot change the covariance. 0 disables. "
          "Live-tunable while running.",
          step=1.0, lo=0.0, hi=60.0,
          visible=_revisit),

    Param("revisit_scan_slowdown", "Revisit scan slowdown", "float", 1.0,
          "localisation.revisit",
          "Divide the scan yaw rate by this while a revisit is in progress. "
          "1.0 is off. Sweeping slower at the revisit target gives the "
          "matcher more overlap per keyframe, which is the whole point of "
          "being there — at the cost of a longer detour.",
          step=0.5, lo=1.0, hi=10.0, visible=_revisit),

    Param("speed_factor", "Move speed", "float", 1.0, "localisation.motion",
          "Multiplies forward/strafe/vertical motion for whichever motion "
          "executor is driving. Commands saturate at the safety gate's +-1.0 "
          "limit, so above roughly 1.1 this stops making the vehicle faster. "
          "Live, no restart. Planner modes only: the teleop keyboard node is "
          "started by hand in its own terminal and takes no launcher "
          "parameters, so this could never reach it.",
          step=0.20, lo=0.1, hi=5.0, visible=_planner),
    Param("turn_factor", "Turn speed", "float", 1.0, "localisation.motion",
          "Multiplies yaw for whichever motion executor is driving, "
          "independent of the move speed. Also saturates at the safety gate's "
          "+-1.0 limit (roughly 6.7x). Live, no restart. Planner modes only, "
          "for the same reason as the move speed.",
          step=0.10, lo=0.1, hi=5.0, visible=_planner),
    Param("robot_depth_target", "Cruise depth (m)", "float", 8.0,
          "localisation.motion",
          "Fixed NED Z the autonomous path controller holds while exploring "
          "(positive is below the surface). Also centres the /projected_map Z "
          "band frontier detection and A* read — Path planning sets how thick "
          "that band is. Under TSDF the band re-centres live and the map is "
          "kept; the planner reads its setpoint at startup, so it restarts "
          "either way. Teleop vertical motion remains manual.",
          step=0.5, lo=0.0, hi=1000.0,
          visible=_planner),
    Param("motion", "Path executor", "enum", "default", "localisation.motion",
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
    Param("scan_style", "Scan style", "enum", "sweep", "localisation.motion",
          "Rotation used when scanning at a waypoint. sweep is the cable-safe "
          "right-then-left motion (net yaw returns to zero). spin is the "
          "legacy full 360 rotation, which piles up keyframes and inflates "
          "loop-closure counts.",
          ["sweep", "spin"],
          {"sweep": "Right half, left full, return — tether safe.",
           "spin": "Legacy 360 degree rotation."},
          visible=_planner),
    Param("scan_sweep_deg", "Sweep width (deg)", "float", 180.0,
          "localisation.motion",
          "Total sweep angle in degrees for scan_style:=sweep.",
          step=5.0, lo=10.0, hi=360.0,
          visible=lambda v: _planner(v) and v["scan_style"] == "sweep"),
    Param("safety_start_enabled", "Safety gate starts enabled", "bool", False,
          "localisation.motion",
          "Start the motion safety gate already enabled. Normally left off so "
          "motion is armed deliberately from the RViz panel after checking the "
          "scene. The gate is fail-closed: it blocks commands that are stale, "
          "oversized, or missing odometry.",
          # IRL_TEST.md: a real launch never starts with motion already armed.
          visible=_sim),
    Param("thrust_boost", "Thrust boost (2.5x ceiling)", "bool", False,
          "localisation.motion",
          "Scale the simulated thruster RPM ceiling 2.5x for fast repositioning "
          "between runs. Body commands still normalize to the safety gate's "
          "+-1.0 range, so this never trips INVALID_COMMAND — it raises what "
          "100% effort means physically, not the command range. Not a "
          "physically realistic BlueROV2/T200 value; restarts the simulator.",
          visible=_sim),

    Param("wall_orientation_offset_deg", "Wall yaw offset (deg)", "float", 30.0,
          "localisation.executor",
          "Degrees to yaw away from the travel bearing toward the nearest "
          "mapped surface, so the sonar keeps the wall in view while moving "
          "along the path.",
          step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: _planner(v) and v["motion"] == "walloriented"),
    Param("wall_orientation_lookahead_m", "Wall lookahead (m)", "float", 0.0,
          "localisation.executor",
          "Lookahead radius along the A* path used to pick the heading. 0 uses "
          "the heading at the current position.",
          step=0.5, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walloriented"),
    Param("wall_z_band_m", "Wall depth slice (m)", "float", 1.5,
          "localisation.executor",
          "Half-thickness of the depth slice used to pick which side the wall "
          "is on. Depth is directly observed, so geometry further above or "
          "below than this cannot be collided with and should not steer the "
          "look direction. Live-tunable while running.",
          step=0.5, lo=0.5, hi=10.0,
          visible=lambda v: _planner(v) and v["motion"] == "walloriented"),
    Param("wall_yaw_only", "Wall guides yaw only", "bool", True,
          "localisation.executor",
          "Let the wall set the look direction and nothing else: translation "
          "follows the planner path, with no standoff regulation and no "
          "wall-tangent travel. Turn it off to restore the wall-orbit "
          "behaviour, where Wall standoff and Path influence apply — that "
          "path holds a distance to the infinite plane through the selected "
          "surface, so it can park the vehicle in open water past the end of "
          "a wall. Live-tunable while running.",
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_standoff", "Wall standoff (m)", "float", 1.5,
          "localisation.executor",
          "Target distance to hold from the wall while wall-looking. "
          "Live-tunable while running.",
          step=0.1, lo=0.1,
          visible=lambda v: (_planner(v) and v["motion"] == "walllooking"
                             and not v["wall_yaw_only"])),
    Param("wall_max_surface_dist", "Wall acquisition range (m)", "float", 8.0,
          "localisation.executor",
          "Furthest a mapped surface can be and still be steered by. Nothing "
          "within this radius means no wall is selected, and the no-wall "
          "response only rotates — so a start pose further out than this from "
          "the structure never acquires and the planner blacklists every goal "
          "as BLOCKED:NO_WALL. Raise it when the vehicle starts far from the "
          "scene. Live-tunable while running.",
          step=0.5, lo=1.0, hi=40.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_switch_goal_distance", "Wall-switch goal dist (m)", "float", 6.0,
          "localisation.executor",
          "When the planner goal is nearer than this, look for another wall "
          "instead of continuing along the current one. Live-tunable.",
          step=0.5, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_switch_scan_angle", "Wall-switch sweep (rad)", "float", 3.14159,
          "localisation.executor",
          "Sweep angle used when searching for the next wall. Live-tunable.",
          step=0.1, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_switch_scan_yaw", "Wall-switch sweep yaw", "float", 0.08,
          "localisation.executor",
          "Yaw rate command used during that sweep. Live-tunable.",
          step=0.01, lo=0.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_path_influence", "Path influence", "float", 0.70,
          "localisation.executor",
          "Travel-direction blend: 0 follows the wall tangent, 1 follows the "
          "planned path. Live-tunable.",
          step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: (_planner(v) and v["motion"] == "walllooking"
                             and not v["wall_yaw_only"])),
    Param("wall_path_look_offset_deg", "Path look offset (deg)", "float", 30.0,
          "localisation.executor",
          "Degrees to turn the path-derived look heading toward the wall. "
          "Live-tunable.",
          step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_normal_offset_deg", "Wall normal offset (deg)", "float", 0.0,
          "localisation.executor",
          "Degrees to turn the wall-facing normal back toward the path "
          "bearing. Live-tunable.",
          step=5.0, lo=-180.0, hi=180.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),
    Param("wall_path_heading_weight", "Look blend", "float", 0.35,
          "localisation.executor",
          "Orientation blend: 0 uses the wall-derived heading, 1 the "
          "path-derived heading. Live-tunable.",
          step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _planner(v) and v["motion"] == "walllooking"),

    # -------------------------------------------------------------------- mapping
    Param("mapper", "Mapper", "enum", "octomap", "mapping",
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

    Param("noise_profile_sonar", "Sonar noise", "enum", "realistic",
          "mapping.sensors",
          "Error injected into the simulated sonar — the range and map quality "
          "knob, and the only sensor the reverberation variants mean anything "
          "for. Volume reverberation is what sprays phantom returns into the "
          "near field; the no_reverb profiles remove it at the source, which "
          "keeps close geometry the near-field filter below would also throw "
          "away.",
          SONAR_NOISE, SONAR_NOISE_HELP, visible=_sim_slam),
    Param("noise_attenuation", "Near-field filter", "enum", "none",
          "mapping.sensors",
          "Post-filter applied over any noise profile, clearing the near-field "
          "volume-reverberation spray around the vehicle so the noised cloud "
          "sits closer to the clean ground truth. cut_close cuts the near field "
          "outright; fade_close thins it, which costs some spray but keeps an "
          "obstacle the vehicle drives up to.",
          ["none", "cut_close", "fade_close"],
          {"none": "Keep every return the profile produces.",
           "cut_close": "Gate out the near-field reverberation spray.",
           "fade_close": "Thin the near field, keeping close geometry visible."},
          visible=_sim_slam),
    Param("near_cutoff_m", "Near-field distance (m)", "float", 1.6,
          "mapping.sensors",
          "Extent of the near field the filter acts on: the range cut_close "
          "cuts at, or the range fade_close fades out to. Raise it to reach a "
          "larger spray (e.g. the degraded profile reaches ~2.5 m); lower it to "
          "leave more close geometry alone.",
          step=0.1, lo=0.2, hi=15.0,
          visible=lambda v: _sim_slam(v) and v["noise_attenuation"] != "none"),
    Param("near_fade_p", "Fade strength", "float", 0.8, "mapping.sensors",
          "fade_close only: probability of dropping a return at the sensor "
          "itself, falling to zero at the near-field distance. 0.8 discards "
          "four of five near returns — enough to clear a sparse spray, while a "
          "surface filling every beam still comes through at a fifth of its "
          "density and refills over successive pings.",
          step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _sim_slam(v) and v["noise_attenuation"] == "fade_close"),

    Param("voxel_size", "Voxel size (m)", "float", 0.2, "mapping.grid",
          "Edge length of one map cell, for whichever backend is selected — "
          "octomap_server's resolution or the TSDF grid's voxel size. Halving "
          "it resolves finer geometry at roughly 8x the voxels, so integration "
          "and the voxel view both get slower. Under TSDF the truncation band "
          "follows at 3x this value unless Truncation band overrides it. The "
          "ground-truth reference map is built at the same size, so the map "
          "metrics keep comparing like with like.",
          step=0.05, lo=0.05, hi=1.0),
    Param("trunc_distance", "Truncation band (m)", "float", 0.0, "mapping.grid",
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
    Param("space_carving", "Space carving", "bool", True, "mapping.grid",
          "Whether a return frees the whole ray back to the sensor, or only the "
          "band just ahead of the surface. On, the map clears space it has flown "
          "through and unknown volume shrinks faster. Off, a ray that misses a "
          "thin target — past the bow, through a gap — can no longer carve away "
          "the far face that an earlier pass mapped, which is the usual reason a "
          "structure degrades as it is circled rather than improving.",
          visible=_tsdf),

    Param("carve_no_return", "Carve no-return rays", "bool", False,
          "mapping.grid",
          "Free the voxels along a sonar ray that came back with no return, "
          "instead of leaving that volume unknown. Open water past the "
          "sensor's range reads as no-return, so this is what clears the space "
          "the vehicle can see through but never gets an echo from — at the "
          "risk of carving away a real surface the beam merely missed.",
          visible=_tsdf),

    Param("voxel_min_solid_confidence", "Wall threshold", "float", 0.80,
          "mapping.surface",
          "How deep behind the reconstructed surface a voxel has to sit before "
          "it counts as a wall, as solid confidence (trunc - d) / (2 * trunc): "
          "0.5 is exactly at the zero crossing, 1.0 is fully saturated solid. "
          "Lowering it calls thinner, less certain returns walls — more "
          "geometry survives, at the cost of noise being mapped as structure. "
          "Applies to the voxel view, the goal-safety cloud and the 2-D "
          "planning map alike, so it moves what A* refuses to route through.",
          step=0.05, lo=0.5, hi=1.0,
          visible=_tsdf),
    Param("voxel_min_weight", "Observations for a wall", "float", 10.0,
          "mapping.surface",
          "How many times a voxel must be observed before it counts as a wall. "
          "Each integrated scan adds weight, so this is the persistence filter: "
          "raise it and single-scan sonar noise stops becoming structure, lower "
          "it and the map fills in sooner from thinner evidence. Same three "
          "consumers as the wall threshold.",
          step=1.0, lo=1.0, hi=200.0,
          visible=_tsdf),

    Param("tsdf_octomap", "TSDF -> OcTree", "bool", True, "mapping.outputs",
          "Rebuild the TSDF grid into an octomap::OcTree on /tsdf/octomap_binary "
          "(tsdf_to_octomap), from its occupied and free voxels. Gives the TSDF "
          "backend the octree interface 3-D frontier detection and 3-D A* "
          "expect. Its own topic, not /octomap_binary, so RViz's OcTree displays "
          "stay empty under TSDF and only TSDFVoxels draws the belief map.",
          visible=_tsdf),
    Param("map_rebuild", "Rebuild map on closure", "bool", False,
          "mapping.outputs",
          "After a large loop closure, reset the TSDF and re-integrate every "
          "keyframe at its corrected pose, so the map geometry is fixed too "
          "rather than just the trajectory. TSDF only — OctoMap cannot "
          "reproduce its raycast free space this way.",
          visible=lambda v: _slam(v) and _tsdf(v) and v["loop_closure"]),

    Param("cache_max_scans", "Rebuild cache (scans)", "int", 6000,
          "mapping.outputs",
          "How many scans the rebuild cache holds. A map rebuild re-integrates "
          "from this cache, so it bounds how far back a corrected trajectory "
          "can be replayed — and the memory the mapper holds to keep that "
          "option open.",
          step=500, lo=100, hi=50000,
          visible=lambda v: _slam(v) and _tsdf(v) and v["loop_closure"]
                            and v["map_rebuild"]),

    # ------------------------------------------------------------------- planning
    Param("mode", "Mode", "enum", "teleop", "planning",
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
           "frontier": "Autonomous frontier-based exploration."},
          # A real run is teleop and only teleop until IRL_TEST.md Phase 5 has
          # been passed on the vehicle; the launcher does not offer autonomy it
          # has no accepted hardware path for.
          visible=_sim),

    Param("frontier_space", "Frontier space", "enum", "2d", "planning.goals",
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
    Param("exploration", "Exploration policy", "enum", "frontier",
          "planning.goals",
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
    Param("goal_order", "Goal ordering", "enum", "greedy", "planning.goals",
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
    Param("tsdf_frontier_standoff_m", "TSDF frontier standoff (m)", "float", 1.0,
          "planning.goals",
          "How far off the reconstructed surface, along its outward normal, a "
          "TSDF frontier goal is placed — keeps goals in free water.",
          step=0.1, lo=0.0,
          visible=lambda v: _frontier(v) and _tsdf(v)),

    Param("survey_radius_m", "Survey radius (m)", "float", 0.0,
          "planning.goals",
          "Bound exploration to a circle this wide around the deployment "
          "point: frontier goals outside it are not candidates. 0 is "
          "unbounded, which in open water lets the planner wander off after "
          "the nearest unknown cell instead of finishing the structure it was "
          "sent to. The centre is wherever the vehicle started.",
          step=1.0, lo=0.0, hi=200.0, visible=_frontier),
    Param("min_goal_separation_m", "Min goal separation (m)", "float", 0.0,
          "planning.goals",
          "Minimum distance between one frontier goal and the next, so the "
          "planner commits to moving on rather than re-picking a cluster it is "
          "already standing in. 0 is off.",
          step=0.5, lo=0.0, hi=50.0, visible=_frontier),

    Param("projected_map_band_m", "Planning band half-height (m)", "float", 1.0,
          "planning.map",
          "Half-thickness of the depth slice the mapper collapses into "
          "/projected_map, the 2-D map both frontier detection and A* work on. "
          "It is centred on the cruise depth, so it reaches this far above and "
          "below it. Too thin and geometry the vehicle would hit never reaches "
          "the planner; too thick and structure well clear of it blocks routes "
          "that are actually open. One value for every backend — octomap_server "
          "takes it as occupancy_min_z/max_z (floored at the vehicle height), "
          "the TSDF mapper as projected_map_band_m — and the planner is handed "
          "the same number so its band marker draws what it is really "
          "planning on.",
          step=0.25, lo=0.05, hi=10.0,
          visible=_planner),
    Param("hard_inflation_m", "Hard wall zone (m)", "float", 1.00,
          "planning.map",
          "A* hard-wall radius around occupied cells: completely blocked "
          "(cost = inf). Small enough that paths can still pass through "
          "narrow corridors.",
          step=0.05, lo=0.0,
          visible=_planner),
    Param("inflation_m", "Soft zone (m)", "float", 1.50, "planning.map",
          "A* soft-zone radius around occupied cells: high but finite cost, "
          "so A* routes around when a free path exists but can pass through "
          "if forced.",
          step=0.05, lo=0.0,
          visible=_planner),
    Param("plan_inflation_m", "Planning margin zone (m)", "float", 3.00,
          "planning.map",
          "A* planning-margin radius around occupied cells: moderate cost "
          "to steer paths away from walls while keeping them usable.",
          step=0.05, lo=0.0,
          visible=_planner),

    # ------------------------------------------------------------------ hardware
    Param("real_sonar", "Sonar 3D-15 mapping", "bool", True, "hardware",
          "Start the WaterLinked Sonar 3D-15 driver and map from its returns. "
          "Off runs the motion path alone — safety gate, ArduSub adapter and "
          "teleop — which is IRL_TEST.md Phase 3/4 and the part that has to "
          "pass before mapping is worth switching on.",
          visible=_real),
    Param("ardusub_backend", "ArduSub interface", "enum", "manual_control",
          "hardware",
          "Which MAVLink message carries the gated body demand. "
          "manual_control sends pilot-axis fractions and needs ALT_HOLD; "
          "local_ned_velocity sends metric velocity setpoints and needs "
          "GUIDED, which ArduSub only supports with a position and depth "
          "solution in its EKF.",
          ["manual_control", "local_ned_velocity"],
          {"manual_control": "MANUAL_CONTROL axes in ALT_HOLD. The first wet "
                             "test uses this.",
           "local_ned_velocity": "SET_POSITION_TARGET_LOCAL_NED in GUIDED. "
                                 "Needs a healthy EKF position solution."},
          visible=_real),
    Param("manual_authority", "Manual authority", "float", 0.15, "hardware",
          "Fraction of the MAVLink pilot axis range ROS is allowed to ask "
          "for. A full-scale normalized demand of 1.0 reaches the vehicle as "
          "this much stick. Raise it only between runs, after reviewing the "
          "log of the one before.",
          step=0.05, lo=0.0, hi=1.0,
          visible=lambda v: _real(v) and v["ardusub_backend"] == "manual_control"),
    Param("enable_vertical", "ROS vertical authority", "bool", False,
          "hardware",
          "Whether ROS may command the vertical axis. Off leaves depth to "
          "ArduSub's own hold loop, which is what the first wet tests use — "
          "the up/down drive keys then do nothing, by design rather than by "
          "fault. Do not run this against a ROS depth controller until both "
          "loops' signs and responsibilities are documented.",
          visible=_real),
    Param("require_odom", "Gate requires odometry", "bool", True, "hardware",
          "Whether the safety gate refuses to pass any command without fresh "
          "pose input. Off is for a motion-only acceptance run with no pose "
          "source at all; it removes one of the gate's fail-closed conditions, "
          "so never leave it off once anything is mapping or planning.",
          visible=_real),

    Param("mavlink_command_url", "Command endpoint", "text",
          "udpin:0.0.0.0:14560", "hardware.link",
          "pymavlink URL the ArduSub adapter listens on. Create a dedicated "
          "external endpoint in BlueOS pointing at this machine and port, and "
          "leave the normal Cockpit/QGroundControl endpoint running — the "
          "pilot's takeover path must not depend on this one.",
          visible=_real),
    Param("mavlink_odom_url", "Navigation endpoint", "text",
          "udpin:0.0.0.0:14561", "hardware.link",
          "Second BlueOS endpoint, read-only, for the navigation estimate. "
          "pymavlink binds the UDP port, so this must differ from the command "
          "endpoint or whichever node starts second fails to open its link.",
          visible=_real),
    Param("require_position", "ArduSub has XY position", "bool", True,
          "hardware.link",
          "Whether ArduSub's EKF has a horizontal position source — a DVL or "
          "GPS. Without one, LOCAL_POSITION_NED X/Y sit at the origin while "
          "depth and attitude stay good, and a map built against that pose is "
          "wrong the moment the vehicle translates. Off publishes depth and "
          "attitude only and marks X/Y unestimated rather than reporting a "
          "confident zero.",
          visible=_real),

    Param("sonar_ip", "Sonar IP", "text", "192.168.2.199", "hardware.sonar",
          "Address the Sonar 3D-15 multicasts from. The driver drops packets "
          "from any other source, so a wrong value here shows up as a driver "
          "that starts cleanly and publishes nothing.",
          visible=_real_sonar),
    # The six below are the lever arm and orientation of a base_link -> sonar3d
    # static transform. The two frames disagree on handedness: base_link is NED,
    # while the driver builds its cloud x-forward / z-up, so the neutral roll is
    # 180 deg, not 0. Translations are read in the NED parent frame.
    Param("sonar_mount_x", "Sonar X (m)", "float", 0.0, "hardware.sonar",
          "Sonar position forward of base_link, in metres.",
          step=0.01, lo=-2.0, hi=2.0, visible=_real_sonar),
    Param("sonar_mount_y", "Sonar Y (m)", "float", 0.0, "hardware.sonar",
          "Sonar position to starboard of base_link, in metres.",
          step=0.01, lo=-2.0, hi=2.0, visible=_real_sonar),
    Param("sonar_mount_z", "Sonar Z (m)", "float", 0.0, "hardware.sonar",
          "Sonar position below base_link, in metres — base_link is NED, so "
          "a sonar mounted above the origin is negative.",
          step=0.01, lo=-2.0, hi=2.0, visible=_real_sonar),
    Param("sonar_mount_roll", "Sonar roll (deg)", "float", 180.0,
          "hardware.sonar",
          "Sonar roll relative to base_link. 180 is the neutral value, not 0: "
          "the driver publishes an ENU-handed cloud (x forward, y to port, z "
          "up) into an NED body frame, and a half turn about x is what "
          "reconciles them. Roll the mount away from flat by adding to it. "
          "Check it before trusting a map: hold a known flat wall in view and "
          "confirm it comes out vertical and on the correct side — a map that "
          "mirrors top for bottom, or port for starboard, is this value.",
          step=5.0, lo=-180.0, hi=180.0, visible=_real_sonar),
    Param("sonar_mount_pitch", "Sonar pitch (deg)", "float", 0.0,
          "hardware.sonar",
          "Sonar pitch relative to base_link. A tilt-down mount is positive. "
          "Applied about the NED parent axes, so the roll above does not "
          "invert it.",
          step=5.0, lo=-180.0, hi=180.0, visible=_real_sonar),
    Param("sonar_mount_yaw", "Sonar yaw (deg)", "float", 0.0,
          "hardware.sonar",
          "Sonar yaw relative to base_link, positive to starboard. Applied "
          "about the NED parent axes, so the roll above does not invert it.",
          step=5.0, lo=-180.0, hi=180.0, visible=_real_sonar),

    # ---------------------------------------------------------------------- scene
    Param("scene", "Scene", "enum", "waterlinked", "scene",
          "Stonefish world to launch. waterlinked is the unchanged baseline; "
          "target contains one selectable static object that can be moved live.",
          ["waterlinked", "target"],
          {"waterlinked": "Baseline offshore-station scene.",
           "target": "One static mesh or built-in pipe for sonar demonstrations."},
          visible=_sim),

    Param("obj_mesh", "Object mesh", "enum", "pipe", "scene.object",
          "Object to place in the target scene. The launcher scans data/obj "
          "each time it starts, so newly added .obj and .stl files appear here. pipe is "
          "a lightweight built-in primitive.",
          ["pipe"], {"pipe": "Built-in 4 m pipe; fast to load and sonar-visible."},
          visible=_target_scene),
    Param("obj_x", "Object X (m)", "float", 8.0, "scene.object",
          "Static object north/X position in world_ned. Changes move the "
          "object live while Stonefish keeps running.",
          step=0.5, lo=-100.0, hi=100.0, visible=_target_scene),
    Param("obj_y", "Object Y (m)", "float", -2.0, "scene.object",
          "Static object east/Y position in world_ned. Live while running.",
          step=0.5, lo=-100.0, hi=100.0, visible=_target_scene),
    Param("obj_z", "Object Z (m)", "float", 8.0, "scene.object",
          "Static object down/Z position in world_ned. Live while running.",
          step=0.5, lo=-100.0, hi=100.0, visible=_target_scene),
    Param("obj_scale", "Object scale", "float", 1.0, "scene.object",
          "Uniform multiplier for the selected object. Applied when the target "
          "scene starts; changing it restarts only Stonefish.",
          step=0.03, lo=0.0001, hi=10.0, visible=_target_scene),
    Param("obj_roll", "Object roll (deg)", "float", 0.0, "scene.object",
          "Static object roll in degrees. Live while running.",
          step=5.0, lo=-180.0, hi=180.0, visible=_target_scene),
    Param("obj_pitch", "Object pitch (deg)", "float", 0.0, "scene.object",
          "Static object pitch in degrees. Live while running.",
          step=5.0, lo=-180.0, hi=180.0, visible=_target_scene),
    Param("obj_yaw", "Object yaw (deg)", "float", 0.0, "scene.object",
          "Static object yaw in degrees. Live while running.",
          step=5.0, lo=-180.0, hi=180.0, visible=_target_scene),

    Param("robot_pose_live", "Move robot in real time", "bool", False,
          "scene.robot",
          "Whether editing a pose field below also moves the running vehicle. "
          "On, every change teleports it to the value shown, so the vehicle can "
          "be placed while the simulator keeps running. Off, the fields are "
          "only where the next start puts it and a running vehicle is left "
          "alone. Either way the flow is one-way: these say where the vehicle "
          "is put, never where it has got to. Where it actually is shows in the "
          "pose table on the right.", visible=_sim),
    Param("robot_x", "Robot X (m)", "float", 0.0, "scene.robot",
          "Spawn north/X position in world_ned, used at start and at r reset. "
          "Driving does not change it; with Move robot in real time on, editing "
          "it teleports the vehicle here.",
          step=0.5, lo=-1000.0, hi=1000.0, visible=_sim),
    Param("robot_y", "Robot Y (m)", "float", 0.0, "scene.robot",
          "Spawn east/Y position in world_ned. Placed, not read back, as above.",
          step=0.5, lo=-1000.0, hi=1000.0, visible=_sim),
    Param("robot_z", "Robot Z (m)", "float", 8.0, "scene.robot",
          "Spawn down/Z position in world_ned. Placed, not read back, as above.",
          step=0.5, lo=-1000.0, hi=1000.0, visible=_sim),
    Param("robot_roll", "Robot roll (deg)", "float", 0.0, "scene.robot",
          "Spawn roll. Placed, not read back, as above.",
          step=5.0, lo=-180.0, hi=180.0, visible=_sim),
    Param("robot_pitch", "Robot pitch (deg)", "float", 0.0, "scene.robot",
          "Spawn pitch. Placed, not read back, as above.",
          step=5.0, lo=-180.0, hi=180.0, visible=_sim),
    Param("robot_yaw", "Robot yaw (deg)", "float", 0.0, "scene.robot",
          "Spawn yaw. Placed, not read back, as above.",
          step=5.0, lo=-180.0, hi=180.0, visible=_sim),

    # ---------------------------------------------------------------------- tools
    Param("rviz", "RViz", "bool", True, "tools",
          "Start RViz on demo.rviz — one view for every mode (OctoMap and "
          "TSDF, belief and ground truth, drift arrow, error HUD, covariance "
          "ellipsoids). Displays with no publisher in this mode draw nothing."),
    Param("rqt", "Planning Dashboard (RQT)", "bool", False, "tools",
          "Start rqt on the planning dashboard — map, inflation zones, path "
          "and robot/goal state, top-down. Off by default: it is a debugging "
          "view, not part of the demo. Offered wherever the planner runs — "
          "frontier and goto.",
          visible=_planner),
    Param("rqt_depthmap", "Sonar DepthMap (RQT)", "bool", False, "tools",
          "Start rqt on the sonar range image (/cloud_in/range_image) — the "
          "2D depth-camera-style view of what SLAM consumes, noise and all. "
          "In rqt rather than RViz because an RViz Image display unticks "
          "itself whenever its dock is hidden, e.g. moving desktop."),
    Param("keyboard", "Keyboard layout", "enum", "qwerty", "tools",
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

    Param("scenario", "Scenario", "enum", "none", "tools.eval",
          "Scripted evaluation run. drift_return leaves the start position and "
          "comes back to it, so loop closure can be observed firing on demand "
          "instead of waiting for exploration to revisit somewhere.",
          ["none", "drift_return"],
          {"none": "Free exploration.",
           "drift_return": "Scripted outbound leg then return to start."},
          visible=_frontier),
    Param("scenario_out_dx", "Outbound dx (m)", "float", 15.0, "tools.eval",
          "drift_return: outbound leg X offset from the captured start pose.",
          step=1.0,
          visible=lambda v: _frontier(v) and v["scenario"] == "drift_return"),
    Param("scenario_out_dy", "Outbound dy (m)", "float", 0.0, "tools.eval",
          "drift_return: outbound leg Y offset from the captured start pose.",
          step=1.0,
          visible=lambda v: _frontier(v) and v["scenario"] == "drift_return"),
    Param("rosbag", "Record rosbag", "bool", False, "tools.eval",
          "Record the run to eval/bags/<platform>_<timestamp>/: commands "
          "either side of the safety gate, safety status, odometry, TF, raw "
          "sonar, map and goals. Named topics rather than everything — the "
          "RViz marker arrays are most of the bandwidth and none of the "
          "evidence. The bag is closed when the stack is stopped or the "
          "launcher quits, both of which send SIGINT; a bag whose recorder was "
          "killed outright has no metadata and will not open. On hardware this "
          "is the run record IRL_TEST.md asks for, so leave it on."),

    Param("noise_seed", "Noise seed", "int", -1, "tools.eval",
          "Seed for the noise draws. -1 uses the profile's own seed; set an "
          "explicit value for reproducible or decorrelated repeat runs.",
          visible=_sim_slam),
    Param("rpe_delta", "RPE window (s)", "float", 1.0, "tools.eval",
          "Time window relative pose error is measured over. RPE reports the "
          "drift accumulated within one window, so this picks what the number "
          "on the right actually means — a short window scores local "
          "odometry, a long one scores whether loop closure is holding the "
          "trajectory together.",
          step=0.5, lo=0.1, hi=60.0, visible=_sim_slam),
    Param("output_dir", "Output dir", "text", "", "tools.eval",
          "Where the eval stack writes TUM trajectories and CSV metrics. "
          "Empty means a timestamped directory under eval/runs/.",
          visible=_sim_slam),
]

PARAM_MAP = {p.id: p for p in PARAMS}
DEFAULTS = {p.id: p.default for p in PARAMS}
DEFAULTS[HIDDEN_SECTIONS_KEY] = list(DEFAULT_HIDDEN_SECTIONS)


def section_options(values, section_id):
    """Visible options sitting directly in one section, in display order."""
    return [p for p in PARAMS
            if p.section == section_id and p.visible(values)]


def populated_sections(values):
    """Section ids worth drawing a heading for.

    A subsection with nothing visible in it is dropped rather than shown
    empty — that is what keeps the executor knobs off the screen until an
    executor that has any is selected. A category survives on its children.
    """
    live = {p.section for p in PARAMS if p.visible(values)}
    live |= {SECTION_PARENT[s] for s in live if SECTION_PARENT.get(s)}
    return live


def visible_params(values):
    """Every visible option, ordered by the section tree."""
    return [p for sid in SECTION_ORDER for p in section_options(values, sid)]


def cycle_enum(p, value, delta):
    """The next choice `delta` away, stepping over the display-only ones."""
    if value not in p.choices:
        return p.choices[0] if p.choices else value
    i = p.choices.index(value)
    for _ in range(len(p.choices)):
        i = (i + delta) % len(p.choices)
        if p.choices[i] not in p.cycle_skip:
            return p.choices[i]
    return value


# demo.launch.py's name for an option, where it differs from the launcher's id.
LAUNCH_ARG_ALIASES = {"robot_depth_target": "depth"}
# Options demo.launch.py has no argument for: launcher-only UI state, the
# noise attenuation switch (which it expresses as near_cutoff/near_fade), and
# the roadmap options below, whose implemented value is the only behaviour
# there is — nothing downstream reads them.
LAUNCH_ARG_SKIP = {"keyboard", "robot_pose_live", "preset", "rqt", "rqt_depthmap",
                   "thrust_boost", "noise_attenuation", "near_cutoff_m",
                   "near_fade_p",
                   "frontier_space", "exploration", "goal_order",
                   "revisit_scoring", "revisit_trigger",
                   # The real-vehicle surface: demo.launch.py is the simulator
                   # demo and declares no argument for any of it.
                   "platform", "real_sonar", "ardusub_backend",
                   "manual_authority", "enable_vertical", "require_odom",
                   "mavlink_command_url", "mavlink_odom_url",
                   "require_position", "sonar_ip",
                   "sonar_mount_x", "sonar_mount_y", "sonar_mount_z",
                   "sonar_mount_roll", "sonar_mount_pitch", "sonar_mount_yaw"}


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

# The ablation ladder the evaluation is built around: each rung adds one piece
# of the SLAM stack, so the trajectory error between two of them is the
# contribution of exactly that piece. Custom is what any hand edit lands on.
PRESETS = [
    ("custom", {}),
    ("ground_truth", {"slam": "none"}),
    ("no_loop_closure", {"slam": "slam", "loop_closure": False,
                         "revisit": False}),
    ("loop_closure", {"slam": "slam", "loop_closure": True, "revisit": False}),
    ("loop_closure_revisit", {"slam": "slam", "loop_closure": True,
                              "revisit": True}),
]
PRESET_OVERRIDES = dict(PRESETS)


def matching_preset(values):
    """The rung of the ladder this configuration is standing on.

    Recomputed after every change rather than trusted from the file, so the
    preset line can never claim a condition the options underneath it no longer
    describe. Custom when nothing matches — reachable because an option a
    preset pins can be left at a stale value while it is hidden.
    """
    for name, overrides in PRESETS:
        if overrides and all(values.get(k) == v for k, v in overrides.items()):
            return name
    return CUSTOM_PRESET


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
