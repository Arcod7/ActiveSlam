#!/usr/bin/env python3
"""
ActiveSlam launcher: a terminal UI for bringing the stack up, changing options
while it runs, and driving the vehicle by keyboard.

Run it from the repo root:

    python3 launcher.py

The stack is split into independently restartable groups (see launcher_core),
so changing the mapper or the pose source only bounces the layers that depend
on it — the simulator, which is the slow part, keeps running.

The same QWEASD cluster (AZERTY supported) drives in every mode; what it
moves is the mode: teleop moves the vehicle directly, goto moves a target
point the planner swims to, frontier explores on its own until a drive key
takes over. This process already owns a raw terminal, which is the only
thing keyboard control needed a separate window for.
"""

import atexit
import base64
import curses
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import textwrap
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launcher_core as core
import launcher_eval as evaluation
import launcher_model as model

MIN_TERM_HEIGHT = 24
MIN_TERM_WIDTH = 80
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(REPO_ROOT, "config.yaml")
# 2 grouped the flat option list into the section tree and renamed every
# section id with it; restore() reads the options either way and only the
# saved fold layout is version-gated.
CONFIG_VERSION = 2
# Kept only as a one-time migration source for users of the previous launcher.
LEGACY_SELECTION_FILE = os.path.expanduser("~/.activeslam_launcher.json")
LOG_DIR = os.path.join(REPO_ROOT, "logs", "launcher")
EVAL_CONFIG_DIR = os.path.join(REPO_ROOT, "eval", "eval_tools", "config")
KEYBOARD_ROW = "Keyboard layout"
WELCOME_OPTIONS = (
    "Launch", "Evaluation", "---",
    "Update", "Rebuild", "---", "Infos", "Exit", "---", KEYBOARD_ROW,
)

C_HEAD, C_ERR, C_INFO, C_OK, C_WARN = 1, 2, 3, 4, 5
# Set but not running: purple for a value the stack has yet to be applied with.
C_PEND = 6

# How long the arm hint stays reversed after a drive key pressed on a
# disarmed gate. Long enough to catch, short enough not to sit there.
BLOCKED_DRIVE_FLASH_S = 1.5

# Right-hand panel: robot state, live error metrics and the GT/belief pose
# table. Dropped whole below PANEL_MIN_TERM_WIDTH so a trimmed terminal keeps
# the option rows readable.
PANEL_W = 32
PANEL_MIN_TERM_WIDTH = 100
# Below this the selected row has no room for a value list worth reading, so it
# is dropped rather than shown as ellipses.
CHOICE_MIN_W = 14
# Space cycles a value forward, so it reads as a Right arrow everywhere on the
# option list. Ascend gave the key up for it (launcher_core.DRIVE_KEYS).
PREV_KEYS = (curses.KEY_LEFT,)
NEXT_KEYS = (curses.KEY_RIGHT, ord(" "))
# Auto-repeat delivers an arrow every ~40 ms; a second deliberate press is far
# slower. Past this gap a press at either end of a list wraps, inside it holds.
NAV_REPEAT_GAP_S = 0.15
# How long the "copied" acknowledgement stays next to the button.
COPY_NOTE_S = 2.5
# Pose table: how far GT and belief may diverge before the row stops being
# background noise and gets coloured.
POSE_DIFF_WARN_M = 0.25
POSE_DIFF_ERR_M = 1.0
METRICS_STALE_S = 8.0
ACTIVITY_STALE_S = 3.0

# What the panel subscribes to: (key, topic, integer?).
EVAL_METRICS = (
    ("abs_error",    "/eval/abs_error",          False),
    ("ate",          "/eval/ate",                False),
    ("rpe_trans",    "/eval/rpe_trans",          False),
    ("rpe_rot",      "/eval/rpe_rot",            False),
    ("dopt",         "/slam/dopt",               False),
    ("sigma_xy",     "/slam/sigma_xy",           False),
    ("sigma_yaw",    "/slam/sigma_yaw",          False),
    ("u_ratio",      "/frontier_slam/uncertainty_ratio", False),
    ("anees",        "/eval/anees",              False),
    ("nis",          "/slam/nis",                False),
    ("keyframes",    "/slam/keyframe_count",     True),
    ("loops",        "/slam/loop_closure_count", True),
    ("map_coverage", "/eval/map_coverage",       False),
    ("map_accuracy", "/eval/map_accuracy",       False),
)

# Labels the motion executors publish on /frontier_slam/activity.
ACTIVITY_TEXT = {
    "INIT_SCAN":         "initial scan of the surroundings",
    "SCAN":              "scanning in place for frontiers",
    "GOAL_REACHED":      "at the goal, scanning",
    "REVISIT_SWEEP":     "at the revisit target, sweeping the wall",
    "FOLLOW_PATH":       "driving to the next waypoint",
    "TRACK":             "following the wall",
    "WALL_SWITCH_SCAN":  "scanning to reacquire the wall",
    "NO_WALL_SCAN":      "scanning, no wall in view",
    "NO_PATH_PROGRESS":  "stalled, waiting for a replan",
    "EMERG_STOP":        "obstacle ahead, backing off",
    "CTRL_STUCK":        "stuck, spinning to escape",
    "CTRL_STUCK_ESCAPE": "stuck, spinning to escape",
}
# Which axis of the pose marginal drove the D-optimality that fired the
# revisit — revisit_planner.py's CAUSE_* strings. The trigger is the combined
# scalar, so this explains a revisit rather than being a second threshold.
# Keep in step with safety_gate's REVISIT_CAUSE_LABELS, the RViz wording.
REVISIT_CAUSE_TEXT = {
    "position": "position uncertainty",
    "heading":  "heading uncertainty",
}
# Activities that mean the vehicle is holding station rather than travelling.
HOLDING_ACTIVITIES = {"INIT_SCAN", "SCAN", "GOAL_REACHED", "WALL_SWITCH_SCAN",
                      "NO_WALL_SCAN", "NO_PATH_PROGRESS", "REVISIT_SWEEP"}


# --------------------------------------------------------------------------
# persistence

def _flatten(mapping):
    """One flat {option id: value} from a nested or flat mapping."""
    flat = {}
    for key, value in mapping.items():
        if isinstance(value, dict):
            flat.update(_flatten(value))
        else:
            flat[key] = value
    return flat


def _load_mapping(path):
    """Load the launcher config without adding a PyYAML runtime dependency.

    Every value is written in JSON scalar syntax, which is also valid YAML, so
    a whole-file JSON parse is tried first — that accepts a compact
    hand-written config too. The fallback reads the grouped file
    save_selection() writes: option ids are unique across the whole tree, so a
    line carrying a value is taken wherever it sits and the grouping headers,
    which carry none, are skipped. The same pass reads the older flat file.
    """
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return None
    try:
        value = json.loads(text)
        return _flatten(value) if isinstance(value, dict) else None
    except json.JSONDecodeError:
        pass

    values = {}
    # A `key:` with nothing after it is either a grouping header or the head of
    # a block list; which one it was is only known once the next line is read.
    list_key = None
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("- "):
            if list_key is not None:
                values.setdefault(list_key, []).append(_scalar(line[2:].strip()))
            continue
        list_key = None
        if ":" not in line:
            continue
        key, raw = line.split(":", 1)
        key, raw = key.strip(), raw.strip()
        if not raw:
            list_key = key    # a grouping header, or a list about to start
            continue
        values[key] = _scalar(raw)
    return values or None


def _scalar(raw):
    """A config value, JSON where it parses and a bare string where it does not
    — which is what makes a hand-edited unquoted value usable."""
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return raw


def load_selection():
    return _load_mapping(CONFIG_FILE) or _load_mapping(LEGACY_SELECTION_FILE)


def config_lines(values):
    """The config file's text, grouped the way the control screen is.

    Nesting is presentation only — the reader flattens it again — so the
    grouping is free to follow the option tree and the file reads as the
    screen does, one block per category with its subsections indented under
    it. JSON scalar syntax keeps every value standards-compliant YAML.
    """
    lines = [
        "# ActiveSlam launcher configuration.",
        "# Grouped as the control screen is: the three parts of active SLAM,",
        "# the scene they run in, and the tools around them.",
        "# Edit while the launcher is stopped — it rewrites the whole file.",
        f"config_version: {CONFIG_VERSION}",
        "",
        "# Sections folded away in the control screen.",
        f"{model.HIDDEN_SECTIONS_KEY}:",
    ]
    lines += [f"  - {s}" for s in sorted(values.get(model.HIDDEN_SECTIONS_KEY, []))]
    written = set()
    for sid in model.SECTION_ORDER:
        params = [p for p in model.PARAMS if p.section == sid]
        depth = model.SECTION_DEPTH[sid]
        if depth == 0:
            lines.append("")
            lines.append(f"# {model.SECTION_TITLES[sid]}")
        pad = "  " * depth
        lines.append(f"{pad}{sid.rsplit('.', 1)[-1]}:")
        for p in params:
            lines.append(f"{pad}  {p.id}: {json.dumps(values[p.id])}")
            written.add(p.id)
    # An option whose section id does not name a real section would otherwise
    # vanish from the file and silently reset on the next start.
    orphans = [p for p in model.PARAMS if p.id not in written]
    if orphans:
        lines += ["", "# Options with no section — this is a bug in the model.",
                  "unfiled:"]
        lines += [f"  {p.id}: {json.dumps(values[p.id])}" for p in orphans]
    return lines


def save_selection(values):
    try:
        temporary = CONFIG_FILE + ".tmp"
        with open(temporary, "w", encoding="utf-8") as f:
            f.write("\n".join(config_lines(values)) + "\n")
        os.replace(temporary, CONFIG_FILE)
    except OSError:
        pass


def migrate_sensor_profiles(saved):
    """Resolve the retired 'inherit' sensor profiles against the master.

    Sensor noise used to be one master profile plus per-sensor overrides that
    defaulted to following it. The sensors now each name a profile outright, so
    a saved 'inherit' has to be resolved to whatever it was inheriting or the
    sensor silently changes underneath the run. The master itself survives as
    the model the pose graph assumes, where only the nav sections are read —
    the no-reverb variants are their base verbatim there, so they collapse onto
    it rather than being dropped.
    """
    master = saved.get("noise_profile")
    if isinstance(master, str):
        for sensor in ("sonar", "dvl", "imu", "compass", "pressure"):
            key = f"noise_profile_{sensor}"
            if saved.get(key) == "inherit":
                saved[key] = master
        saved["noise_profile"] = master.replace("_no_reverb", "")
    return saved


def restore(values):
    saved = load_selection()
    if saved:
        saved = migrate_sensor_profiles(saved)
        hidden = saved.get(model.HIDDEN_SECTIONS_KEY)
        # Section ids changed with the option tree, so a layout written by an
        # older launcher names sections that no longer exist. Falling through
        # to the default folds the subsections again rather than unfolding
        # everything at once.
        if (isinstance(hidden, list)
                and saved.get("config_version", 0) >= CONFIG_VERSION):
            values[model.HIDDEN_SECTIONS_KEY] = [s for s in hidden
                                                 if s in model.SECTION_ORDER]
        for k, v in saved.items():
            p = model.PARAM_MAP.get(k)
            if p is None:
                continue
            if p.kind == "enum" and v in p.choices:
                values[k] = v
            elif p.kind == "bool" and isinstance(v, bool):
                values[k] = v
            elif p.kind == "int" and isinstance(v, int) and not isinstance(v, bool):
                values[k] = int(p.clamp(v))
            elif (p.kind == "float" and isinstance(v, (int, float))
                  and not isinstance(v, bool) and math.isfinite(v)):
                values[k] = p.clamp(float(v))
            elif p.kind == "text" and isinstance(v, str):
                values[k] = v


# --------------------------------------------------------------------------
# environment

def ros_env(ws_root):
    """Environment shared by every spawned group.

    LD_PRELOAD is deliberately not set here: it is an RViz-specific workaround
    for an octomap symbol clash (demo.launch.py scopes it to the rviz2 node
    alone), and preloading it into every Python node is both unnecessary and a
    good way to cause trouble at interpreter teardown. It is attached to the
    rviz group's own environment instead — see build_groups().
    """
    env = os.environ.copy()
    env.setdefault("QT_QPA_PLATFORM", "xcb")   # RViz needs xcb on this GPU stack
    # A newly copied data/obj mesh becomes selectable after restarting the
    # launcher; it does not need to wait for a package install.
    source_world = os.path.join(REPO_ROOT, "sim", "world")
    if os.path.isdir(source_world):
        env.setdefault("STONEFISH_WORLD_DIR", source_world)
    return env


def available_object_meshes():
    """Names of mesh assets which the TUI may offer in its object line."""
    obj_dir = os.path.join(REPO_ROOT, "sim", "world", "data", "obj")
    try:
        with os.scandir(obj_dir) as entries:
            return sorted(entry.name for entry in entries
                          if entry.is_file()
                          and entry.name.lower().endswith((".obj", ".stl")))
    except OSError:
        return []


def ros_ready():
    return bool(_which("ros2"))


# The container bootstrap.sh creates for a ROS-less host (keep in sync with it).
DISTROBOX_NAME = "activeslam-jazzy"


def _distrobox_has(name):
    try:
        r = subprocess.run(["distrobox", "list"], capture_output=True,
                           text=True, timeout=8)
    except (OSError, subprocess.SubprocessError):
        return False
    return r.returncode == 0 and any(
        field.strip() == name
        for line in r.stdout.splitlines()
        for field in line.split("|"))


def ensure_sourced_environment():
    """Re-exec so the launcher runs from a plain shell, sourcing what it needs.

    Two cases. When ROS is installed on this filesystem, re-source the
    workspace and its venv: colcon's install/setup.bash chains in the ROS
    underlay it was built against, and the venv activate puts gtsam/rclpy on the
    path the spawned nodes inherit. When ROS is not here but was installed into
    the Distrobox bootstrap.sh builds, hop into that container and re-run there,
    where the first case then applies. A no-op once ros2 is on PATH with the
    workspace venv active; sentinels keep each hop to a single re-exec.
    """
    if os.environ.get("ACTIVESLAM_LAUNCHER_SOURCED") == "1":
        return
    ws_root = core.find_workspace_root(__file__)
    if not ws_root:
        return
    venv_dir = os.path.join(ws_root, ".venv")
    venv_active = os.environ.get("VIRTUAL_ENV") == venv_dir
    if _which("ros2") and (venv_active or not os.path.isdir(venv_dir)):
        return

    if glob.glob("/opt/ros/*/setup.bash"):
        ws_setup = os.path.join(ws_root, "install", "setup.bash")
        if not os.path.isfile(ws_setup):
            return
        parts = [f'source "{ws_setup}"']
        venv_activate = os.path.join(venv_dir, "bin", "activate")
        if os.path.isfile(venv_activate):
            parts.append(f'source "{venv_activate}"')
        parts.append(f'exec python3 "{os.path.abspath(__file__)}" "$@"')
        os.environ["ACTIVESLAM_LAUNCHER_SOURCED"] = "1"
        try:
            os.execvp("bash",
                      ["bash", "-c", " && ".join(parts), "bash", *sys.argv[1:]])
        except OSError:
            pass   # fall through and run unsourced — ros_ready() reports it
        return

    # ROS is not on this filesystem. If we are on a bare host (not already in a
    # container) and the bootstrap Distrobox exists, re-run inside it — where
    # the branch above sources the workspace. The env sentinel avoids a second
    # hop; SOURCED is deliberately left unset so sourcing still happens inside.
    in_container = os.path.exists("/run/.containerenv") or os.path.exists("/.dockerenv")
    if (not in_container
            and os.environ.get("ACTIVESLAM_LAUNCHER_IN_DISTROBOX") != "1"
            and _which("distrobox") and _distrobox_has(DISTROBOX_NAME)):
        try:
            os.execvp("distrobox", [
                "distrobox", "enter", DISTROBOX_NAME, "--",
                "env", "ACTIVESLAM_LAUNCHER_IN_DISTROBOX=1",
                "python3", os.path.abspath(__file__), *sys.argv[1:]])
        except OSError:
            pass   # fall through — ros_ready() reports the missing ros2


def normalize_rmw_implementation():
    """Drop RMW_IMPLEMENTATION when its implementation is not installed.

    A shell that exports an RMW whose library is missing makes every node fail
    to start or silently not discover peers; falling back to the distro default
    is what running the launcher under `env -u RMW_IMPLEMENTATION` did by hand.
    Returns a note for the session log, or None. Only unsets a genuinely
    missing implementation — a working non-default RMW is left untouched.
    """
    impl = os.environ.get("RMW_IMPLEMENTATION")
    if not impl:
        return None
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", impl],
                           capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.SubprocessError):
        return None
    if r.returncode != 0:
        del os.environ["RMW_IMPLEMENTATION"]
        return (f"RMW_IMPLEMENTATION={impl} is not installed; unset it and fell "
                f"back to the distribution default.")
    return None


def _which(prog):
    for d in os.environ.get("PATH", "").split(os.pathsep):
        p = os.path.join(d, prog)
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def bringup_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("bringup")
    except Exception:
        return ""


# --------------------------------------------------------------------------
# teleop (rclpy imported lazily so the UI runs before the workspace is built)

class RosLink:
    """Teleop publisher, motion-gate arming, and gate status — in this process.

    The safety gate is fail-closed and starts disabled, so without a way to arm
    it from here the vehicle cannot move at all when RViz (and its Motion Safety
    panel) is switched off.
    """

    STEP = 0.9
    # Mirrors safety_gate.py's max_abs_command default (1.0): the gate rejects
    # (and latches INVALID_COMMAND on) any component outside this range, so a
    # speed_factor/turn_factor above ~1.1x/~6.7x must saturate here rather than
    # publish an out-of-range command.
    MAX_ABS_COMMAND = 1.0

    def __init__(self):
        self.node = None
        self.pub = None
        self.enable_pub = None
        self.marker_pub = None
        self.point_suspend_pub = None
        self.point_goal_pub = None
        self.rclpy = None
        self.error = None
        self.gate_state = None
        self.enabled = None
        self.robot_pose = None
        self.slam_pose = None
        # Which odometry the drive/point controller works in: "slam" when the
        # pose graph owns the TF frame the operator sees, "gt" otherwise.
        # Synced from values["slam"] every control_screen tick.
        self.control_source = "gt"
        # Synced from values["speed_factor"/"turn_factor"] every control_screen
        # tick; multiply STEP so runs can be piloted faster without touching
        # /motion/body_command's magnitude elsewhere.
        self.speed_factor = 1.0
        self.turn_factor = 1.0
        # Right-hand panel feeds: last value per EVAL_METRICS key, and what the
        # running motion executor says it is doing.
        self.metrics = {}
        self.metrics_at = 0.0
        self.activity = None
        self.activity_at = 0.0
        self.revisit_state = None
        self.revisit_cause = None

    def start(self):
        if self.node:
            return True
        try:
            import rclpy
            from geometry_msgs.msg import PointStamped, Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Bool, Float64, Int32, String
            from visualization_msgs.msg import Marker
        except Exception as e:
            self.error = f"rclpy unavailable ({e}). Source the workspace first."
            return False
        try:
            self.rclpy = rclpy
            self.Twist, self.Bool, self.Marker = Twist, Bool, Marker
            self.PointStamped = PointStamped
            if not rclpy.ok():
                # The launcher owns SIGINT handling; rclpy must not take it over.
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node("activeslam_launcher")
            # No /motion/body_command publisher here on purpose: the safety gate
            # counts *publishers*, so merely having teleop available would read
            # as MULTIPLE_COMMAND_SOURCES and zero all motion. It is created
            # only while teleop is active (see open_teleop).
            self.enable_pub = self.node.create_publisher(Bool, "/motion/enable", 10)
            # The target-point marker is safe to keep around: the gate only
            # counts publishers on the command topic.
            self.marker_pub = self.node.create_publisher(
                Marker, "/activeslam/target_point", 1)
            # Target-point mode routes through the same A* planner frontier
            # exploration uses (frontier_extractor + waypoint_controller),
            # via the suspend/goal handoff revisit_planner.py also uses —
            # not a body-command publisher, so these never trip the gate's
            # command-source count either.
            self.point_suspend_pub = self.node.create_publisher(
                Bool, "/frontier_slam/suspend", 1)
            self.point_goal_pub = self.node.create_publisher(
                PointStamped, "/frontier_slam/goal", 1)
            self.node.create_subscription(
                String, "/motion/safety_status", self._status_cb, 10)
            self.node.create_subscription(
                Odometry, "/StoneFish/Odometry", self._odom_cb, 10)
            # Only published while slam:=slam; subscribing unconditionally is
            # harmless — the topic simply stays silent otherwise.
            self.node.create_subscription(
                Odometry, "/slam/odometry", self._slam_odom_cb, 10)
            # Feeds for the right-hand panel. Same story: silent when the layer
            # that publishes them is not running.
            for key, topic, is_int in EVAL_METRICS:
                self.node.create_subscription(
                    Int32 if is_int else Float64, topic,
                    lambda msg, key=key: self._metric_cb(key, msg), 10)
            self.node.create_subscription(
                String, "/frontier_slam/activity", self._activity_cb, 1)
            self.node.create_subscription(
                String, "/frontier_slam/revisit_state", self._revisit_cb, 1)
            self.node.create_subscription(
                String, "/frontier_slam/revisit_cause", self._revisit_cause_cb, 1)
        except Exception as e:
            self.error = f"could not create launcher node: {e}"
            self.node = None
            return False
        return True

    def _status_cb(self, msg):
        self.gate_state = msg.data

    def _metric_cb(self, key, msg):
        self.metrics[key] = msg.data
        self.metrics_at = time.time()

    def _activity_cb(self, msg):
        self.activity = msg.data
        self.activity_at = time.time()

    def _revisit_cb(self, msg):
        self.revisit_state = msg.data

    def _revisit_cause_cb(self, msg):
        self.revisit_cause = msg.data or None

    @staticmethod
    def _pose_from_msg(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rpy = core.quaternion_to_rpy(q.x, q.y, q.z, q.w)
        return {
            "robot_x": p.x, "robot_y": p.y, "robot_z": p.z,
            "robot_roll": math.degrees(rpy[0]),
            "robot_pitch": math.degrees(rpy[1]),
            "robot_yaw": math.degrees(rpy[2]),
        }

    def _odom_cb(self, msg):
        """Keep the TUI's robot fields aligned with simulation/teleop motion."""
        self.robot_pose = self._pose_from_msg(msg)

    def _slam_odom_cb(self, msg):
        self.slam_pose = self._pose_from_msg(msg)

    def forget_pose(self):
        """Drop the readback once the vehicle it describes is gone."""
        self.robot_pose = None
        self.slam_pose = None

    def control_pose(self):
        """Pose the drive/point controller works in: the SLAM estimate when
        that owns the TF frame the operator sees, ground truth otherwise —
        with a fallback to whichever source has published at all."""
        if self.control_source == "slam":
            return self.slam_pose or self.robot_pose
        return self.robot_pose or self.slam_pose

    SPIN_BATCH = 32

    def spin(self):
        """Drain the callback queue. spin_once handles one item, and a dozen
        subscriptions at simulator rates would starve the slow ones at one
        callback per UI tick."""
        if not self.node:
            return
        for _ in range(self.SPIN_BATCH):
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0)
            except Exception:
                return

    def peer_count(self):
        """Nodes this process can see, excluding itself. 0 means discovery is
        not working, whatever else is wrong."""
        if not self.node:
            return 0
        try:
            return max(0, len(self.node.get_node_names()) - 1)
        except Exception:
            return 0

    def _unreachable_reason(self, specific):
        if self.peer_count() > 0:
            return specific
        rmw = os.environ.get("RMW_IMPLEMENTATION") or "distribution default"
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        return (f"no other ROS nodes are visible — discovery is not working "
                f"(RMW_IMPLEMENTATION={rmw}, ROS_DOMAIN_ID={domain}). "
                f"See docs/TROUBLESHOOTING.md.")

    def set_enabled(self, on, timeout=3.0):
        """Arm/disarm the motion gate.

        Waits for the gate's subscription to match before publishing: a message
        sent immediately after create_publisher is dropped, because discovery
        has not connected the two ends yet.
        """
        if not self.start():
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.enable_pub.get_subscription_count() > 0:
                break
            self.spin()
            time.sleep(0.05)
        else:
            # "Nobody is subscribed" reads as a gate problem, but the usual
            # cause is that this process cannot see the stack at all — a
            # mismatched RMW or domain. Those need different fixes, so say
            # which one it is.
            self.error = self._unreachable_reason(
                "motion gate is not subscribed to /motion/enable")
            return False
        msg = self.Bool()
        msg.data = bool(on)
        for _ in range(3):          # cheap redundancy on a best-effort link
            self.enable_pub.publish(msg)
            time.sleep(0.02)
        self.enabled = bool(on)
        return True

    def respawn(self, name, xyz, rpy, timeout=10.0):
        """Teleport the robot back to its scenario spawn pose.

        Uses Stonefish's respawn_robot service, so the simulator itself keeps
        running — only the vehicle is moved.
        """
        if not self.start():
            return False, self.error or "no ROS connection"
        try:
            from stonefish_ros2.srv import Respawn
        except Exception as e:
            return False, f"stonefish_ros2 srv unavailable: {e}"
        try:
            client = self.node.create_client(Respawn, "/respawn_robot")
            if not client.wait_for_service(timeout_sec=timeout):
                return False, "/respawn_robot did not appear (is the simulator up?)"
            req = Respawn.Request()
            req.name = name
            req.origin.position.x, req.origin.position.y, req.origin.position.z = xyz
            q = core.rpy_to_quaternion(*rpy)
            (req.origin.orientation.x, req.origin.orientation.y,
             req.origin.orientation.z, req.origin.orientation.w) = q
            future = client.call_async(req)
            deadline = time.time() + timeout
            while time.time() < deadline and not future.done():
                self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if not future.done():
                return False, "respawn timed out"
            result = future.result()
            self.node.destroy_client(client)
            if result is None:
                return False, "respawn returned no result"
            return bool(result.success), (result.message or "respawned")
        except Exception as e:
            return False, f"respawn failed: {e}"

    def set_static_entity_pose(self, name, xyz, rpy_degrees, timeout=5.0):
        """Move a static Stonefish entity without reconstructing the scene."""
        if not self.start():
            return False, self.error or "no ROS connection"
        try:
            from stonefish_ros2.srv import SetEntityPose
        except Exception as e:
            return False, f"stonefish_ros2 srv unavailable: {e}"
        client = None
        try:
            client = self.node.create_client(SetEntityPose, "/set_entity_pose")
            if not client.wait_for_service(timeout_sec=timeout):
                return False, "/set_entity_pose did not appear (is the target scene up?)"
            req = SetEntityPose.Request()
            req.name = name
            req.pose.position.x, req.pose.position.y, req.pose.position.z = xyz
            rpy = tuple(math.radians(angle) for angle in rpy_degrees)
            q = core.rpy_to_quaternion(*rpy)
            (req.pose.orientation.x, req.pose.orientation.y,
             req.pose.orientation.z, req.pose.orientation.w) = q
            future = client.call_async(req)
            deadline = time.time() + timeout
            while time.time() < deadline and not future.done():
                self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if not future.done():
                return False, "moving target timed out"
            result = future.result()
            if result is None:
                return False, "moving target returned no result"
            return bool(result.success), (result.message or "target moved")
        except Exception as e:
            return False, f"moving target failed: {e}"
        finally:
            if client is not None:
                try:
                    self.node.destroy_client(client)
                except Exception:
                    pass

    def open_teleop(self):
        """Become a command source. Only valid once the planner is suspended."""
        if not self.start():
            return False
        if self.pub is None:
            self.pub = self.node.create_publisher(
                self.Twist, "/motion/body_command", 10)
            deadline = time.time() + 2.0
            while time.time() < deadline and self.pub.get_subscription_count() == 0:
                self.spin()
                time.sleep(0.05)
        return True

    def close_teleop(self):
        """Stop being a command source so the planner can own the topic again."""
        if self.pub is not None:
            try:
                self.halt()
                time.sleep(0.05)
                self.node.destroy_publisher(self.pub)
            except Exception:
                pass
            self.pub = None

    def send_action(self, action):
        """Publish the body demand for a canonical drive action.

        The action->demand mapping mirrors launch_tools/keyboard_control.py so
        the standalone keyboard node and the launcher drive identically."""
        if not self.pub:
            return
        cap = self.MAX_ABS_COMMAND
        step = max(-cap, min(cap, self.STEP * self.speed_factor))
        turn = max(-cap, min(cap, (self.STEP / 6) * self.turn_factor))
        self.send_body(*core.action_to_command(action, step, turn))

    def send_body(self, f, s, y, v):
        """Publish an explicit body demand, saturated at the gate's limit."""
        if not self.pub:
            return
        cap = self.MAX_ABS_COMMAND
        clamp = lambda u: max(-cap, min(cap, u))
        msg = self.Twist()
        msg.linear.x = clamp(f)
        msg.linear.y = clamp(s)
        msg.linear.z = -clamp(v)   # NED: negative body Z demand moves upward
        msg.angular.z = clamp(y)
        self.pub.publish(msg)

    def halt(self):
        if self.pub:
            self.pub.publish(self.Twist())

    def publish_target_marker(self, point):
        """Draw the follow-point target in RViz (world_ned sphere).

        Brown at the size frontier mode uses (visualizer.py C_GOAL/GOAL_SCALE):
        goto and frontier exploration drive the same waypoint controller, so
        one goal marker, not two that read as different things.

        Purple while the motion gate is disabled: the point can be driven
        around with the gate closed, and a live-looking target the vehicle is
        never going to move toward reads as a stuck planner. Same purple as
        the gate's robot arrow (safety_gate.py).
        """
        if self.marker_pub is None:
            return
        m = self.Marker()
        m.header.frame_id = "world_ned"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = "activeslam"
        m.id = 0
        m.type = self.Marker.SPHERE
        m.action = self.Marker.ADD
        (m.pose.position.x, m.pose.position.y, m.pose.position.z) = point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.8
        disabled = self.gate_state == "DISABLED"
        m.color.r, m.color.g, m.color.b, m.color.a = (
            (0.7, 0.2, 0.9, 0.9) if disabled else (0.588, 0.353, 0.157, 0.95))
        self.marker_pub.publish(m)

    def publish_point_goal(self, point):
        """Hand the target point to frontier_extractor as an external goal.

        Only takes effect while suspended (see set_planner_suspended) — the
        same /frontier_slam/goal handoff revisit_planner.py uses. Its A*
        replan loop then drives waypoint_controller at the point, same as
        any frontier goal.
        """
        if self.point_goal_pub is None:
            return
        g = self.PointStamped()
        g.header.frame_id = "world_ned"
        g.header.stamp = self.node.get_clock().now().to_msg()
        (g.point.x, g.point.y, g.point.z) = point
        self.point_goal_pub.publish(g)

    def set_planner_suspended(self, suspended):
        """Pause/resume frontier_extractor's own frontier-goal picking."""
        if self.point_suspend_pub is None:
            return
        self.point_suspend_pub.publish(self.Bool(data=suspended))

    def clear_target_marker(self):
        if self.marker_pub is None:
            return
        try:
            m = self.Marker()
            m.header.frame_id = "world_ned"
            m.ns = "activeslam"
            m.id = 0
            m.action = self.Marker.DELETE
            self.marker_pub.publish(m)
        except Exception:
            pass

    def close(self):
        if self.node:
            try:
                self.halt()
                self.clear_target_marker()
                self.set_planner_suspended(False)
                self.node.destroy_node()
            except Exception:
                pass
        self.node = self.pub = self.marker_pub = None
        self.point_suspend_pub = self.point_goal_pub = None


def set_live_param(node_basename, param, value):
    """Push a value to a running node. Returns (ok, message)."""
    try:
        listing = subprocess.run(["ros2", "node", "list"], capture_output=True,
                                 text=True, timeout=8).stdout.split()
    except (subprocess.SubprocessError, OSError) as e:
        return False, f"node list failed: {e}"
    target = next((n for n in listing if n.rstrip("/").split("/")[-1] == node_basename), None)
    if target is None:
        return False, f"{node_basename} is not running"
    try:
        r = subprocess.run(["ros2", "param", "set", target, param, str(value)],
                           capture_output=True, text=True, timeout=10)
    except (subprocess.SubprocessError, OSError) as e:
        return False, f"param set failed: {e}"
    if r.returncode != 0 or "Set parameter failed" in r.stdout:
        return False, (r.stdout or r.stderr).strip().splitlines()[-1:] and \
            (r.stdout or r.stderr).strip().splitlines()[-1] or "param set failed"
    return True, f"{node_basename}.{param} = {value}"


# --------------------------------------------------------------------------
# drawing helpers

def init_colors():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(C_HEAD, curses.COLOR_YELLOW, -1)
    curses.init_pair(C_ERR, curses.COLOR_RED, -1)
    curses.init_pair(C_INFO, curses.COLOR_CYAN, -1)
    curses.init_pair(C_OK, curses.COLOR_GREEN, -1)
    curses.init_pair(C_WARN, curses.COLOR_MAGENTA, -1)
    # A brighter purple than the magenta warnings use where the terminal has
    # 256 colours; magenta is the fallback, which still reads as "not green".
    curses.init_pair(C_PEND, 141 if curses.COLORS >= 256 else curses.COLOR_MAGENTA, -1)
    # Button clicks only; leaving position reporting off keeps the terminal's
    # own text selection working.
    try:
        curses.mousemask(curses.BUTTON1_PRESSED | curses.BUTTON1_CLICKED)
    except curses.error:
        pass


def _clicked(button) -> bool:
    """True when the pending mouse event is a press inside `button`, given as
    (row, x_start, x_end)."""
    if button is None:
        return False
    try:
        _, mx, my, _, state = curses.getmouse()
    except curses.error:
        return False
    if not state & (curses.BUTTON1_PRESSED | curses.BUTTON1_CLICKED):
        return False
    row, x0, x1 = button
    return my == row and x0 <= mx < x1


def copy_to_clipboard(text: str) -> str:
    """Put `text` on the system clipboard, returning what was used so the
    caller can say so. Falls back to the OSC 52 escape sequence, which the
    terminal emulator handles itself and so works over SSH too."""
    for argv, name in ((["wl-copy"], "wl-copy"),
                       (["xclip", "-selection", "clipboard"], "xclip"),
                       (["xsel", "--clipboard", "--input"], "xsel")):
        if shutil.which(argv[0]) is None:
            continue
        try:
            subprocess.run(argv, input=text.encode(), check=True, timeout=2)
            return name
        except (subprocess.SubprocessError, OSError):
            continue
    payload = base64.b64encode(text.encode()).decode()
    try:
        sys.stdout.write(f"\033]52;c;{payload}\a")
        sys.stdout.flush()
        return "terminal"
    except OSError:
        return ""


def put(stdscr, y, x, text, attr=0, maxx=None):
    """Draw clipped to the screen, or to `maxx` when a column is reserved."""
    h, w = stdscr.getmaxyx()
    limit = w if maxx is None else min(w, maxx)
    if y < 0 or y >= h or x >= limit:
        return
    try:
        stdscr.addstr(y, x, str(text)[:max(0, limit - x - 1)], attr)
    except curses.error:
        pass


def too_small(stdscr):
    h, w = stdscr.getmaxyx()
    stdscr.erase()
    put(stdscr, h // 2 - 1, max(0, (w - 28) // 2), "Please increase terminal size",
        curses.A_BOLD)
    put(stdscr, h // 2, max(0, (w - 24) // 2),
        f"({MIN_TERM_WIDTH}x{MIN_TERM_HEIGHT} required)", curses.A_DIM)
    stdscr.refresh()


def fits(stdscr):
    h, w = stdscr.getmaxyx()
    return h >= MIN_TERM_HEIGHT and w >= MIN_TERM_WIDTH


def read_key(stdscr, timeout):
    """The next key, and whether it was already waiting to be read.

    A key found in the buffer was typed before the read that returned it, so
    the time since the previous key says how long this frame took, not how long
    the finger was off the arrow."""
    stdscr.timeout(0)
    key = stdscr.getch()
    if key != -1:
        return key, True
    stdscr.timeout(timeout)
    return stdscr.getch(), False


class NavRepeat:
    """Tells a held arrow from a fresh press.

    Two signals feed this, because neither covers both screens. A key already
    waiting when the frame read it was typed during the previous redraw — on
    the control screen, which spins ROS and repaints between reads, that is
    what auto-repeat looks like. Where the redraw is fast enough that nothing
    is ever waiting, the gap since the last press shows the hold instead.

    Neither is trusted on its own: a deliberate press can land mid-redraw and
    look buffered, and a finger can tap faster than the gap. So they only
    lengthen a run, and it takes a run no tapping would sustain before the
    arrow counts as held.
    """

    # Presses in an unbroken fast run before it is one. Auto-repeat reaches
    # this in well under a second; getting there by hand means tapping at the
    # keyboard's own repeat rate without a single gap.
    HOLD_RUN = 3

    def __init__(self):
        self.key = None
        self.at = 0.0
        self.run = 0

    def held(self, key, buffered=False, queued=0):
        now = time.time()
        fast = key == self.key and (buffered
                                    or now - self.at < NAV_REPEAT_GAP_S)
        # Presses swallowed inside one frame are one burst by definition, so
        # they count in full rather than as a single step.
        self.run = (self.run + 1 if fast else 0) + queued
        self.key, self.at = key, now
        return self.run >= self.HOLD_RUN


def step_row(idx, delta, count, skip=None, repeated=False):
    """Next selectable index `delta` away, stepping over rows `skip` rejects.

    A held arrow stops at the first or last row; only a fresh press from there
    wraps to the other end, so overshooting a long list takes a deliberate
    second press."""
    if count <= 0:
        return idx
    nxt = idx
    for _ in range(count):
        nxt += delta
        if not 0 <= nxt < count:
            if repeated:
                return idx
            nxt %= count
        if skip is None or not skip(nxt):
            return nxt
    return idx


def nav_step(stdscr, nav, key, buffered, idx, delta, count, skip=None):
    """Move the cursor for one arrow press and any repeats queued behind it.

    The whole burst is spent in this frame rather than one press per redraw, so
    a held arrow crosses a long list at the keyboard's pace, not the screen's.
    The caller's loop sets its own timeout again before the next read."""
    queued = 0
    stdscr.timeout(0)
    while True:
        k = stdscr.getch()
        if k == key:
            queued += 1
            continue
        if k != -1:
            curses.ungetch(k)          # not ours — leave it for the main loop
        break
    repeated = nav.held(key, buffered, queued)
    for n in range(queued + 1):
        idx = step_row(idx, delta, count, skip, repeated or n > 0)
    return idx


# --------------------------------------------------------------------------
# screens

def welcome_screen(stdscr, built, values, eval_runner):
    options = WELCOME_OPTIONS
    idx = 0
    nav = NavRepeat()
    while True:
        if not fits(stdscr):
            too_small(stdscr)
            stdscr.timeout(120)
            if stdscr.getch() == curses.KEY_RESIZE:
                continue
            continue
        stdscr.timeout(-1)
        eval_runner.poll()
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        title = "ActiveSlam Workspace Manager"
        put(stdscr, 0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)
        sub = "Active SLAM for underwater volumetric exploration"
        put(stdscr, 1, max(0, (w - len(sub)) // 2), sub, curses.A_DIM)

        if not built:
            put(stdscr, 3, 4, "Workspace is not built yet — run Rebuild first.",
                curses.color_pair(C_WARN) | curses.A_BOLD)

        row = 5
        for i, opt in enumerate(options):
            if opt == "---":
                put(stdscr, row + i, 6, "-" * 22, curses.A_DIM)
                continue
            if opt == KEYBOARD_ROW:
                label = f"{KEYBOARD_ROW}: {values['keyboard'].upper()}"
            elif opt == "Evaluation" and eval_runner.state != "idle":
                progress = eval_runner.progress()
                done = progress.get("completed", 0)
                total = progress.get("total") or (
                    eval_runner.matrix.total_runs if eval_runner.matrix else 0)
                suffix = (f"{done}/{total}" if total
                          else eval_runner.state)
                label = f"Evaluation  [{suffix}]"
            else:
                label = opt
            prefix = "> " if idx == i else "  "
            attr = curses.A_REVERSE if idx == i else curses.A_NORMAL
            if opt in ("Launch", "Evaluation"):
                attr |= curses.A_BOLD
            if opt == "Evaluation" and eval_runner.running:
                attr |= curses.color_pair(C_OK)
            put(stdscr, row + i, 4, f"{prefix}{label}", attr)

        hints = {
            "Launch": "Configure options and bring the stack up.",
            "Evaluation": ("Run a repeatable matrix and watch data collection "
                           "progress."),
            KEYBOARD_ROW: "Enter to switch between QWERTY and AZERTY.",
            "Update": "git pull, then reinstall dependencies (./bootstrap.sh).",
            "Rebuild": "colcon build --symlink-install.",
            "Infos": "Technologies this project is built on.",
            "Exit": "Leave the launcher.",
        }
        put(stdscr, row + len(options) + 1, 4, hints.get(options[idx], ""),
            curses.color_pair(C_INFO))
        put(stdscr, h - 1, 2, "Up/Down navigate   Enter select   q quit", curses.A_DIM)
        stdscr.refresh()

        # Keep the badge moving while a background evaluation is active.
        key, buffered = read_key(stdscr, 500 if eval_runner.running else -1)
        if key == -1:
            continue
        if key == curses.KEY_RESIZE:
            continue
        if key in (curses.KEY_UP, curses.KEY_DOWN):
            idx = nav_step(stdscr, nav, key, buffered, idx,
                           -1 if key == curses.KEY_UP else 1, len(options),
                           skip=lambda i: options[i] == "---")
        elif key in (ord("\n"), curses.KEY_ENTER, 10, 13):
            if options[idx] == KEYBOARD_ROW:
                values["keyboard"] = ("azerty" if values["keyboard"] == "qwerty"
                                      else "qwerty")
                save_selection(values)
                continue
            return options[idx]
        elif key in (ord("q"), ord("Q"), 27):
            return "Exit"


def infos_screen(stdscr):
    scroll = 0
    lines = []
    for heading, body in model.INFOS:
        lines.append(("head", heading))
        lines.extend(("body", b) for b in body)
        lines.append(("body", ""))
    while True:
        if not fits(stdscr):
            too_small(stdscr)
            stdscr.timeout(120)
            stdscr.getch()
            continue
        stdscr.timeout(-1)
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        title = "Technologies"
        put(stdscr, 0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)
        # A row above and below the text is kept clear for the cut marks.
        view = h - 4
        scroll = max(0, min(scroll, max(0, len(lines) - view)))
        for i, (kind, text) in enumerate(lines[scroll:scroll + view]):
            if kind == "head":
                put(stdscr, 2 + i, 2, text, curses.A_BOLD | curses.color_pair(C_HEAD))
            else:
                put(stdscr, 2 + i, 4, text)
        if scroll:
            put(stdscr, 1, 4, "...", curses.A_DIM)
        if scroll + view < len(lines):
            put(stdscr, 2 + view, 4, "...", curses.A_DIM)
        put(stdscr, h - 1, 2, "Up/Down scroll   q back", curses.A_DIM)
        stdscr.refresh()
        key = stdscr.getch()
        if key == curses.KEY_UP:
            scroll -= 1
        elif key == curses.KEY_DOWN:
            scroll += 1
        elif key == curses.KEY_NPAGE:
            scroll += view
        elif key == curses.KEY_PPAGE:
            scroll -= view
        elif key in (ord("q"), ord("Q"), 27, ord("\n")):
            return


def _eval_float(row, *names):
    for name in names:
        raw = row.get(name)
        if raw not in (None, ""):
            try:
                return float(raw)
            except (TypeError, ValueError):
                pass
    return None


def _eval_metric(value, unit="", percent=False):
    if value is None or not math.isfinite(value):
        return "—"
    if percent:
        return f"{100.0 * value:.1f}%"
    return f"{value:.3f}{unit}"


def _eval_progress_bar(width, fraction):
    width = max(8, width)
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(width * fraction))
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def confirm_evaluation_stop(stdscr):
    """The active run is kept as partial data, but stopping it is deliberate."""
    while True:
        h, w = stdscr.getmaxyx()
        lines = [
            "Stop this evaluation?",
            "The active run will be terminated; completed and partial data",
            "will remain in the batch directory.",
            "",
            "y stop     n continue",
        ]
        box_w = min(w - 8, max(len(line) for line in lines) + 6)
        top = max(2, (h - len(lines) - 2) // 2)
        left = max(2, (w - box_w) // 2)
        for row in range(top, top + len(lines) + 2):
            put(stdscr, row, left, " " * box_w, curses.A_REVERSE)
        for i, line in enumerate(lines):
            attr = curses.A_REVERSE | (curses.A_BOLD if i == 0 else 0)
            put(stdscr, top + 1 + i, left + 3, line, attr,
                maxx=left + box_w)
        stdscr.refresh()
        key = stdscr.getch()
        if key in (ord("y"), ord("Y")):
            return True
        if key in (ord("n"), ord("N"), 27):
            return False


def evaluation_screen(stdscr, runner, built, ws_root):
    """Choose, run, and monitor a repeatable evaluation matrix."""
    matrices = evaluation.discover_matrices(EVAL_CONFIG_DIR)
    selected = 0
    if runner.matrix is not None:
        selected = next(
            (i for i, info in enumerate(matrices)
             if info.path == runner.matrix.path), selected)
    nav = NavRepeat()
    note = ""
    note_kind = C_INFO

    while True:
        runner.poll()
        if not fits(stdscr):
            too_small(stdscr)
            stdscr.timeout(120)
            stdscr.getch()
            continue
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        title = "Evaluation"
        put(stdscr, 0, max(0, (w - len(title)) // 2), title,
            curses.A_BOLD | curses.color_pair(C_HEAD))
        put(stdscr, 1, 2,
            "Run a configuration × seed matrix and collect comparable metrics.",
            curses.A_DIM)

        list_w = min(32, max(25, w // 3))
        put(stdscr, 3, 2, "Matrices", curses.A_BOLD)
        put(stdscr, 3, list_w + 1, "│", curses.A_DIM)
        visible = h - 7
        if matrices:
            selected = max(0, min(selected, len(matrices) - 1))
            first = max(0, min(selected - visible // 2,
                               len(matrices) - visible))
            for screen_row, i in enumerate(
                    range(first, min(len(matrices), first + visible)), 4):
                info = matrices[i]
                marker = "▶" if runner.matrix and info.path == runner.matrix.path \
                    and runner.running else " "
                label = f"{marker} {info.name}"
                attr = curses.A_REVERSE if i == selected else curses.A_NORMAL
                if info.error:
                    attr |= curses.color_pair(C_ERR)
                put(stdscr, screen_row, 2, label, attr, maxx=list_w)
            if first:
                put(stdscr, 3, list_w - 3, "↑", curses.A_DIM)
            if first + visible < len(matrices):
                put(stdscr, h - 3, list_w - 3, "↓", curses.A_DIM)
        else:
            put(stdscr, 5, 4, "No matrix_*.yaml files found",
                curses.color_pair(C_WARN), maxx=list_w)

        for row in range(4, h - 2):
            put(stdscr, row, list_w + 1, "│", curses.A_DIM)

        x = list_w + 4
        right_w = max(20, w - x - 2)
        info = matrices[selected] if matrices else None
        if info:
            put(stdscr, 3, x, info.name, curses.A_BOLD)
            put(stdscr, 4, x, info.filename, curses.A_DIM)
            plan = (f"{info.configs} configs × {info.seeds} seeds = "
                    f"{info.total_runs} runs")
            put(stdscr, 6, x, plan, curses.color_pair(C_INFO))
            put(stdscr, 7, x,
                f"{evaluation.format_duration(info.duration_s)} per run  ·  "
                f"{evaluation.format_duration(info.settle_s)} settle  ·  "
                f"~{evaluation.format_duration(info.estimated_s)} total")
            bag_text = "recording rosbag" if info.record_bag else "metrics only"
            common = info.common_args
            setup = "  ".join(
                f"{key}={common[key]}" for key in
                ("mode", "motion", "mapper", "slam", "noise_profile")
                if key in common)
            put(stdscr, 8, x, bag_text + (f"  ·  {setup}" if setup else ""),
                curses.A_DIM)
            description = info.error or info.description
            desc_attr = curses.color_pair(C_ERR) if info.error else curses.A_NORMAL
            for i, line in enumerate(textwrap.wrap(description, right_w)[:2]):
                put(stdscr, 10 + i, x, line, desc_attr)

        progress = runner.progress()
        if runner.state != "idle":
            active = runner.matrix or info
            total = int(progress.get("total") or (
                active.total_runs if active else 0))
            completed = int(progress.get("completed", 0))
            phase = str(progress.get("phase") or runner.state)
            elapsed = time.time() - (runner.started_at or time.time())
            run_elapsed = time.time() - float(
                progress.get("run_started_at") or time.time())
            partial = 0.0
            if phase == "running" and active and active.duration_s > 0:
                partial = min(0.99, run_elapsed / active.duration_s)
            fraction = (completed + partial) / total if total else 0.0
            row = 13
            state_attr = {
                "running": C_OK, "stopping": C_WARN, "complete": C_OK,
                "stopped": C_WARN, "failed": C_ERR,
            }.get(runner.state, C_INFO)
            put(stdscr, row, x,
                f"{runner.state.upper()}  ·  {phase.replace('_', ' ')}",
                curses.A_BOLD | curses.color_pair(state_attr))
            row += 1
            bar_w = max(8, min(34, right_w - 15))
            put(stdscr, row, x,
                f"{_eval_progress_bar(bar_w, fraction)} "
                f"{completed}/{total}  {100 * fraction:4.1f}%")
            row += 1
            current_name = progress.get("name")
            current_seed = progress.get("seed")
            if current_name is not None:
                put(stdscr, row, x,
                    f"Run {progress.get('current', completed + 1)}/{total}: "
                    f"{current_name}  seed {current_seed}")
                row += 1
            run_time = (f"  ·  run {evaluation.format_duration(run_elapsed)}"
                        if phase == "running" else "")
            put(stdscr, row, x,
                f"Elapsed {evaluation.format_duration(elapsed)}{run_time}",
                curses.A_DIM)
            row += 1

            data = runner.data_snapshot()
            metrics = data["metrics"]
            map_metrics = data["map"]
            ate = _eval_float(metrics, "ate")
            abs_error = _eval_float(metrics, "abs_error")
            coverage = _eval_float(map_metrics, "coverage")
            put(stdscr, row, x,
                f"Collected  {data['metrics_rows']} pose rows  ·  "
                f"{data['map_rows']} map rows",
                curses.color_pair(C_INFO))
            row += 1
            put(stdscr, row, x,
                f"Latest     ATE {_eval_metric(ate, ' m')}  ·  "
                f"error {_eval_metric(abs_error, ' m')}  ·  "
                f"coverage {_eval_metric(coverage, percent=True)}")
            row += 1
            failed = int(progress.get("failed", 0))
            put(stdscr, row, x,
                f"Validated  {max(0, completed - failed)} ok  ·  {failed} flagged",
                curses.color_pair(C_WARN if failed else C_OK))
            row += 1
            if runner.batch_dir:
                put(stdscr, row, x,
                    "Output: " + os.path.relpath(runner.batch_dir, REPO_ROOT),
                    curses.A_DIM)
                row += 2
            log_room = h - 3 - row
            if log_room > 1 and runner.output:
                put(stdscr, row, x, "Recent", curses.A_BOLD)
                for i, line in enumerate(list(runner.output)[-(log_room - 1):]):
                    put(stdscr, row + 1 + i, x, line, curses.A_DIM)

        if note:
            put(stdscr, h - 2, 2, note, curses.color_pair(note_kind) | curses.A_BOLD)
        if runner.running:
            keys = "Esc menu (keeps running)   s stop evaluation"
        else:
            keys = "Up/Down choose   Enter run evaluation   r reload   Esc back"
        put(stdscr, h - 1, 2, keys, curses.A_DIM)
        stdscr.refresh()

        key, buffered = read_key(stdscr, 250 if runner.running else -1)
        if key == -1 or key == curses.KEY_RESIZE:
            continue
        note = ""
        if key in (curses.KEY_UP, curses.KEY_DOWN) and matrices:
            if runner.running:
                note, note_kind = "Stop the active evaluation before choosing another.", C_WARN
                continue
            selected = nav_step(
                stdscr, nav, key, buffered, selected,
                -1 if key == curses.KEY_UP else 1, len(matrices))
        elif key in (ord("r"), ord("R")) and not runner.running:
            selected_path = info.path if info else None
            matrices = evaluation.discover_matrices(EVAL_CONFIG_DIR)
            selected = next(
                (i for i, item in enumerate(matrices)
                 if item.path == selected_path), 0)
            note, note_kind = "Evaluation matrices reloaded.", C_OK
        elif key in (ord("s"), ord("S")) and runner.running:
            if confirm_evaluation_stop(stdscr):
                runner.request_stop()
                note, note_kind = "Stopping safely; finalising partial data…", C_WARN
        elif key in (ord("\n"), curses.KEY_ENTER, 10, 13):
            if runner.running:
                note, note_kind = "An evaluation is already running.", C_WARN
            elif info is None:
                note, note_kind = "No evaluation matrix is available.", C_ERR
            elif info.error:
                note, note_kind = f"Invalid matrix: {info.error}", C_ERR
            elif not ros_ready():
                note, note_kind = "ROS 2 is not available in this environment.", C_ERR
            elif not built:
                note, note_kind = "Build the workspace before evaluating.", C_WARN
            else:
                try:
                    runner.start(info, env=ros_env(ws_root))
                    note, note_kind = f"Started {info.name}.", C_OK
                except (OSError, RuntimeError, ValueError) as exc:
                    note, note_kind = f"Could not start: {exc}", C_ERR
        elif key in (27, ord("q"), ord("Q")):
            return


def row_id(p, section):
    """Identity of a drawn row, stable across rebuilds of the option list."""
    return p.id if p is not None else ("#", section)


def resolve_cursor(rows, cursor, idx):
    """Where the selection sits after the list was rebuilt under it.

    Changing an option can add or drop whole sections above the selected row,
    which moves it without the operator touching the cursor — switching Mode to
    teleop takes the planner sections away, so an index kept from the previous
    frame lands somewhere else entirely. Following the row's identity keeps the
    cursor on the option that was just changed; a row that has genuinely gone
    falls back to its own section heading, then to the old index.
    """
    ids = [row_id(p, s) for p, s in rows]
    if cursor is not None:
        wanted, section = cursor
        for candidate in (wanted, ("#", section)):
            if candidate in ids:
                return ids.index(candidate), ids
    return max(0, min(idx, len(rows) - 1)), ids


def option_rows(values, collapsed):
    """The drawn list, as one (param, section id) per line.

    Headings are (None, section id) so the cursor can sit on one and fold it.
    Folding a category takes its subsections with it, and a subsection with
    nothing visible in it is never drawn — that is what keeps the executor
    knobs off the screen until an executor that has any is selected.
    """
    populated = model.populated_sections(values)
    rows = []
    for sid in model.SECTION_ORDER:
        if sid not in populated:
            continue
        parent = model.SECTION_PARENT.get(sid)
        if parent is not None and parent in collapsed:
            continue
        rows.append((None, sid))
        if sid in collapsed:
            continue
        rows.extend((p, None) for p in model.section_options(values, sid))
    return rows


def pending_tag(pend, limit=2):
    """'[restarts mapper, planner +2]' from Supervisor.pending_for().

    Names are capped so the tag stays on the option's row; the footer carries
    the full list."""
    verbs = {"restart": "restarts", "start": "starts", "stop": "stops"}
    parts = []
    for action in ("restart", "start", "stop"):
        gids = [gid for gid, act in pend if act == action]
        if not gids:
            continue
        extra = f" +{len(gids) - limit}" if len(gids) > limit else ""
        parts.append(f"{verbs[action]} {', '.join(gids[:limit])}{extra}")
    return "[" + "; ".join(parts) + "]"


def unapplied_ids(applied, values):
    """Visible option ids the running stack has not taken yet.

    A change reaches the stack either by restarting a group or by a live push;
    a mode switch does neither — goto and frontier launch the same nodes and
    differ in what the launcher publishes — so pending_for() finds nothing to
    report for it. Diffing against the configuration apply() last committed to
    catches those, and the option's own row is coloured from it.
    """
    if not applied:
        return set()
    return {p.id for p in model.PARAMS
            if not model.is_live(p, values, applied) and p.visible(values)
            and p.id in applied and applied[p.id] != values.get(p.id)}


def slam_seed(live_pose, launch_pose):
    """Where dead reckoning starts: the live vehicle, or the launch pose once
    there is no vehicle — that is where the next run respawns, and a kept
    readback would offset SLAM by the distance last driven. Outside every
    group's `depends`, so a seed change never reads as a pending restart."""
    live = live_pose or launch_pose
    return {"slam_seed_x": live["robot_x"], "slam_seed_y": live["robot_y"]}


def fmt_value(p, v):
    if p.kind == "bool":
        return "[x]" if v else "[ ]"
    if p.kind == "enum":
        return f"< {v} >"
    if p.kind == "text":
        return v if v else "(default)"
    if p.kind == "float":
        return f"{v:g}"
    return str(v)


def robot_state_text(link, values, running, driving, drive_active):
    """One sentence for what the vehicle is doing right now.

    The clause after the dash goes on its own line (draw_side_panel keeps
    newlines) so the qualifier never wraps mid-phrase.
    """
    if not running:
        return "stack stopped"
    if link.gate_state == "DISABLED":
        return "motion disabled\n— holding position"
    if driving:
        return ("driven from the keyboard" if drive_active
                else "teleop ready\n— holding position")
    if values["mode"] == "teleop":
        return "waiting for a drive key"
    # A stale activity means the executor stopped publishing, not that it is
    # repeating its last state.
    if time.time() - link.activity_at >= ACTIVITY_STALE_S:
        return "planner is starting up\n— holding position"
    doing = ACTIVITY_TEXT.get(link.activity,
                              str(link.activity).lower().replace("_", " "))
    # Checked before the mode: a revisit detour preempts the goal in goto mode
    # too, and reporting the target point while driving away from it is a lie.
    if link.revisit_state == "revisiting":
        where = ("at the revisit site, holding until d-opt drops"
                 if link.activity in HOLDING_ACTIVITIES
                 else "driving to the revisit site")
        why = REVISIT_CAUSE_TEXT.get(link.revisit_cause)
        return f"{where}\n— triggered by {why}" if why else where
    cooling = " (revisit cooldown)" if link.revisit_state == "cooldown" else ""
    if values["mode"] == "goto":
        return f"heading for the target point{cooling}\n— {doing}"
    return f"exploring{cooling}\n— {doing}"


def _metric(metrics, key, spec, unit=""):
    value = metrics.get(key)
    return "-" if value is None else format(value, spec) + unit


def metric_rows(metrics, mapper, values=None):
    """(label, value) pairs for the metrics block, in display order."""
    rpe = ("-" if metrics.get("rpe_trans") is None else
           f"{_metric(metrics, 'rpe_trans', '.3f')}m "
           f"{_metric(metrics, 'rpe_rot', '.1f')}deg")
    # Coverage and accuracy come from map_metrics.py, which scores the belief
    # map against the ground-truth reference map — so accuracy means belief->GT
    # RMSE under TSDF and occupied-cell IoU under octomap.
    accuracy = (("RMSE", _metric(metrics, "map_accuracy", ".3f", " m"))
                if mapper.startswith("tsdf")
                else ("IoU", _metric(metrics, "map_accuracy", ".3f")))
    # The two axes d-opt rolls into one scalar, in the units the allowable
    # sigmas are stated in, so the pair below reads against the pair above it.
    sigma = (f'{_metric(metrics, "sigma_xy", ".3f")}m '
             f'{_metric(metrics, "sigma_yaw", ".3f")}rad')
    rows = [
        ("err",      _metric(metrics, "abs_error", ".3f", " m")),
        ("ATE",      _metric(metrics, "ate", ".3f", " m")),
        ("RPE",      rpe),
        ("kf",       _metric(metrics, "keyframes", "d")),
        ("lc",       _metric(metrics, "loops", "d")),
        ("sigma",    sigma),
        ("d-opt",    _metric(metrics, "dopt", ".4f")),
        ("U_r",      _metric(metrics, "u_ratio", ".2f")),
        # Consistency pair: ANEES wants ~3 against ground truth, NIS ~6 against
        # the scan-matching model. Both far off means the sigmas are wrong.
        ("ANEES/NIS", (f'{_metric(metrics, "anees", ".1f")}'
                       f' / {_metric(metrics, "nis", ".1f")}')),
    ]
    # The live d-opt only means something against the level that triggers a
    # revisit, so the thresholds sit under it whenever revisit is armed.
    if (values and values.get("revisit") and values.get("loop_closure")
            and values.get("mode") in ("frontier", "goto")):
        rows.append(("allow", f"{values['sigma_allow_xy_m']:g}m "
                              f"{values['sigma_allow_yaw_rad']:g}rad"))
        rows.append(("trig/res", f"{values['ratio_trigger']:g}"
                                 f" / {values['ratio_resume']:g}"))
    rows += [
        ("coverage", _metric(metrics, "map_coverage", ".1%")),
        accuracy,
    ]
    return rows


def choice_lines(p, value):
    """What else the selected option can be set to, as (text, is_current) entries.

    Enums list their values; the numeric kinds have no list, so they show the
    bounds and the arrow-key step instead. Bools have none at all — [x]/[ ] on
    the row already says both states. A planned-work value carries (soon) here
    rather than only in the description, so the list itself reads as the
    roadmap.
    """
    if p.kind == "enum":
        return [(f"{c} (soon)" if p.is_soon(c) else c, c == value)
                for c in p.choices]
    if p.kind == "bool":
        return []
    if p.kind == "text":
        return [("free text — i to edit", False)]
    lo = "-inf" if p.lo is None else f"{p.lo:g}"
    hi = "+inf" if p.hi is None else f"{p.hi:g}"
    return [(f"{lo} .. {hi}", False), (f"step {p.step:g}", False)]


def draw_side_panel(stdscr, link, values, running, driving, drive_active,
                    top, height, x, width):
    """Robot state, error metrics and the GT/belief pose table, right of the
    option list."""
    right = x + width
    end = top + height
    row = top

    def note(message, attr=curses.A_DIM):
        nonlocal row
        # Newlines in the message are kept as breaks; each segment wraps on its own.
        lines = [w for seg in message.split("\n")
                 for w in textwrap.wrap(seg, width - 2)]
        for line in lines[:max(0, end - row)]:
            put(stdscr, row, x + 1, line, attr, maxx=right)
            row += 1

    def heading(text):
        nonlocal row
        put(stdscr, row, x, text, curses.A_BOLD | curses.color_pair(C_HEAD),
            maxx=right)
        row += 1

    heading("Robot state")
    note(robot_state_text(link, values, running, driving, drive_active),
         curses.color_pair(C_OK))
    row += 1
    if row >= end:
        return

    heading("Error metrics")
    if not running:
        note("no run in progress")
    elif values["slam"] != "slam":
        note("pose source is ground truth — no error to measure")
    elif not link.metrics:
        note("waiting for the eval node")
    else:
        # The eval layer going down is otherwise invisible: the last numbers
        # would just sit there looking live.
        stale = time.time() - link.metrics_at > METRICS_STALE_S
        attr = curses.A_DIM if stale else curses.color_pair(C_INFO)
        for label, value in metric_rows(link.metrics, values["mapper"], values):
            if row >= end:
                return
            put(stdscr, row, x + 1, f"{label:<9}", curses.A_DIM, maxx=right)
            put(stdscr, row, x + 10, value, attr, maxx=right)
            row += 1
        if stale:
            note("(stale — is eval running?)", curses.color_pair(C_WARN))
    row += 1
    if row >= end:
        return

    heading("Pos" + f"{'GT':>9}{'belief':>8}{'diff':>7}")
    gt, belief = link.robot_pose, link.slam_pose
    for axis, key in (("x", "robot_x"), ("y", "robot_y"), ("z", "robot_z")):
        if row >= end:
            return
        cell = lambda pose: "-" if pose is None else f"{pose[key]:.2f}"
        put(stdscr, row, x + 1, f"{axis:<3}{cell(gt):>9}{cell(belief):>8}",
            curses.color_pair(C_INFO), maxx=right)
        # The per-axis gap is the number being looked for; it stays grey while
        # it is noise and only takes colour once it is worth reacting to.
        diff = (None if gt is None or belief is None
                else belief[key] - gt[key])
        if diff is None:
            put(stdscr, row, x + 21, f"{'-':>7}", curses.A_DIM, maxx=right)
        else:
            magnitude = abs(diff)
            if magnitude >= POSE_DIFF_ERR_M:
                attr = curses.color_pair(C_ERR) | curses.A_BOLD
            elif magnitude >= POSE_DIFF_WARN_M:
                attr = curses.color_pair(C_WARN)
            else:
                attr = curses.A_DIM
            put(stdscr, row, x + 21, f"{diff:>+7.2f}", attr, maxx=right)
        row += 1


def draw_choice_inline(stdscr, p, value, row, col, maxx, unapplied=False):
    """The selected option's values along its own row, the current one in < >.

    A list too long for the row scrolls under the cursor instead of sliding it:
    the current value is held near the middle and the others move past it, so
    the eye keeps one place to look. Near either end the list stops and the
    cursor travels the last stretch itself. `...` marks a cut side.

    `unapplied` draws the selected value purple rather than green: the option
    is set to it, the running stack is not."""
    entries = choice_lines(p, value)
    width = maxx - col - 1
    if not entries or width < CHOICE_MIN_W:
        return
    segments, total = [], 0
    for text, current in entries:
        if total:
            total += 2
        label = f"< {text} >" if current else text
        segments.append((total, label, current))
        total += len(label)
    offset = 0
    if total > width:
        # Numeric and text options mark nothing current, so they scroll from the
        # head; there is no cursor to centre on.
        cursor = next((s for s in segments if s[2]), None)
        if cursor is not None:
            offset = max(0, min(cursor[0] - (width - len(cursor[1])) // 2,
                                total - width))
    for start, label, current in segments:
        clip_lo = max(start, offset)
        clip_hi = min(start + len(label), offset + width)
        if clip_hi <= clip_lo:
            continue
        # A value cut by the edge is left out — half of one reads as a value of
        # its own. The current one is drawn cut rather than dropped, so the
        # cursor is on the row even when it alone is wider than the space.
        if (clip_lo, clip_hi) != (start, start + len(label)) and not current:
            continue
        put(stdscr, row, col + clip_lo - offset,
            label[clip_lo - start:clip_hi - start],
            (curses.color_pair(C_PEND if unapplied else C_OK) | curses.A_BOLD)
            if current else curses.A_DIM, maxx=maxx)
    if offset:
        put(stdscr, row, col, "...", curses.A_DIM, maxx=maxx)
    if offset + width < total:
        put(stdscr, row, col + width - 3, "...", curses.A_DIM, maxx=maxx)


def control_screen(stdscr, sup, values, link, session):
    idx = 0
    scroll = 0
    nav = NavRepeat()
    status = None
    status_kind = C_OK
    # Driving is not a mode any more: in teleop mode the launcher becomes a
    # command source as soon as the stack is up, and in frontier mode the
    # first drive keypress suspends the planner and takes over (Esc hands
    # control back). The suspension exists because the safety gate treats two
    # publishers on /motion/body_command as MULTIPLE_COMMAND_SOURCES and
    # zeroes all motion.
    driving = False
    planner_suspended = False
    point = None              # [x, y, z] goto target, control-odom frame
    point_dist = None         # last controller distance, for the footer
    # Whether goto mode is the one holding frontier_extractor suspended. Only
    # what we suspended may be resumed: revisit_planner suspends it too, and
    # resuming its detour would put two publishers on the goal topic.
    suspended_for_goto = False
    last_point_pub_at = 0.0   # last /frontier_slam/goal republish
    last_drive_at = 0.0
    last_action = "halt"
    # When a drive key was last pressed with the gate disarmed. The footer
    # highlights the arm hint for a moment after, so a dead key press says
    # why it did nothing instead of looking like a broken teleop.
    last_blocked_drive_at = 0.0
    # Click target and acknowledgement for the copy-launch-command button.
    copy_button = None
    copy_note = None
    copy_note_at = 0.0
    # Where the vehicle is placed at start, at r reset, and — with Move robot
    # in real time on — the moment one of these is edited. The flow is one-way,
    # TUI to Stonefish: odometry never writes back here, so the configured
    # spawn pose survives the run that drives away from it. Where the vehicle
    # actually is lives in the pose table on the right.
    robot_pose_fields = ("robot_x", "robot_y", "robot_z",
                         "robot_roll", "robot_pitch", "robot_yaw")
    # Section ids folded away by their < SHOW > / < HIDE > heading row, kept in
    # `values` so save_config() persists the layout to config.yaml.
    collapsed = set(values.get(model.HIDDEN_SECTIONS_KEY,
                               model.DEFAULT_HIDDEN_SECTIONS))
    # Row 0 is a section heading, where Enter folds rather than applies; park
    # the cursor on the first real option instead whenever the list is rebuilt
    # from scratch (first draw, preset).
    snap_to_option = True
    # (row identity, its section) of the selected row, carried across rebuilds
    # so the cursor follows the option rather than its index. See resolve_cursor.
    cursor = None

    def set_status(msg, kind=C_OK):
        nonlocal status, status_kind
        status = msg
        status_kind = kind

    def launch_values():
        """`values` with the dead-reckoning seed a mid-run SLAM restart needs.

        The seed follows the live vehicle rather than the spawn pose, so a
        pose graph restarted mid-run picks up where the vehicle is instead of
        re-anchoring at the start. It sits outside every group's `depends`, so
        driving never reads as a pending restart.
        """
        merged = dict(values)
        merged.update(slam_seed(link.robot_pose,
                                {k: values[k] for k in robot_pose_fields}))
        return merged

    def save_config():
        save_selection(values)

    def apply_live_change(param, old):
        """Push a changed live value, restoring it if the simulator rejects it."""
        if values[param.id] == old:
            return
        if (not model.is_live(param, values, sup.applied_values)
                or not sup.running_ids()):
            save_config()
            return
        mapper_targets = model.mapper_live_targets(param.id, values)
        if mapper_targets:
            # Belief and ground-truth mappers take the same threshold; a push
            # that reached only one would leave the map metrics comparing two
            # different definitions of a wall.
            pushes = [set_live_param(node, pname, values[param.id])
                      for node, pname in mapper_targets]
            ok = all(done for done, _ in pushes)
            msg = "; ".join(text for _, text in pushes)
        elif param.id in model.LIVE:
            node, pname = model.LIVE[param.id]
            ok, msg = set_live_param(node, pname, values[param.id])
        elif param.id in model.LIVE_ROBOT_POSE:
            if "core" not in sup.running_ids():
                save_config()
                return
            xyz = (values["robot_x"], values["robot_y"], values["robot_z"])
            rpy = tuple(math.radians(values[key]) for key in
                        ("robot_roll", "robot_pitch", "robot_yaw"))
            ok, msg = link.respawn("bluerov2", xyz, rpy)
        elif param.id in model.LIVE_SPEED_TURN:
            # Teleop and the goto point read link.speed_factor/turn_factor
            # straight from `values` every tick — nothing to push. Only a
            # running motion executor needs an explicit ros2 param set.
            if (values["mode"] not in ("frontier", "goto")
                    or "planner" not in sup.running_ids()):
                save_config()
                return
            node = model.MOTION_EXECUTOR_NODE[values["motion"]]
            ok, msg = set_live_param(node, param.id, values[param.id])
        elif (sup.applied.get("core", {}).get("scene") != "target"
              or "core" not in sup.running_ids()):
            # A pose has no live target to apply to yet, but is still the
            # desired launch-time value and should be retained.
            save_config()
            return
        else:
            xyz = (values["obj_x"], values["obj_y"], values["obj_z"])
            rpy = (values["obj_roll"], values["obj_pitch"], values["obj_yaw"])
            ok, msg = link.set_static_entity_pose("SonarTarget", xyz, rpy)
        if ok:
            save_config()
            set_status("live: " + msg, C_OK)
        else:
            values[param.id] = old
            set_status("live failed: " + msg, C_WARN)

    def apply_preset_change(param):
        """Keep the preset line and the options it names in step.

        Selecting a rung writes the options it pins; touching one of those
        afterwards recomputes which rung that leaves the configuration on,
        which is custom unless it happens to land on another.
        """
        if param.id == "preset":
            values.update(model.PRESET_OVERRIDES.get(values["preset"], {}))
            if values["preset"] != model.CUSTOM_PRESET:
                set_status(f"Preset: {values['preset']}", C_INFO)
        else:
            values["preset"] = model.matching_preset(values)

    def ensure_driving():
        """Become the command source, suspending autonomy first in frontier
        mode. Returns True while driving."""
        nonlocal driving, planner_suspended
        if driving:
            return True
        running = sup.running_ids()
        if not running:
            set_status("Start the stack before driving", C_WARN)
            return False
        if not link.start():
            set_status(link.error or "teleop unavailable", C_ERR)
            return False
        # Suspending the planner keeps teleop the only command source.
        # frontier_slam.launch.py also owns the safety gate and thruster
        # mixer, so teleop_support has to take over or nothing moves.
        if "planner" in running:
            draw_busy(stdscr, "Suspending autonomy for teleop...")
            was_armed = bool(link.enabled)
            sup.stop("planner")
            sup.start("teleop_support", values)
            planner_suspended = True
            if was_armed:
                draw_busy(stdscr, "Re-arming motion for teleop...")
                link.set_enabled(True, timeout=8.0)
        draw_busy(stdscr, "Opening teleop...")
        link.open_teleop()
        driving = True
        set_status("Driving" + (" — autonomy suspended" if planner_suspended
                                else ""), C_OK)
        return True

    def drop_point():
        """Forget the goto target and hand goal picking back to the planner."""
        nonlocal point, point_dist, suspended_for_goto
        if point is not None:
            link.clear_target_marker()
        if suspended_for_goto:
            link.set_planner_suspended(False)
            suspended_for_goto = False
        point = None
        point_dist = None

    def release_driving(resume=True):
        """Stop being a command source; optionally hand control back to the
        planner after a frontier takeover."""
        nonlocal driving, planner_suspended
        link.close_teleop()
        driving = False
        if planner_suspended:
            if resume:
                draw_busy(stdscr, "Resuming autonomy...")
                was_armed = bool(link.enabled)
                sup.stop("teleop_support")
                sup.start("planner", values)
                if was_armed:
                    draw_busy(stdscr, "Re-arming motion...")
                    link.set_enabled(True, timeout=8.0)
                set_status("Autonomy resumed", C_INFO)
            planner_suspended = False

    while True:
        if not fits(stdscr):
            too_small(stdscr)
            stdscr.timeout(150)
            stdscr.getch()
            continue

        rows = option_rows(values, collapsed)
        if snap_to_option:
            idx = next((r for r, (p, _) in enumerate(rows) if p is not None), 0)
            snap_to_option = cursor = None
            ids = [row_id(p, s) for p, s in rows]
        else:
            idx, ids = resolve_cursor(rows, cursor, idx)
        cur, cur_section = rows[idx]
        # Carried into the next frame, where the list may have been rebuilt by
        # whatever this one changed.
        cursor = (row_id(cur, cur_section),
                  cur.section if cur is not None else cur_section)

        link.speed_factor = values["speed_factor"]
        link.turn_factor = values["turn_factor"]

        stdscr.erase()
        h, w = stdscr.getmaxyx()

        running = sup.running_ids()
        # A layer that goes down on its own is the thing hardest to notice on a
        # screen that only shows current state, so it is recorded as it happens.
        for gid, was, now, codes in sup.poll_transitions():
            session.event(f"{gid}: {was} -> {now} (exit codes: {codes})")
            if now in ("exited", "partial"):
                set_status(f"{gid} {now} — see {os.path.basename(session.path)}",
                           C_ERR)
        if "core" not in running:
            # No simulator, no vehicle: a kept readback would seed the next SLAM
            # launch where the stopped run ended, not where the respawn puts it.
            link.forget_pose()
        if running:
            # Brought up as soon as the stack is, so discovery has connected
            # before the first arm/teleop keypress rather than dropping it.
            link.start()
            link.spin()   # pick up /motion/safety_status
            link.control_source = "slam" if values["slam"] == "slam" else "gt"
            # Teleop mode drives by default: become the command source as soon
            # as its gate/mixer layer exists, with no mode key to press first.
            if (not driving and values["mode"] == "teleop"
                    and "teleop_support" in running):
                ensure_driving()
        elif driving:
            # The whole stack went away under us (stopped or crashed): stop
            # being a command source. Anything still running gets no fresh
            # command and fails closed on its own.
            release_driving(resume=False)

        # goto mode runs every tick (the loop wakes at 5 Hz while the stack is
        # up). Suspend is republished every tick, not throttled: the goal is
        # ours for as long as the mode lasts, and frontier_extractor picks its
        # own the moment it stops hearing otherwise. The goal point itself
        # stays throttled — the topic isn't latched, but doesn't need spamming.
        point_dist = None
        # The mode the planner layer is actually running under, not the one
        # selected in the list — arrowing the Mode row must not move the keys
        # out from under the operator before Enter applies it. Read from the
        # applied configuration, not the planner group's own snapshot: goto
        # and frontier launch the planner identically, so switching between
        # them deliberately does not restart it.
        goto_mode = (running and "planner" in running
                     and sup.applied_values.get("mode") == "goto")
        if not goto_mode and (point is not None or suspended_for_goto):
            drop_point()
        if goto_mode:
            link.set_planner_suspended(True)
            suspended_for_goto = True
            pose = link.control_pose()
            if point is None and pose:
                # Start from the vehicle position, so entering the mode never
                # commands a jump; the point then travels with the keys.
                point = [pose["robot_x"], pose["robot_y"], pose["robot_z"]]
            if point is not None:
                if pose:
                    point_dist = math.dist(
                        point, (pose["robot_x"], pose["robot_y"], pose["robot_z"]))
                link.publish_target_marker(point)
                # revisit_planner owns /frontier_slam/goal while it drives an
                # uncertainty detour, so the point yields for the duration —
                # the same rule trajectory_mission.py follows. Clearing the
                # timestamp resends the point the moment the detour ends.
                if link.revisit_state not in (None, "exploring"):
                    last_point_pub_at = 0.0
                elif time.time() - last_point_pub_at >= core.POINT_GOAL_REPUBLISH_S:
                    link.publish_point_goal(point)
                    last_point_pub_at = time.time()
        title = "ActiveSlam Control Center"
        put(stdscr, 0, 2, title, curses.A_BOLD)
        title_end = 2 + len(title)
        # A layer that goes down on its own is named next to the title: nothing
        # else on this screen reports one group being gone while the rest runs.
        down = [gid for gid in core.GROUP_ORDER
                if sup.status(gid) in ("exited", "partial")]
        if down:
            put(stdscr, 0, title_end + 2, "! " + ", ".join(down) + " down",
                curses.color_pair(C_ERR) | curses.A_BOLD)
        state = "RUNNING" if running else "STOPPED"
        # Gate state is what decides whether the vehicle can move at all, so it
        # sits next to the run state rather than buried in a pane.
        gate = link.gate_state if running else None
        if gate:
            gtxt = f"motion: {gate}"
            gattr = curses.color_pair(C_OK if gate == "ACTIVE" else C_WARN) | curses.A_BOLD
            put(stdscr, 0, w - len(state) - len(gtxt) - 6, gtxt, gattr)
        put(stdscr, 0, w - len(state) - 3, state,
            curses.color_pair(C_OK if running else C_INFO) | curses.A_BOLD)
        # The single-shot equivalent of what is configured here, listing only
        # what differs from the defaults. Built from the launch pose, not the
        # odometry readback, so it stays a command worth copying. The command
        # itself is far too long for one row, so only a button is shown (on the
        # key line at the bottom) and the text goes to the clipboard.
        pending_values = launch_values()
        launch_cmd = model.launch_command(pending_values)
        unapplied = (unapplied_ids(sup.applied_values, pending_values)
                     if running else set())

        # -- parameter list
        desc_h = 5
        list_top = 2
        list_h = max(4, h - list_top - desc_h - 4)
        panel = w >= PANEL_MIN_TERM_WIDTH
        panel_x = w - PANEL_W if panel else w
        list_right = panel_x - 2 if panel else w

        # Section headings occupy rows too, so scrolling counts them: over the
        # item index alone the selection can fall past the bottom of the pane
        # and get edited unseen behind the description.
        sel_row = idx
        if sel_row < scroll:
            # Keep a heading with the first option under it when scrolling up.
            scroll = (sel_row - 1 if sel_row > 0 and rows[sel_row - 1][0] is None
                      else sel_row)
        if sel_row >= scroll + list_h:
            scroll = sel_row - list_h + 1
        scroll = max(0, min(scroll, max(0, len(rows) - list_h)))

        for offset, (p, section) in enumerate(rows[scroll:scroll + list_h]):
            row = list_top + offset
            sel = (scroll + offset == idx)
            # Every row is indented by its depth in the section tree, so a
            # subsection's options read as belonging to it rather than to the
            # category above.
            depth = model.SECTION_DEPTH[section if p is None else p.section]
            if p is None:
                title = model.SECTION_TITLES[section]
                toggle = "< SHOW >" if section in collapsed else "< HIDE >"
                head_x = 2 + 2 * depth
                put(stdscr, row, head_x, title,
                    curses.A_BOLD | curses.color_pair(C_HEAD)
                    | (curses.A_REVERSE if sel else 0), maxx=list_right)
                # Left plain: the reversed title already marks the cursor.
                put(stdscr, row, head_x + 2 + len(title), toggle, curses.A_DIM,
                    maxx=list_right)
                continue
            # One step in from its own heading, which the two-character cursor
            # marker supplies — so an option and a subsection heading at the
            # same level of the tree start in the same column.
            opt_x = 2 + 2 * depth
            marker = "> " if sel else "  "
            label = f"{marker}{p.label}:"
            # What applying this option costs sits on the option's own row. A
            # live push doesn't update the group's launch snapshot, so an option
            # can be both live and still due a restart — say both. Measured
            # before anything is drawn: where the value list starts decides
            # whether it fits, and that decides what the row itself says.
            tags = []
            # Not gated on `running`: a planned-work value blocks apply, so it
            # has to be findable on a stopped stack too.
            if p.is_soon(values[p.id]):
                tags.append(("(soon)", curses.color_pair(C_WARN) | curses.A_BOLD))
            # Runs, unlike (soon) — the tag marks results as provisional.
            if p.is_wip(values[p.id]):
                tags.append(("(WIP)", curses.color_pair(C_WARN) | curses.A_BOLD))
            if running:
                if model.is_live(p, values, sup.applied_values):
                    tags.append(("(live)", curses.color_pair(C_OK)))
                pend = sup.pending_for(p.id, pending_values)
                if pend:
                    tags.append((pending_tag(pend),
                                 curses.color_pair(C_WARN) | curses.A_BOLD))
            choice_col = opt_x + 2 + len(label) + sum(len(t) + 1 for t, _ in tags)
            # The value list carries the current value in < >, so the row does
            # not repeat it — unless the pane is too narrow to draw the list.
            inline = (sel and p.kind == "enum"
                      and list_right - choice_col - 1 >= CHOICE_MIN_W)
            due = p.id in unapplied
            text = label if inline else f"{label} {fmt_value(p, values[p.id])}"
            base = curses.A_REVERSE if sel else curses.A_NORMAL
            if due and not inline:
                # Only the value goes purple: the label names the option either
                # way, and colouring the whole row would read as an error.
                put(stdscr, row, opt_x, label, base, maxx=list_right)
                put(stdscr, row, opt_x + 1 + len(label),
                    fmt_value(p, values[p.id]),
                    base | curses.color_pair(C_PEND) | curses.A_BOLD,
                    maxx=list_right)
            else:
                put(stdscr, row, opt_x, text, base, maxx=list_right)
            tag_col = opt_x + 1 + len(text)
            for tag, attr in tags:
                put(stdscr, row, tag_col, tag, attr, maxx=list_right)
                tag_col += len(tag) + 1
            # The values this option can take, after the tags on its own row —
            # only for the selection, so there is one such list on screen.
            if sel:
                draw_choice_inline(stdscr, p, values[p.id], row, tag_col + 1,
                                   list_right, unapplied=due)

        if panel:
            # The panel runs the full column height, past the description pane:
            # the metrics and the pose table need more rows than the option
            # list alone leaves.
            panel_h = h - 2 - list_top
            try:
                stdscr.vline(list_top, panel_x - 2, curses.ACS_VLINE, panel_h)
            except curses.error:
                pass
            draw_side_panel(stdscr, link, values, running, driving,
                            driving and time.time() - last_drive_at < 0.6,
                            list_top, panel_h, panel_x, PANEL_W - 1)

        # -- description pane for the selected option
        # The value-specific line is rendered first and on its own, so cycling a
        # value changes only that line; the shared text below it stays put
        # instead of reflowing.
        dtop = list_top + list_h
        put(stdscr, dtop, 2, "-" * max(0, list_right - 4), curses.A_DIM,
            maxx=list_right)
        # A list scrolled past its pane otherwise ends there as far as the eye goes.
        if scroll:
            put(stdscr, list_top - 1, 4, "...", curses.A_DIM, maxx=list_right)
        if scroll + list_h < len(rows):
            put(stdscr, dtop, 4, "...", curses.A_DIM, maxx=list_right)
        width = max(20, list_right - 6)
        drow = dtop + 1
        if cur is None:
            # What the section holds, not how to work it: the row already
            # carries < SHOW >/< HIDE > and the key line explains Left/Right.
            text = model.section_pane_text(cur_section)
            for line in textwrap.wrap(text, width)[:desc_h - 1]:
                put(stdscr, drow, 3, line, curses.color_pair(C_INFO),
                    maxx=list_right)
                drow += 1
        elif cur.kind == "enum" and values[cur.id] in cur.choice_help:
            head = f"{values[cur.id]}: {cur.choice_help[values[cur.id]]}"
            # Planned work reads in the warning colour the (soon) tag and the
            # refused apply use, so one value never looks runnable in green.
            soon = cur.is_soon(values[cur.id])
            attr = curses.color_pair(C_WARN if soon else C_OK) | curses.A_BOLD
            for line in textwrap.wrap(head, width)[:3 if soon else 2]:
                put(stdscr, drow, 3, line, attr, maxx=list_right)
                drow += 1
        remaining = (dtop + desc_h) - drow
        if cur is not None and remaining > 0:
            for line in textwrap.wrap(cur.description, width)[:remaining]:
                put(stdscr, drow, 3, line, curses.color_pair(C_INFO),
                    maxx=list_right)
                drow += 1

        # -- pending changes / status
        _, to_start, to_restart = sup.plan(pending_values)
        pending = sorted(set(to_start + to_restart))
        planned = model.unimplemented(values)
        foot = h - 2
        # The gate zeroes every command while disarmed, so the drive keys are
        # dead until it is armed — say that instead of listing them.
        disarmed = bool(running) and link.gate_state == "DISABLED"
        blocked = time.time() - last_blocked_drive_at < BLOCKED_DRIVE_FLASH_S
        arm_hint = "press M to enable motion"
        # A blocked key gets the line reversed for a moment — the same text,
        # impossible to read past.
        flash = curses.A_REVERSE if blocked else 0
        if goto_mode and point is not None:
            dist_txt = (f"  dist {point_dist:.1f} m" if point_dist is not None
                        else "")
            # The distance still refers to the point while a revisit detour
            # drives the other way, so say the point is on hold.
            if link.revisit_state not in (None, "exploring"):
                dist_txt += "  [revisit: point on hold]"
            # The point still moves while disarmed; the vehicle just won't
            # swim to it, which is what the middle of the line says.
            gate_txt = (f"  —  MOTION DISABLED, {arm_hint}" if disarmed else "")
            put(stdscr, foot, 2,
                f"GOTO ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})"
                f"{dist_txt}{gate_txt}  —  "
                + model.POINT_KEYS.get(values["keyboard"],
                                       model.POINT_KEYS["qwerty"]),
                curses.color_pair(C_WARN if disarmed else C_OK)
                | curses.A_BOLD | (flash if disarmed else 0))
        elif disarmed and (driving or blocked):
            put(stdscr, foot, 2, f"MOTION DISABLED — {arm_hint}",
                curses.color_pair(C_WARN) | curses.A_BOLD | flash)
        elif driving and running:
            take = " (autonomy suspended)  " if planner_suspended else "  "
            put(stdscr, foot, 2,
                "DRIVE" + take
                + model.TELEOP_KEYS.get(values["keyboard"],
                                        model.TELEOP_KEYS["qwerty"]),
                curses.color_pair(C_OK) | curses.A_BOLD)
        elif status:
            put(stdscr, foot, 2, status[:w - 4], curses.color_pair(status_kind) | curses.A_BOLD)
        elif planned:
            # Said before the pending list: Enter is refused while one of these
            # is selected, so promising a restart would be wrong.
            put(stdscr, foot, 2,
                ", ".join(p.label for p, _ in planned)
                + (" is" if len(planned) == 1 else " are")
                + " planned work — Enter is blocked until set back",
                curses.color_pair(C_WARN))
        elif running and (pending or unapplied):
            # An option can be due without anything to restart — a mode switch
            # only changes what this launcher publishes — so the purple rows
            # need a footer that does not promise a restart.
            put(stdscr, foot, 2, "Pending — Enter applies "
                + (f"(restarts: {', '.join(pending)})" if pending
                   else "(nothing restarts)"), curses.color_pair(C_WARN))
        elif not running:
            put(stdscr, foot, 2, "Enter starts the stack", curses.A_DIM)

        keys = ("Up/Down move  Left/Right change  " +
                ("i edit  " if cur is not None
                 and cur.kind in ("int", "float", "text") else "") +
                "Enter apply  m arm  r reset  p preset  "
                "k stop all  Esc quit")
        put(stdscr, h - 1, 2, keys, curses.A_DIM)
        # Last item on the key line, after Esc: it reads as one more key rather
        # than a separate control.
        copy_label = "C copy launch command"
        copy_x = 4 + len(keys)
        copy_button = (h - 1, copy_x, copy_x + len(copy_label))  # for the mouse
        put(stdscr, h - 1, copy_x, copy_label, curses.A_DIM)
        if copy_note and time.time() - copy_note_at < COPY_NOTE_S:
            put(stdscr, h - 1, copy_x + len(copy_label) + 2, copy_note,
                curses.color_pair(C_OK))
        stdscr.refresh()

        # -- input
        key, buffered = read_key(stdscr, 200 if running else -1)

        if key == -1:
            if driving:
                # Republish the last command so the safety gate keeps seeing a
                # fresh command; it fails closed on a stale one. (point mode
                # never sets driving — see the loop-top block above, which
                # keeps the planner's goal fresh instead.)
                if time.time() - last_drive_at < 0.6:
                    link.send_action(last_action)
                else:
                    link.halt()
            continue
        status = None
        if key == curses.KEY_RESIZE:
            continue

        # The drive cluster is always live on this screen; the mode decides
        # what it moves. In frontier mode the first press takes control,
        # suspending the planner (Esc hands it back).
        ch = chr(key) if 0 <= key < 256 else ""
        if goto_mode:
            # goto never takes over the command topic — the planner drives to
            # the point, so a key just edits the point and republishes it.
            paction = core.point_axis_action(ch, values["keyboard"])
            if paction is not None:
                if point is None:
                    # The point is placed on the vehicle as soon as odometry
                    # arrives; until then there is nothing to move.
                    set_status("No odometry yet — no target point to move",
                               C_WARN)
                    continue
                if disarmed:
                    # The point moves either way; flag that nothing will
                    # follow it until the gate is armed.
                    last_blocked_drive_at = time.time()
                if paction == "halt":
                    # F recalls the point to the vehicle's current position.
                    pose = link.control_pose()
                    if pose:
                        point = [pose["robot_x"], pose["robot_y"],
                                 pose["robot_z"]]
                else:
                    point = core.move_point(
                        point, paction, core.POINT_STEP_M * link.speed_factor)
                link.publish_target_marker(point)
                link.publish_point_goal(point)
                last_point_pub_at = time.time()
                continue
        else:
            action = core.drive_action(ch, values["keyboard"])
            if action is not None:
                if disarmed:
                    # Don't take control (nor suspend autonomy) for a command
                    # the gate will zero anyway — flash the arm hint instead.
                    last_blocked_drive_at = time.time()
                    continue
                if ensure_driving():
                    link.send_action(action)
                    last_action = action
                    last_drive_at = time.time()
                continue

        if key in (curses.KEY_UP, curses.KEY_DOWN):
            idx = nav_step(stdscr, nav, key, buffered, idx,
                           -1 if key == curses.KEY_UP else 1, len(rows))
            # A deliberate move wins over following the previous row.
            moved, moved_section = rows[idx]
            cursor = (ids[idx], moved.section if moved is not None
                      else moved_section)
        elif cur is None and key in NEXT_KEYS + PREV_KEYS:
            # Left/Right/Space only: Enter stays the apply/launch key.
            if cur_section in collapsed:
                collapsed.discard(cur_section)
            else:
                collapsed.add(cur_section)
            values[model.HIDDEN_SECTIONS_KEY] = sorted(collapsed)
            save_config()
        elif key in NEXT_KEYS + PREV_KEYS:
            delta = -1 if key in PREV_KEYS else 1
            old = values[cur.id]
            if cur.kind == "enum":
                values[cur.id] = model.cycle_enum(cur, values[cur.id], delta)
            elif cur.kind == "bool":
                values[cur.id] = not values[cur.id]
            elif cur.kind == "int":
                values[cur.id] = int(cur.clamp(values[cur.id] + delta * cur.step))
            elif cur.kind == "float":
                values[cur.id] = round(cur.clamp(values[cur.id] + delta * cur.step), 4)
            if values[cur.id] != old:
                apply_preset_change(cur)
                apply_live_change(cur, old)
        elif (key in (ord("i"), ord("I")) and cur is not None
              and cur.kind in ("int", "float", "text")):
            raw = prompt(stdscr, f"{cur.label} =")
            if raw:
                try:
                    old = values[cur.id]
                    if cur.kind == "int":
                        values[cur.id] = int(cur.clamp(int(raw)))
                    elif cur.kind == "float":
                        values[cur.id] = cur.clamp(float(raw))
                    else:
                        values[cur.id] = raw
                    if values[cur.id] != old:
                        apply_preset_change(cur)
                        apply_live_change(cur, old)
                except ValueError:
                    set_status(f"not a valid {cur.kind}: {raw}", C_ERR)
        elif key in (ord("c"), ord("C")) or (
                key == curses.KEY_MOUSE and _clicked(copy_button)):
            via = copy_to_clipboard(launch_cmd)
            if via:
                copy_note = f"copied ({via})"
                copy_note_at = time.time()
                set_status("launch command copied to the clipboard", C_OK)
            else:
                set_status("no clipboard tool found (wl-copy/xclip/xsel)", C_ERR)
        elif key == curses.KEY_MOUSE:
            # Any other click is ignored rather than falling through to a
            # keyboard branch that would read the mouse code as a character.
            pass
        elif key in (ord("p"), ord("P")):
            # The same step the Preset row takes, reachable without walking
            # back up to it.
            param = model.PARAM_MAP["preset"]
            was = values["preset"]
            values["preset"] = model.cycle_enum(param, was, 1)
            if values["preset"] != was:
                apply_preset_change(param)
                apply_live_change(param, was)
        elif key in (ord("r"), ord("R")):
            if not sup.running_ids():
                set_status("Start the stack before resetting", C_WARN)
            elif confirm_reset(stdscr):
                # A suspended planner is invisible to the reset's stateful
                # group list and its takeover gate would fight the planner's
                # own — hand control back first so the reset then stops and
                # restarts the planner like normal. In teleop mode
                # teleop_support is not stateful and driving survives.
                if planner_suspended:
                    release_driving(resume=True)
                else:
                    drop_point()   # the target would drag the teleported
                    link.halt()    # vehicle; drop it and hold position
                was_armed = bool(link.enabled)
                name = "bluerov2"
                xyz = tuple(values[key] for key in
                            ("robot_x", "robot_y", "robot_z"))
                rpy = tuple(math.radians(values[key]) for key in
                            ("robot_roll", "robot_pitch", "robot_yaw"))
                # Stop the state-holding groups first so no mapper integrates
                # scans while the vehicle is being teleported.
                draw_busy(stdscr, "Reset: stopping mapper/SLAM/planner...")
                stateful = [g for g in core.STATEFUL_GROUPS
                            if g in sup.running_ids()]
                sup.stop_many(stateful)
                draw_busy(stdscr, f"Reset: respawning {name} at "
                                  f"({xyz[0]:g}, {xyz[1]:g}, {xyz[2]:g})...")
                ok, msg = link.respawn(name, xyz, rpy)
                # Nothing has spun since before the teleport, so the readback
                # still describes the pre-reset pose — it must not seed SLAM.
                link.forget_pose()
                time.sleep(1.0)   # let the physics settle before mapping resumes
                draw_busy(stdscr, "Reset: restarting mapper/SLAM/planner...")
                # Snapshot the launch pose, not the odometry readback: the
                # settled pose is never exactly the commanded one, and a
                # snapshot that misses it reads as a pending restart forever.
                restart_values = launch_values()
                for gid in [g for g in core.GROUP_ORDER if g in stateful]:
                    sup.start(gid, restart_values)
                if was_armed:
                    draw_busy(stdscr, "Reset: re-arming motion...")
                    link.set_enabled(True, timeout=8.0)
                set_status(("Reset complete — " if ok else "Reset partial — ") + msg,
                           C_OK if ok else C_WARN)
        elif key in (ord("m"), ord("M")):
            if not sup.running_ids():
                set_status("Start the stack before arming motion", C_WARN)
            elif link.set_enabled(not (link.enabled or False)):
                set_status("Motion gate " + ("ARMED — the vehicle can move"
                                             if link.enabled else "disarmed"),
                           C_OK if link.enabled else C_INFO)
            else:
                set_status(link.error or "could not reach the motion gate", C_ERR)
        elif key in (ord("k"), ord("K")):
            owned = bool(sup.running_ids())
            draw_busy(stdscr, "Stopping every instance...")
            if owned:
                release_driving(resume=False)
                sup.shutdown_all(on_event=session.event)
            # A stack orphaned by an earlier launcher holds the same topics, so
            # K has not stopped anything until those are gone too. Our own group
            # is spared: the launcher would otherwise stop itself.
            stopped, survived = core.stop_stray_processes(
                sup.ws_root, exclude_pgids=(os.getpgid(0),),
                on_event=session.event)
            stray_txt = (f"{stopped} stray process{'es' if stopped != 1 else ''} "
                         "from an earlier session")
            if survived:
                set_status(f"{survived} process(es) would not stop — see the "
                           "session log", C_ERR)
            elif owned and stopped:
                set_status(f"Stack stopped — and {stray_txt}", C_WARN)
            elif owned:
                set_status("Stack stopped", C_INFO)
            elif stopped:
                set_status(stray_txt.capitalize() + " stopped", C_WARN)
            else:
                set_status("Nothing running", C_INFO)
        elif key in (ord("\n"), curses.KEY_ENTER, 10, 13):
            # Nothing downstream implements these, so applying would start a
            # stack doing something other than what the screen says. Refused as
            # a whole rather than per option: a partial apply would leave the
            # unimplemented one sitting there looking applied.
            planned = model.unimplemented(values)
            if planned:
                names = ", ".join(f"{p.label} = {v}" for p, v in planned)
                set_status(f"Not implemented yet: {names} — see Coming next on "
                           f"the info screen", C_WARN)
                continue
            # `core` is running whenever the stack is up, so its snapshot holds
            # the pose source the running stack was actually started with.
            last_slam = sup.applied.get("core", {}).get("slam")
            if sup.running_ids() and last_slam is not None and last_slam != values["slam"]:
                if not confirm_slam_switch(stdscr):
                    continue
            if driving and values["mode"] in ("frontier", "goto"):
                # The planner is a wanted group in both those modes: apply
                # would start it alongside our publisher, and the gate would
                # read two command sources. Hand the topic back first; apply
                # (re)starts the planner below.
                release_driving(resume=False)
            if values["mode"] != "goto":
                drop_point()
            # A live odometry readback must not silently become the next core
            # launch pose (for example if changing object scale restarts
            # Stonefish mid-run), nor restart the pose graph it seeds.
            wanted = launch_values()
            save_config()
            was_armed = bool(link.enabled)
            # Committing the switch is what arms live placement, so it is also
            # what puts the vehicle where the pose fields say — the fields are
            # one-way, and until now nothing had acted on them.
            place_robot = (values["robot_pose_live"]
                           and not sup.applied_values.get("robot_pose_live")
                           and "core" in sup.running_ids())
            # Only announce work there is: apply() returns immediately when the
            # plan is empty, and the busy line would flash for one frame.
            if any(sup.plan(wanted)):
                draw_busy(stdscr, "Applying...")
            session.event(f"apply: {wanted}")
            stopped, started, restarted = sup.apply(wanted, on_event=session.event)
            changed = sorted(set(started + restarted))
            # planner and teleop_support each bring their own motion safety
            # gate, and a fresh gate starts disabled — re-arm so applying an
            # unrelated option doesn't silently stop the vehicle.
            regated = [g for g in changed if g in ("planner", "teleop_support")]
            if was_armed and regated:
                draw_busy(stdscr, "Re-arming motion...")
                link.set_enabled(True, timeout=8.0)
            # Re-seeding from ground truth zeroes the drift the run had already
            # accumulated, so ATE either side of this is not one trajectory.
            placed = ""
            if place_robot:
                draw_busy(stdscr, "Placing the vehicle at the configured pose...")
                xyz = tuple(values[k] for k in ("robot_x", "robot_y", "robot_z"))
                rpy = tuple(math.radians(values[k]) for k in
                            ("robot_roll", "robot_pitch", "robot_yaw"))
                ok, msg = link.respawn("bluerov2", xyz, rpy)
                placed = (f" — vehicle placed at ({xyz[0]:g}, {xyz[1]:g}, "
                          f"{xyz[2]:g})" if ok else f" — could not place: {msg}")
            reseeded = "slam" in restarted
            set_status("Applied" + (f" — (re)started: {', '.join(changed)}" if changed
                                    else " — no changes") + placed
                       + (" — motion re-armed" if was_armed and regated else "")
                       + (" — SLAM re-seeded at the live pose, drift reset"
                          if reseeded else ""),
                       C_WARN if reseeded else C_OK)
        elif key == 27:
            # Esc peels off one layer at a time: the frontier takeover first,
            # then the screen itself. (q/Q is a drive key.)
            if planner_suspended:
                release_driving(resume=True)
                continue
            save_config()
            return


def confirm_reset(stdscr):
    lines = [
        "Reset the run?",
        "",
        "  - the vehicle returns to its scenario spawn pose",
        "  - the map is cleared and rebuilt from scratch",
        "  - the SLAM pose graph resets to empty",
        "  - eval logging restarts into a new eval/runs/ dir",
        "  - the planner's frontier blacklist is cleared",
        "",
        "  the simulator itself keeps running",
        "",
        "Enter to reset, Esc to cancel.",
    ]
    h, w = stdscr.getmaxyx()
    top = max(1, h // 2 - len(lines) // 2 - 1)
    stdscr.erase()
    put(stdscr, top - 1, 4, "Confirm reset", curses.A_BOLD | curses.color_pair(C_WARN))
    for i, line in enumerate(lines):
        put(stdscr, top + i + 1, 4, line)
    stdscr.refresh()
    stdscr.timeout(-1)
    while True:
        k = stdscr.getch()
        if k in (ord("\n"), curses.KEY_ENTER, 10, 13):
            return True
        if k in (27, ord("q"), ord("n"), ord("N")):
            return False


def confirm_slam_switch(stdscr):
    lines = [
        "Switching the pose source restarts SLAM.",
        "",
        "  - the pose graph resets to empty",
        "  - ATE/RPE logging restarts into a new eval/runs/ dir",
        "  - the simulator and the current map keep running",
        "",
        "Enter to proceed, Esc to cancel.",
    ]
    h, w = stdscr.getmaxyx()
    top = max(1, h // 2 - len(lines) // 2 - 1)
    stdscr.erase()
    put(stdscr, top - 1, 4, "Confirm", curses.A_BOLD | curses.color_pair(C_WARN))
    for i, line in enumerate(lines):
        put(stdscr, top + i + 1, 4, line)
    stdscr.refresh()
    stdscr.timeout(-1)
    while True:
        k = stdscr.getch()
        if k in (ord("\n"), curses.KEY_ENTER, 10, 13):
            return True
        if k in (27, ord("q"), ord("n"), ord("N")):
            return False


def draw_busy(stdscr, msg):
    h, w = stdscr.getmaxyx()
    put(stdscr, h - 2, 2, " " * (w - 4))
    put(stdscr, h - 2, 2, msg, curses.A_BOLD | curses.color_pair(C_WARN))
    stdscr.refresh()


def prompt(stdscr, label):
    h, w = stdscr.getmaxyx()
    row = h - 2
    put(stdscr, row, 2, " " * (w - 4))
    put(stdscr, row, 2, f"{label} ")
    curses.echo()
    curses.curs_set(1)
    try:
        raw = stdscr.getstr(row, 3 + len(label), 48).decode("utf-8", "replace")
    except (curses.error, UnicodeDecodeError):
        raw = ""
    curses.noecho()
    curses.curs_set(0)
    return raw.strip()


# --------------------------------------------------------------------------
# foreground tasks (curses torn down, output straight to the shell)

def run_foreground(cmd, cwd=None, env=None):
    print(f"--- {' '.join(cmd)} ---", flush=True)
    try:
        rc = subprocess.run(cmd, cwd=cwd, env=env).returncode
    except (OSError, KeyboardInterrupt) as e:
        print(f"failed: {e}")
        rc = 1
    print(f"--- finished with exit code {rc} ---")
    input("Press Enter to return to the menu.")
    return rc


# --------------------------------------------------------------------------

LOCK_FILE = os.path.expanduser("~/.activeslam_launcher.pid")


def other_instance_pid():
    """PID of another running launcher, or None.

    Two launchers each bring up their own copy of every group; the duplicates
    fight over the same topics (the safety gate sees two command sources and
    stops all motion) with nothing on screen to explain it.
    """
    try:
        with open(LOCK_FILE) as f:
            pid = int(f.read().strip())
    except (OSError, ValueError):
        return None
    if pid == os.getpid():
        return None
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            if b"launcher.py" in f.read():
                return pid
    except OSError:
        pass
    return None


def claim_lock():
    try:
        with open(LOCK_FILE, "w") as f:
            f.write(str(os.getpid()))
    except OSError:
        pass


def release_lock():
    try:
        if other_instance_pid() is None:
            os.remove(LOCK_FILE)
    except OSError:
        pass


def main():
    ensure_sourced_environment()
    rmw_note = normalize_rmw_implementation()

    other = other_instance_pid()
    if other is not None:
        print(f"Another launcher is already running (pid {other}).\n"
              "Two launchers would each start their own copy of the stack and "
              "fight over the same topics. Close that one first, or stop it "
              f"with: kill {other}", file=sys.stderr)
        sys.exit(1)
    claim_lock()
    atexit.register(release_lock)

    ws_root = core.find_workspace_root(__file__)
    built = ws_root is not None and os.path.isdir(os.path.join(ws_root, "install", "bringup"))

    model.configure_object_meshes(available_object_meshes())
    values = dict(model.DEFAULTS)
    restore(values)
    # Create/update the repository-owned config even on the first run, and
    # migrate old home-directory selections to it transparently.
    save_selection(values)

    # Refuse to start rather than present controls that do not reach the run.
    # A knob that reads as configured while the node uses its own default is a
    # silent wrong answer, and this stack has produced two: sigma_allow_xy_m
    # ran at 0.045 against a configured 0.3, and wall_z_band_m pinned the
    # vehicle to a 1.4 m depth slice. Both were live-tunable and neither was
    # applied at startup. Set ACTIVESLAM_SKIP_WIRING_CHECK=1 to bypass.
    if not os.environ.get("ACTIVESLAM_SKIP_WIRING_CHECK"):
        report = core.wiring_report(values, model.PARAMS, bringup_share())
        if report:
            print(report, file=sys.stderr)
            sys.exit(1)

    session = core.SessionLog(LOG_DIR)
    session.start()
    session.event(f"launcher started (ws_root={ws_root}, built={built})")
    # Recorded because every symptom of a mismatched middleware looks like a
    # different bug — a gate that never reports, an empty RViz, a stack that is
    # up and mute. Cheaper to read here than to infer.
    session.event("middleware: RMW_IMPLEMENTATION={} ROS_DOMAIN_ID={} "
                  "ROS_LOCALHOST_ONLY={} ROS_DISTRO={}".format(
                      os.environ.get("RMW_IMPLEMENTATION") or "(default)",
                      os.environ.get("ROS_DOMAIN_ID", "0"),
                      os.environ.get("ROS_LOCALHOST_ONLY", "0"),
                      os.environ.get("ROS_DISTRO", "?")))
    if rmw_note:
        session.event(rmw_note)

    sup = core.Supervisor(ws_root or REPO_ROOT, LOG_DIR,
                          core.build_groups(bringup_share()),
                          env=ros_env(ws_root), session=session)
    link = RosLink()
    eval_runner = evaluation.EvaluationRunner(REPO_ROOT)

    def emergency(*_):
        link.close()
        sup.shutdown_all(on_event=session.event)
        eval_runner.close()
        session.close()

    atexit.register(emergency)
    for s in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(s, lambda *a: (emergency(), sys.exit(0)))
        except (ValueError, OSError):
            pass

    while True:
        choice = curses.wrapper(lambda scr: (init_colors(),
                                             welcome_screen(
                                                 scr, built, values,
                                                 eval_runner))[1])

        if choice == "Exit":
            break
        if choice == "Evaluation":
            curses.wrapper(lambda scr: (
                init_colors(),
                evaluation_screen(scr, eval_runner, built, ws_root or REPO_ROOT)
            )[1])
            continue
        if choice == "Infos":
            curses.wrapper(lambda scr: (init_colors(), infos_screen(scr))[1])
            continue
        if eval_runner.running:
            print("An evaluation is running. Stop it from the Evaluation panel "
                  f"before using {choice}; concurrent ROS stacks would corrupt "
                  "the collected data.", file=sys.stderr)
            input("Press Enter to return to the menu.")
            continue
        if choice == "Update":
            run_foreground(["./bootstrap.sh"], cwd=REPO_ROOT)
            built = os.path.isdir(os.path.join(ws_root or "", "install", "bringup"))
            continue
        if choice == "Rebuild":
            run_foreground(["colcon", "build", "--symlink-install",
                            "--cmake-args", "-Wno-dev"],
                           cwd=ws_root or REPO_ROOT,
                           env=core.venv_env(ws_root))
            built = os.path.isdir(os.path.join(ws_root or "", "install", "bringup"))
            continue

        # Launch
        if not ros_ready():
            print(f"ros2 was not found on PATH, and the '{DISTROBOX_NAME}' "
                  "Distrobox could not be entered automatically. Create it with "
                  "./bootstrap.sh, or enter a ROS 2 environment yourself (source "
                  "/opt/ros/<distro>/setup.bash) and run this again.",
                  file=sys.stderr)
            input("Press Enter to return to the menu.")
            continue
        if not built:
            print("The workspace is not built yet — choose Rebuild first.",
                  file=sys.stderr)
            input("Press Enter to return to the menu.")
            continue

        try:
            # fd 2 goes to the session log for the duration: the middleware
            # writes warnings straight to it, and they land mid-screen.
            with core.stderr_to(session.path):
                curses.wrapper(lambda scr: (init_colors(),
                                            control_screen(scr, sup, values,
                                                           link, session))[1])
        finally:
            link.close()
            if sup.running_ids():
                print("Stopping the stack...", flush=True)
                sup.shutdown_all(on_event=lambda m: (session.event(m),
                                                     print(f"  {m}", flush=True)))
                print("Stopped.")
            print(f"Session log: {session.latest_path}")

    emergency()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the stack.")
        sys.exit(0)
