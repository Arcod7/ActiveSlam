#!/usr/bin/env python3
"""
ActiveSlam launcher: a terminal UI for bringing the stack up, changing options
while it runs, and driving the vehicle by keyboard.

Run it from the repo root:

    python3 launcher.py

The stack is split into independently restartable groups (see launcher_core),
so changing the mapper or the pose source only bounces the layers that depend
on it — the simulator, which is the slow part, keeps running.

Teleop is built in and always live: the drive keys (QWEASD cluster, AZERTY
supported) move the vehicle directly, and Y toggles a target point the
vehicle follows on its own. This process already owns a raw terminal, which
is the only thing keyboard control needed a separate window for.
"""

import atexit
import curses
import glob
import json
import math
import os
import signal
import subprocess
import sys
import textwrap
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import launcher_core as core
import launcher_model as model

MIN_TERM_HEIGHT = 24
MIN_TERM_WIDTH = 80
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(REPO_ROOT, "config.yaml")
# Kept only as a one-time migration source for users of the previous launcher.
LEGACY_SELECTION_FILE = os.path.expanduser("~/.activeslam_launcher.json")
LOG_DIR = os.path.join(REPO_ROOT, "logs", "launcher")

C_HEAD, C_ERR, C_INFO, C_OK, C_WARN = 1, 2, 3, 4, 5


# --------------------------------------------------------------------------
# persistence

def _load_mapping(path):
    """Load the launcher config without adding a PyYAML runtime dependency.

    Each generated value is JSON syntax, which is also valid YAML.  Reading
    JSON first accepts a compact hand-written config too; the small fallback
    handles the one-key-per-line YAML file produced by save_selection().
    """
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return None
    try:
        value = json.loads(text)
        return value if isinstance(value, dict) else None
    except json.JSONDecodeError:
        pass

    values = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw = line.split(":", 1)
        key, raw = key.strip(), raw.strip()
        try:
            values[key] = json.loads(raw)
        except json.JSONDecodeError:
            # This makes a simple unquoted string value usable too.
            values[key] = raw
    return values or None


def load_selection():
    return _load_mapping(CONFIG_FILE) or _load_mapping(LEGACY_SELECTION_FILE)


def save_selection(values):
    try:
        # JSON scalar syntax makes this a standards-compliant YAML mapping
        # while keeping the launcher dependency-free.
        lines = [
            "# ActiveSlam launcher configuration.",
            "# Edit while the launcher is stopped; it writes every option here.",
            "config_version: 1",
        ]
        lines.extend(f"{p.id}: {json.dumps(values[p.id])}"
                     for p in model.PARAMS)
        temporary = CONFIG_FILE + ".tmp"
        with open(temporary, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        os.replace(temporary, CONFIG_FILE)
    except OSError:
        pass


def restore(values):
    saved = load_selection()
    if saved:
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
        # revisit_planner's own state ('exploring' when idle, something else
        # mid-revisit), or None if it isn't running/hasn't published yet.
        # Point mode yields its goal to an active revisit rather than
        # fighting it — see control_screen's revisit_active().
        self.revisit_state = None
        # Which odometry the drive/point controller works in: "slam" when the
        # pose graph owns the TF frame the operator sees, "gt" otherwise.
        # Synced from values["slam"] every control_screen tick.
        self.control_source = "gt"
        # Synced from values["speed_factor"/"turn_factor"] every control_screen
        # tick; multiply STEP so runs can be piloted faster without touching
        # /motion/body_command's magnitude elsewhere.
        self.speed_factor = 1.0
        self.turn_factor = 1.0

    def start(self):
        if self.node:
            return True
        try:
            import rclpy
            from geometry_msgs.msg import PointStamped, Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Bool, String
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
            # revisit_planner republishes /frontier_slam/suspend at 1 Hz
            # regardless of whether it wants control — subscribing here is
            # only for the revisit_state yield check, not for suspend itself.
            self.node.create_subscription(
                String, "/frontier_slam/revisit_state", self._revisit_state_cb, 10)
        except Exception as e:
            self.error = f"could not create launcher node: {e}"
            self.node = None
            return False
        return True

    def _status_cb(self, msg):
        self.gate_state = msg.data

    def _revisit_state_cb(self, msg):
        self.revisit_state = msg.data

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

    def control_pose(self):
        """Pose the drive/point controller works in: the SLAM estimate when
        that owns the TF frame the operator sees, ground truth otherwise —
        with a fallback to whichever source has published at all."""
        if self.control_source == "slam":
            return self.slam_pose or self.robot_pose
        return self.robot_pose or self.slam_pose

    def spin(self):
        if self.node:
            try:
                self.rclpy.spin_once(self.node, timeout_sec=0)
            except Exception:
                pass

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
        """Draw the follow-point target in RViz (world_ned sphere)."""
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
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.3, 0.9
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


def put(stdscr, y, x, text, attr=0):
    h, w = stdscr.getmaxyx()
    if y < 0 or y >= h or x >= w:
        return
    try:
        stdscr.addstr(y, x, str(text)[:max(0, w - x - 1)], attr)
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


# --------------------------------------------------------------------------
# screens

def welcome_screen(stdscr, built):
    options = ["Launch", "---", "Update", "Rebuild", "---", "Infos", "Exit"]
    idx = 0
    while True:
        if not fits(stdscr):
            too_small(stdscr)
            stdscr.timeout(120)
            if stdscr.getch() == curses.KEY_RESIZE:
                continue
            continue
        stdscr.timeout(-1)
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
            prefix = "> " if idx == i else "  "
            attr = curses.A_REVERSE if idx == i else curses.A_NORMAL
            if opt == "Launch":
                attr |= curses.A_BOLD
            put(stdscr, row + i, 4, f"{prefix}{opt}", attr)

        hints = {
            "Launch": "Configure options and bring the stack up.",
            "Update": "git pull, then reinstall dependencies (./bootstrap.sh).",
            "Rebuild": "colcon build --symlink-install.",
            "Infos": "Technologies this project is built on.",
            "Exit": "Leave the launcher.",
        }
        put(stdscr, row + len(options) + 1, 4, hints.get(options[idx], ""),
            curses.color_pair(C_INFO))
        put(stdscr, h - 1, 2, "Up/Down navigate   Enter select   q quit", curses.A_DIM)
        stdscr.refresh()

        key = stdscr.getch()
        if key == curses.KEY_RESIZE:
            continue
        if key == curses.KEY_UP:
            idx = (idx - 1) % len(options)
            while options[idx] == "---":
                idx = (idx - 1) % len(options)
        elif key == curses.KEY_DOWN:
            idx = (idx + 1) % len(options)
            while options[idx] == "---":
                idx = (idx + 1) % len(options)
        elif key in (ord("\n"), curses.KEY_ENTER, 10, 13):
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
        view = h - 3
        scroll = max(0, min(scroll, max(0, len(lines) - view)))
        for i, (kind, text) in enumerate(lines[scroll:scroll + view]):
            if kind == "head":
                put(stdscr, 2 + i, 2, text, curses.A_BOLD | curses.color_pair(C_HEAD))
            else:
                put(stdscr, 2 + i, 4, text)
        more = "" if scroll + view >= len(lines) else "   (more below)"
        put(stdscr, h - 1, 2, f"Up/Down scroll   q back{more}", curses.A_DIM)
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


def visible_params(values, show_advanced):
    out = []
    for section in model.SECTION_ORDER:
        for p in model.PARAMS:
            if p.section != section or not p.visible(values):
                continue
            if p.advanced and not show_advanced:
                continue
            out.append(p)
    return out


def current_preset_name(values):
    """Name of the preset the current values match, else '(custom)'. Avoids a
    stale label after a restored selection or a hand-edited option."""
    for name, overrides in model.PRESETS:
        expected = dict(model.DEFAULTS)
        expected.update(overrides)
        if all(values.get(k) == expected[k] for k in expected):
            return name
    return "(custom)"


def fmt_value(p, v):
    if p.kind == "bool":
        return "[x] on" if v else "[ ] off"
    if p.kind == "enum":
        return f"< {v} >"
    if p.kind == "text":
        return v if v else "(default)"
    if p.kind == "float":
        return f"{v:g}"
    return str(v)


def control_screen(stdscr, sup, values, link, session):
    show_advanced = False
    preset_idx = 0
    idx = 0
    scroll = 0
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
    point_mode = False
    point = None              # [x, y, z] follow target, control-odom frame
    point_dist = None         # last controller distance, for the footer
    last_point_pub_at = 0.0   # last /frontier_slam/goal republish
    last_drive_at = 0.0
    last_action = "halt"
    robot_pose_fields = ("robot_x", "robot_y", "robot_z",
                         "robot_roll", "robot_pitch", "robot_yaw")
    # This is the deliberately configured launch pose. Odometry updates the
    # displayed fields continuously, but must not overwrite it unless the
    # operator explicitly edits a robot field or enables save-on-exit.
    saved_robot_pose = {key: values[key] for key in robot_pose_fields}

    def set_status(msg, kind=C_OK):
        nonlocal status, status_kind
        status = msg
        status_kind = kind

    def save_config():
        """Persist settings without accidentally saving teleop motion."""
        persisted = dict(values)
        if not values["robot_save_pose_on_exit"]:
            persisted.update(saved_robot_pose)
        save_selection(persisted)

    def apply_live_change(param, old):
        """Push a changed live value, restoring it if the simulator rejects it."""
        nonlocal saved_robot_pose
        if values[param.id] == old:
            return
        if not param.live or not sup.running_ids():
            if param.id in model.LIVE_ROBOT_POSE:
                saved_robot_pose = {key: values[key] for key in robot_pose_fields}
            save_config()
            return
        if param.id in model.LIVE:
            node, pname = model.LIVE[param.id]
            ok, msg = set_live_param(node, pname, values[param.id])
        elif param.id in model.LIVE_ROBOT_POSE:
            if "core" not in sup.running_ids():
                saved_robot_pose = {key: values[key] for key in robot_pose_fields}
                save_config()
                return
            xyz = (values["robot_x"], values["robot_y"], values["robot_z"])
            rpy = tuple(math.radians(values[key]) for key in
                        ("robot_roll", "robot_pitch", "robot_yaw"))
            ok, msg = link.respawn("bluerov2", xyz, rpy)
        elif param.id in model.LIVE_SPEED_TURN:
            # Teleop reads link.speed_factor/turn_factor straight from `values`
            # every tick — nothing to push. Only a running frontier motion
            # executor needs an explicit ros2 param set.
            if values["mode"] != "frontier" or "planner" not in sup.running_ids():
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
            if param.id in model.LIVE_ROBOT_POSE:
                saved_robot_pose = {key: values[key] for key in robot_pose_fields}
            save_config()
            set_status("live: " + msg, C_OK)
        else:
            values[param.id] = old
            set_status("live failed: " + msg, C_WARN)

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

    def revisit_active():
        """True while revisit_planner owns /frontier_slam/goal for a real,
        uncertainty-triggered revisit — point mode yields its own goal
        publication rather than fighting it (same precedence trajectory_
        mission.py gives revisit_planner). None/'exploring' both mean no
        active revisit (None covers revisit_planner not running at all)."""
        return link.revisit_state not in (None, "exploring")

    def stop_point_mode():
        nonlocal point_mode, point, point_dist
        if point_mode:
            link.clear_target_marker()
            link.set_planner_suspended(False)
        point_mode = False
        point = None
        point_dist = None

    def release_driving(resume=True):
        """Stop being a command source; optionally hand control back to the
        planner after a frontier takeover."""
        nonlocal driving, planner_suspended
        stop_point_mode()
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

        items = visible_params(values, show_advanced)
        if not items:
            items = visible_params(values, True)
        idx = max(0, min(idx, len(items) - 1))
        cur = items[idx]

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
            if link.robot_pose and "core" in running:
                values.update({key: round(value, 4)
                               for key, value in link.robot_pose.items()})
            if point_mode and "planner" not in running:
                # The planner stopped/crashed under us — nothing is left to
                # drive to the point, so drop it rather than leave it stale.
                stop_point_mode()
                set_status("Target point off — planner stopped", C_WARN)
        elif driving:
            # The whole stack went away under us (stopped or crashed): stop
            # being a command source. Anything still running gets no fresh
            # command and fails closed on its own.
            release_driving(resume=False)
        elif point_mode:
            stop_point_mode()

        # Target-point following runs every tick (the loop wakes at 5 Hz while
        # the stack is up). Suspend is republished every tick, not throttled:
        # revisit_planner reasserts it at 1 Hz whenever it's idle (its own
        # normal behaviour, not a bug — see revisit_active()), and a slower
        # republish here used to lose that race, letting frontier_extractor
        # un-suspend and pick its own goal between our sends. The goal point
        # itself stays throttled (the topic isn't latched, but doesn't need
        # spamming), and is withheld entirely while yielding to a real revisit.
        point_dist = None
        if point_mode and point is not None:
            pose = link.control_pose()
            if pose:
                pos = (pose["robot_x"], pose["robot_y"], pose["robot_z"])
                point_dist = math.dist(point, pos)
            link.publish_target_marker(point)
            link.set_planner_suspended(True)
            if (not revisit_active()
                    and time.time() - last_point_pub_at >= core.POINT_GOAL_REPUBLISH_S):
                link.publish_point_goal(point)
                last_point_pub_at = time.time()
        title = "ActiveSlam Control Center"
        put(stdscr, 0, 2, title, curses.A_BOLD)
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

        put(stdscr, 1, 2, f"Preset: {current_preset_name(values)}",
            curses.color_pair(C_INFO))

        # -- group status line
        gstat = []
        for gid in core.GROUP_ORDER:
            g = sup.groups.get(gid)
            if not g or not g.visible(values):
                continue
            st = sup.status(gid)
            mark = {"running": "+", "stopped": "-", "exited": "!", "partial": "~"}[st]
            gstat.append((f"{mark}{gid}", st))
        col = 2
        put(stdscr, 2, 0, " " * (w - 1))
        for label, st in gstat:
            attr = {"running": curses.color_pair(C_OK),
                    "exited": curses.color_pair(C_ERR) | curses.A_BOLD,
                    "partial": curses.color_pair(C_WARN),
                    "stopped": curses.A_DIM}[st]
            if col + len(label) + 1 < w - 1:
                put(stdscr, 2, col, label, attr)
                col += len(label) + 2

        # -- parameter list
        desc_h = 5
        list_top = 4
        list_h = max(4, h - list_top - desc_h - 4)
        if idx < scroll:
            scroll = idx
        if idx >= scroll + list_h:
            scroll = idx - list_h + 1
        scroll = max(0, min(scroll, max(0, len(items) - list_h)))

        last_section = None
        row = list_top
        for i in range(scroll, min(len(items), scroll + list_h)):
            p = items[i]
            if p.section != last_section:
                if row < list_top + list_h:
                    put(stdscr, row, 2, model.SECTION_TITLES[p.section],
                        curses.A_BOLD | curses.color_pair(C_HEAD))
                    row += 1
                last_section = p.section
            if row >= list_top + list_h:
                break
            sel = (i == idx)
            marker = "> " if sel else "  "
            live = " (live)" if p.live and running else ""
            text = f"{marker}{p.label}: {fmt_value(p, values[p.id])}{live}"
            put(stdscr, row, 4, text,
                curses.A_REVERSE if sel else curses.A_NORMAL)
            row += 1

        # -- description pane for the selected option
        # The value-specific line is rendered first and on its own, so cycling a
        # value changes only that line; the shared text below it stays put
        # instead of reflowing.
        dtop = list_top + list_h
        put(stdscr, dtop, 2, "-" * (w - 4), curses.A_DIM)
        width = max(20, w - 6)
        drow = dtop + 1
        if cur.kind == "enum" and values[cur.id] in cur.choice_help:
            head = f"{values[cur.id]}: {cur.choice_help[values[cur.id]]}"
            for line in textwrap.wrap(head, width)[:2]:
                put(stdscr, drow, 3, line, curses.color_pair(C_OK) | curses.A_BOLD)
                drow += 1
        remaining = (dtop + desc_h) - drow
        if remaining > 0:
            for line in textwrap.wrap(cur.description, width)[:remaining]:
                put(stdscr, drow, 3, line, curses.color_pair(C_INFO))
                drow += 1

        # -- pending changes / status
        _, to_start, to_restart = sup.plan(values)
        pending = sorted(set(to_start + to_restart))
        foot = h - 2
        if point_mode and point is not None:
            dist_txt = (f"  dist {point_dist:.1f} m" if point_dist is not None
                        else "")
            yield_txt = "  — YIELDING TO REVISIT" if revisit_active() else ""
            put(stdscr, foot, 2,
                f"TARGET ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})"
                f"{dist_txt}{yield_txt}  —  " + model.POINT_KEYS,
                curses.color_pair(C_WARN if yield_txt else C_OK) | curses.A_BOLD)
        elif driving and running:
            take = " (autonomy suspended)  " if planner_suspended else "  "
            put(stdscr, foot, 2,
                "DRIVE" + take
                + model.TELEOP_KEYS.get(values["keyboard"],
                                        model.TELEOP_KEYS["qwerty"])
                + "  Y target point",
                curses.color_pair(C_OK) | curses.A_BOLD)
        elif status:
            put(stdscr, foot, 2, status[:w - 4], curses.color_pair(status_kind) | curses.A_BOLD)
        elif running and pending:
            put(stdscr, foot, 2, "Pending — Enter applies (restarts: "
                + ", ".join(pending) + ")", curses.color_pair(C_WARN))
        elif not running:
            put(stdscr, foot, 2, "Enter starts the stack", curses.A_DIM)

        keys = ("Up/Down move  Left/Right change  " +
                ("i edit  " if cur.kind in ("int", "float", "text") else "") +
                "Enter apply  m arm  y point  r reset  o advanced  p preset  k stop  Esc quit")
        put(stdscr, h - 1, 2, keys, curses.A_DIM)
        stdscr.refresh()

        # -- input
        stdscr.timeout(200 if running else -1)
        key = stdscr.getch()

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

        # Drive keys are always live on this screen — there is no teleop mode
        # to enter any more. The first press takes control, suspending the
        # planner first if autonomy is running.
        ch = chr(key) if 0 <= key < 256 else ""
        if point_mode and point is not None:
            # Point mode doesn't take over teleop — the planner (already
            # running to own this) drives to the point on its own, so
            # dragging the point just edits local state and republishes.
            paction = core.point_axis_action(ch, values["keyboard"])
            if paction is not None:
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
                if not revisit_active():
                    link.publish_point_goal(point)
                    last_point_pub_at = time.time()
                continue
        else:
            action = core.drive_action(ch, values["keyboard"])
            if action is not None:
                if ensure_driving():
                    link.send_action(action)
                    last_action = action
                    last_drive_at = time.time()
                continue
        if ch and ch.lower() == "y":
            if point_mode:
                stop_point_mode()
                set_status("Target point off — planner resumes its own goals",
                           C_INFO)
            elif driving:
                set_status("Stop driving (Esc) before setting a target point",
                           C_WARN)
            elif "planner" not in running:
                set_status("Target point needs the frontier planner running "
                           "(mode:=frontier)", C_WARN)
            else:
                pose = link.control_pose()
                if pose is None:
                    set_status("No odometry yet — cannot place the target point",
                               C_WARN)
                else:
                    # Start from the vehicle position, so toggling on never
                    # commands a jump; the point then travels with the keys.
                    point = [pose["robot_x"], pose["robot_y"], pose["robot_z"]]
                    point_mode = True
                    link.publish_target_marker(point)
                    link.set_planner_suspended(True)
                    if revisit_active():
                        last_point_pub_at = 0.0   # publish as soon as the yield clears
                        set_status("Target point set — yielding to an active "
                                   "revisit until it clears", C_WARN)
                    else:
                        link.publish_point_goal(point)
                        last_point_pub_at = time.time()
                        set_status("Target point ON — drive keys move it, "
                                   "the planner paths to it", C_OK)
            continue

        if key == curses.KEY_UP:
            idx = (idx - 1) % len(items)
        elif key == curses.KEY_DOWN:
            idx = (idx + 1) % len(items)
        elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
            delta = -1 if key == curses.KEY_LEFT else 1
            old = values[cur.id]
            if cur.kind == "enum":
                i = cur.choices.index(values[cur.id])
                values[cur.id] = cur.choices[(i + delta) % len(cur.choices)]
            elif cur.kind == "bool":
                values[cur.id] = not values[cur.id]
            elif cur.kind == "int":
                values[cur.id] = int(cur.clamp(values[cur.id] + delta * cur.step))
            elif cur.kind == "float":
                values[cur.id] = round(cur.clamp(values[cur.id] + delta * cur.step), 4)
            if values[cur.id] != old:
                apply_live_change(cur, old)
        elif key in (ord("i"), ord("I")) and cur.kind in ("int", "float", "text"):
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
                        apply_live_change(cur, old)
                except ValueError:
                    set_status(f"not a valid {cur.kind}: {raw}", C_ERR)
        elif key in (ord("p"), ord("P")):
            preset_idx = (preset_idx + 1) % len(model.PRESETS)
            values.clear()
            values.update(model.DEFAULTS)
            values.update(model.PRESETS[preset_idx][1])
            if not values["robot_save_pose_on_exit"]:
                values.update(saved_robot_pose)
            save_config()
            idx = 0
            set_status(f"Preset: {model.PRESETS[preset_idx][0]}", C_INFO)
        elif key in (ord("o"), ord("O")):
            show_advanced = not show_advanced
            idx = 0
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
                    stop_point_mode()   # the target would drag the teleported
                    link.halt()         # vehicle; drop it and hold position
                was_armed = bool(link.enabled)
                name = "bluerov2"
                xyz = tuple(saved_robot_pose[key] for key in
                            ("robot_x", "robot_y", "robot_z"))
                rpy = tuple(math.radians(saved_robot_pose[key]) for key in
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
                time.sleep(1.0)   # let the physics settle before mapping resumes
                draw_busy(stdscr, "Reset: restarting mapper/SLAM/planner...")
                for gid in [g for g in core.GROUP_ORDER if g in stateful]:
                    sup.start(gid, values)
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
            if sup.running_ids():
                release_driving(resume=False)
                draw_busy(stdscr, "Stopping the stack...")
                sup.shutdown_all()
                set_status("Stack stopped", C_INFO)
        elif key in (ord("\n"), curses.KEY_ENTER, 10, 13):
            # `core` is running whenever the stack is up, so its snapshot holds
            # the pose source the running stack was actually started with.
            last_slam = sup.applied.get("core", {}).get("slam")
            if sup.running_ids() and last_slam is not None and last_slam != values["slam"]:
                if not confirm_slam_switch(stdscr):
                    continue
            if driving and values["mode"] == "frontier":
                # The planner is a wanted group in frontier mode: apply would
                # start it alongside our publisher, and the gate would read
                # two command sources. Hand the topic back first; apply
                # (re)starts the planner below.
                release_driving(resume=False)
            # A live odometry readback must not silently become the next core
            # launch pose when save-on-exit is off (for example if changing
            # object scale restarts Stonefish mid-run).
            launch_values = dict(values)
            if not values["robot_save_pose_on_exit"]:
                launch_values.update(saved_robot_pose)
            save_config()
            draw_busy(stdscr, "Applying...")
            was_armed = bool(link.enabled)
            session.event(f"apply: {launch_values}")
            stopped, started, restarted = sup.apply(launch_values, on_event=session.event)
            changed = sorted(set(started + restarted))
            # planner and teleop_support each bring their own motion safety
            # gate, and a fresh gate starts disabled — re-arm so applying an
            # unrelated option doesn't silently stop the vehicle.
            regated = [g for g in changed if g in ("planner", "teleop_support")]
            if was_armed and regated:
                draw_busy(stdscr, "Re-arming motion...")
                link.set_enabled(True, timeout=8.0)
            set_status("Applied" + (f" — (re)started: {', '.join(changed)}" if changed
                                    else " — no changes")
                       + (" — motion re-armed" if was_armed and regated else ""), C_OK)
        elif key == 27:
            # Esc peels off one layer at a time: target point first, then the
            # frontier takeover, then the screen itself. (q/Q is a drive key.)
            if point_mode:
                stop_point_mode()
                set_status("Target point off — planner resumes its own goals",
                           C_INFO)
                continue
            if planner_suspended:
                release_driving(resume=True)
                continue
            if values["robot_save_pose_on_exit"]:
                saved_robot_pose = {key: values[key] for key in robot_pose_fields}
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

    def emergency(*_):
        link.close()
        sup.shutdown_all(on_event=session.event)
        session.close()

    atexit.register(emergency)
    for s in (signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(s, lambda *a: (emergency(), sys.exit(0)))
        except (ValueError, OSError):
            pass

    while True:
        choice = curses.wrapper(lambda scr: (init_colors(), welcome_screen(scr, built))[1])

        if choice == "Exit":
            break
        if choice == "Infos":
            curses.wrapper(lambda scr: (init_colors(), infos_screen(scr))[1])
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
