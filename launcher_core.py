#!/usr/bin/env python3
"""
Launch model and process supervisor for launcher.py.

Kept separate from the curses UI so the group/parameter model and the teardown
logic can be unit-tested headlessly. Pure standard library — importable before
the workspace is built.

The stack is split into independently restartable groups so changing an option
only bounces the layers that actually depend on it; `core` (the simulator) is
expensive to start and is never restarted by a parameter change.
"""

import contextlib
import datetime
import math
import os
import shutil
import signal
import subprocess
import sys
import sysconfig
import tempfile
import threading
import time

# Order matters: groups are started top-down and stopped bottom-up.
GROUP_ORDER = ["core", "tf", "cloud", "mapper", "gt_map", "slam", "eval",
               "planner", "teleop_support", "rviz", "rqt"]

# Groups that accumulate state across a run (maps, pose graph, eval output,
# planner blacklists). A reset restarts exactly these; the simulator, TF, point
# cloud and RViz are stateless in this sense and stay up.
STATEFUL_GROUPS = ["mapper", "gt_map", "slam", "eval", "planner"]

DEFAULT_SPAWN = ("bluerov2", (0.0, 0.0, 8.0), (0.0, 0.0, 0.0))

STOP_SIGINT_TIMEOUT = 12.0   # ros2 launch handles SIGINT gracefully
STOP_SIGTERM_TIMEOUT = 5.0
STOP_SIGKILL_TIMEOUT = 3.0


def octomap_preload_path():
    """LD_PRELOAD for RViz's octomap symbol clash, with the arch triplet derived
    rather than hardcoded (demo.launch.py hardcodes aarch64-linux-gnu)."""
    triplet = sysconfig.get_config_var("MULTIARCH") or ""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix("octomap")
    except Exception:
        return None
    for candidate in ([os.path.join(prefix, "lib", triplet, "liboctomap.so")] if triplet else []) + [
        os.path.join(prefix, "lib", "liboctomap.so"),
    ]:
        if os.path.isfile(candidate):
            return candidate
    return None


def _motion_arg(motion):
    """demo.launch.py's motion alias mapping, mirrored for the planner group."""
    if motion in ("walllooking", "wallfollow"):
        return "walllooking"
    if motion in ("walloriented", "wall_oriented"):
        return "walloriented"
    return "forward"


def _odom_topic(v):
    return "/slam/odometry" if v["slam"] == "slam" else "/StoneFish/Odometry"


def _rviz_config(v, bringup_share):
    """One view for every mode — mirrors demo.launch.py.

    RViz is handed a scratch copy: it rewrites its whole config on exit, and
    the installed path is a symlink into the source tree under
    --symlink-install, so pointing it at the real file let each session edit
    the tracked view. See session_rviz_config in demo.launch.py.
    """
    session_dir = os.path.join(tempfile.gettempdir(), "activeslam_rviz")
    os.makedirs(session_dir, exist_ok=True)
    dst = os.path.join(session_dir, f"demo_{os.getpid()}.rviz")
    shutil.copyfile(os.path.join(bringup_share, "rviz", "demo.rviz"), dst)
    return dst


class Group:
    """One independently restartable layer of the stack."""

    def __init__(self, id, label, description, build, depends=(),
                 visible=lambda v: True, env_extra=None, graceful=True):
        self.id = id
        self.label = label
        self.description = description
        self.build = build          # values -> list of argv lists
        self.depends = tuple(depends)  # parameter ids that force a restart
        self.visible = visible
        # Extra environment for this group only — LD_PRELOAD in particular must
        # not leak into the Python nodes it was never meant for.
        self.env_extra = env_extra or {}
        # graceful=False: skip the SIGINT/SIGTERM ladder and kill outright.
        # For a pure viewer there is nothing to flush, and RViz's own signal
        # teardown aborts on this platform, which surfaces as a crash dialog.
        self.graceful = graceful

    def commands(self, values):
        return self.build(values)


def build_groups(bringup_share=""):
    """The group table. `bringup_share` is only needed for the RViz config path."""

    def core(v):
        return [["ros2", "launch", "stonefish_groundtruth_mapping", "core.launch.py",
                 f"scene:={v['scene']}",
                 f"obj_mesh:={v['obj_mesh']}",
                 f"obj_x:={v['obj_x']}", f"obj_y:={v['obj_y']}",
                 f"obj_z:={v['obj_z']}", f"obj_scale:={v['obj_scale']}",
                 f"obj_roll:={v['obj_roll']}", f"obj_pitch:={v['obj_pitch']}",
                 f"obj_yaw:={v['obj_yaw']}",
                 f"robot_x:={v['robot_x']}", f"robot_y:={v['robot_y']}",
                 f"robot_z:={v['robot_z']}",
                 f"robot_roll:={v['robot_roll']}",
                 f"robot_pitch:={v['robot_pitch']}",
                 f"robot_yaw:={v['robot_yaw']}",
                 f"thrust_boost:={'true' if v['thrust_boost'] else 'false'}"]]

    def tf(v):
        use_gt = "false" if v["slam"] == "slam" else "true"
        return [["ros2", "launch", "stonefish_groundtruth_mapping",
                 "tf_only.launch.py", f"use_gt_tf:={use_gt}"]]

    def cloud(v):
        noise = "true" if v["slam"] == "slam" else "false"
        cut = str(v["near_cutoff_m"]) if v.get("noise_attenuation") == "cut_close" else "-1.0"
        return [["ros2", "launch", "stonefish_groundtruth_mapping",
                 "pointcloud_only.launch.py",
                 f"sonar_noise:={noise}",
                 f"noise_profile:={v['noise_profile']}",
                 f"noise_seed:={v['noise_seed']}",
                 f"near_cutoff:={cut}"]]

    def mapper(v):
        # Under mode:=frontier mapper:=tsdf the mapper derives /projected_map
        # from its own grid (banded around the cruise depth), so frontier
        # detection + A* share the belief map — no separate octomap_server.
        publish_projected = v["mode"] in ("frontier", "goto") and v["mapper"] == "tsdf"
        return [["ros2", "launch", "stonefish_groundtruth_mapping",
                 "mapper_only.launch.py",
                 f"mapper:={v['mapper']}",
                 f"map_rebuild:={'true' if v['map_rebuild'] else 'false'}",
                 # octomap_server bands its own /projected_map around this depth
                 # (z_band.py); the TSDF mapper bands its projection through
                 # target_depth_m. Same cruise depth, one knob.
                 f"depth:={v['robot_depth_target']}",
                 f"publish_projected_map:={'true' if publish_projected else 'false'}",
                 f"target_depth_m:={v['robot_depth_target']}",
                 f"tsdf_octomap:={'true' if v['tsdf_octomap'] else 'false'}"]]

    def gt_map(v):
        return [["ros2", "launch", "stonefish_groundtruth_mapping",
                 "gt_map.launch.py", f"mapper:={v['mapper']}"]]

    def slam(v):
        return [["ros2", "launch", "slam_backend", "slam.launch.py",
                 f"noise_profile:={v['noise_profile']}",
                 f"loop_closure:={'true' if v['loop_closure'] else 'false'}",
                 f"noise_seed:={v['noise_seed']}",
                 f"map_rebuild:={'true' if v['map_rebuild'] else 'false'}",
                 f"initial_x:={v['robot_x']}",
                 f"initial_y:={v['robot_y']}"]]

    def evaluation(v):
        cmd = ["ros2", "launch", "eval_tools", "eval.launch.py",
               f"mapper:={v['mapper']}"]
        if v["output_dir"]:
            cmd.append(f"output_dir:={v['output_dir']}")
        return [cmd]

    def planner(v):
        # goto mode: the operator owns the goal, so revisit_planner must not
        # preempt it with a loop-closure detour.
        revisit = "true" if (v["revisit"] and v["slam"] == "slam"
                             and v["mode"] == "frontier") else "false"
        return [["ros2", "launch", "frontier_slam", "frontier_slam.launch.py",
                 f"hard_inflation_m:={v['hard_inflation_m']}",
                 f"inflation_m:={v['inflation_m']}",
                 f"plan_inflation_m:={v['plan_inflation_m']}",
                 f"odom_topic:={_odom_topic(v)}",
                 f"revisit:={revisit}",
                 f"scenario:={v['scenario']}",
                 f"scenario_out_dx:={v['scenario_out_dx']}",
                 f"scenario_out_dy:={v['scenario_out_dy']}",
                 f"scan_style:={v['scan_style']}",
                 f"scan_sweep_deg:={v['scan_sweep_deg']}",
                 f"depth:={v['robot_depth_target']}",
                 f"safety_start_enabled:={'true' if v['safety_start_enabled'] else 'false'}",
                 f"motion:={_motion_arg(v['motion'])}",
                 f"wall_orientation_offset_deg:={v['wall_orientation_offset_deg']}",
                 f"wall_orientation_lookahead_m:={v['wall_orientation_lookahead_m']}",
                 f"tsdf_frontier_standoff_m:={v['tsdf_frontier_standoff_m']}",
                 "wall_points_topic:=" + ("/tsdf/surface_cloud" if v["mapper"] == "tsdf"
                                          else "/octomap_point_cloud_centers"),
                 f"wall_standoff:={v['wall_standoff']}",
                 f"wall_switch_goal_distance:={v['wall_switch_goal_distance']}",
                 f"wall_switch_scan_angle:={v['wall_switch_scan_angle']}",
                 f"wall_switch_scan_yaw:={v['wall_switch_scan_yaw']}",
                 f"wall_path_influence:={v['wall_path_influence']}",
                 f"wall_path_look_offset_deg:={v['wall_path_look_offset_deg']}",
                 f"wall_normal_offset_deg:={v['wall_normal_offset_deg']}",
                 f"wall_path_heading_weight:={v['wall_path_heading_weight']}"]]

    def teleop_support(v):
        # demo.launch.py starts these two only for mode:=teleop.
        return [
            ["ros2", "run", "frontier_slam", "motion_safety_gate", "--ros-args",
             "-p", f"odom_topic:={_odom_topic(v)}",
             "-p", f"start_enabled:={'true' if v['safety_start_enabled'] else 'false'}"],
            ["ros2", "run", "frontier_slam", "heavy_sim_mixer"],
        ]

    def rviz(v):
        return [["rviz2", "-d", _rviz_config(v, bringup_share)]]

    def rqt(v):
        return [["rqt"]]

    is_slam = lambda v: v["slam"] == "slam"

    return [
        Group("core", "Simulator (Stonefish)",
              "Stonefish underwater simulator: BlueROV2 + scene meshes, and the "
              "depth camera standing in for a wide-FoV 3D sonar. Expensive to "
              "start, so only scene/object selection and object scale restart it.",
              core, depends=["scene", "obj_mesh", "obj_scale", "thrust_boost"]),
        Group("tf", "TF chain",
              "world_ned -> bluerov2/base_link -> bluerov2/Dcam. Under "
              "slam:=none this is broadcast from ground truth (odom_tf_sync); "
              "under slam:=slam the pose graph owns it instead.",
              tf, depends=["slam"]),
        Group("cloud", "Point cloud + sonar noise",
              "depth_image_proc turns the depth image into /cloud_in. Under "
              "slam:=slam a datasheet-grounded WaterLinked Sonar 3D-15 noise "
              "model is spliced in ahead of every consumer.",
              cloud, depends=["slam", "noise_profile", "noise_seed",
                              "noise_attenuation", "near_cutoff_m"]),
        Group("mapper", "Map backend",
              "OctoMap occupancy grid or VDBFusion TSDF. Consumes /cloud_in "
              "only, so the backend can be swapped without touching the sim.",
              mapper, depends=["mapper", "map_rebuild", "mode", "tsdf_octomap",
                               "robot_depth_target"]),
        Group("gt_map", "Ground-truth reference map",
              "A second map built from the exact simulator pose, overlaid "
              "against the belief map so map drift is visible directly.",
              gt_map, depends=["mapper"], visible=is_slam),
        Group("slam", "SLAM backend (GTSAM)",
              "Simulated pressure/IMU/DVL sensors, dead-reckoning fusion and a "
              "GTSAM iSAM2 pose graph with loop closure.",
              slam, depends=["noise_profile", "loop_closure", "noise_seed",
                             "map_rebuild", "robot_x", "robot_y"], visible=is_slam),
        Group("eval", "Benchmark + eval",
              "ATE/RPE against ground truth, TUM trajectory export and map "
              "metrics, written to eval/runs/<timestamp>/.",
              evaluation, depends=["output_dir", "mapper"], visible=is_slam),
        Group("planner", "Planner",
              "Frontier detection, A* planning and the path executor. Runs "
              "under frontier (it picks its own goals) and under goto (the "
              "operator's target point is the goal).",
              planner, depends=["mode", "motion", "scan_style", "scenario",
                                "revisit", "slam", "mapper",
                                "scenario_out_dx", "scenario_out_dy",
                                "scan_sweep_deg", "robot_depth_target",
                                "safety_start_enabled",
                                "wall_orientation_offset_deg",
                                "wall_orientation_lookahead_m",
                                "tsdf_frontier_standoff_m", "wall_standoff",
                                "wall_switch_goal_distance",
                                "wall_switch_scan_angle", "wall_switch_scan_yaw",
                                "wall_path_influence", "wall_path_look_offset_deg",
                                "wall_normal_offset_deg", "wall_path_heading_weight",
                                "hard_inflation_m", "inflation_m", "plan_inflation_m"],
              visible=lambda v: v["mode"] in ("frontier", "goto")),
        Group("teleop_support", "Safety gate + thruster mixer",
              "Fail-closed motion safety gate and the thruster mixer that "
              "teleop drives through.",
              teleop_support, depends=["slam", "safety_start_enabled"],
              visible=lambda v: v["mode"] == "teleop"),
        Group("rviz", "RViz",
              "Visualisation. The view is picked automatically from the "
              "mapper/slam combination.",
              rviz, depends=["slam", "mapper"],
              visible=lambda v: bool(v["rviz"]),
              env_extra=({"LD_PRELOAD": _preload} if (_preload := octomap_preload_path())
                         else {}),
              graceful=False),
        Group("rqt", "rqt",
              "Introspection GUI: node graph, topic monitor, plots and "
              "parameter reconfigure. Restores whatever perspective was left "
              "open last time. Offered under frontier mode only.",
              rqt, visible=lambda v: bool(v["rqt"]) and v["mode"] == "frontier",
              graceful=False),
    ]


SESSION_POLL_INTERVAL = 0.3


@contextlib.contextmanager
def stderr_to(path):
    """Point this process's fd 2 at path.

    rclpy's middleware writes to the file descriptor directly, not through
    sys.stderr, so it lands on top of the curses screen. Only fd 2 is moved:
    curses draws through fd 1, and redirecting that would send the UI to the
    file instead of the terminal.
    """
    sys.stderr.flush()
    saved = os.dup(2)
    fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        os.dup2(fd, 2)
        yield
    finally:
        sys.stderr.flush()
        os.dup2(saved, 2)
        os.close(saved)
        os.close(fd)


class SessionLog:
    """One merged, timestamped log of a whole run.

    The per-group logs stay as they are; this tails them and interleaves their
    lines with the launcher's own events, so there is a single file that tells
    the story in order. `latest.log` always points at the newest session.

    Tailing rather than sitting in the children's write path is deliberate: a
    stalled writer here must never be able to block the simulator on a pipe.
    """

    def __init__(self, log_dir, clock=time.time):
        self.log_dir = log_dir
        self._clock = clock
        self._lock = threading.Lock()
        self._sources = []       # (label, path, offset, pending bytes)
        self._stop = threading.Event()
        self._thread = None
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.fromtimestamp(clock()).strftime("%Y%m%d-%H%M%S")
        self.path = os.path.join(log_dir, f"session-{stamp}.log")
        self._handle = open(self.path, "ab", buffering=0)
        self._link_latest()

    def _link_latest(self):
        self.latest_path = os.path.join(self.log_dir, "latest.log")
        try:
            if os.path.islink(self.latest_path) or os.path.exists(self.latest_path):
                os.unlink(self.latest_path)
            os.symlink(os.path.basename(self.path), self.latest_path)
        except OSError:
            # A filesystem without symlinks still gets the session file itself.
            self.latest_path = self.path

    def _write(self, label, text):
        stamp = datetime.datetime.fromtimestamp(self._clock()).strftime("%H:%M:%S.%f")[:-3]
        line = f"{stamp}  {label:<14}  {text}\n"
        with self._lock:
            try:
                self._handle.write(line.encode("utf-8", "replace"))
            except (OSError, ValueError):
                pass

    def event(self, text):
        """Record a launcher-level event: a start, a signal, an apply."""
        self._write("launcher", text)

    def follow(self, label, path, offset=0):
        """Interleave a per-group log into the session log from offset on.

        Replaces any existing source for the same file: a restarted group
        appends to the log it used before, and following it twice emits every
        line once per restart.

        The existing offset wins when there is one. It is at or behind the new
        one, and the bytes between them are the previous process's last words —
        skipping to the restarted process's offset would drop exactly the
        output that says why it was restarted.
        """
        with self._lock:
            for src in self._sources:
                if src[1] == path:
                    src[0] = label
                    return
            self._sources.append([label, path, offset, b""])

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self):
        while not self._stop.wait(SESSION_POLL_INTERVAL):
            self.drain()
        self.drain()

    def drain(self):
        """Copy whatever the followed logs have grown by since the last pass."""
        with self._lock:
            sources = list(self._sources)
        for src in sources:
            label, path, offset, pending = src
            try:
                size = os.path.getsize(path)
            except OSError:
                continue
            if size < offset:      # truncated underneath us; start over
                offset = 0
            if size == offset:
                continue
            try:
                with open(path, "rb") as fh:
                    fh.seek(offset)
                    chunk = fh.read(size - offset)
            except OSError:
                continue
            src[2] = offset + len(chunk)
            buf = pending + chunk
            # Hold an unterminated tail back: a line half-written when we read
            # would otherwise be split across two entries.
            lines = buf.split(b"\n")
            src[3] = lines.pop()
            for raw in lines:
                text = raw.decode("utf-8", "replace").rstrip("\r")
                if text.strip():
                    self._write(label, text)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.drain()
        # Collected under the lock but written outside it: _write takes the
        # same lock, which is not reentrant.
        with self._lock:
            tails = [(src[0], src[3]) for src in self._sources if src[3].strip()]
            for src in self._sources:
                src[3] = b""
        for label, raw in tails:
            self._write(label, raw.decode("utf-8", "replace"))
        with self._lock:
            try:
                self._handle.close()
            except OSError:
                pass


class Proc:
    """One spawned process, in its own process group."""

    def __init__(self, argv, log_path, env=None, graceful=True):
        self.argv = argv
        self.log_path = log_path
        self.graceful = graceful
        self.log_handle = open(log_path, "ab", buffering=0)
        # Where this run's output starts: the per-group logs are appended to
        # across runs, and the session log must not re-ingest older ones.
        try:
            self.log_offset = self.log_handle.tell()
        except OSError:
            self.log_offset = 0
        self.popen = subprocess.Popen(
            argv,
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,   # own process group -> killpg reaches children
            env=env,
        )
        # Captured now: once the child is reaped, getpgid() no longer resolves,
        # and the group is what teardown actually targets.
        try:
            self.pgid = os.getpgid(self.popen.pid)
        except OSError:
            self.pgid = self.popen.pid

    @property
    def pid(self):
        return self.popen.pid

    def poll(self):
        return self.popen.poll()

    def alive(self):
        """Group-level liveness: the launched work is still running if anything
        in the process group survives, even after the direct child exits."""
        return not self.group_empty()

    def _killpg(self, sig):
        if self.pgid is None:
            return False
        try:
            os.killpg(self.pgid, sig)
            return True
        except (ProcessLookupError, PermissionError, OSError):
            return False

    def group_empty(self):
        """True once no process remains in the group.

        Checked instead of the direct child's exit status: `ros2 launch` dying
        does not mean its nodes died, and a background child inherits SIG_IGN
        for SIGINT, so watching only the parent reports success while the real
        work is still running.
        """
        if self.pgid is None:
            return True
        # Reap the direct child first — a zombie still counts as a group member.
        self.popen.poll()
        try:
            os.killpg(self.pgid, 0)
        except ProcessLookupError:
            return True
        except PermissionError:
            return False
        except OSError:
            return True
        return False

    def stop(self, on_event=None):
        """SIGINT -> SIGTERM -> SIGKILL to the whole process group, escalating
        until the group is actually empty."""
        if self.group_empty():
            self._reap()
            self._close()
            return True
        for sig, timeout, name in (
            (signal.SIGINT, STOP_SIGINT_TIMEOUT, "SIGINT"),
            (signal.SIGTERM, STOP_SIGTERM_TIMEOUT, "SIGTERM"),
            (signal.SIGKILL, STOP_SIGKILL_TIMEOUT, "SIGKILL"),
        ):
            if on_event:
                on_event(f"{name} -> pgid {self.pgid}")
            self._killpg(sig if self.graceful else signal.SIGKILL)
            deadline = time.time() + timeout
            while time.time() < deadline:
                if self.group_empty():
                    break
                time.sleep(0.1)
            if self.group_empty():
                break
        self._reap()
        self._close()
        return self.group_empty()

    def _reap(self):
        try:
            self.popen.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass

    def _close(self):
        try:
            self.log_handle.close()
        except OSError:
            pass


class Supervisor:
    """Starts, stops and restarts groups; guarantees teardown on exit."""

    def __init__(self, ws_root, log_dir, groups, env=None, session=None):
        self.ws_root = ws_root
        self.log_dir = log_dir
        self.session = session
        self.groups = {g.id: g for g in groups}
        self.env = env or os.environ.copy()
        self.procs = {}          # group id -> [Proc]
        self.applied = {}        # group id -> values snapshot it was started with
        self._last_status = {}   # group id -> status at the last poll
        self._shutting_down = False
        os.makedirs(log_dir, exist_ok=True)

    # -- state ---------------------------------------------------------------
    def running_ids(self):
        return [gid for gid, ps in self.procs.items() if any(p.alive() for p in ps)]

    def status(self, gid):
        ps = self.procs.get(gid)
        if not ps:
            return "stopped"
        if all(p.alive() for p in ps):
            return "running"
        if any(p.alive() for p in ps):
            return "partial"
        return "exited"

    def exit_codes(self, gid):
        return [p.poll() for p in self.procs.get(gid, [])]

    def poll_transitions(self):
        """Status changes since the last call: (gid, was, now, exit codes).

        Only groups still tracked are considered, and a deliberate stop drops
        its entry, so what this reports is a group going down on its own.
        """
        changes = []
        for gid in list(self.procs):
            now = self.status(gid)
            was = self._last_status.get(gid)
            self._last_status[gid] = now
            if was is not None and was != now:
                changes.append((gid, was, now, self.exit_codes(gid)))
        for gid in list(self._last_status):
            if gid not in self.procs:
                self._last_status.pop(gid, None)
        return changes

    # -- lifecycle -----------------------------------------------------------
    def start(self, gid, values, on_event=None):
        if gid in self.procs and any(p.alive() for p in self.procs[gid]):
            return
        group = self.groups[gid]
        env = dict(self.env)
        env.update(group.env_extra)
        procs = []
        for i, argv in enumerate(group.commands(values)):
            log_path = os.path.join(self.log_dir, f"{gid}{'' if i == 0 else f'_{i}'}.log")
            if on_event:
                on_event(f"start {gid}: {' '.join(argv)}")
            p = Proc(argv, log_path, env=env, graceful=group.graceful)
            if self.session:
                label = gid if i == 0 else f"{gid}_{i}"
                self.session.follow(label, p.log_path, p.log_offset)
            procs.append(p)
        self.procs[gid] = procs
        self.applied[gid] = dict(values)

    def stop(self, gid, on_event=None):
        self.stop_many([gid], on_event=on_event)

    def stop_many(self, gids, on_event=None):
        """Tear down several groups at once.

        Signals are delivered to every group up front and then waited on
        collectively, so a slow or signal-ignoring group costs the escalation
        ladder once overall instead of once per group (serial teardown of the
        full stack would otherwise take minutes).
        """
        # Known groups tear down in reverse start order; any group not listed in
        # GROUP_ORDER still goes through this parallel path rather than a slow
        # serial fallback.
        ordered = [g for g in reversed(GROUP_ORDER) if g in gids and g in self.procs]
        ordered += [g for g in gids if g in self.procs and g not in GROUP_ORDER]
        remaining = [p for gid in ordered for p in self.procs.get(gid, [])]
        for sig, timeout, name in (
            (signal.SIGINT, STOP_SIGINT_TIMEOUT, "SIGINT"),
            (signal.SIGTERM, STOP_SIGTERM_TIMEOUT, "SIGTERM"),
            (signal.SIGKILL, STOP_SIGKILL_TIMEOUT, "SIGKILL"),
        ):
            remaining = [p for p in remaining if not p.group_empty()]
            if not remaining:
                break
            if on_event:
                on_event(f"{name} -> {len(remaining)} process group(s)")
            for p in remaining:
                # Non-graceful groups (viewers) skip straight to SIGKILL rather
                # than run a signal teardown that is known to abort.
                p._killpg(sig if p.graceful else signal.SIGKILL)
            deadline = time.time() + timeout
            while time.time() < deadline:
                if all(p.group_empty() for p in remaining):
                    break
                time.sleep(0.1)
        for gid in ordered:
            for p in self.procs.get(gid, []):
                p._reap()
                p._close()
            self.procs.pop(gid, None)
            self.applied.pop(gid, None)

    def needs_restart(self, gid, values):
        """True when a running group was started with a value it depends on that
        has since changed."""
        old = self.applied.get(gid)
        if old is None:
            return False
        return any(old.get(k) != values.get(k) for k in self.groups[gid].depends)

    def pending_for(self, pid, values):
        """Groups a single edited parameter would restart, start or stop.

        Returns [(gid, action)] so the change can be flagged on the option's own
        row instead of only in a footer summary.
        """
        actions = {}
        running = set(self.running_ids())
        for gid in GROUP_ORDER:
            group = self.groups.get(gid)
            old = self.applied.get(gid)
            if (group and gid in running and old is not None
                    and pid in group.depends and old.get(pid) != values.get(pid)):
                actions[gid] = "restart"
        # Visibility: compare against the same values with this one parameter put
        # back to what the stack was started with, so only its own effect shows.
        base = self.applied.get("core")
        if base is not None and pid in base and base[pid] != values.get(pid):
            reverted = dict(values)
            reverted[pid] = base[pid]
            for gid in GROUP_ORDER:
                group = self.groups.get(gid)
                if not group:
                    continue
                # A group appearing or disappearing outranks a restart — that is
                # what apply() would do with it.
                if group.visible(values) and not group.visible(reverted) and gid not in running:
                    actions[gid] = "start"
                elif group.visible(reverted) and not group.visible(values) and gid in running:
                    actions[gid] = "stop"
        return [(gid, actions[gid]) for gid in GROUP_ORDER if gid in actions]

    def plan(self, values):
        """(to_stop, to_start, to_restart) for the requested configuration."""
        wanted = [gid for gid in GROUP_ORDER
                  if gid in self.groups and self.groups[gid].visible(values)]
        running = set(self.running_ids())
        to_stop = [g for g in GROUP_ORDER if g in running and g not in wanted]
        to_start = [g for g in wanted if g not in running]
        to_restart = [g for g in wanted if g in running and self.needs_restart(g, values)]
        return to_stop, to_start, to_restart

    def apply(self, values, on_event=None):
        to_stop, to_start, to_restart = self.plan(values)
        self.stop_many(to_stop + to_restart, on_event=on_event)
        for gid in [g for g in GROUP_ORDER if g in to_start + to_restart]:
            self.start(gid, values, on_event=on_event)
        return to_stop, to_start, to_restart

    def shutdown_all(self, on_event=None):
        """Idempotent full teardown. Safe to call from a signal handler."""
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self.stop_many(list(self.procs), on_event=on_event)
            # Anything spawned outside GROUP_ORDER still gets reaped.
            while self.procs:
                gid = next(iter(self.procs))
                for p in self.procs.pop(gid, []):
                    p.stop(on_event=on_event)
                self.applied.pop(gid, None)
        finally:
            self._shutting_down = False


def robot_spawn_pose():
    """(name, xyz, rpy) the robot is spawned at, read from the scenario.

    Parsed rather than hardcoded so a change to the .scn is picked up; falls
    back to the shipped BlueROV2 values if the scenario cannot be read.
    """
    import xml.etree.ElementTree as ET
    world_dir = os.environ.get("STONEFISH_WORLD_DIR")
    if not world_dir:
        try:
            from ament_index_python.packages import get_package_share_directory
            world_dir = get_package_share_directory("world")
        except Exception:
            return DEFAULT_SPAWN
    path = os.path.join(world_dir, "data", "robot", "bluerov2_unphy.scn")
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError):
        return DEFAULT_SPAWN
    robot = root.find(".//robot")
    if robot is None:
        return DEFAULT_SPAWN
    name = robot.get("name") or DEFAULT_SPAWN[0]
    wt = robot.find("world_transform")
    if wt is None:
        return DEFAULT_SPAWN
    def triple(attr, default):
        try:
            parts = [float(x) for x in (wt.get(attr) or "").split()]
            return tuple(parts) if len(parts) == 3 else default
        except ValueError:
            return default
    return (name, triple("xyz", DEFAULT_SPAWN[1]), triple("rpy", DEFAULT_SPAWN[2]))


def rpy_to_quaternion(roll, pitch, yaw):
    """ZYX intrinsic roll/pitch/yaw to (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quaternion_to_rpy(x, y, z, w):
    """Quaternion to ZYX intrinsic roll/pitch/yaw, in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny_cosp, cosy_cosp)


# --------------------------------------------------------------------------
# drive keys (teleop + target point)
#
# The launcher drives on the physical QWEASD cluster, so the mapping is per
# keyboard layout: an AZERTY keyboard produces different letters at those
# positions. Canonical actions keep the rest of the code layout-independent.

DRIVE_KEYS = {
    "qwerty": {"w": "fwd", "s": "back", "q": "strafe_l", "e": "strafe_r",
               "a": "yaw_l", "d": "yaw_r", " ": "up", "x": "down", "f": "halt"},
    # Same physical cluster as qwerty. W stays as a second forward binding:
    # it exists on AZERTY too (bottom row) and conflicts with nothing.
    "azerty": {"z": "fwd", "w": "fwd", "s": "back", "a": "strafe_l",
               "e": "strafe_r", "q": "yaw_l", "d": "yaw_r", " ": "up",
               "x": "down", "f": "halt"},
}


def drive_action(ch, layout="qwerty"):
    """Canonical drive action for a typed character, or None if not a drive key."""
    if not ch:
        return None
    return DRIVE_KEYS.get(layout, DRIVE_KEYS["qwerty"]).get(ch.lower())


def action_to_command(action, step, turn):
    """(f, s, y, v) body demand for a canonical action.

    Same convention as launch_tools/keyboard_control.py: v > 0 moves the
    vehicle up (it is published negated — NED body Z points down).
    """
    f = s = y = v = 0.0
    if action == "fwd":
        f = step
    elif action == "back":
        f = -step
    elif action == "strafe_l":
        s = -step
    elif action == "strafe_r":
        s = step
    elif action == "yaw_l":
        y = -turn
    elif action == "yaw_r":
        y = turn
    elif action == "up":
        v = step
    elif action == "down":
        v = -step
    return f, s, y, v


# Target-point following (Y toggles it on the control screen). The point is
# driven to by the same A* planner that frontier exploration uses — routed
# through frontier_extractor/waypoint_controller via /frontier_slam/suspend
# + /frontier_slam/goal (see revisit_planner.py for the same handoff
# pattern) — rather than a bespoke pursuit controller in the launcher.
POINT_STEP_M = 0.5         # point travel per keypress, before speed_factor
POINT_MIN_Z = 0.2          # m — NED z is down; keep the target under the surface
# Matches revisit_planner.GOAL_REPUBLISH_S / frontier_extractor's own
# republish cadence — the goal topic isn't latched, so a fresh subscriber
# (or one that missed the initial publish) needs a periodic resend.
POINT_GOAL_REPUBLISH_S = 2.0

# Point mode reuses the same physical QWEASD cluster, but a target point has
# no heading, so a yaw-relative fwd/strafe mapping (like DRIVE_KEYS) made the
# point drift sideways as the vehicle turned. These map straight onto world
# axes instead: W/S -> X, A/D -> Y, Q/E -> Z.
POINT_AXIS_KEYS = {
    "qwerty": {"w": "x_pos", "s": "x_neg", "a": "y_neg", "d": "y_pos",
               "q": "z_down", "e": "z_up", "f": "halt"},
    "azerty": {"z": "x_pos", "w": "x_pos", "s": "x_neg", "q": "y_neg",
               "d": "y_pos", "a": "z_down", "e": "z_up", "f": "halt"},
}


def point_axis_action(ch, layout="qwerty"):
    """Canonical point-move action for a typed character, or None."""
    if not ch:
        return None
    return POINT_AXIS_KEYS.get(layout, POINT_AXIS_KEYS["qwerty"]).get(ch.lower())


def move_point(point, action, step):
    """New target after one point-move keypress, along world axes.

    z_up decreases z since NED z is down (positive z is deeper).
    """
    x, y, z = point
    if action == "x_pos":
        x += step
    elif action == "x_neg":
        x -= step
    elif action == "y_pos":
        y += step
    elif action == "y_neg":
        y -= step
    elif action == "z_up":
        z = max(POINT_MIN_Z, z - step)
    elif action == "z_down":
        z += step
    return [x, y, z]


def venv_env(ws_root):
    """Environment with the workspace venv on PATH, as activating it would give.

    colcon has to run under the venv's interpreter: the ament_python entry
    points inherit their shebang from it, and only the venv can import gtsam
    and the other wheels.
    """
    env = os.environ.copy()
    if ws_root:
        venv_bin = os.path.join(ws_root, ".venv", "bin")
        if os.path.isdir(venv_bin):
            env["PATH"] = venv_bin + os.pathsep + env.get("PATH", "")
    return env


def find_workspace_root(start=None):
    """Nearest ancestor containing install/setup.bash."""
    d = os.path.dirname(os.path.abspath(start or __file__))
    for _ in range(6):
        if os.path.isfile(os.path.join(d, "install", "setup.bash")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent
    return None
