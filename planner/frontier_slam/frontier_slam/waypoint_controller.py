"""Waypoint controller node.

Drives the BlueROV2 toward the current frontier goal by following A*-planned
path waypoints published on /frontier_slam/path.  Responsibilities:

  1. Path following  — advance through waypoints published by the extractor,
                       yaw toward the current waypoint, surge when aligned.
  2. Depth hold      — track the active goal's Z (whoever published it —
                       frontier's own pick, a revisit/scenario/mission
                       waypoint, the launcher's target point); with no goal
                       (waiting or scanning), hold depth_setpoint, captured
                       on the first odometry message unless set explicitly.
  3. Emergency stop  — zero surge if the forward depth camera detects an
                       obstacle closer than EMERGENCY_STOP_DIST (last-resort
                       safety; A* inflation should prevent this normally).
  4. Initial scan    — populate an initial map before navigating; `scan_style`
                       picks a cable-safe sweep (default, scan_sweep.py) or
                       the legacy spin. Same behaviour while waiting/at goal.

Why not just track the goal's Z unconditionally? See Progress.md "Session 1 —
Findings": frontier_extractor used to set the goal's Z to the robot's own
live Z every cycle, a self-referential loop that let the robot sink
undetected (Change 12). It now anchors its own picks to depth_setpoint
instead (locked from first odom, same as here), so goal.point.z is always a
real target by the time it reaches this node — never a copy of the robot's
own position.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import String

from frontier_slam.control_utils import wrap_angle, yaw_from_quat
from frontier_slam.scan_sweep import SweepScan, scan_yaw_command
from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'rx', 'ry', 'rz', 'gx', 'gy', 'gz',
    'dist_m', 'hdg_err_deg', 'depth_err_m',
    'surge', 'yaw_cmd', 'heave',
    'obs_m', 'path_len', 'wp_idx', 'event',
]


class WaypointController(Node):
    # P-gains
    KP_YAW   = 0.07
    # Keep yaw behaviour unchanged, but slow path translation by 10% to give
    # scan matching more overlap between consecutive mapping keyframes.
    KP_SURGE = 0.25
    KP_HEAVE = 0.35

    MAX_SURGE             = 0.25
    GOAL_RADIUS           = 2.0    # m
    GOAL_REACHED_TIMEOUT  = 10.0   # s — clear stale goal after this long at goal
    SCAN_YAW              = 0.026  # rad/s cmd — slow, for sonar frame overlap at 5 Hz
    INIT_SCAN_DURATION    = 10.0    # s — initial spin before navigating (~1 rotation)
    SCAN_YAW_RATE_RAD_S   = 0.22    # achieved rate at SCAN_YAW; sizes the sweep timeout
    WAYPOINT_ADVANCE_DIST = 1.5    # m — advance to next waypoint when this close
    OBS_SLOW_DIST         = 1.5    # m — begin linearly reducing surge at this distance
    EMERGENCY_STOP_DIST   = 0.4    # m — ramp reaches zero; switch to back-surge below this
    BACK_SURGE_SPEED      = 0.12   # m/s backward when obstacle is inside EMERGENCY_STOP_DIST
    ESCAPE_YAW            = 0.20   # spin rate for CTRL_STUCK escape
    ESCAPE_DURATION       = 4.0    # s — CTRL_STUCK escape spin duration
    STUCK_SURGE_MIN       = 0.15   # min surge to consider "trying to move"
    STUCK_WINDOW          = 5.0    # s — position-stuck detection window
    STUCK_MOVE_MIN        = 0.25   # m — minimum movement expected in STUCK_WINDOW
    CTRL_HZ               = 10.0
    LOG_EVERY_N_TICKS     = 10     # CSV row rate = CTRL_HZ / this → 1 Hz

    def __init__(self):
        super().__init__('waypoint_controller')

        self.declare_parameter('depth_setpoint', -1.0)
        v = float(self.get_parameter('depth_setpoint').value)
        self._depth_setpoint: float | None = None if v < 0 else v
        if self._depth_setpoint is not None:
            self.get_logger().info(f'depth setpoint from launch param: {self._depth_setpoint:.2f} m')

        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.declare_parameter('goal_topic', '/frontier_slam/goal')
        self.declare_parameter('path_topic', '/frontier_slam/path')
        self.declare_parameter('command_topic', '/motion/body_command')
        goal_topic = str(self.get_parameter('goal_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)

        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('turn_factor', 1.0)

        self.declare_parameter('scan_style', 'sweep')
        self.declare_parameter('scan_sweep_deg', 180.0)
        self._scan_style = str(self.get_parameter('scan_style').value)
        scan_sweep_deg = float(self.get_parameter('scan_sweep_deg').value)
        self._sweep = SweepScan(scan_sweep_deg, self.SCAN_YAW_RATE_RAD_S)

        self._goal: np.ndarray | None = None
        self._pose: np.ndarray | None = None
        self._yaw  = 0.0
        self._min_front_dist      = float('inf')
        self._init_scan_end: float | None  = None   # set on first odom
        self._goal_reached_at: float | None = None
        self._path: list  = []    # [(wx, wy), ...] from /frontier_slam/path
        self._wp_idx: int = 0
        self._escape_until: float | None  = None
        self._stuck_ref_pos: np.ndarray | None = None
        self._stuck_ref_t:   float | None = None
        self._tick = 0

        self._log = open_session_log('controller', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(PointStamped, goal_topic,                   self._goal_cb,  1)
        self.create_subscription(Path,         path_topic,                   self._path_cb,  1)
        self.create_subscription(Odometry,     odom_topic,                   self._odom_cb,  10)
        self.create_subscription(Image,        '/sensor_msgs/image_depth',   self._depth_cb, 1)
        self._command_pub = self.create_publisher(Twist, command_topic, 1)

        # Same label the CSV `event` column carries — what the vehicle is
        # doing, for the launcher's status panel.
        self._activity_pub = self.create_publisher(
            String, '/frontier_slam/activity', 1)

        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info(
            f'waypoint_controller ready — goal_topic={goal_topic} path_topic={path_topic} '
            f'— logging to {self._log.path}')

    # ------------------------------------------------------------------
    # ROS callbacks
    def _depth_cb(self, msg: Image) -> None:
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        v = data[np.isfinite(data) & (data > 0.1)]
        self._min_front_dist = float(v.min()) if v.size > 0 else float('inf')

    def _goal_cb(self, msg: PointStamped) -> None:
        new_goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        # Only reset navigation state when the goal actually moved by more than 1 m.
        # The extractor republishes at 0.5 Hz even when the goal is unchanged; resetting
        # unconditionally prevents the 5 s CTRL_STUCK window from ever accumulating.
        goal_changed = (self._goal is None or
                        np.hypot(new_goal[0] - self._goal[0],
                                 new_goal[1] - self._goal[1]) > 1.0)
        self._goal = new_goal
        if goal_changed:
            self._goal_reached_at = None
            self._path          = []
            self._wp_idx        = 0
            self._stuck_ref_pos = None
            self._stuck_ref_t   = None

    def _path_cb(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not self._path or self._pose is None:
            self._wp_idx = 0
            return
        # Resume from the nearest waypoint to the current position
        dists = [math.hypot(wp[0] - self._pose[0], wp[1] - self._pose[1])
                 for wp in self._path]
        self._wp_idx = int(np.argmin(dists))

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw  = yaw_from_quat(msg.pose.pose.orientation)
        if self._init_scan_end is None:   # first odom
            if self._depth_setpoint is None:
                self._depth_setpoint = float(p.z)
                self.get_logger().info(
                    f'depth setpoint locked from odom at {self._depth_setpoint:.2f} m'
                )
            self._init_scan_end = self._t_ros() + self.INIT_SCAN_DURATION
            if self._scan_style == 'spin':
                self.get_logger().info(
                    f'initial {self.INIT_SCAN_DURATION:.0f}s scan starting'
                )
            else:
                self._sweep.start(self._yaw, self._t_ros())
                self.get_logger().info('initial cable-safe sweep scan starting')

    def _t_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Control primitives
    def _heave_cmd(self) -> float:
        """Hold depth: the active goal's Z while pursuing one, the locked/
        configured setpoint otherwise (no goal — waiting or scanning).

        goal.point.z is a real target here regardless of who published it —
        a frontier pick (now locked to depth_setpoint at the source, see
        frontier_extractor's own _cruise_z), a revisit/scenario/mission
        waypoint, or the launcher's target point: one goal message, one Z,
        no separate depth channel to keep in sync (Change 12 is still
        respected — see frontier_extractor.py for why its own picks don't
        just copy the robot's live Z).
        """
        target_z = self._goal[2] if self._goal is not None else self._depth_setpoint
        if target_z is None:
            return 0.0
        depth_err = self._pose[2] - target_z    # +ve = too deep
        return float(np.clip(-self.KP_HEAVE * depth_err, -1.0, 1.0))

    def _xy_drive(self, target_xy: np.ndarray) -> tuple:
        """Return (surge_cmd, yaw_cmd, dist, heading_err)."""
        dx, dy = target_xy - self._pose[:2]
        dist = math.hypot(dx, dy)
        heading_err = wrap_angle(math.atan2(dy, dx) - self._yaw)
        yaw_cmd   = float(np.clip(self.KP_YAW * heading_err, -1.0, 1.0))
        surge_raw = self.KP_SURGE * dist * max(0.0, math.cos(heading_err))
        surge_cmd = float(np.clip(surge_raw, 0.0, self.MAX_SURGE))
        return surge_cmd, yaw_cmd, dist, heading_err

    # ------------------------------------------------------------------
    # Main loop
    def _loop(self) -> None:
        self._tick += 1
        write_csv = (self._tick % self.LOG_EVERY_N_TICKS == 0)

        if self._pose is None:
            return

        now   = self._t_ros()
        heave = self._heave_cmd()

        # Initial scan — spin, or cable-safe sweep, to populate the map before first navigation
        if self._init_scan_end is not None:
            if self._scan_style == 'spin':
                if now < self._init_scan_end:
                    self._send_thrust(0.0, self.SCAN_YAW, heave)
                    if write_csv:
                        self._write_csv(0.0, self.SCAN_YAW, heave, 'INIT_SCAN')
                    return
            elif self._sweep.active:
                yaw_cmd = scan_yaw_command(self._sweep, self._scan_style, self._yaw, now,
                                           self.SCAN_YAW, repeat=False)
                self._send_thrust(0.0, yaw_cmd, heave)
                if write_csv:
                    self._write_csv(0.0, yaw_cmd, heave, 'INIT_SCAN')
                return

        if self._goal is None:
            yaw_cmd = scan_yaw_command(self._sweep, self._scan_style, self._yaw, now,
                                       self.SCAN_YAW, repeat=True)
            self._send_thrust(0.0, yaw_cmd, heave)
            if write_csv:
                self._write_csv(0.0, yaw_cmd, heave, 'SCAN')
            return

        dist_xy = float(np.hypot(self._goal[0] - self._pose[0],
                                 self._goal[1] - self._pose[1]))

        if dist_xy < self.GOAL_RADIUS:
            if self._goal_reached_at is None:
                self._goal_reached_at = now
            elif now - self._goal_reached_at > self.GOAL_REACHED_TIMEOUT:
                self.get_logger().info(
                    f'No new goal after {self.GOAL_REACHED_TIMEOUT:.0f}s — '
                    'clearing goal, entering scan'
                )
                self._goal = None
                self._goal_reached_at = None
            yaw_cmd = scan_yaw_command(self._sweep, self._scan_style, self._yaw, now,
                                       self.SCAN_YAW, repeat=True)
            self._send_thrust(0.0, yaw_cmd, heave)
            self.get_logger().info('Goal reached — scanning', throttle_duration_sec=2.0)
            if write_csv:
                self._write_csv(0.0, yaw_cmd, heave, 'GOAL_REACHED', dist=dist_xy)
            return

        self._sweep.reset()   # clear any interrupted scan before path-following resumes

        # Choose navigation target: current path waypoint, or raw goal as fallback
        if self._path:
            while (self._wp_idx < len(self._path) - 1 and
                   math.hypot(self._path[self._wp_idx][0] - self._pose[0],
                              self._path[self._wp_idx][1] - self._pose[1])
                   < self.WAYPOINT_ADVANCE_DIST):
                self._wp_idx += 1
            target_xy = np.array(self._path[self._wp_idx])
        else:
            target_xy = self._goal[:2]

        surge_cmd, yaw_cmd, _, heading_err = self._xy_drive(target_xy)
        event = ''

        # Obstacle response: ramp down then back-surge.
        if self._min_front_dist < self.EMERGENCY_STOP_DIST:
            # Too close — reverse to escape the inflation zone so A* can replan.
            surge_cmd = -self.BACK_SURGE_SPEED
            event = 'EMERG_STOP'
        elif self._min_front_dist < self.OBS_SLOW_DIST:
            factor = (self._min_front_dist - self.EMERGENCY_STOP_DIST) \
                   / (self.OBS_SLOW_DIST - self.EMERGENCY_STOP_DIST)
            surge_cmd *= factor

        # Active escape spin (CTRL_STUCK only)
        if self._escape_until is not None:
            if now < self._escape_until:
                self._send_thrust(0.0, self.ESCAPE_YAW, heave)
                if write_csv:
                    self._write_csv(0.0, self.ESCAPE_YAW, heave, 'CTRL_STUCK_ESCAPE',
                                    dist=dist_xy, hdg_err_deg=math.degrees(heading_err))
                return
            self._escape_until  = None
            self._stuck_ref_pos = None
            self._stuck_ref_t   = None

        # Position-stuck detection
        if surge_cmd >= self.STUCK_SURGE_MIN:
            if self._stuck_ref_pos is None:
                self._stuck_ref_pos = self._pose.copy()
                self._stuck_ref_t   = now
            else:
                moved = float(np.hypot(self._pose[0] - self._stuck_ref_pos[0],
                                       self._pose[1] - self._stuck_ref_pos[1]))
                if moved >= self.STUCK_MOVE_MIN:
                    self._stuck_ref_pos = self._pose.copy()
                    self._stuck_ref_t   = now
                elif now - self._stuck_ref_t >= self.STUCK_WINDOW:
                    self.get_logger().warn(
                        f'CTRL_STUCK: {moved:.2f}m in {self.STUCK_WINDOW:.0f}s at '
                        f'({self._pose[0]:.1f},{self._pose[1]:.1f}) — escape spin'
                    )
                    self._escape_until  = now + self.ESCAPE_DURATION
                    self._stuck_ref_pos = None
                    self._stuck_ref_t   = None
                    self._send_thrust(0.0, self.ESCAPE_YAW, heave)
                    if write_csv:
                        self._write_csv(0.0, self.ESCAPE_YAW, heave, 'CTRL_STUCK',
                                        dist=dist_xy, hdg_err_deg=math.degrees(heading_err))
                    return
        else:
            self._stuck_ref_pos = None
            self._stuck_ref_t   = None

        self._send_thrust(surge_cmd, yaw_cmd, heave)

        self.get_logger().info(
            f'pos=({self._pose[0]:.1f},{self._pose[1]:.1f},{self._pose[2]:.1f}) '
            f'goal=({self._goal[0]:.1f},{self._goal[1]:.1f}) '
            f'wp={self._wp_idx}/{len(self._path)} '
            f'dist={dist_xy:.1f}m  hdg_err={math.degrees(heading_err):+.0f}°  '
            f'surge={surge_cmd:.2f}  yaw={yaw_cmd:+.3f}  heave={heave:+.2f}  '
            f'obs={self._min_front_dist:.1f}m'
            + (f'  [{event}]' if event else ''),
            throttle_duration_sec=2.0,
        )

        if write_csv:
            self._write_csv(surge_cmd, yaw_cmd, heave, event,
                            dist=dist_xy, hdg_err_deg=math.degrees(heading_err))

    # ------------------------------------------------------------------
    # Output
    # Matches safety_gate.py's max_abs_command default (1.0): the gate rejects
    # (and latches INVALID_COMMAND on) any out-of-range component, so a
    # speed_factor/turn_factor above 1x must saturate here, not there.
    MAX_ABS_COMMAND = 1.0

    def _send_thrust(self, surge: float, yaw: float, heave: float) -> None:
        """Single publish choke point — applies the operator speed/turn factors
        (live-tunable from the launcher TUI) uniformly to every caller: path
        following, scanning, escape spin, obstacle backup."""
        speed_factor = float(self.get_parameter('speed_factor').value)
        turn_factor = float(self.get_parameter('turn_factor').value)
        cap = self.MAX_ABS_COMMAND
        msg = Twist()
        msg.linear.x = float(np.clip(surge * speed_factor, -cap, cap))
        msg.linear.z = float(np.clip(heave * speed_factor, -cap, cap))
        msg.angular.z = float(np.clip(yaw * turn_factor, -cap, cap))
        self._command_pub.publish(msg)

    def _write_csv(self, surge, yaw_cmd, heave, event,
                   dist=float('nan'), hdg_err_deg=float('nan')) -> None:
        self._activity_pub.publish(String(data=event or 'FOLLOW_PATH'))
        p = self._pose
        g = self._goal
        depth_err = ((p[2] - self._depth_setpoint)
                     if self._depth_setpoint is not None else float('nan'))
        self._log.write([
            self._t_ros(),
            float(p[0]), float(p[1]), float(p[2]),
            float(g[0]) if g is not None else float('nan'),
            float(g[1]) if g is not None else float('nan'),
            float(g[2]) if g is not None else float('nan'),
            dist, hdg_err_deg, depth_err,
            surge, yaw_cmd, heave,
            self._min_front_dist, len(self._path), self._wp_idx, event,
        ])


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._log.close()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
