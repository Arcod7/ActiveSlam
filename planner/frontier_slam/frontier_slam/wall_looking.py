"""Path-following motion controller with TSDF wall-looking guidance.

The planner remains responsible for goals and routes. This executor follows
that route while using the nearest TSDF wall as a soft preference for travel
and sensor orientation. The path and wall blends allow it to round poorly
resolved tips instead of treating each surface normal as an infinite wall.

Subscribed topics:
  /tsdf/surface_normals_cloud  (sensor_msgs/PointCloud2)
      fields x y z normal_x normal_y normal_z — sampled marching-cubes
      surface points with TSDF-gradient normals, from tsdf_mapper
  odometry                     (nav_msgs/Odometry, param `odom_topic`)
  /sensor_msgs/image_depth     (sensor_msgs/Image)  emergency back-off only

Published topics:
  /motion/body_command  (geometry_msgs/Twist; normalized safety-gate input)

Behaviour:
  SEARCH — no usable wall (empty/stale TSDF, or nothing wall-like within
           `max_surface_dist_m` of the robot's depth band): slow rotation +
           depth hold, letting the mapper build surface before reporting
           BLOCKED after `no_wall_timeout_s`.
  TRACK — blend planner travel with the wall tangent, blend viewing orientation
          from the wall normal toward that travel heading, and regulate
          perpendicular distance to `standoff_m`.
  SWITCH — after no path progress to a nearby planner goal, turn to observe
           the target side, exclude the trapping wall briefly, and select an
           alternative goal-aligned wall.

Because the vehicle can sway, translation is holonomic: the desired
world-frame velocity is projected onto the body axes each tick, so the
standoff/tangent control stays correct even while the heading is still
converging on the normal.

Frame conventions (NED / world_ned):
  yaw ψ from +X(N) toward +Y(E); body forward = (cos ψ, sin ψ);
  body starboard = (−sin ψ, cos ψ); positive sway = starboard.
  TSDF normals point INTO free space (VDBFusion: d<0 occupied, gradient
  points outward); any normal facing away from the robot is flipped as a
  guard against noisy gradients.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String

from frontier_slam.control_utils import wrap_angle, yaw_from_quat
from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'rx', 'ry', 'rz', 'wx', 'wy', 'wz', 'nx', 'ny',
    'd_m', 'hdg_err_deg', 'depth_err_m',
    'surge', 'sway', 'yaw_cmd', 'heave',
    'obs_m', 'n_wall_pts', 'event',
]

_NORMAL_FIELDS = ('x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z')


class WallLooking(Node):
    # P-gains
    KP_YAW      = 0.07   # same heading gain as waypoint_controller
    KP_STANDOFF = 0.50   # standoff-distance error → approach speed
    KP_HEAVE    = 0.40

    MAX_SURGE           = 0.20   # approach/retreat clamp
    MAX_SWAY            = 0.25   # tangential clamp
    SCAN_YAW            = 0.08   # rotation while no wall is in range
    Z_BAND_M            = 1.5    # only surface points within ±this of robot depth
    NORMAL_Z_MAX        = 0.7    # |n_z| above this = floor/ceiling, not a wall
    NORMAL_EMA_ALPHA    = 0.3    # smoothing on the tracked normal (nearest-point
                                 # jumps between samples cause yaw jitter otherwise)
    CLOUD_STALE_S       = 5.0    # no normals cloud for this long → SCAN
    PROGRESS_TIMEOUT_S  = 15.0   # no path-waypoint progress → BLOCKED
    PROGRESS_MIN_M      = 0.5
    WAYPOINT_ADVANCE_DIST = 1.5
    GOAL_RADIUS         = 2.0
    EMERGENCY_STOP_DIST = 0.4    # m — front camera floor; below: forced back-off
    BACK_SURGE_SPEED    = 0.12   # m/s backward during emergency back-off
    CTRL_HZ             = 10.0
    LOG_EVERY_N_TICKS   = 10     # CSV row rate = CTRL_HZ / this → 1 Hz

    def __init__(self) -> None:
        super().__init__('wall_looking')

        self.declare_parameter('standoff_m',    1.5)
        self.declare_parameter('tangent_speed', 0.15)
        self.declare_parameter('max_surface_dist_m', 8.0)
        self.declare_parameter('no_wall_timeout_s', 10.0)
        self.declare_parameter('switch_goal_distance_m', 6.0)
        self.declare_parameter('switch_scan_s', 25.0)
        self.declare_parameter('switch_scan_angle_rad', math.pi)
        self.declare_parameter('switch_scan_yaw', 0.08)
        self.declare_parameter('switch_exclusion_m', 2.5)
        self.declare_parameter('path_influence', 0.70)
        self.declare_parameter('path_look_offset_deg', 30.0)
        self.declare_parameter('wall_normal_offset_deg', 0.0)
        self.declare_parameter('path_heading_weight', 0.35)
        self.declare_parameter('progress_timeout_s', 30.0)
        # +1 = strafe to the robot's right while facing the wall, -1 = left.
        self.declare_parameter('direction',     1)
        self.declare_parameter('depth_setpoint', -1.0)
        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('normals_topic', '/tsdf/surface_normals_cloud')
        self.declare_parameter('goal_topic', '/frontier_slam/goal')
        self.declare_parameter('path_topic', '/frontier_slam/path')
        self.declare_parameter('status_topic', '/motion/status')
        self.declare_parameter('command_topic', '/motion/body_command')
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('turn_factor', 1.0)

        self._standoff   = float(self.get_parameter('standoff_m').value)
        self._tan_speed  = float(self.get_parameter('tangent_speed').value)
        self._direction  = 1 if int(self.get_parameter('direction').value) >= 0 else -1
        v = float(self.get_parameter('depth_setpoint').value)
        self._depth_setpoint: float | None = None if v < 0 else v
        odom_topic    = str(self.get_parameter('odom_topic').value)
        normals_topic = str(self.get_parameter('normals_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)

        self._pose: np.ndarray | None = None
        self._yaw  = 0.0
        self._min_front_dist = float('inf')
        self._wall_pts: np.ndarray | None = None   # (N,3) world
        self._wall_nrm: np.ndarray | None = None   # (N,3) unit
        self._cloud_t: float | None = None
        self._n_ema: np.ndarray | None = None      # smoothed planar normal (2,)
        self._goal: np.ndarray | None = None
        self._path: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._no_wall_since: float | None = None
        self._blocked_reported = False
        self._progress_target: np.ndarray | None = None
        self._progress_best_dist = float('inf')
        self._progress_since: float | None = None
        self._search_until: float | None = None
        self._search_yaw_cmd: float | None = None
        self._search_last_yaw: float | None = None
        self._search_turned_rad = 0.0
        self._switch_exclude_xy: np.ndarray | None = None
        self._switched_for_goal = False
        self._tick = 0

        self._log = open_session_log('wall_looking', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(PointCloud2, normals_topic,             self._cloud_cb, 1)
        self.create_subscription(Odometry,    odom_topic,                self._odom_cb,  10)
        self.create_subscription(Image,       '/sensor_msgs/image_depth', self._depth_cb, 1)
        self.create_subscription(PointStamped, goal_topic, self._goal_cb, 1)
        self.create_subscription(Path, path_topic, self._path_cb, 1)
        self._command_pub = self.create_publisher(Twist, command_topic, 1)
        self._status_pub = self.create_publisher(String, status_topic, 1)

        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info(
            f'wall_looking ready — standoff={self._standoff:.1f}m '
            f'tangent_speed={self._tan_speed:.2f} direction={self._direction:+d} '
            f'goal_topic={goal_topic} path_topic={path_topic} '
            f'— logging to {self._log.path}'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    def _cloud_cb(self, msg: PointCloud2) -> None:
        pts, nrm = _parse_normals_cloud(msg)
        if pts is None:
            self.get_logger().warn(
                'normals cloud missing x/y/z/normal_* fields — ignoring',
                throttle_duration_sec=10.0)
            return
        self._wall_pts = pts
        self._wall_nrm = nrm
        self._cloud_t  = self._t_ros()

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        new_yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self._search_last_yaw is not None:
            self._search_turned_rad += abs(wrap_angle(new_yaw - self._search_last_yaw))
        self._yaw = new_yaw
        if self._search_last_yaw is not None:
            self._search_last_yaw = new_yaw
        if self._depth_setpoint is None:
            self._depth_setpoint = float(p.z)
            self.get_logger().info(
                f'depth setpoint locked from odom at {self._depth_setpoint:.2f} m')

    def _depth_cb(self, msg: Image) -> None:
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        v = data[np.isfinite(data) & (data > 0.1)]
        self._min_front_dist = float(v.min()) if v.size > 0 else float('inf')

    def _goal_cb(self, msg: PointStamped) -> None:
        new_goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self._goal is None or np.hypot(*(new_goal[:2] - self._goal[:2])) > 1.0:
            self._path = []
            self._wp_idx = 0
            self._no_wall_since = None
            self._blocked_reported = False
            self._progress_target = None
            self._progress_best_dist = float('inf')
            self._progress_since = None
            self._search_until = None
            self._search_yaw_cmd = None
            self._search_last_yaw = None
            self._search_turned_rad = 0.0
            self._switch_exclude_xy = None
            self._switched_for_goal = False
        self._goal = new_goal

    def _path_cb(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self._path and self._pose is not None:
            dists = [math.hypot(x - self._pose[0], y - self._pose[1])
                     for x, y in self._path]
            self._wp_idx = int(np.argmin(dists))

    def _target_xy(self) -> np.ndarray | None:
        if self._goal is None:
            return None
        while (self._wp_idx < len(self._path) - 1 and
               math.hypot(self._path[self._wp_idx][0] - self._pose[0],
                          self._path[self._wp_idx][1] - self._pose[1])
               < self.WAYPOINT_ADVANCE_DIST):
            self._wp_idx += 1
        return np.array(self._path[self._wp_idx]) if self._path else self._goal[:2]

    def _report_blocked(self, reason: str) -> None:
        if self._blocked_reported:
            return
        self._status_pub.publish(String(data=f'BLOCKED:{reason}'))
        self._blocked_reported = True
        self.get_logger().warn(f'BLOCKED: {reason}; planner must choose a new path/goal')

    def _path_progress_blocked(self, target_xy: np.ndarray, now: float) -> bool:
        """Return true when wall motion cannot advance the planner's waypoint."""
        target_changed = (self._progress_target is None or
                          np.hypot(*(target_xy - self._progress_target)) > 1.0)
        dist = float(np.hypot(*(target_xy - self._pose[:2])))
        if target_changed:
            self._progress_target = target_xy.copy()
            self._progress_best_dist = dist
            self._progress_since = now
            return False
        if dist < self._progress_best_dist - self.PROGRESS_MIN_M:
            self._progress_best_dist = dist
            self._progress_since = now
            return False
        timeout = float(self.get_parameter('progress_timeout_s').value)
        return now - self._progress_since >= timeout

    def _t_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Wall selection
    def _nearest_wall(self, target_xy: np.ndarray | None = None) -> 'tuple[np.ndarray, np.ndarray, int] | None':
        """Select a usable wall, optionally preferring one aligned to a target.

        Normal tracking normally uses the nearest wall. During a short
        wall-switch search the previous wall is excluded and candidates whose
        tangents point toward the planner waypoint are preferred.
        """
        if self._wall_pts is None or self._cloud_t is None:
            return None
        if self._t_ros() - self._cloud_t > self.CLOUD_STALE_S:
            return None

        pts, nrm = self._wall_pts, self._wall_nrm
        # Wall-like only: near the robot's depth band, mostly-horizontal normal
        # (rules out seafloor / hull-top hits whose normals point up/down).
        mask = ((np.abs(pts[:, 2] - self._pose[2]) <= self.Z_BAND_M) &
                (np.abs(nrm[:, 2]) <= self.NORMAL_Z_MAX))
        if not np.any(mask):
            return None

        pts, nrm = pts[mask], nrm[mask]
        d_xy = np.hypot(pts[:, 0] - self._pose[0], pts[:, 1] - self._pose[1])
        max_surface_dist = float(self.get_parameter('max_surface_dist_m').value)
        valid = d_xy <= max_surface_dist
        if self._switch_exclude_xy is not None:
            exclusion = float(self.get_parameter('switch_exclusion_m').value)
            valid &= np.hypot(pts[:, 0] - self._switch_exclude_xy[0],
                              pts[:, 1] - self._switch_exclude_xy[1]) > exclusion
        if not np.any(valid):
            return None

        candidates = np.flatnonzero(valid)
        if target_xy is None or self._switch_exclude_xy is None:
            i = int(candidates[np.argmin(d_xy[candidates])])
        else:
            to_target = target_xy - self._pose[:2]
            target_norm = float(np.hypot(*to_target))
            if target_norm < 1e-6:
                i = int(candidates[np.argmin(d_xy[candidates])])
            else:
                normals_xy = nrm[candidates, :2].astype(np.float64)
                normal_norms = np.hypot(normals_xy[:, 0], normals_xy[:, 1])
                tangents = np.column_stack((normals_xy[:, 1], -normals_xy[:, 0]))
                alignment = np.zeros(len(candidates))
                usable = normal_norms > 1e-6
                tangents[usable] /= normal_norms[usable, None]
                alignment[usable] = np.abs(tangents[usable] @ (to_target / target_norm))
                # Alignment dominates; nearer equivalent surfaces win.
                score = alignment - 0.05 * d_xy[candidates] / max_surface_dist
                i = int(candidates[np.argmax(score)])

        n_xy = nrm[i, :2].astype(np.float64)
        norm = float(np.hypot(n_xy[0], n_xy[1]))
        if norm < 1e-6:
            return None
        n_xy /= norm

        # Orientation guard: the TSDF gradient should already point into free
        # space (toward the robot's side); flip if a noisy gradient doesn't.
        to_robot = self._pose[:2] - pts[i, :2]
        if float(np.dot(to_robot, n_xy)) < 0.0:
            n_xy = -n_xy

        return pts[i], n_xy, len(candidates)

    def _search_yaw(self, target_xy: np.ndarray | None) -> float:
        """Turn toward the requested route while scanning for another wall."""
        if self._search_yaw_cmd is not None:
            return self._search_yaw_cmd
        if target_xy is None:
            return self.SCAN_YAW
        heading = math.atan2(target_xy[1] - self._pose[1], target_xy[0] - self._pose[0])
        err = wrap_angle(heading - self._yaw)
        return self.SCAN_YAW if err >= 0.0 else -self.SCAN_YAW

    def _start_wall_switch(self, wall_point: np.ndarray, target_xy: np.ndarray) -> None:
        """Pause translation, inspect the target side, then select another wall."""
        now = self._t_ros()
        self._search_until = now + float(self.get_parameter('switch_scan_s').value)
        scan_speed = float(self.get_parameter('switch_scan_yaw').value)
        heading = math.atan2(target_xy[1] - self._pose[1], target_xy[0] - self._pose[0])
        self._search_yaw_cmd = scan_speed if wrap_angle(heading - self._yaw) >= 0.0 else -scan_speed
        self._search_last_yaw = self._yaw
        self._search_turned_rad = 0.0
        self._switch_exclude_xy = wall_point[:2].copy()
        self._switched_for_goal = True
        self._progress_target = None
        self._progress_since = None
        self._n_ema = None
        self.get_logger().info(
            f'Wall switch: goal is {np.hypot(*(target_xy - self._pose[:2])):.1f}m away; '
            f'turning {math.degrees(float(self.get_parameter("switch_scan_angle_rad").value)):.0f}° '
            'to inspect and select another wall')

    # ------------------------------------------------------------------
    # Main loop
    def _loop(self) -> None:
        self._tick += 1
        write_csv = (self._tick % self.LOG_EVERY_N_TICKS == 0)

        if self._pose is None:
            return

        heave = self._heave_cmd()
        target_xy = self._target_xy()
        now = self._t_ros()

        # At startup, and while changing walls, rotate to make the TSDF observe
        # the surroundings instead of holding a fixed view of empty water or
        # the wall that just prevented route progress.
        scan_angle = float(self.get_parameter('switch_scan_angle_rad').value)
        scan_complete = self._search_turned_rad >= scan_angle
        if self._search_until is not None and now < self._search_until and not scan_complete:
            yaw = self._search_yaw(target_xy)
            self._send_thrust(0.0, yaw, heave, 0.0)
            if write_csv:
                self._write_csv(0.0, 0.0, yaw, heave, 'WALL_SWITCH_SCAN')
            return
        if self._search_until is not None:
            if scan_complete:
                self.get_logger().info(
                    f'Wall switch scan completed after {math.degrees(self._search_turned_rad):.0f}°')
            else:
                self.get_logger().warn(
                    f'Wall switch scan timed out after {math.degrees(self._search_turned_rad):.0f}°')
            self._search_until = None
            self._search_yaw_cmd = None
            self._search_last_yaw = None
            self._search_turned_rad = 0.0

        if target_xy is None:
            wall = self._nearest_wall()
            yaw = 0.0 if wall is not None else self.SCAN_YAW
            self._send_thrust(0.0, yaw, heave, 0.0)
            if write_csv:
                self._write_csv(0.0, 0.0, yaw, heave,
                                'IDLE' if wall is not None else 'STARTUP_SCAN')
            return

        goal_dist = float(np.hypot(self._goal[0] - self._pose[0],
                                   self._goal[1] - self._pose[1]))
        if goal_dist < self.GOAL_RADIUS:
            self._send_thrust(0.0, 0.0, heave, 0.0)
            if write_csv:
                self._write_csv(0.0, 0.0, 0.0, heave, 'GOAL_REACHED')
            return

        wall = self._nearest_wall(target_xy)

        if wall is None:
            # Rotate to acquire structure. The planner only receives BLOCKED
            # after this bounded observation attempt has failed.
            self._n_ema = None
            yaw = self._search_yaw(target_xy)
            self._send_thrust(0.0, yaw, heave, 0.0)
            if self._no_wall_since is None:
                self._no_wall_since = now
            elif now - self._no_wall_since >= float(self.get_parameter('no_wall_timeout_s').value):
                self._report_blocked('NO_WALL')
            self.get_logger().info('No wall for requested path — scanning',
                                   throttle_duration_sec=2.0)
            if write_csv:
                self._write_csv(0.0, 0.0, yaw, heave, 'NO_WALL_SCAN')
            return

        self._no_wall_since = None

        # A wall-switch search excludes the trapping wall for one selection.
        # Once another wall is acquired, normal nearest-wall tracking resumes.
        if self._switch_exclude_xy is not None:
            self.get_logger().info('Wall switch selected an alternative wall')
            self._switch_exclude_xy = None

        if self._path_progress_blocked(target_xy, now):
            switch_dist = float(self.get_parameter('switch_goal_distance_m').value)
            if not self._switched_for_goal and goal_dist <= switch_dist:
                self._start_wall_switch(wall[0], target_xy)
                yaw = self._search_yaw(target_xy)
                self._send_thrust(0.0, yaw, heave, 0.0)
                if write_csv:
                    self._write_csv(0.0, 0.0, yaw, heave, 'WALL_SWITCH_SCAN')
                return
            self._send_thrust(0.0, 0.0, heave, 0.0)
            self._report_blocked('NO_PATH_PROGRESS')
            if write_csv:
                self._write_csv(0.0, 0.0, 0.0, heave, 'NO_PATH_PROGRESS')
            return

        wp, n_raw, n_pts = wall

        # Smooth the tracked normal: the nearest sample hops between
        # marching-cubes vertices as the robot moves, and raw hops = yaw jitter.
        if self._n_ema is None:
            self._n_ema = n_raw
        else:
            blended = (1.0 - self.NORMAL_EMA_ALPHA) * self._n_ema \
                      + self.NORMAL_EMA_ALPHA * n_raw
            norm = float(np.hypot(blended[0], blended[1]))
            self._n_ema = blended / norm if norm > 1e-6 else n_raw
        n = self._n_ema

        # Perpendicular distance to the wall plane through the nearest sample.
        d = float(np.dot(self._pose[:2] - wp[:2], n))
        standoff = float(self.get_parameter('standoff_m').value)

        # Start from wall-tangent travel, then blend in the planner direction.
        # This lets the vehicle round low-resolution tips instead of treating
        # every smoothed normal as a perfectly flat, infinite wall.
        tangent = np.array([n[1], -n[0]])
        target_delta = target_xy - self._pose[:2]
        tangent_dot = float(np.dot(tangent, target_delta))
        tangent_sign = (1.0 if tangent_dot > 1e-6 else
                        -1.0 if tangent_dot < -1e-6 else float(self._direction))
        wall_travel = tangent_sign * tangent
        path_travel = _unit_xy(target_delta)
        path_influence = float(np.clip(self.get_parameter('path_influence').value, 0.0, 1.0))
        travel = _unit_xy((1.0 - path_influence) * wall_travel + path_influence * path_travel)
        if not np.any(travel):
            travel = wall_travel

        # Orientation has two independently tunable candidates:
        #   path-look: start on the raw planner bearing, turn toward the wall;
        #   wall-look: start facing the wall normal, turn toward the path.
        # The final weight is circular interpolation between those headings.
        route_heading = math.atan2(path_travel[1], path_travel[0])
        wall_normal_heading = math.atan2(-n[1], -n[0])
        path_look_heading = _offset_heading_towards(
            route_heading, wall_normal_heading,
            float(self.get_parameter('path_look_offset_deg').value))
        wall_look_heading = _offset_heading_towards(
            wall_normal_heading, route_heading,
            float(self.get_parameter('wall_normal_offset_deg').value))
        path_heading_weight = float(np.clip(
            self.get_parameter('path_heading_weight').value, 0.0, 1.0))
        yaw_des = _blended_heading(
            wall_look_heading, path_look_heading, path_heading_weight)
        hdg_err = wrap_angle(yaw_des - self._yaw)
        yaw_cmd = float(np.clip(self.KP_YAW * hdg_err, -1.0, 1.0))

        # Standoff correction is perpendicular to the selected wall; route
        # travel remains planner-influenced instead of a constant wall orbit.
        v_des = (self.KP_STANDOFF * (d - standoff) * -n
                 + self._tan_speed * travel)

        fwd  = np.array([math.cos(self._yaw), math.sin(self._yaw)])
        stbd = np.array([-math.sin(self._yaw), math.cos(self._yaw)])
        surge = float(np.clip(np.dot(v_des, fwd),  -self.MAX_SURGE, self.MAX_SURGE))
        sway  = float(np.clip(np.dot(v_des, stbd), -self.MAX_SWAY,  self.MAX_SWAY))

        event = 'TRACK'
        # Last-resort floor from the forward camera.  No progressive ramp here:
        # the standoff P-controller is the primary distance regulator and a ramp
        # would fight it at standoff distances near the ramp threshold.
        if self._min_front_dist < self.EMERGENCY_STOP_DIST:
            surge = -self.BACK_SURGE_SPEED
            event = 'EMERG_STOP'

        self._send_thrust(surge, yaw_cmd, heave, sway)

        self.get_logger().info(
            f'pos=({self._pose[0]:.1f},{self._pose[1]:.1f},{self._pose[2]:.1f}) '
            f'wall=({wp[0]:.1f},{wp[1]:.1f})  d={d:.2f}m (target {standoff:.1f}) '
            f'path_wp=({target_xy[0]:.1f},{target_xy[1]:.1f}) {self._wp_idx}/{len(self._path)} '
            f'hdg_err={math.degrees(hdg_err):+.0f}° path_w={path_heading_weight:.2f}  '
            f'surge={surge:+.2f}  sway={sway:+.2f}  yaw={yaw_cmd:+.3f}  '
            f'heave={heave:+.2f}  obs={self._min_front_dist:.1f}m'
            + ('  [EMERG_STOP]' if event == 'EMERG_STOP' else ''),
            throttle_duration_sec=2.0,
        )

        if write_csv:
            self._write_csv(surge, sway, yaw_cmd, heave, event,
                            wall_pt=wp, n=n, d=d,
                            hdg_err_deg=math.degrees(hdg_err), n_pts=n_pts)

    # ------------------------------------------------------------------
    # Control primitives / output
    def _heave_cmd(self) -> float:
        """Hold the depth setpoint. Negative output = upward thrust in Stonefish."""
        if self._depth_setpoint is None:
            return 0.0
        depth_err = self._pose[2] - self._depth_setpoint    # +ve = too deep
        return float(np.clip(-self.KP_HEAVE * depth_err, -1.0, 1.0))

    # Matches safety_gate.py's max_abs_command default (1.0): the gate rejects
    # (and latches INVALID_COMMAND on) any out-of-range component, so a
    # speed_factor/turn_factor above 1x must saturate here, not there.
    MAX_ABS_COMMAND = 1.0

    def _send_thrust(self, surge: float, yaw: float, heave: float, sway: float) -> None:
        """Single publish choke point — applies the operator speed/turn factors
        (live-tunable from the launcher TUI) uniformly to every caller."""
        speed_factor = float(self.get_parameter('speed_factor').value)
        turn_factor = float(self.get_parameter('turn_factor').value)
        cap = self.MAX_ABS_COMMAND
        msg = Twist()
        msg.linear.x = float(np.clip(surge * speed_factor, -cap, cap))
        msg.linear.y = float(np.clip(sway * speed_factor, -cap, cap))
        msg.linear.z = float(np.clip(heave * speed_factor, -cap, cap))
        msg.angular.z = float(np.clip(yaw * turn_factor, -cap, cap))
        self._command_pub.publish(msg)

    def _write_csv(self, surge, sway, yaw_cmd, heave, event,
                   wall_pt=None, n=None, d=float('nan'),
                   hdg_err_deg=float('nan'), n_pts=0) -> None:
        p  = self._pose
        depth_err = ((p[2] - self._depth_setpoint)
                     if self._depth_setpoint is not None else float('nan'))
        self._log.write([
            self._t_ros(),
            float(p[0]), float(p[1]), float(p[2]),
            float(wall_pt[0]) if wall_pt is not None else float('nan'),
            float(wall_pt[1]) if wall_pt is not None else float('nan'),
            float(wall_pt[2]) if wall_pt is not None else float('nan'),
            float(n[0]) if n is not None else float('nan'),
            float(n[1]) if n is not None else float('nan'),
            d, hdg_err_deg, depth_err,
            surge, sway, yaw_cmd, heave,
            self._min_front_dist, n_pts, event,
        ])


# ══════════════════════════════════════════════════════════════════════════════
# Module-level helpers
# ══════════════════════════════════════════════════════════════════════════════

def _unit_xy(v: np.ndarray) -> np.ndarray:
    norm = float(np.hypot(v[0], v[1]))
    return v / norm if norm > 1e-9 else np.zeros(2)


def _blended_heading(wall_heading: float, path_heading: float,
                     path_influence: float) -> float:
    """Interpolate smoothly from wall normal (0) to route heading (1)."""
    return wrap_angle(wall_heading + path_influence *
                      wrap_angle(path_heading - wall_heading))


def _offset_heading_towards(source_heading: float, target_heading: float,
                            offset_deg: float) -> float:
    """Move `source_heading` toward `target_heading` by at most `offset_deg`.

    The offset is an unsigned magnitude in [0, 180]. The shortest circular
    direction supplies the sign, so the same configuration works with walls
    on either side of the planned route.
    """
    max_offset = math.radians(float(np.clip(offset_deg, 0.0, 180.0)))
    delta = wrap_angle(target_heading - source_heading)
    applied = math.copysign(min(abs(delta), max_offset), delta)
    return wrap_angle(source_heading + applied)


def _parse_normals_cloud(msg: PointCloud2) -> 'tuple[np.ndarray, np.ndarray] | tuple[None, None]':
    """Parse a float32 PointCloud2 with x,y,z,normal_x,normal_y,normal_z fields.

    Returns (points (N,3), normals (N,3)) as float32, or (None, None) if the
    expected fields are missing.  N may be 0.
    """
    offsets = {f.name: f.offset for f in msg.fields
               if f.datatype == PointField.FLOAT32}
    if (any(k not in offsets for k in _NORMAL_FIELDS)
            or msg.point_step % 4 != 0 or msg.point_step < 24):
        return None, None
    floats = msg.point_step // 4
    data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats)
    cols = [offsets[k] // 4 for k in _NORMAL_FIELDS]
    arr  = data[:, cols]
    return arr[:, :3].copy(), arr[:, 3:].copy()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallLooking()
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
