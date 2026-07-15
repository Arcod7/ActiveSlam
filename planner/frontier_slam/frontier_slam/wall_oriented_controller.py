"""
Forward path controller with a fixed viewing offset toward the nearest wall.

Unlike ``wall_follower``, this controller does not estimate wall normals,
regulate standoff, or alter the planner route.  It follows the same path as
the ordinary waypoint controller while yawing ``look_offset_deg`` to the left
or right of the route bearing.  The nearest occupied/surface point in the map
chooses the sign of that offset.

The BlueROV2 is holonomic in the horizontal plane.  Route velocity is therefore
projected onto body surge and sway, allowing the vehicle to keep travelling
along the planned path while its camera looks slightly toward the wall.
"""
import math
import os

from frontier_slam.control_utils import mix_thrusters, wrap_angle, yaw_from_quat
from frontier_slam.session_log import open_session_log
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'rx', 'ry', 'rz', 'gx', 'gy', 'gz',
    'dist_m', 'route_hdg_deg', 'look_hdg_deg', 'hdg_err_deg',
    'wall_side', 'wall_dist_m', 'depth_err_m',
    'surge', 'sway', 'yaw_cmd', 'heave',
    'obs_m', 'path_len', 'wp_idx', 'event',
]


def offset_heading(route_heading: float, wall_side: int, offset_deg: float) -> float:
    """Offset route heading toward the wall (-1 left, +1 right, 0 absent)."""
    side = 0 if wall_side == 0 else (1 if wall_side > 0 else -1)
    offset = math.radians(float(np.clip(offset_deg, 0.0, 89.0)))
    return wrap_angle(route_heading + side * offset)


def wall_side_distances(points: np.ndarray, pose: np.ndarray,
                        route_heading: float, z_band_m: float,
                        max_distance_m: float,
                        lateral_deadband_m: float = 0.1) -> tuple[float, float]:
    """
    Return nearest (left, right) planar wall distances relative to the route.

    Points close to the route centreline do not provide a reliable side and are
    ignored.  Positive lateral displacement is starboard/right in the NED XY
    convention used by this project.
    """
    if points is None or len(points) == 0:
        return float('inf'), float('inf')

    pts = np.asarray(points, dtype=np.float64)
    p = np.asarray(pose, dtype=np.float64)
    finite = np.isfinite(pts).all(axis=1)
    depth_ok = np.abs(pts[:, 2] - p[2]) <= z_band_m
    delta = pts[:, :2] - p[:2]
    distance = np.hypot(delta[:, 0], delta[:, 1])
    valid = finite & depth_ok & (distance <= max_distance_m)
    if not np.any(valid):
        return float('inf'), float('inf')

    # Starboard unit vector for a route bearing in world NED.
    right = np.array([-math.sin(route_heading), math.cos(route_heading)])
    lateral = delta @ right
    left_mask = valid & (lateral < -lateral_deadband_m)
    right_mask = valid & (lateral > lateral_deadband_m)
    left_dist = float(np.min(distance[left_mask])) if np.any(left_mask) else float('inf')
    right_dist = float(np.min(distance[right_mask])) if np.any(right_mask) else float('inf')
    return left_dist, right_dist


def choose_wall_side(left_distance: float, right_distance: float,
                     current_side: int = 0,
                     switch_margin_m: float = 0.3) -> int:
    """Choose the nearest side, retaining the current side within a margin."""
    left_ok = math.isfinite(left_distance)
    right_ok = math.isfinite(right_distance)
    if not left_ok and not right_ok:
        return 0
    if not left_ok:
        return 1
    if not right_ok:
        return -1

    margin = max(0.0, switch_margin_m)
    if current_side < 0 and right_distance + margin >= left_distance:
        return -1
    if current_side > 0 and left_distance + margin >= right_distance:
        return 1
    return -1 if left_distance <= right_distance else 1


def lookahead_path_heading(path: list[tuple[float, float]], waypoint_index: int,
                           robot_xy: np.ndarray, radius_m: float,
                           fallback_heading: float) -> tuple[np.ndarray, float]:
    """
    Return the first forward path/circle intersection and its tangent heading.

    A zero radius deliberately returns the current route heading, preserving
    the controller's original behaviour.  For a positive radius, the first
    intersection reached while walking forward along the path is used.  The
    virtual first segment from the robot to its current waypoint covers a
    robot that has drifted slightly off the discretised A* path.
    """
    if radius_m <= 0.0 or not path or waypoint_index >= len(path):
        return np.asarray(robot_xy, dtype=np.float64), fallback_heading

    centre = np.asarray(robot_xy, dtype=np.float64)
    points = [centre] + [np.asarray(p, dtype=np.float64)
                         for p in path[max(0, waypoint_index):]]
    radius_sq = radius_m * radius_m
    for start, end in zip(points, points[1:]):
        direction = end - start
        length_sq = float(direction @ direction)
        if length_sq < 1e-12:
            continue
        relative = start - centre
        roots = np.roots([
            length_sq,
            2.0 * float(relative @ direction),
            float(relative @ relative) - radius_sq,
        ])
        candidates = sorted(
            float(root.real) for root in roots
            if abs(root.imag) < 1e-9 and 1e-8 < root.real <= 1.0 + 1e-8)
        if candidates:
            t = min(candidates[0], 1.0)
            point = start + t * direction
            return point, math.atan2(direction[1], direction[0])
    return centre, fallback_heading


def parse_xyz_cloud(msg: PointCloud2) -> 'np.ndarray | None':
    """Parse finite XYZ points from an arbitrary float32 PointCloud2 layout."""
    offsets = {
        field.name: field.offset for field in msg.fields
        if field.datatype == PointField.FLOAT32
    }
    if (any(name not in offsets for name in ('x', 'y', 'z'))
            or msg.point_step <= 0 or msg.width == 0 or msg.height == 0):
        return None

    byte_order = '>' if msg.is_bigendian else '<'
    cloud_dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [byte_order + 'f4'] * 3,
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': msg.point_step,
    })
    try:
        data = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=cloud_dtype,
            buffer=msg.data,
            strides=(msg.row_step, msg.point_step),
        )
    except (TypeError, ValueError):
        return None
    xyz = np.column_stack((data['x'].ravel(), data['y'].ravel(), data['z'].ravel()))
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64, copy=False)


class WallOrientedController(Node):
    KP_YAW = 0.07
    KP_SPEED = 0.35
    KP_HEAVE = 0.40

    MAX_SPEED = 0.35
    GOAL_RADIUS = 2.0
    GOAL_REACHED_TIMEOUT = 10.0
    SCAN_YAW = 0.08
    INIT_SCAN_DURATION = 10.0
    WAYPOINT_ADVANCE_DIST = 1.5
    OBS_SLOW_DIST = 1.5
    EMERGENCY_STOP_DIST = 0.4
    BACK_SURGE_SPEED = 0.12
    ESCAPE_YAW = 0.20
    ESCAPE_DURATION = 4.0
    STUCK_SPEED_MIN = 0.15
    STUCK_WINDOW = 5.0
    STUCK_MOVE_MIN = 0.25
    MAP_STALE_S = 5.0
    CTRL_HZ = 10.0
    LOG_EVERY_N_TICKS = 10

    def __init__(self) -> None:
        super().__init__('wall_oriented_controller')

        self.declare_parameter('depth_setpoint', -1.0)
        self.declare_parameter('look_offset_deg', 30.0)
        self.declare_parameter('lookahead_m', 0.0)
        self.declare_parameter('speed_scale', 0.9)
        self.declare_parameter('map_points_topic', '/octomap_point_cloud_centers')
        self.declare_parameter('max_wall_distance_m', 8.0)
        self.declare_parameter('wall_z_band_m', 1.5)
        self.declare_parameter('side_switch_margin_m', 0.3)
        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('goal_topic', '/frontier_slam/goal')
        self.declare_parameter('path_topic', '/frontier_slam/path')
        self.declare_parameter('thruster_topic', '/bluerov2/controller/thruster_setpoints_sim')

        depth = float(self.get_parameter('depth_setpoint').value)
        self._depth_setpoint: float | None = None if depth < 0 else depth
        self._look_offset_deg = float(np.clip(
            self.get_parameter('look_offset_deg').value, 0.0, 89.0))
        self._lookahead_m = max(0.0, float(self.get_parameter('lookahead_m').value))
        self._speed_scale = float(np.clip(
            self.get_parameter('speed_scale').value, 0.0, 1.0))
        self._max_wall_distance = float(self.get_parameter('max_wall_distance_m').value)
        self._wall_z_band = float(self.get_parameter('wall_z_band_m').value)
        self._side_switch_margin = float(self.get_parameter('side_switch_margin_m').value)
        map_topic = str(self.get_parameter('map_points_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        thruster_topic = str(self.get_parameter('thruster_topic').value)

        self._goal: np.ndarray | None = None
        self._pose: np.ndarray | None = None
        self._yaw = 0.0
        self._path: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._map_points: np.ndarray | None = None
        self._map_received_at: float | None = None
        self._wall_side = 0
        self._wall_distance = float('nan')
        self._min_front_dist = float('inf')
        self._init_scan_end: float | None = None
        self._goal_reached_at: float | None = None
        self._escape_until: float | None = None
        self._stuck_ref_pos: np.ndarray | None = None
        self._stuck_ref_t: float | None = None
        self._tick = 0

        self._log = open_session_log('wall_oriented', CSV_COLUMNS, _LOG_DIR)
        self.create_subscription(PointStamped, goal_topic, self._goal_cb, 1)
        self.create_subscription(Path, path_topic, self._path_cb, 1)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Image, '/sensor_msgs/image_depth', self._depth_cb, 1)
        self.create_subscription(PointCloud2, map_topic, self._map_cb, 1)
        self._thrust_pub = self.create_publisher(Float64MultiArray, thruster_topic, 1)
        self.create_timer(1.0 / self.CTRL_HZ, self._loop)

        self.get_logger().info(
            f'wall_oriented_controller ready — offset={self._look_offset_deg:.1f}deg '
            f'lookahead={self._lookahead_m:.1f}m speed={self._speed_scale:.0%} '
            f'map={map_topic} goal={goal_topic} path={path_topic} '
            f'— logging to {self._log.path}')

    def _t_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_cb(self, msg: PointCloud2) -> None:
        points = parse_xyz_cloud(msg)
        if points is None:
            self.get_logger().warn(
                'map cloud has no readable float32 x/y/z fields — ignoring',
                throttle_duration_sec=10.0)
            return
        self._map_points = points
        self._map_received_at = self._t_ros()

    def _depth_cb(self, msg: Image) -> None:
        try:
            data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(
                msg.height, msg.width)
        except ValueError:
            return
        valid = data[np.isfinite(data) & (data > 0.1)]
        self._min_front_dist = float(valid.min()) if valid.size else float('inf')

    def _goal_cb(self, msg: PointStamped) -> None:
        new_goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        changed = (self._goal is None or
                   np.hypot(*(new_goal[:2] - self._goal[:2])) > 1.0)
        self._goal = new_goal
        if changed:
            self._goal_reached_at = None
            self._path = []
            self._wp_idx = 0
            self._stuck_ref_pos = None
            self._stuck_ref_t = None

    def _path_cb(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not self._path or self._pose is None:
            self._wp_idx = 0
            return
        distances = [math.hypot(x - self._pose[0], y - self._pose[1])
                     for x, y in self._path]
        self._wp_idx = int(np.argmin(distances))

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self._init_scan_end is None:
            if self._depth_setpoint is None:
                self._depth_setpoint = float(p.z)
                self.get_logger().info(
                    f'depth setpoint locked from odom at {self._depth_setpoint:.2f} m')
            self._init_scan_end = self._t_ros() + self.INIT_SCAN_DURATION
            self.get_logger().info(
                f'initial {self.INIT_SCAN_DURATION:.0f}s scan starting')

    def _heave_cmd(self) -> float:
        if self._depth_setpoint is None:
            return 0.0
        error = self._pose[2] - self._depth_setpoint
        return float(np.clip(-self.KP_HEAVE * error, -1.0, 1.0))

    def _select_wall_side(self, route_heading: float, now: float) -> None:
        if (self._map_points is None or self._map_received_at is None
                or now - self._map_received_at > self.MAP_STALE_S):
            self._wall_side = 0
            self._wall_distance = float('nan')
            return
        left, right = wall_side_distances(
            self._map_points, self._pose, route_heading,
            self._wall_z_band, self._max_wall_distance)
        self._wall_side = choose_wall_side(
            left, right, self._wall_side, self._side_switch_margin)
        chosen = left if self._wall_side < 0 else right
        self._wall_distance = chosen if math.isfinite(chosen) else float('nan')

    def _drive(self, target_xy: np.ndarray, now: float) -> tuple:
        delta = target_xy - self._pose[:2]
        distance = float(np.hypot(*delta))
        travel_heading = math.atan2(delta[1], delta[0])
        _, look_path_heading = lookahead_path_heading(
            self._path, self._wp_idx, self._pose[:2], self._lookahead_m,
            travel_heading)
        self._select_wall_side(look_path_heading, now)
        look_heading = offset_heading(
            look_path_heading, self._wall_side, self._look_offset_deg)
        heading_error = wrap_angle(look_heading - self._yaw)
        yaw_cmd = float(np.clip(self.KP_YAW * heading_error, -1.0, 1.0))

        # Reduce travel while the requested viewing heading is far away, then
        # project the unchanged route velocity onto the current body axes.
        alignment = max(0.0, math.cos(heading_error))
        speed = float(np.clip(self.KP_SPEED * self._speed_scale * distance * alignment,
                              0.0, self.MAX_SPEED * self._speed_scale))
        route_in_body = wrap_angle(travel_heading - self._yaw)
        surge = speed * math.cos(route_in_body)
        sway = speed * math.sin(route_in_body)
        return (surge, sway, yaw_cmd, distance, travel_heading, look_heading,
                heading_error, look_path_heading)

    def _loop(self) -> None:
        self._tick += 1
        write_csv = self._tick % self.LOG_EVERY_N_TICKS == 0
        if self._pose is None:
            return

        now = self._t_ros()
        heave = self._heave_cmd()
        if self._init_scan_end is not None and now < self._init_scan_end:
            self._send_thrust(0.0, 0.0, self.SCAN_YAW, heave)
            if write_csv:
                self._write_csv(0.0, 0.0, self.SCAN_YAW, heave, 'INIT_SCAN')
            return
        if self._goal is None:
            self._send_thrust(0.0, 0.0, self.SCAN_YAW, heave)
            if write_csv:
                self._write_csv(0.0, 0.0, self.SCAN_YAW, heave, 'SCAN')
            return

        goal_dist = float(np.hypot(*(self._goal[:2] - self._pose[:2])))
        if goal_dist < self.GOAL_RADIUS:
            if self._goal_reached_at is None:
                self._goal_reached_at = now
            elif now - self._goal_reached_at > self.GOAL_REACHED_TIMEOUT:
                self._goal = None
                self._goal_reached_at = None
            self._send_thrust(0.0, 0.0, self.SCAN_YAW, heave)
            if write_csv:
                self._write_csv(0.0, 0.0, self.SCAN_YAW, heave,
                                'GOAL_REACHED', distance=goal_dist)
            return

        if self._path:
            while (self._wp_idx < len(self._path) - 1 and
                   math.hypot(self._path[self._wp_idx][0] - self._pose[0],
                              self._path[self._wp_idx][1] - self._pose[1])
                   < self.WAYPOINT_ADVANCE_DIST):
                self._wp_idx += 1
            target_xy = np.array(self._path[self._wp_idx])
        else:
            target_xy = self._goal[:2]

        (surge, sway, yaw_cmd, _, route_heading,
         look_heading, heading_error, look_path_heading) = self._drive(target_xy, now)
        event = ''
        if self._min_front_dist < self.EMERGENCY_STOP_DIST:
            surge, sway = -self.BACK_SURGE_SPEED, 0.0
            event = 'EMERG_STOP'
        elif self._min_front_dist < self.OBS_SLOW_DIST:
            factor = ((self._min_front_dist - self.EMERGENCY_STOP_DIST) /
                      (self.OBS_SLOW_DIST - self.EMERGENCY_STOP_DIST))
            surge *= factor
            sway *= factor

        if self._escape_until is not None:
            if now < self._escape_until:
                self._send_thrust(0.0, 0.0, self.ESCAPE_YAW, heave)
                if write_csv:
                    self._write_csv(0.0, 0.0, self.ESCAPE_YAW, heave,
                                    'CTRL_STUCK_ESCAPE', goal_dist,
                                    route_heading, look_heading, heading_error)
                return
            self._escape_until = None
            self._stuck_ref_pos = None
            self._stuck_ref_t = None

        planar_speed = math.hypot(surge, sway)
        if planar_speed >= self.STUCK_SPEED_MIN:
            if self._stuck_ref_pos is None:
                self._stuck_ref_pos = self._pose.copy()
                self._stuck_ref_t = now
            else:
                moved = float(np.hypot(*(self._pose[:2] - self._stuck_ref_pos[:2])))
                if moved >= self.STUCK_MOVE_MIN:
                    self._stuck_ref_pos = self._pose.copy()
                    self._stuck_ref_t = now
                elif now - self._stuck_ref_t >= self.STUCK_WINDOW:
                    self._escape_until = now + self.ESCAPE_DURATION
                    self._stuck_ref_pos = None
                    self._stuck_ref_t = None
                    self._send_thrust(0.0, 0.0, self.ESCAPE_YAW, heave)
                    event = 'CTRL_STUCK'
                    return
        else:
            self._stuck_ref_pos = None
            self._stuck_ref_t = None

        self._send_thrust(surge, sway, yaw_cmd, heave)
        side_name = {-1: 'left', 0: 'none', 1: 'right'}[self._wall_side]
        self.get_logger().info(
            f'goal_dist={goal_dist:.1f}m wp={self._wp_idx}/{len(self._path)} '
            f'wall={side_name}@{self._wall_distance:.1f}m '
            f'route={math.degrees(route_heading):+.0f}deg '
            f'path_look={math.degrees(look_path_heading):+.0f}deg '
            f'look={math.degrees(look_heading):+.0f}deg '
            f'surge={surge:+.2f} sway={sway:+.2f} yaw={yaw_cmd:+.3f}',
            throttle_duration_sec=2.0)
        if write_csv:
            self._write_csv(surge, sway, yaw_cmd, heave, event, goal_dist,
                            route_heading, look_heading, heading_error)

    def _send_thrust(self, surge: float, sway: float,
                     yaw: float, heave: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in mix_thrusters(surge, yaw, heave, sway)]
        self._thrust_pub.publish(msg)

    def _write_csv(self, surge: float, sway: float, yaw_cmd: float,
                   heave: float, event: str, distance: float = float('nan'),
                   route_heading: float = float('nan'),
                   look_heading: float = float('nan'),
                   heading_error: float = float('nan')) -> None:
        p, g = self._pose, self._goal
        depth_error = (p[2] - self._depth_setpoint
                       if self._depth_setpoint is not None else float('nan'))
        self._log.write([
            self._t_ros(), float(p[0]), float(p[1]), float(p[2]),
            float(g[0]) if g is not None else float('nan'),
            float(g[1]) if g is not None else float('nan'),
            float(g[2]) if g is not None else float('nan'),
            distance, math.degrees(route_heading), math.degrees(look_heading),
            math.degrees(heading_error), self._wall_side, self._wall_distance,
            depth_error, surge, sway, yaw_cmd, heave, self._min_front_dist,
            len(self._path), self._wp_idx, event,
        ])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallOrientedController()
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
