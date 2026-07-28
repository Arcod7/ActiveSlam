"""
Forward path controller with a fixed viewing offset toward the nearest wall.

Unlike ``wall_looking``, this controller does not estimate wall normals,
regulate standoff, or alter the planner route.  It follows the same path as
the ordinary waypoint controller while yawing ``look_offset_deg`` to the left
or right of the route bearing.  The nearest occupied/surface point in the map
chooses the sign of that offset.

The BlueROV2 is holonomic in the horizontal plane.  Route velocity is therefore
projected onto body surge and sway, allowing the vehicle to keep travelling
along the planned path while its camera looks slightly toward the wall.

Published topics:
  /motion/body_command    (geometry_msgs/Twist; normalized safety-gate input)
  /motion/selected_wall   (visualization_msgs/MarkerArray) the nearest point on
      the side currently chosen, and a link to it from the vehicle — the map
      evidence behind the viewing offset (see wall_markers)
"""

import math
import os

from frontier_slam.control_utils import (
    depth_hold_effort,
    LowPassRate,
    SlewLimiter,
    wrap_angle,
    yaw_from_quat,
)
from frontier_slam.session_log import open_session_log
from frontier_slam.wall_markers import selected_wall_markers, TOPIC as WALL_MARKER_TOPIC
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    "logs",
)

CSV_COLUMNS = [
    "t_ros",
    "rx",
    "ry",
    "rz",
    "gx",
    "gy",
    "gz",
    "dist_m",
    "route_hdg_deg",
    "look_hdg_deg",
    "hdg_err_deg",
    "wall_side",
    "wall_dist_m",
    "depth_err_m",
    "surge",
    "sway",
    "yaw_cmd",
    "yaw_rate",
    "heave",
    "obs_m",
    "path_len",
    "wp_idx",
    "event",
]


def offset_heading(route_heading: float, wall_side: int, offset_deg: float) -> float:
    """Offset route heading toward the wall (-1 left, +1 right, 0 absent)."""
    side = 0 if wall_side == 0 else (1 if wall_side > 0 else -1)
    offset = math.radians(float(np.clip(offset_deg, 0.0, 89.0)))
    return wrap_angle(route_heading + side * offset)



def _side_geometry(
    points: np.ndarray,
    pose: np.ndarray,
    route_heading: float,
    z_band_m: float,
    max_distance_m: float,
    lateral_deadband_m: float,
):
    """Gated candidates as (points, planar distance, lateral offset), or None.

    Points close to the route centreline do not provide a reliable side and are
    dropped.  Positive lateral displacement is starboard/right in the NED XY
    convention used by this project.
    """
    if points is None or len(points) == 0:
        return None

    pts = np.asarray(points, dtype=np.float64)
    p = np.asarray(pose, dtype=np.float64)

    # Gate before the vector maths; non-finite points fail both gates anyway.
    near = np.abs(pts[:, 2] - p[2]) <= z_band_m
    if not np.any(near):
        return None
    pts = pts[near]
    dx = pts[:, 0] - p[0]
    dy = pts[:, 1] - p[1]
    distance = np.hypot(dx, dy)
    in_range = distance <= max_distance_m
    if not np.any(in_range):
        return None
    pts, dx, dy, distance = pts[in_range], dx[in_range], dy[in_range], distance[in_range]

    # Starboard unit vector for a route bearing in world NED. Elementwise, not a
    # matmul: a two-column gemv dispatches to threaded BLAS, and at control rate
    # the worker spin-wait costs far more than the arithmetic.
    lateral = dx * -math.sin(route_heading) + dy * math.cos(route_heading)
    return pts, distance, lateral


def wall_side_distances(
    points: np.ndarray,
    pose: np.ndarray,
    route_heading: float,
    z_band_m: float,
    max_distance_m: float,
    lateral_deadband_m: float = 0.1,
) -> tuple[float, float]:
    """Return nearest (left, right) planar wall distances relative to the route."""
    geometry = _side_geometry(
        points, pose, route_heading, z_band_m, max_distance_m, lateral_deadband_m
    )
    if geometry is None:
        return float("inf"), float("inf")
    _, distance, lateral = geometry

    left_mask = lateral < -lateral_deadband_m
    right_mask = lateral > lateral_deadband_m
    left_dist = (
        float(np.min(distance[left_mask])) if np.any(left_mask) else float("inf")
    )
    right_dist = (
        float(np.min(distance[right_mask])) if np.any(right_mask) else float("inf")
    )
    return left_dist, right_dist


def nearest_wall_point(
    points: np.ndarray,
    pose: np.ndarray,
    route_heading: float,
    z_band_m: float,
    max_distance_m: float,
    side: int,
    lateral_deadband_m: float = 0.1,
) -> 'np.ndarray | None':
    """The point behind wall_side_distances' answer for `side` (-1 left, +1 right).

    Display only: the controller steers on the distance, but the distance alone
    cannot be checked against the map by eye. None when no side is held or the
    side is empty, which is exactly when there is nothing to point at.
    """
    if side == 0:
        return None
    geometry = _side_geometry(
        points, pose, route_heading, z_band_m, max_distance_m, lateral_deadband_m
    )
    if geometry is None:
        return None
    pts, distance, lateral = geometry

    mask = lateral < -lateral_deadband_m if side < 0 else lateral > lateral_deadband_m
    if not np.any(mask):
        return None
    candidates = np.flatnonzero(mask)
    return pts[candidates[np.argmin(distance[candidates])]]


def choose_wall_side(
    left_distance: float,
    right_distance: float,
    current_side: int = 0,
    switch_margin_m: float = 0.3,
) -> int:
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


def lookahead_path_heading(
    path: list[tuple[float, float]],
    waypoint_index: int,
    robot_xy: np.ndarray,
    radius_m: float,
    fallback_heading: float,
) -> tuple[np.ndarray, float]:
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
    points = [centre] + [
        np.asarray(p, dtype=np.float64) for p in path[max(0, waypoint_index) :]
    ]
    radius_sq = radius_m * radius_m
    for start, end in zip(points, points[1:]):
        direction = end - start
        length_sq = float(direction @ direction)
        if length_sq < 1e-12:
            continue
        relative = start - centre
        roots = np.roots(
            [
                length_sq,
                2.0 * float(relative @ direction),
                float(relative @ relative) - radius_sq,
            ]
        )
        candidates = sorted(
            float(root.real)
            for root in roots
            if abs(root.imag) < 1e-9 and 1e-8 < root.real <= 1.0 + 1e-8
        )
        if candidates:
            t = min(candidates[0], 1.0)
            point = start + t * direction
            return point, math.atan2(direction[1], direction[0])
    return centre, fallback_heading


def parse_xyz_cloud(msg: PointCloud2) -> "np.ndarray | None":
    """Parse finite XYZ points from an arbitrary float32 PointCloud2 layout."""
    offsets = {
        field.name: field.offset
        for field in msg.fields
        if field.datatype == PointField.FLOAT32
    }
    if (
        any(name not in offsets for name in ("x", "y", "z"))
        or msg.point_step <= 0
        or msg.width == 0
        or msg.height == 0
    ):
        return None

    byte_order = ">" if msg.is_bigendian else "<"
    cloud_dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [byte_order + "f4"] * 3,
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": msg.point_step,
        }
    )
    try:
        data = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=cloud_dtype,
            buffer=msg.data,
            strides=(msg.row_step, msg.point_step),
        )
    except (TypeError, ValueError):
        return None
    xyz = np.column_stack((data["x"].ravel(), data["y"].ravel(), data["z"].ravel()))
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64, copy=False)


class WallOrientedController(Node):
    # Heading is held by a direct heading->thrust law, because the only yaw rate
    # available here is a differentiated heading estimate and that cannot close a
    # loop: compass sigma is 0.05 rad at 10 Hz, so differentiation yields ~0.24
    # rad/s of noise against a 0.25 rad/s target -- measured on a constant-spin
    # scan, the fed-back rate read backwards on 4 of 18 samples and the command
    # flipped sign on 10 of 17. No gain or filter fixes that: resolving the rate
    # at 10:1 would need ~3 s of averaging. Heading error itself is clean (under
    # 2 deg of noise), so the law that uses it directly is smooth.
    #
    # yaw_rate_command()/MAX_YAW_RATE in control_utils stay: a real gyro measures
    # rate without differentiating, and the cascade goes back on top of one.
    # Until then YAW_EFFORT_LIMIT bounds authority instead of achieved rate --
    # equivalent only while thrust_boost is off, which is what caps spin here.
    # Yaw is inertia plus a little drag -- close to a double integrator, which
    # proportional control alone cannot stabilise; only drag damps it, so the
    # gain has to stay under what drag can absorb. At 0.60 the loop diverged on
    # a smooth setpoint: heading error grew 8.4 -> 24.2 -> 25.7 deg mean across
    # a run, peaking at 146, while look_hdg moved only 3 deg/s. 0.15 is twice
    # the 0.07 that was stable before the cascade, and still 4x under 0.60.
    # The real fix is the D term, which needs a rate a gyro can supply and a
    # differentiated heading cannot -- see the note above.
    # Effective loop gain is KP_YAW * turn_factor, so the launcher's turn_factor
    # is the live knob if this still rings.
    KP_YAW = 0.15
    YAW_RATE_TAU = 0.15  # diagnostic only; the CSV logs it, control ignores it
    YAW_EFFORT_LIMIT = 0.30
    # Effort per second on the published yaw command: a step to full authority
    # now takes 1 s, not one 0.1 s tick. The effort limit above bounds the rate
    # eventually reached; this bounds the acceleration used to reach it.
    YAW_SLEW_PER_S = 0.30
    KP_SPEED = 0.25
    KP_HEAVE = 0.35
    KD_HEAVE = 0.50  # damps the 6.1 s depth limit cycle P alone sustains
    DEPTH_RATE_TAU = 0.20  # s, low-pass on the differentiated depth

    # Both rates are differentiated from a pose estimate that steps on graph
    # corrections and stalls while the optimiser runs. Beyond these bounds the
    # sample is one of those, not motion: measured true motion peaks at 1.26
    # rad/s and 0.17 m/s.
    YAW_RATE_MAX = 1.5  # rad/s (~86 deg/s)
    DEPTH_RATE_MAX = 0.6  # m/s
    ODOM_GAP_S = 0.5  # a longer gap carries no usable rate
    ODOM_STALE_S = 1.0  # past this the pose is too old to steer on

    MAX_SPEED = 0.35
    GOAL_RADIUS = 2.0
    GOAL_REACHED_TIMEOUT = 10.0
    SCAN_YAW = 0.08  # open-loop effort; ~0.22 rad/s achieved without boost
    INIT_SCAN_DURATION = 10.0
    # wall-oriented heading — narrow and slow keeps the wall in the sonar.
    # Patrol dwell: half-extent of the beat walked along the wall either side of
    # the target, and how close counts as reaching one end.
    REVISIT_PATROL_M = 3.0
    REVISIT_PATROL_REACHED_M = 1.0
    WAYPOINT_ADVANCE_DIST = 1.5
    OBS_SLOW_DIST = 1.5
    EMERGENCY_STOP_DIST = 0.4
    BACK_SURGE_SPEED = 0.12
    ESCAPE_YAW = 0.20  # open-loop effort, same basis as SCAN_YAW
    ESCAPE_DURATION = 4.0
    STUCK_SPEED_MIN = 0.15
    STUCK_WINDOW = 5.0
    STUCK_MOVE_MIN = 0.25
    SIDE_SWITCH_DWELL_S = 4.0  # a side change must persist this long to be taken
    # ...and the previous re-sign must have converged first. 25 deg is a little
    # over half a 45 deg offset, so the vehicle has clearly committed to the
    # side it holds before it is allowed to abandon it.
    SIDE_SWITCH_SETTLED_RAD = math.radians(25.0)
    MAP_STALE_S = 5.0
    CTRL_HZ = 10.0
    LOG_EVERY_N_TICKS = 10

    def __init__(self) -> None:
        super().__init__("wall_oriented_controller")

        self.declare_parameter("depth_setpoint", -1.0)
        self.declare_parameter("look_offset_deg", 30.0)
        self.declare_parameter("lookahead_m", 0.0)
        self.declare_parameter("map_points_topic", "/octomap_point_cloud_centers")
        self.declare_parameter("max_wall_distance_m", 8.0)
        self.declare_parameter("wall_z_band_m", 1.5)
        self.declare_parameter("side_switch_margin_m", 0.3)
        # Scan this many times slower while a revisit is in progress: the
        # vehicle went back to re-observe known structure, so a slower sweep
        # puts more sonar frames on it and gives the pose graph and the map
        # rebuild more time on geometry where a closure is actually possible.
        # 1.0 = off.
        self.declare_parameter("revisit_scan_slowdown", 1.0)
        self.declare_parameter("odom_topic", "/StoneFish/Odometry")
        self.declare_parameter("goal_topic", "/frontier_slam/goal")
        self.declare_parameter("path_topic", "/frontier_slam/path")
        self.declare_parameter("command_topic", "/motion/body_command")
        self.declare_parameter("speed_factor", 1.0)
        self.declare_parameter("turn_factor", 1.0)
        # Only sizes the selected-wall highlight; must match the mapper's grid
        # or the marker straddles two cubes.
        self.declare_parameter("voxel_size", 0.2)

        self._voxel_size = float(self.get_parameter("voxel_size").value)
        depth = float(self.get_parameter("depth_setpoint").value)
        self._depth_setpoint: float | None = None if depth < 0 else depth
        self._look_offset_deg = float(
            np.clip(self.get_parameter("look_offset_deg").value, 0.0, 89.0)
        )
        self._lookahead_m = max(0.0, float(self.get_parameter("lookahead_m").value))
        self._max_wall_distance = float(self.get_parameter("max_wall_distance_m").value)
        self._wall_z_band = float(self.get_parameter("wall_z_band_m").value)
        self._side_switch_margin = float(
            self.get_parameter("side_switch_margin_m").value
        )
        self._revisit_scan_slowdown = max(
            1.0, float(self.get_parameter("revisit_scan_slowdown").value)
        )
        self._scan_slowdown = 1.0
        self._revisiting = False
        # Beat endpoints are anchored to where the vehicle arrived, so the walk
        # stays centred on the target instead of drifting along the wall.
        self._patrol_anchor = None
        self._patrol_sign = 1.0
        # Route heading last driven on, so a stopped vehicle still has one to
        # offset its viewing heading from.
        self._last_route_heading: float | None = None
        # Previous tick's |look_heading - yaw|, read by the side-switch gate.
        # _select_wall_side runs before this tick's error exists, so one tick of
        # staleness is inherent and harmless at control rate.
        self._last_heading_error = 0.0
        self.create_subscription(
            String, "/frontier_slam/revisit_state", self._revisit_state_cb, 10
        )
        map_topic = str(self.get_parameter("map_points_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)

        self._goal: np.ndarray | None = None
        self._pose: np.ndarray | None = None
        self._depth_rate = LowPassRate(
            self.DEPTH_RATE_TAU, max_rate=self.DEPTH_RATE_MAX, max_gap_s=self.ODOM_GAP_S
        )
        self._yaw_rate = LowPassRate(
            self.YAW_RATE_TAU,
            wrap=True,
            max_rate=self.YAW_RATE_MAX,
            max_gap_s=self.ODOM_GAP_S,
        )
        self._yaw_slew = SlewLimiter(self.YAW_SLEW_PER_S)
        self._yaw = 0.0
        self._odom_at: float | None = None
        self._path: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._map_points: np.ndarray | None = None
        self._map_received_at: float | None = None
        self._wall_side = 0
        self._side_candidate = 0
        self._side_candidate_t = 0.0
        self._wall_distance = float("nan")
        self._min_front_dist = float("inf")
        self._init_scan_end: float | None = None
        self._goal_reached_at: float | None = None
        self._escape_until: float | None = None
        self._stuck_ref_pos: np.ndarray | None = None
        self._stuck_ref_t: float | None = None
        self._tick = 0

        self._log = open_session_log("wall_oriented", CSV_COLUMNS, _LOG_DIR)
        self.create_subscription(PointStamped, goal_topic, self._goal_cb, 1)
        self.create_subscription(Path, path_topic, self._path_cb, 1)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(Image, "/sensor_msgs/image_depth", self._depth_cb, 1)
        self.create_subscription(PointCloud2, map_topic, self._map_cb, 1)
        self._command_pub = self.create_publisher(Twist, command_topic, 1)
        # Same label the CSV `event` column carries — what the vehicle is
        # doing, for the launcher's status panel.
        self._activity_pub = self.create_publisher(String, "/frontier_slam/activity", 1)
        self._selected_wall_pub = self.create_publisher(
            MarkerArray, WALL_MARKER_TOPIC, 1)
        self.create_timer(1.0 / self.CTRL_HZ, self._loop)

        self.get_logger().info(
            f"wall_oriented_controller ready — offset={self._look_offset_deg:.1f}deg "
            f"lookahead={self._lookahead_m:.1f}m "
            f"map={map_topic} goal={goal_topic} path={path_topic} "
            f"— logging to {self._log.path}"
        )

    def _t_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_cb(self, msg: PointCloud2) -> None:
        points = parse_xyz_cloud(msg)
        if points is None:
            self.get_logger().warn(
                "map cloud has no readable float32 x/y/z fields — ignoring",
                throttle_duration_sec=10.0,
            )
            return
        self._map_points = points
        self._map_received_at = self._t_ros()

    def _depth_cb(self, msg: Image) -> None:
        try:
            data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(
                msg.height, msg.width
            )
        except ValueError:
            return
        valid = data[np.isfinite(data) & (data > 0.1)]
        self._min_front_dist = float(valid.min()) if valid.size else float("inf")

    def _goal_cb(self, msg: PointStamped) -> None:
        new_goal = np.array([msg.point.x, msg.point.y, msg.point.z])
        changed = self._goal is None or np.hypot(*(new_goal[:2] - self._goal[:2])) > 1.0
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
        distances = [
            math.hypot(x - self._pose[0], y - self._pose[1]) for x, y in self._path
        ]
        self._wp_idx = int(np.argmin(distances))

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._odom_at = self._t_ros()
        self._depth_rate.update(float(p.z), self._odom_at)
        self._yaw_rate.update(self._yaw, self._odom_at)
        if self._init_scan_end is None:
            if self._depth_setpoint is None:
                self._depth_setpoint = float(p.z)
                self.get_logger().info(
                    f"depth setpoint locked from odom at {self._depth_setpoint:.2f} m"
                )
            self._init_scan_end = self._t_ros() + self.INIT_SCAN_DURATION
            self.get_logger().info(
                f"initial {self.INIT_SCAN_DURATION:.0f}s scan starting"
            )

    def _scan_yaw(self) -> float:
        """Scan yaw effort, divided down while a revisit is in progress."""
        return self.SCAN_YAW / self._scan_slowdown

    def _revisit_patrol_target(self, now: float) -> np.ndarray:
        """Waypoint for a short beat along the wall, centred on the target.

        The viewing offset already points look_heading at the wall, so the wall
        runs roughly perpendicular to it; stepping along that perpendicular
        walks the structure rather than into or away from it. The vehicle turns
        round at each end, so it stays within REVISIT_PATROL_M of where it
        arrived for the whole dwell.

        Falls back to the current pose when no wall has been selected, which
        makes the vehicle hold station exactly as the sweep would.
        """
        if self._patrol_anchor is None:
            self._patrol_anchor = self._pose[:2].copy()
        if self._wall_side == 0 or self._last_route_heading is None:
            return self._patrol_anchor
        look = offset_heading(
            self._last_route_heading, self._wall_side, self._look_offset_deg)
        tangent = look + math.pi / 2.0
        step = self._patrol_sign * self.REVISIT_PATROL_M
        target = self._patrol_anchor + step * np.array(
            [math.cos(tangent), math.sin(tangent)])
        if float(np.hypot(*(target - self._pose[:2]))) < self.REVISIT_PATROL_REACHED_M:
            self._patrol_sign = -self._patrol_sign
        return target

    def _revisit_state_cb(self, msg) -> None:
        was = self._revisiting
        self._revisiting = msg.data == "revisiting"
        if not self._revisiting and was:
            self._patrol_anchor = None
        slowdown = self._revisit_scan_slowdown if msg.data == "revisiting" else 1.0
        if slowdown != self._scan_slowdown:
            self._scan_slowdown = slowdown
            self.get_logger().info(
                f"Scan yaw {'slowed x%.1f for revisit' % slowdown}"
                if slowdown > 1.0
                else "Scan yaw back to normal"
            )

    def _heave_cmd(self) -> float:
        if self._depth_setpoint is None:
            return 0.0
        error = self._pose[2] - self._depth_setpoint
        return depth_hold_effort(
            error, self._depth_rate.value, self.KP_HEAVE, self.KD_HEAVE
        )

    def _select_wall_side(self, route_heading: float, now: float) -> None:
        if (
            self._map_points is None
            or self._map_received_at is None
            or now - self._map_received_at > self.MAP_STALE_S
        ):
            self._wall_side = 0
            self._wall_distance = float("nan")
            self._publish_selected_wall(None)
            return
        # Re-read live: the launcher pushes this with `ros2 param set` while
        # running, and a value cached at startup would ignore it silently.
        self._wall_z_band = float(self.get_parameter("wall_z_band_m").value)
        left, right = wall_side_distances(
            self._map_points,
            self._pose,
            route_heading,
            self._wall_z_band,
            self._max_wall_distance,
        )
        candidate = choose_wall_side(
            left, right, self._wall_side, self._side_switch_margin
        )
        # Switching sides re-signs the viewing offset, so each change steps the
        # yaw setpoint by 2*look_offset_deg at once -- measured at 48.6 deg,
        # against 2.4 deg/s while the side holds. The 0.3 m margin alone does
        # not settle it when both sides sit at a similar range: the side flipped
        # 8 times in 107 s, which is the left-right sweep. Make a change earn
        # itself over SIDE_SWITCH_DWELL_S before the setpoint follows it.
        #
        # The time dwell alone is not enough, because it is shorter than the
        # manoeuvre it authorises. Measured 2026-07-28 over 4 runs: the vehicle
        # yaws at 5.7 deg/s, so a 2*45 deg re-sign takes ~16 s, while the dwell
        # commits to it on 4 s of evidence. The side could therefore flip again
        # before the previous swing finished -- 8-12 flips per run, about 24% of
        # the run spent chasing a setpoint it never reached, which is what made
        # the vehicle look like it was ignoring the wall and driving straight.
        # So also require the previous swing to have converged: no new side is
        # taken while the heading is still far from the one currently commanded.
        settled = abs(self._last_heading_error) <= self.SIDE_SWITCH_SETTLED_RAD
        if candidate != self._wall_side:
            if candidate != self._side_candidate:
                self._side_candidate = candidate
                self._side_candidate_t = now
            elif (settled
                  and now - self._side_candidate_t >= self.SIDE_SWITCH_DWELL_S):
                self._wall_side = candidate
                self._side_candidate_t = now
        else:
            self._side_candidate = candidate
            self._side_candidate_t = now
        chosen = left if self._wall_side < 0 else right
        self._wall_distance = chosen if math.isfinite(chosen) else float("nan")

        # Drawn from the side actually held, not the fresh candidate: during the
        # switch dwell the marker must show what is steering the vehicle now.
        self._publish_selected_wall(nearest_wall_point(
            self._map_points, self._pose, route_heading, self._wall_z_band,
            self._max_wall_distance, self._wall_side))

    def _publish_selected_wall(self, wall_point) -> None:
        self._selected_wall_pub.publish(selected_wall_markers(
            self._pose, wall_point, self._voxel_size,
            self.get_clock().now().to_msg()))

    def _drive(self, target_xy: np.ndarray, now: float) -> tuple:
        delta = target_xy - self._pose[:2]
        distance = float(np.hypot(*delta))
        travel_heading = math.atan2(delta[1], delta[0])
        _, look_path_heading = lookahead_path_heading(
            self._path, self._wp_idx, self._pose[:2], self._lookahead_m, travel_heading
        )
        self._select_wall_side(look_path_heading, now)
        self._last_route_heading = look_path_heading
        look_heading = offset_heading(
            look_path_heading, self._wall_side, self._look_offset_deg
        )
        heading_error = wrap_angle(look_heading - self._yaw)
        self._last_heading_error = heading_error
        yaw_cmd = float(
            np.clip(
                self.KP_YAW * heading_error,
                -self.YAW_EFFORT_LIMIT,
                self.YAW_EFFORT_LIMIT,
            )
        )

        # Reduce travel while the requested viewing heading is far away, then
        # project the unchanged route velocity onto the current body axes.
        alignment = max(0.0, math.cos(heading_error))
        speed = float(
            np.clip(self.KP_SPEED * distance * alignment, 0.0, self.MAX_SPEED)
        )
        route_in_body = wrap_angle(travel_heading - self._yaw)
        surge = speed * math.cos(route_in_body)
        sway = speed * math.sin(route_in_body)
        return (
            surge,
            sway,
            yaw_cmd,
            distance,
            travel_heading,
            look_heading,
            heading_error,
            look_path_heading,
        )

    def _loop(self) -> None:
        self._tick += 1
        write_csv = self._tick % self.LOG_EVERY_N_TICKS == 0
        if self._pose is None:
            return

        now = self._t_ros()
        if self._odom_at is not None and now - self._odom_at > self.ODOM_STALE_S:
            # Steering on a pose seconds old is what makes the vehicle lurch.
            # The stuck reference has to clear too: a pose that is not being
            # updated is not evidence that the vehicle failed to move.
            self._send_thrust(0.0, 0.0, 0.0, 0.0)
            self._stuck_ref_pos = None
            self._stuck_ref_t = None
            self.get_logger().warn(
                f"odometry {now - self._odom_at:.1f}s stale — holding",
                throttle_duration_sec=5.0,
            )
            if write_csv:
                self._write_csv(0.0, 0.0, 0.0, 0.0, "ODOM_STALE")
            return

        heave = self._heave_cmd()
        if self._init_scan_end is not None and now < self._init_scan_end:
            self._send_thrust(0.0, 0.0, self._scan_yaw(), heave)
            if write_csv:
                self._write_csv(0.0, 0.0, self._scan_yaw(), heave, "INIT_SCAN")
            return
        if self._goal is None:
            self._send_thrust(0.0, 0.0, self._scan_yaw(), heave)
            if write_csv:
                self._write_csv(0.0, 0.0, self._scan_yaw(), heave, "SCAN")
            return

        goal_dist = float(np.hypot(*(self._goal[:2] - self._pose[:2])))
        patrol_target = None
        if goal_dist < self.GOAL_RADIUS:
            if self._revisiting:
                # Walk the wall instead of holding station. Falls through to the
                # normal drive below rather than commanding thrust here, so the
                # dwell keeps the obstacle slowdown and emergency stop that
                # guard every other metre of the mission.
                patrol_target = self._revisit_patrol_target(now)
            if patrol_target is None:
                if self._goal_reached_at is None:
                    self._goal_reached_at = now
                elif now - self._goal_reached_at > self.GOAL_REACHED_TIMEOUT:
                    self._goal = None
                    self._goal_reached_at = None
                self._send_thrust(0.0, 0.0, self._scan_yaw(), heave)
                if write_csv:
                    self._write_csv(
                        0.0,
                        0.0,
                        self._scan_yaw(),
                        heave,
                        "GOAL_REACHED",
                        distance=goal_dist,
                    )
                return

        if patrol_target is not None:
            target_xy = patrol_target
        elif self._path:
            while (
                self._wp_idx < len(self._path) - 1
                and math.hypot(
                    self._path[self._wp_idx][0] - self._pose[0],
                    self._path[self._wp_idx][1] - self._pose[1],
                )
                < self.WAYPOINT_ADVANCE_DIST
            ):
                self._wp_idx += 1
            target_xy = np.array(self._path[self._wp_idx])
        else:
            target_xy = self._goal[:2]

        (
            surge,
            sway,
            yaw_cmd,
            _,
            route_heading,
            look_heading,
            heading_error,
            look_path_heading,
        ) = self._drive(target_xy, now)
        event = "REVISIT_PATROL" if patrol_target is not None else ""
        if self._min_front_dist < self.EMERGENCY_STOP_DIST:
            surge, sway = -self.BACK_SURGE_SPEED, 0.0
            event = "EMERG_STOP"
        elif self._min_front_dist < self.OBS_SLOW_DIST:
            factor = (self._min_front_dist - self.EMERGENCY_STOP_DIST) / (
                self.OBS_SLOW_DIST - self.EMERGENCY_STOP_DIST
            )
            surge *= factor
            sway *= factor

        if self._escape_until is not None:
            if now < self._escape_until:
                self._send_thrust(0.0, 0.0, self.ESCAPE_YAW, heave)
                if write_csv:
                    self._write_csv(
                        0.0,
                        0.0,
                        self.ESCAPE_YAW,
                        heave,
                        "CTRL_STUCK_ESCAPE",
                        goal_dist,
                        route_heading,
                        look_heading,
                        heading_error,
                    )
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
                    event = "CTRL_STUCK"
                    return
        else:
            self._stuck_ref_pos = None
            self._stuck_ref_t = None

        self._send_thrust(surge, sway, yaw_cmd, heave)
        side_name = {-1: "left", 0: "none", 1: "right"}[self._wall_side]
        self.get_logger().info(
            f"goal_dist={goal_dist:.1f}m wp={self._wp_idx}/{len(self._path)} "
            f"wall={side_name}@{self._wall_distance:.1f}m "
            f"route={math.degrees(route_heading):+.0f}deg "
            f"path_look={math.degrees(look_path_heading):+.0f}deg "
            f"look={math.degrees(look_heading):+.0f}deg "
            f"surge={surge:+.2f} sway={sway:+.2f} yaw={yaw_cmd:+.3f}",
            throttle_duration_sec=2.0,
        )
        if write_csv:
            self._write_csv(
                surge,
                sway,
                yaw_cmd,
                heave,
                event,
                goal_dist,
                route_heading,
                look_heading,
                heading_error,
            )

    # Matches safety_gate.py's max_abs_command default (1.0): the gate rejects
    # (and latches INVALID_COMMAND on) any out-of-range component, so a
    # speed_factor/turn_factor above 1x must saturate here, not there.
    MAX_ABS_COMMAND = 1.0

    def _send_thrust(self, surge: float, sway: float, yaw: float, heave: float) -> None:
        """Single publish choke point — applies the operator speed/turn factors
        (live-tunable from the launcher TUI) uniformly to every caller, then
        slew-limits yaw so no caller can step the actuators."""
        speed_factor = float(self.get_parameter("speed_factor").value)
        turn_factor = float(self.get_parameter("turn_factor").value)
        cap = self.MAX_ABS_COMMAND
        msg = Twist()
        msg.linear.x = float(np.clip(surge * speed_factor, -cap, cap))
        msg.linear.y = float(np.clip(sway * speed_factor, -cap, cap))
        msg.linear.z = float(np.clip(heave * speed_factor, -cap, cap))
        # Slewed after turn_factor, so raising the factor ramps rather than steps.
        msg.angular.z = self._yaw_slew.update(
            float(np.clip(yaw * turn_factor, -cap, cap)), self._t_ros()
        )
        self._command_pub.publish(msg)

    def _write_csv(
        self,
        surge: float,
        sway: float,
        yaw_cmd: float,
        heave: float,
        event: str,
        distance: float = float("nan"),
        route_heading: float = float("nan"),
        look_heading: float = float("nan"),
        heading_error: float = float("nan"),
    ) -> None:
        self._activity_pub.publish(String(data=event or "FOLLOW_PATH"))
        p, g = self._pose, self._goal
        depth_error = (
            p[2] - self._depth_setpoint
            if self._depth_setpoint is not None
            else float("nan")
        )
        self._log.write(
            [
                self._t_ros(),
                float(p[0]),
                float(p[1]),
                float(p[2]),
                float(g[0]) if g is not None else float("nan"),
                float(g[1]) if g is not None else float("nan"),
                float(g[2]) if g is not None else float("nan"),
                distance,
                math.degrees(route_heading),
                math.degrees(look_heading),
                math.degrees(heading_error),
                self._wall_side,
                self._wall_distance,
                depth_error,
                surge,
                sway,
                yaw_cmd,
                self._yaw_rate.value,
                heave,
                self._min_front_dist,
                len(self._path),
                self._wp_idx,
                event,
            ]
        )


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
