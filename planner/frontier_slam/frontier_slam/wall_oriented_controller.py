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

Heading is held by a cascade -- error to desired yaw rate to effort -- damped by
a sensed rate, so the gyro topic below is a control input, not telemetry.

Subscribed topics of note:
  gyro_topic  (geometry_msgs/Vector3Stamped; default
      /slam/sensors/imu_angular_velocity) body-frame angular velocity

Published topics:
  /motion/body_command    (geometry_msgs/Twist; normalized safety-gate input)
  /motion/selected_wall   (visualization_msgs/MarkerArray) the nearest point on
      the side currently chosen, and a link to it from the vehicle — the map
      evidence behind the viewing offset (see wall_markers)
  /motion/commanded_heading (visualization_msgs/MarkerArray) the viewing heading
      being asked for, against the one held — the yaw setpoint and its tracking
      error, live (see heading_markers)
"""

import math
import os

from frontier_slam.control_utils import (
    depth_hold_effort,
    HeadingReference,
    LowPass,
    LowPassRate,
    SlewLimiter,
    wrap_angle,
    yaw_from_quat,
    yaw_rate_command,
)
from frontier_slam.heading_markers import (
    heading_markers,
    TOPIC as HEADING_MARKER_TOPIC,
)
from frontier_slam.session_log import open_session_log
from frontier_slam.wall_markers import selected_wall_markers, TOPIC as WALL_MARKER_TOPIC
from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
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
    # A non-positive or non-finite cap means no range gate: the nearest wall is
    # whatever is nearest. A finite cap makes the vehicle lose the wall outright
    # once it drifts past it — reported as wall=none, which drops the viewing
    # offset entirely — rather than merely track a more distant one.
    if math.isfinite(max_distance_m) and max_distance_m > 0.0:
        in_range = distance <= max_distance_m
        if not np.any(in_range):
            return None
        pts, dx, dy, distance = (
            pts[in_range], dx[in_range], dy[in_range], distance[in_range])

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
    # Heading is held by a cascade: error -> desired yaw rate -> effort, with the
    # rate measured by the gyro (/slam/sensors/imu_angular_velocity). The direct
    # heading->thrust law this replaces had no damping term -- yaw is inertia
    # plus a little drag, so only drag damped it, and it arrived at the setpoint
    # still turning: 20 deg mean overshoot, 47% of the incoming error, measured
    # 2026-07-31 over 25 swings. Raising the gain could not fix that (at 0.60 the
    # loop diverged, error 8.4 -> 25.7 deg mean, peaking at 146) and neither
    # could the slew limiter, which had already taken it from 52 deg to 20.
    # A differentiated heading estimate cannot supply the rate -- sd 0.37 rad/s
    # against a 0.47 rad/s mean slew, same run. A gyro senses rate directly.
    # Chosen so a 180 deg step settles no slower than the law this replaces
    # (6.3 s against 6.1 s) with the overshoot gone, and stays that way against
    # an unmodelled actuator lag of up to 0.6 s. Raising them to 1.6/1.5 settles
    # 0.8 s sooner but starts ringing again past 0.4 s of lag.
    KP_HEADING = 1.2      # heading error -> desired rate, 1/s; saturates at 38 deg
    # Halved from 1.0 on 2026-08-01, with the shaped reference in place. At 1.0
    # the damping term dominated the command on steady headings -- corr(cmd,
    # -rate) 0.51 vs corr(cmd, err) 0.30, median 0.13 effort from ambient rate
    # wobble, saturated 20% of ticks -- so damping sized for killing 90 deg
    # swings was amplifying noise the rest of the time, and through the ~0.25 s
    # of filter+ZOH+thrust delay it reinjects energy near 1 Hz rather than
    # removing it. 0.5 still brakes an overrun hard (a 0.5 rad/s excess is a
    # -0.25 command) and keeps the loop overdamped (zeta ~1.1).
    # ...and settled at 0.35 by pole placement against the measured plant
    # (ACC ~5.1 rad/s^2 per unit effort at the shipped turn_factor, drag
    # ~1.4/s): inner-loop bandwidth ACC*kd + drag = 3.2 rad/s, the 45 deg
    # phase-margin limit for the ~0.25 s filter+ZOH+thrust delay, and the
    # heading loop lands at wn 1.46 rad/s, zeta 1.09 -- no overshoot, no idle
    # dither (0.35 * 0.07 rad/s ambient rate = 0.025 effort, the old P law's
    # level).
    KP_YAW_RATE = 0.35    # rate error -> effort, s
    # Bounds the spin actually achieved rather than the authority used to get
    # there, so it holds whatever the thrust ceiling is. The overshooting P law
    # reached 1.36 rad/s; the hull's steady scan rate is 0.22. A rate loop this
    # shallow keeps a standing offset, so the rate reached is ~0.63 of this.
    MAX_YAW_RATE = 0.8    # rad/s (~46 deg/s)
    YAW_RATE_TAU = 0.10   # s, low-pass on the gyro
    # Complementary fusion of the steering yaw (see _fuse_steering_yaw).
    YAW_FUSE_TAU_S = 1.5
    YAW_FUSE_SNAP_RAD = 0.35  # past this the innovation is a graph correction
    # Heading gain used when no gyro is publishing — the direct law that
    # preceded the cascade, at the gain it shipped with. Overshoots (that is
    # what the cascade is for) but is the behaviour this hull is known to
    # survive. Applies to a real-robot run too, where mavlink_odometry supplies
    # orientation but no rate.
    KP_YAW_FALLBACK = 0.30
    # Effort is torque, so this is the peak angular acceleration.
    YAW_EFFORT_LIMIT = 0.30
    # The commanded heading is shaped, not the actuator command: the raw
    # viewing target is a staircase (path geometry on ~1 Hz map updates, SLAM
    # pose jitter, mode-change re-signs) and slew-limiting the command instead
    # put a rate limiter inside the loop -- measured 2026-08-01 as a limit
    # cycle: mean effort 0.15 near the setpoint against 0.03 for the old P
    # law, saturated 18% of the time, sustaining ~0.18 rad/s of yaw wobble the
    # loop itself was injecting. The reference glides at the rate the hull
    # actually holds (~0.63 * MAX_YAW_RATE), so tracking it never saturates.
    LOOK_REF_TAU = 0.4    # s, pull toward the raw target
    LOOK_REF_RATE = 0.6   # rad/s, cap on how fast the commanded heading moves
    # Backstop only, for the open-loop callers (scan, escape); the closed-loop
    # command is smooth by construction now the reference is shaped. 1.0/s sat
    # inside the loop and was the limit cycle's phase lag.
    YAW_SLEW_PER_S = 3.0
    KP_SPEED = 0.25
    KP_HEAVE = 0.35
    KD_HEAVE = 0.50  # damps the 6.1 s depth limit cycle P alone sustains
    DEPTH_RATE_TAU = 0.20  # s, low-pass on the differentiated depth

    # Depth rate is differentiated from a pose estimate that steps on graph
    # corrections and stalls while the optimiser runs. Beyond this bound the
    # sample is one of those, not motion: true motion peaks at 0.17 m/s. Yaw
    # rate needs no such guard now it is sensed rather than differenced.
    DEPTH_RATE_MAX = 0.6  # m/s
    ODOM_GAP_S = 0.5  # a longer gap carries no usable rate
    ODOM_STALE_S = 1.0  # past this the pose is too old to steer on
    GYRO_STALE_S = 0.5  # past this the cascade has no damping term — see _loop

    MAX_SPEED = 0.35
    GOAL_RADIUS = 4.0
    # Translation tapers between here and GOAL_RADIUS. KP_SPEED alone
    # does not brake: it only bites below MAX_SPEED/KP_SPEED = 1.4 m, which is
    # inside the arrival radius, so the vehicle used to reach the goal at full
    # speed and have its thrust cut in a single tick -- it then coasted through.
    ARRIVAL_BRAKE_M = 5.0
    # Floor of the ramp. Tapering all the way to zero is an asymptote: thrust
    # vanishes just outside GOAL_RADIUS and the vehicle stalls there without
    # ever registering arrival. A slowdown must still cross the radius.
    ARRIVAL_MIN_SCALE = 0.3
    # Misalignment slows travel, it must not stop it: cos() clamped at zero
    # froze the vehicle for the 3-5 s of every re-aim (measured 2026-08-01 at
    # each direction change). The creep this floor allows cannot overshoot
    # WAYPOINT_ADVANCE_DIST in that time.
    ALIGN_MIN_SCALE = 0.2
    GOAL_REACHED_TIMEOUT = 5.0
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
    # ...and it must have held that way, not merely touched it. Sampled at one
    # instant the gate leaked: the error passes through the band on its way
    # across, so a switch could still be taken mid-swing and reverse a turn
    # already under way. Measured 2026-07-31: 19% of switches came within 20 s
    # of the previous one and the closest pair was 5 s apart, against the ~10 s
    # a 90 deg re-sign needs.
    SIDE_SWITCH_SETTLED_S = 2.0
    # Below this lateral offset the remembered wall sits on the route axis and
    # cannot sign a side (see _resign_side_from_memory).
    MEMORY_SIDE_MIN_LATERAL_M = 0.3
    MAP_STALE_S = 5.0
    # The extractor replans at 3 Hz, so this tolerates 6 missed publications
    # before the path is abandoned for the straight-line bearing to the goal.
    PATH_STALE_S = 2.0
    CTRL_HZ = 10.0
    LOG_EVERY_N_TICKS = 10

    def __init__(self) -> None:
        super().__init__("wall_oriented_controller")

        self.declare_parameter("depth_setpoint", -1.0)
        self.declare_parameter("look_offset_deg", 30.0)
        self.declare_parameter("lookahead_m", 0.0)
        self.declare_parameter("map_points_topic", "/octomap_point_cloud_centers")
        self.declare_parameter("max_wall_distance_m", 0.0)  # 0 = no range gate
        self.declare_parameter("wall_z_band_m", 1.5)
        self.declare_parameter("side_switch_margin_m", 0.3)
        # Scan this many times slower while a revisit is in progress: the
        # vehicle went back to re-observe known structure, so a slower sweep
        # puts more sonar frames on it and gives the pose graph and the map
        # rebuild more time on geometry where a closure is actually possible.
        # 1.0 = off.
        self.declare_parameter("revisit_scan_slowdown", 1.0)
        self.declare_parameter("odom_topic", "/StoneFish/Odometry")
        # Sensed, not differentiated from odom_topic — the cascade's damping
        # term. Retargetable so a real gyro can feed the same loop.
        self.declare_parameter(
            "gyro_topic", "/slam/sensors/imu_angular_velocity")
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
        # Axis the patrol beat walks along, fixed for the dwell. Recomputing it
        # per tick fed the route heading it is derived from back into itself.
        self._patrol_tangent: float | None = None
        # Viewing heading held during the dwell, as an offset from the bearing
        # to the wall so it survives the beat reversing.
        self._patrol_look_delta: float | None = None
        # Last wall actually seen, kept so a gap in the map does not drop the
        # side and re-sign the viewing offset. A wall does not stop existing
        # because this tick's scan missed it.
        self._wall_memory: np.ndarray | None = None
        # Route heading last driven on, so a stopped vehicle still has one to
        # offset its viewing heading from.
        self._last_route_heading: float | None = None
        # Previous tick's |look_heading - yaw|, read by the side-switch gate.
        # _select_wall_side runs before this tick's error exists, so one tick of
        # staleness is inherent and harmless at control rate.
        self._last_heading_error = 0.0
        # Yaw setpoint behind the command about to be sent, for the RViz arrow.
        # None while no heading is being closed on (open-loop spin, stale pose).
        self._last_look_heading: float | None = None
        # When the heading error last entered SIDE_SWITCH_SETTLED_RAD and stayed.
        self._settled_since: float | None = None
        self.create_subscription(
            String, "/frontier_slam/revisit_state", self._revisit_state_cb, 10
        )
        map_topic = str(self.get_parameter("map_points_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        gyro_topic = self._gyro_topic = str(self.get_parameter("gyro_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)

        self._goal: np.ndarray | None = None
        self._pose: np.ndarray | None = None
        self._depth_rate = LowPassRate(
            self.DEPTH_RATE_TAU, max_rate=self.DEPTH_RATE_MAX, max_gap_s=self.ODOM_GAP_S
        )
        self._yaw_rate = LowPass(self.YAW_RATE_TAU, max_gap_s=self.ODOM_GAP_S)
        self._gyro_at: float | None = None
        self._yaw_slew = SlewLimiter(self.YAW_SLEW_PER_S)
        self._look_ref = HeadingReference(self.LOOK_REF_TAU, self.LOOK_REF_RATE)
        self._yaw = 0.0
        # Complementary steering yaw: gyro-integrated, pulled toward the SLAM
        # estimate by _fuse_steering_yaw. None until the first estimate.
        self._yaw_ctrl: float | None = None
        self._odom_at: float | None = None
        self._path: list[tuple[float, float]] = []
        self._path_at: float | None = None
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
        self.create_subscription(Vector3Stamped, gyro_topic, self._gyro_cb, 10)
        self.create_subscription(Image, "/sensor_msgs/image_depth", self._depth_cb, 1)
        self.create_subscription(PointCloud2, map_topic, self._map_cb, 1)
        self._command_pub = self.create_publisher(Twist, command_topic, 1)
        # Same label the CSV `event` column carries — what the vehicle is
        # doing, for the launcher's status panel.
        self._activity_pub = self.create_publisher(String, "/frontier_slam/activity", 1)
        self._selected_wall_pub = self.create_publisher(
            MarkerArray, WALL_MARKER_TOPIC, 1)
        self._heading_pub = self.create_publisher(
            MarkerArray, HEADING_MARKER_TOPIC, 1)
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
            self._stuck_ref_pos = None
            self._stuck_ref_t = None
            # The path is deliberately kept: clearing it here made the fallback
            # below steer on the straight-line bearing to the new goal for the
            # ~0.3 s until the replan landed, and that bearing can sit almost
            # opposite the route. Measured 2026-07-31 at every goal change --
            # route heading 57 -> -179 -> 57 deg in consecutive ticks, a 236 deg
            # setpoint spike that saturated yaw authority the wrong way and left
            # the vehicle swinging for seconds after the setpoint snapped back.
            # One stale leg is a far smaller error; PATH_STALE_S bounds it.

    def _path_cb(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._path_at = self._t_ros()
        if not self._path or self._pose is None:
            self._wp_idx = 0
            return
        distances = [
            math.hypot(x - self._pose[0], y - self._pose[1]) for x, y in self._path
        ]
        self._wp_idx = int(np.argmin(distances))

    def _gyro_cb(self, msg: Vector3Stamped) -> None:
        """Body-frame yaw rate. Level enough here that z is the heading rate."""
        now = self._t_ros()
        prev = self._gyro_at
        self._gyro_at = now
        self._yaw_rate.update(msg.vector.z, now)
        # Carry the steering yaw between estimate corrections. Raw rate, not
        # the low-passed one: integration wants an unbiased sample.
        if (self._yaw_ctrl is not None and prev is not None
                and 0.0 < now - prev < self.ODOM_GAP_S):
            self._yaw_ctrl = wrap_angle(
                self._yaw_ctrl + msg.vector.z * (now - prev))

    def _fuse_steering_yaw(self, now: float, dt: 'float | None') -> None:
        """Pull the gyro-carried steering yaw toward the SLAM estimate.

        The estimate's yaw wanders at 0.11 rad/s sd against a true hull rate
        of 0.07 median (2026-08-01) -- steering on it directly turns estimate
        noise into real motion, which no gain choice can prevent. The gyro
        carries the high frequencies; the estimate wins over YAW_FUSE_TAU_S,
        so heading stays anchored to the SLAM frame without shaking with it.
        """
        if self._yaw_ctrl is None or self._gyro_stale(now):
            self._yaw_ctrl = self._yaw
            return
        innovation = wrap_angle(self._yaw - self._yaw_ctrl)
        if abs(innovation) > self.YAW_FUSE_SNAP_RAD:
            # A step this size is a graph correction, not noise -- follow it.
            self._yaw_ctrl = self._yaw
            return
        if dt is not None and dt > 0.0:
            alpha = dt / (self.YAW_FUSE_TAU_S + dt)
            self._yaw_ctrl = wrap_angle(self._yaw_ctrl + alpha * innovation)

    def _steering_yaw(self) -> float:
        return self._yaw if self._yaw_ctrl is None else self._yaw_ctrl

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._pose = np.array([p.x, p.y, p.z])
        self._yaw = yaw_from_quat(msg.pose.pose.orientation)
        prev_odom = self._odom_at
        self._odom_at = self._t_ros()
        self._fuse_steering_yaw(
            self._odom_at,
            None if prev_odom is None else self._odom_at - prev_odom)
        self._depth_rate.update(float(p.z), self._odom_at)
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

        The axis is taken from the bearing to the wall itself and then held for
        the whole dwell. Deriving it from the route heading instead closed a
        loop: the route heading defined the axis, the axis placed the target,
        and driving to the target redefined the route heading -- measured
        2026-08-01 spinning the viewing heading through four bearings 90 deg
        apart once a second, for the whole patrol.
        """
        if self._patrol_anchor is None:
            self._patrol_anchor = self._pose[:2].copy()
            self._patrol_tangent = None
            self._patrol_look_delta = None
        if self._patrol_tangent is None:
            wall = self._wall_memory
            if wall is None or self._wall_side == 0:
                return self._patrol_anchor
            to_wall = math.atan2(wall[1] - self._pose[1], wall[0] - self._pose[0])
            self._patrol_tangent = wrap_angle(to_wall + math.pi / 2.0)
            # Held relative to the wall, but starting from the heading transit
            # was already commanding, so entering the dwell asks for no turn at
            # all. Pointing straight at the wall instead costs a 45 deg swing on
            # arrival -- the offset is 45 deg off the route and the wall is
            # roughly abeam of it -- which is a turn the dwell has no use for.
            self._patrol_look_delta = (
                0.0 if self._last_look_heading is None
                else wrap_angle(self._last_look_heading - to_wall))
        tangent = self._patrol_tangent
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
            self._patrol_tangent = None
            self._patrol_look_delta = None
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

    def _resign_side_from_memory(self, route_heading: float, now: float) -> None:
        """Keep the held side attached to the wall, not to the route.

        The side is route-relative, so a direction change flips which side the
        same physical wall is on. That is a frame change, not a wall change --
        making it earn the switch dwell left the viewing offset pointing into
        open water for seconds at every reversal (measured 2026-08-01: a
        ~180 deg look sweep and a 3-5 s stall at each waypoint or revisit
        direction change). The dwell in _select_wall_side still gates changes
        of wall.
        """
        wall = self._wall_memory
        if wall is None or self._wall_side == 0:
            return
        lateral = ((wall[0] - self._pose[0]) * -math.sin(route_heading)
                   + (wall[1] - self._pose[1]) * math.cos(route_heading))
        if abs(lateral) < self.MEMORY_SIDE_MIN_LATERAL_M:
            return
        side = 1 if lateral > 0.0 else -1
        if side != self._wall_side:
            self._wall_side = side
            self._side_candidate = side
            self._side_candidate_t = now

    def _select_wall_side(self, route_heading: float, now: float) -> None:
        self._resign_side_from_memory(route_heading, now)
        if (
            self._map_points is None
            or self._map_received_at is None
            or now - self._map_received_at > self.MAP_STALE_S
        ):
            # The side is deliberately kept. Zeroing it drops the viewing offset
            # to zero and then re-signs it by 2*look_offset_deg when the map
            # returns -- a 90 deg setpoint step for what was only a gap in the
            # scan. Distance goes NaN because that is genuinely unmeasured now.
            self._wall_distance = float("nan")
            self._publish_selected_wall(self._wall_memory)
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
        if candidate == 0 and self._wall_side != 0:
            # Nothing in the depth band this tick — a turn sweeps the sonar off
            # the structure for seconds at a time. Hold the side already chosen
            # rather than treating a gap in the scan as the wall having gone.
            self._wall_distance = float("nan")
            self._publish_selected_wall(self._wall_memory)
            return
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
        if abs(self._last_heading_error) > self.SIDE_SWITCH_SETTLED_RAD:
            self._settled_since = None
        elif self._settled_since is None:
            self._settled_since = now
        settled = (self._settled_since is not None
                   and now - self._settled_since >= self.SIDE_SWITCH_SETTLED_S)
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
        wall_point = nearest_wall_point(
            self._map_points, self._pose, route_heading, self._wall_z_band,
            self._max_wall_distance, self._wall_side)
        if wall_point is not None:
            self._wall_memory = np.asarray(wall_point, dtype=np.float64).copy()
        self._publish_selected_wall(wall_point if wall_point is not None
                                    else self._wall_memory)

    def _publish_selected_wall(self, wall_point) -> None:
        self._selected_wall_pub.publish(selected_wall_markers(
            self._pose, wall_point, self._voxel_size,
            self.get_clock().now().to_msg()))

    def _arrival_brake(self, goal_dist: float) -> float:
        """Translation scale: 1.0 beyond the ramp, ARRIVAL_MIN_SCALE at the radius."""
        span = self.ARRIVAL_BRAKE_M - self.GOAL_RADIUS
        if span <= 0.0:
            return 1.0
        return float(np.clip((goal_dist - self.GOAL_RADIUS) / span,
                             self.ARRIVAL_MIN_SCALE, 1.0))

    def _gyro_stale(self, now: float) -> bool:
        return self._gyro_at is None or now - self._gyro_at > self.GYRO_STALE_S

    def _yaw_effort(self, heading_error: float, now: float) -> float:
        """Cascaded heading hold, shared by the hold and drive paths.

        Without a rate the cascade must not simply run with measured_rate=0:
        that leaves a proportional law of effective gain KP_HEADING *
        KP_YAW_RATE = 1.2, four times the law this replaced and twice the 0.60
        measured to diverge, saturating at 14 deg of error instead of 57. Losing
        damping should not also mean losing the gain ceiling, so the fallback is
        the explicit pre-cascade law rather than a degenerate case of this one.
        """
        if self._gyro_stale(now):
            return float(np.clip(self.KP_YAW_FALLBACK * heading_error,
                                 -self.YAW_EFFORT_LIMIT, self.YAW_EFFORT_LIMIT))
        return yaw_rate_command(
            heading_error,
            self._yaw_rate.value,
            self.KP_HEADING,
            self.KP_YAW_RATE,
            self.MAX_YAW_RATE,
            limit=self.YAW_EFFORT_LIMIT,
        )

    def _hold_yaw_cmd(self, now: float) -> tuple:
        """Yaw effort holding the viewing heading while the vehicle waits.

        Waiting pointed at the wall beats scanning: the sonar already faces the
        structure, and a spin throws away a heading the loop needs ~8 s to win
        back at the yaw rate this vehicle achieves.
        """
        route_heading = self._last_route_heading
        if route_heading is None:
            return 0.0, float("nan"), float("nan"), float("nan")
        self._select_wall_side(route_heading, now)
        raw_look = offset_heading(
            route_heading, self._wall_side, self._look_offset_deg
        )
        # Raw error gates the side switch (distance to the final target);
        # control tracks the shaped reference.
        steering_yaw = self._steering_yaw()
        self._last_heading_error = wrap_angle(raw_look - steering_yaw)
        look_heading = self._look_ref.update(raw_look, now, steering_yaw)
        heading_error = wrap_angle(look_heading - steering_yaw)
        self._last_look_heading = look_heading
        yaw_cmd = self._yaw_effort(heading_error, now)
        return yaw_cmd, route_heading, look_heading, heading_error

    def _wall_look_heading(self) -> 'float | None':
        """Bearing to the wall last seen, or None if none has been yet."""
        wall = self._wall_memory
        if wall is None:
            return None
        bearing = math.atan2(wall[1] - self._pose[1], wall[0] - self._pose[0])
        return wrap_angle(bearing + (self._patrol_look_delta or 0.0))

    def _drive(self, target_xy: np.ndarray, now: float,
               look_at_wall: bool = False) -> tuple:
        delta = target_xy - self._pose[:2]
        distance = float(np.hypot(*delta))
        travel_heading = math.atan2(delta[1], delta[0])
        _, look_path_heading = lookahead_path_heading(
            self._path, self._wp_idx, self._pose[:2], self._lookahead_m, travel_heading
        )
        self._select_wall_side(look_path_heading, now)
        self._last_route_heading = look_path_heading
        # The patrol beat reverses at each end, which swings a route-relative
        # viewing heading by 180 deg for a vehicle that has not turned. Facing
        # the wall itself is invariant to which way along it we are going, and
        # the hull is holonomic, so it strafes the beat without re-aiming.
        raw_look = self._wall_look_heading() if look_at_wall else None
        if raw_look is None:
            raw_look = offset_heading(
                look_path_heading, self._wall_side, self._look_offset_deg
            )
        # Raw error gates the side switch (distance to the final target);
        # control tracks the shaped reference.
        steering_yaw = self._steering_yaw()
        self._last_heading_error = wrap_angle(raw_look - steering_yaw)
        look_heading = self._look_ref.update(raw_look, now, steering_yaw)
        heading_error = wrap_angle(look_heading - steering_yaw)
        self._last_look_heading = look_heading
        yaw_cmd = self._yaw_effort(heading_error, now)

        # Reduce travel while the requested viewing heading is far away, then
        # project the unchanged route velocity onto the current body axes.
        # Raw error, deliberately not the shaped one. This gate doubles as the
        # interlock the waypoint tracker depends on: translation waits until
        # the vehicle points roughly at its final aim. Gating on the shaped
        # error let it translate mid-glide, overshoot the (densified) waypoints
        # between 1 Hz replans, and put the current waypoint behind it -- the
        # route heading then flipped ~180 deg once a second (54 look jumps
        # >60 deg in 13 min, measured 2026-08-01). Shaping is for the yaw loop
        # only. Floored at ALIGN_MIN_SCALE rather than zero, so a re-aim slows
        # the vehicle instead of freezing it (see the constant).
        alignment = max(self.ALIGN_MIN_SCALE, math.cos(self._last_heading_error))
        speed = float(
            np.clip(self.KP_SPEED * distance * alignment, 0.0, self.MAX_SPEED)
        )
        route_in_body = wrap_angle(travel_heading - self._steering_yaw())
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

        # Cleared every tick, re-set only by the paths that close on a heading
        # (_hold_yaw_cmd, _drive). An open-loop spin — initial scan, stuck
        # escape, stale-pose hold — therefore draws no commanded arrow, and a
        # future one inherits that without needing to know about the marker.
        self._last_look_heading = None

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

        # Unlike the pose, a missing rate is not a reason to stop — _yaw_effort
        # falls back to the pre-cascade law. reset() rather than assignment so
        # the filter re-seeds on the first sample back instead of ramping up
        # from zero, and so the CSV logs no rate rather than a stale one.
        if self._gyro_stale(now):
            if self._yaw_rate.value != 0.0:
                self._yaw_rate.reset()
            self.get_logger().warn(
                f"no gyro on {self._gyro_topic} — yaw damping off, holding "
                f"heading with the pre-cascade law (KP {self.KP_YAW_FALLBACK})",
                throttle_duration_sec=10.0,
            )

        heave = self._heave_cmd()
        if self._init_scan_end is not None and now < self._init_scan_end:
            self._send_thrust(0.0, 0.0, self._scan_yaw(), heave)
            if write_csv:
                self._write_csv(0.0, 0.0, self._scan_yaw(), heave, "INIT_SCAN")
            return
        if self._goal is None:
            yaw_cmd, route_hdg, look_hdg, hdg_err = self._hold_yaw_cmd(now)
            self._send_thrust(0.0, 0.0, yaw_cmd, heave)
            if write_csv:
                self._write_csv(0.0, 0.0, yaw_cmd, heave, "WALL_HOLD",
                                float("nan"), route_hdg, look_hdg, hdg_err)
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
                yaw_cmd, route_hdg, look_hdg, hdg_err = self._hold_yaw_cmd(now)
                self._send_thrust(0.0, 0.0, yaw_cmd, heave)
                if write_csv:
                    self._write_csv(
                        0.0,
                        0.0,
                        yaw_cmd,
                        heave,
                        "GOAL_REACHED",
                        goal_dist,
                        route_hdg,
                        look_hdg,
                        hdg_err,
                    )
                return

        if self._path and self._path_at is not None and (
            now - self._path_at > self.PATH_STALE_S
        ):
            self._path = []
            self._wp_idx = 0
            self.get_logger().warn(
                f"no replan for {self.PATH_STALE_S:.0f}s — steering straight at the goal",
                throttle_duration_sec=5.0,
            )

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
        ) = self._drive(target_xy, now, look_at_wall=patrol_target is not None)
        event = "REVISIT_PATROL" if patrol_target is not None else ""
        if patrol_target is None:
            # Not while patrolling: the beat runs inside GOAL_RADIUS, where the
            # ramp is zero, and would brake the vehicle to a standstill.
            brake = self._arrival_brake(goal_dist)
            surge *= brake
            sway *= brake
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
        # Drawn here rather than per caller: every command leaves through this
        # method, so the arrow cannot show a setpoint no command was sent for.
        self._heading_pub.publish(heading_markers(
            self._pose, self._last_look_heading, self._steering_yaw(),
            self.get_clock().now().to_msg()))

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
