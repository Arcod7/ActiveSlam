"""Wall-normal following motion controller.

A standalone motion mode, independent of the frontier/waypoint stack: the
robot locks its heading onto the nearest mapped surface — facing along the
inward TSDF surface normal — and strafes sideways along the wall at a fixed
standoff distance.  This is the "wall-normal + standoff targeting" idea from
the roadmap in its purest form, using the TSDF gradient as a control input.

Subscribed topics:
  /tsdf/surface_normals_cloud  (sensor_msgs/PointCloud2)
      fields x y z normal_x normal_y normal_z — sampled marching-cubes
      surface points with TSDF-gradient normals, from tsdf_mapper
  odometry                     (nav_msgs/Odometry, param `odom_topic`)
  /sensor_msgs/image_depth     (sensor_msgs/Image)  emergency back-off only

Published topics:
  /bluerov2/controller/thruster_setpoints_sim  (std_msgs/Float64MultiArray)

Behaviour:
  SCAN  — no usable wall (empty/stale TSDF, or nothing wall-like within
          MAX_SURFACE_DIST_M of the robot's depth band): slow rotation +
          depth hold, letting the mapper build surface.
  TRACK — yaw to face the wall (heading = −normal), regulate perpendicular
          distance to `standoff_m` with surge, strafe along the wall tangent
          with sway at `tangent_speed`.

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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray

from frontier_slam.control_utils import mix_thrusters, wrap_angle, yaw_from_quat
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


class WallFollower(Node):
    # P-gains
    KP_YAW      = 0.07   # same heading gain as waypoint_controller
    KP_STANDOFF = 0.50   # standoff-distance error → approach speed
    KP_HEAVE    = 0.40

    MAX_SURGE           = 0.20   # approach/retreat clamp
    MAX_SWAY            = 0.25   # tangential clamp
    SCAN_YAW            = 0.08   # rotation while no wall is in range
    MAX_SURFACE_DIST_M  = 8.0    # ignore surface farther than this (XY)
    Z_BAND_M            = 1.5    # only surface points within ±this of robot depth
    NORMAL_Z_MAX        = 0.7    # |n_z| above this = floor/ceiling, not a wall
    NORMAL_EMA_ALPHA    = 0.3    # smoothing on the tracked normal (nearest-point
                                 # jumps between samples cause yaw jitter otherwise)
    CLOUD_STALE_S       = 5.0    # no normals cloud for this long → SCAN
    EMERGENCY_STOP_DIST = 0.4    # m — front camera floor; below: forced back-off
    BACK_SURGE_SPEED    = 0.12   # m/s backward during emergency back-off
    CTRL_HZ             = 10.0
    LOG_EVERY_N_TICKS   = 10     # CSV row rate = CTRL_HZ / this → 1 Hz

    def __init__(self) -> None:
        super().__init__('wall_follower')

        self.declare_parameter('standoff_m',    1.5)
        self.declare_parameter('tangent_speed', 0.15)
        # +1 = strafe to the robot's right while facing the wall, -1 = left.
        self.declare_parameter('direction',     1)
        self.declare_parameter('depth_setpoint', -1.0)
        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('normals_topic', '/tsdf/surface_normals_cloud')

        self._standoff   = float(self.get_parameter('standoff_m').value)
        self._tan_speed  = float(self.get_parameter('tangent_speed').value)
        self._direction  = 1 if int(self.get_parameter('direction').value) >= 0 else -1
        v = float(self.get_parameter('depth_setpoint').value)
        self._depth_setpoint: float | None = None if v < 0 else v
        odom_topic    = str(self.get_parameter('odom_topic').value)
        normals_topic = str(self.get_parameter('normals_topic').value)

        self._pose: np.ndarray | None = None
        self._yaw  = 0.0
        self._min_front_dist = float('inf')
        self._wall_pts: np.ndarray | None = None   # (N,3) world
        self._wall_nrm: np.ndarray | None = None   # (N,3) unit
        self._cloud_t: float | None = None
        self._n_ema: np.ndarray | None = None      # smoothed planar normal (2,)
        self._tick = 0

        self._log = open_session_log('wall_follower', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(PointCloud2, normals_topic,             self._cloud_cb, 1)
        self.create_subscription(Odometry,    odom_topic,                self._odom_cb,  10)
        self.create_subscription(Image,       '/sensor_msgs/image_depth', self._depth_cb, 1)
        self._thrust_pub = self.create_publisher(
            Float64MultiArray, '/bluerov2/controller/thruster_setpoints_sim', 1,
        )

        self.create_timer(1.0 / self.CTRL_HZ, self._loop)
        self.get_logger().info(
            f'wall_follower ready — standoff={self._standoff:.1f}m '
            f'tangent_speed={self._tan_speed:.2f} direction={self._direction:+d} '
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
        self._yaw  = yaw_from_quat(msg.pose.pose.orientation)
        if self._depth_setpoint is None:
            self._depth_setpoint = float(p.z)
            self.get_logger().info(
                f'depth setpoint locked from odom at {self._depth_setpoint:.2f} m')

    def _depth_cb(self, msg: Image) -> None:
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        v = data[np.isfinite(data) & (data > 0.1)]
        self._min_front_dist = float(v.min()) if v.size > 0 else float('inf')

    def _t_ros(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Wall selection
    def _nearest_wall(self) -> 'tuple[np.ndarray, np.ndarray, int] | None':
        """Return (point_xyz, planar_normal_xy_unit, n_candidates) of the nearest
        wall-like surface sample, or None if no usable wall is in range."""
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
        i = int(np.argmin(d_xy))
        if d_xy[i] > self.MAX_SURFACE_DIST_M:
            return None

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

        return pts[i], n_xy, len(pts)

    # ------------------------------------------------------------------
    # Main loop
    def _loop(self) -> None:
        self._tick += 1
        write_csv = (self._tick % self.LOG_EVERY_N_TICKS == 0)

        if self._pose is None:
            return

        heave = self._heave_cmd()
        wall  = self._nearest_wall()

        if wall is None:
            # No usable surface yet — rotate slowly so the mapper sees more.
            self._n_ema = None
            self._send_thrust(0.0, self.SCAN_YAW, heave, 0.0)
            self.get_logger().info('No wall in range — scanning',
                                   throttle_duration_sec=2.0)
            if write_csv:
                self._write_csv(0.0, 0.0, self.SCAN_YAW, heave, 'SCAN')
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

        # Heading: face the wall (along −normal).
        yaw_des = math.atan2(-n[1], -n[0])
        hdg_err = wrap_angle(yaw_des - self._yaw)
        yaw_cmd = float(np.clip(self.KP_YAW * hdg_err, -1.0, 1.0))

        # Desired world-frame velocity: standoff regulation along −n,
        # constant cruise along the wall tangent.  tangent = (n_y, −n_x) so
        # that direction=+1 strafes to the robot's right once aligned.
        v_des = (self.KP_STANDOFF * (d - self._standoff) * -n
                 + self._tan_speed * self._direction * np.array([n[1], -n[0]]))

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
            f'wall=({wp[0]:.1f},{wp[1]:.1f})  d={d:.2f}m (target {self._standoff:.1f}) '
            f'hdg_err={math.degrees(hdg_err):+.0f}°  '
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

    def _send_thrust(self, surge: float, yaw: float, heave: float, sway: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in mix_thrusters(surge, yaw, heave, sway)]
        self._thrust_pub.publish(msg)

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
    node = WallFollower()
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
