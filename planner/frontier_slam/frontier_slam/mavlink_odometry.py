"""ArduSub navigation state as ROS odometry and TF, for real-vehicle runs.

Stonefish publishes /StoneFish/Odometry and broadcasts world_ned -> base_link;
on hardware nothing does. This node fills that hole from the autopilot's own
estimate so the safety gate has a freshness source and the mapper has a pose to
integrate clouds against. It is read-only MAVLink: it requests message
intervals and never arms, disarms or changes mode.

Use a MAVLink endpoint of its own. pymavlink binds the UDP port, so pointing
this and ardusub_adapter at the same `udpin` URL makes the second one fail.

ArduSub only has a horizontal position solution when a DVL or GPS feeds its
EKF. Without one, LOCAL_POSITION_NED x/y stay at the origin while depth and
attitude remain good — set require_position false to say so explicitly rather
than mapping against a position that is not being estimated.
"""

import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PointStamped, QuaternionStamped, TransformStamped, TwistStamped,
    Vector3Stamped,
)
from pymavlink import mavutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster

from frontier_slam.control_utils import quat_from_rpy, world_to_body

# Position that never leaves the origin means the EKF has no horizontal source.
STATIONARY_WARN_S = 20.0
STATIONARY_EPS_M = 0.01


class MavlinkOdometry(Node):
    """Publish ArduSub LOCAL_POSITION_NED + ATTITUDE as odometry and TF."""

    def __init__(self) -> None:
        """Open a read-only MAVLink endpoint and start requesting messages."""
        super().__init__('mavlink_odometry')
        self.declare_parameter('connection_url', 'udpin:0.0.0.0:14561')
        self.declare_parameter('source_system', 192)
        self.declare_parameter('source_component', 192)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('odometry_topic', '/mavlink/odometry')
        self.declare_parameter('world_frame', 'world_ned')
        self.declare_parameter('base_frame', 'bluerov2/base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('poll_rate_hz', 100.0)
        self.declare_parameter('message_rate_hz', 20.0)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('require_position', True)
        self.declare_parameter('position_variance_m2', 0.25)
        self.declare_parameter('depth_variance_m2', 0.01)
        self.declare_parameter('orientation_variance_rad2', 0.02)
        # The four topics dead_reckoning.py fuses. Publishing them here is what
        # lets the existing pose graph run on hardware unmodified: the same
        # interface the simulated sensor node fills, filled from the autopilot.
        self.declare_parameter('publish_slam_sensors', True)
        self.declare_parameter('slam_sensor_prefix', '/slam/sensors')
        # ArduSub's barometric depth (VFR_HUD) is a measurement; the EKF's z is
        # an estimate that drifts with it. Default to the measurement.
        self.declare_parameter('depth_source', 'vfr_hud')
        # The only thing this node ever transmits, and it is a telemetry-rate
        # request, never a command. A freshly created BlueOS endpoint carries
        # ATTITUDE/LOCAL_POSITION_NED too slowly to hold the gate's freshness
        # window, so this defaults on; set it false to make the link strictly
        # receive-only and accept whatever rate the endpoint already has.
        self.declare_parameter('request_streams', True)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._target_system = int(self.get_parameter('target_system').value)
        self._target_component = int(
            self.get_parameter('target_component').value)
        self._message_rate_hz = self._positive_parameter('message_rate_hz')
        self._stale_timeout_s = self._positive_parameter('stale_timeout_s')
        self._require_position = bool(
            self.get_parameter('require_position').value)
        self._position_var = float(
            self.get_parameter('position_variance_m2').value)
        self._depth_var = float(
            self.get_parameter('depth_variance_m2').value)
        self._orientation_var = float(
            self.get_parameter('orientation_variance_rad2').value)

        connection_url = str(self.get_parameter('connection_url').value)
        self._link = mavutil.mavlink_connection(
            connection_url,
            source_system=int(self.get_parameter('source_system').value),
            source_component=int(
                self.get_parameter('source_component').value),
            autoreconnect=True,
        )

        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter('odometry_topic').value), 10)
        self._tf = (TransformBroadcaster(self)
                    if bool(self.get_parameter('publish_tf').value) else None)

        self._depth_source = str(self.get_parameter('depth_source').value)
        if self._depth_source not in ('vfr_hud', 'local_ned'):
            raise ValueError("depth_source must be 'vfr_hud' or 'local_ned'")
        prefix = str(self.get_parameter('slam_sensor_prefix').value).rstrip('/')
        self._sensor_pubs = {}
        if bool(self.get_parameter('publish_slam_sensors').value):
            self._sensor_pubs = {
                'depth': self.create_publisher(
                    PointStamped, f'{prefix}/pressure_depth', 10),
                'imu': self.create_publisher(
                    QuaternionStamped, f'{prefix}/imu_orientation', 10),
                'compass': self.create_publisher(
                    Vector3Stamped, f'{prefix}/compass_heading', 10),
                'dvl': self.create_publisher(
                    TwistStamped, f'{prefix}/dvl_velocity', 10),
            }

        self._position = None
        self._position_time: float | None = None
        self._attitude = None
        self._attitude_time: float | None = None
        self._vfr_hud = None
        self._heartbeat_time: float | None = None
        self._request_streams_enabled = bool(
            self.get_parameter('request_streams').value)
        self._requested = not self._request_streams_enabled
        self._origin: tuple[float, float] | None = None
        self._origin_time: float | None = None
        self._stationary_warned = False
        self._publishing = False

        self.create_timer(
            1.0 / self._positive_parameter('poll_rate_hz'),
            self._poll_mavlink)
        self.create_timer(
            1.0 / self._positive_parameter('publish_rate_hz'),
            self._publish_tick)
        self.get_logger().info(
            f'mavlink_odometry listening on {connection_url}; '
            f'{self._world_frame} -> {self._base_frame}, '
            f'require_position={self._require_position} '
            f'request_streams={self._request_streams_enabled}')

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _request_streams(self) -> None:
        """Ask the autopilot for the two messages this node reads."""
        interval_us = 1e6 / self._message_rate_hz
        for message_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                           mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                           mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD):
            self._link.mav.command_long_send(
                self._target_system, self._target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                message_id, interval_us, 0, 0, 0, 0, 0)
        self._requested = True

    def _poll_mavlink(self) -> None:
        now = self._now()
        try:
            while True:
                msg = self._link.recv_match(blocking=False)
                if msg is None:
                    break
                if int(msg.get_srcSystem()) != self._target_system:
                    continue
                kind = msg.get_type()
                if kind == 'HEARTBEAT':
                    self._heartbeat_time = now
                elif kind == 'LOCAL_POSITION_NED':
                    self._position, self._position_time = msg, now
                elif kind == 'ATTITUDE':
                    self._attitude, self._attitude_time = msg, now
                elif kind == 'VFR_HUD':
                    self._vfr_hud = msg
        except Exception as exc:      # a dead link must stop the odometry
            self._position_time = self._attitude_time = None
            self.get_logger().warn(
                f'MAVLink receive failed: {type(exc).__name__}')
            self._requested = not self._request_streams_enabled
            return

        # Re-request after a gap: a reconnected autopilot forgets the interval.
        if not self._request_streams_enabled:
            return
        if self._heartbeat_time is not None and not self._requested:
            self._request_streams()
        if (self._heartbeat_time is not None
                and now - self._heartbeat_time > self._stale_timeout_s):
            self._requested = False

    def _fresh(self, stamp: float | None, now: float) -> bool:
        return stamp is not None and now - stamp <= self._stale_timeout_s

    def _check_stationary(self, x: float, y: float, now: float) -> None:
        """Warn once if the EKF's horizontal position never leaves its origin."""
        if not self._require_position or self._stationary_warned:
            return
        if self._origin is None:
            self._origin, self._origin_time = (x, y), now
            return
        if math.dist((x, y), self._origin) > STATIONARY_EPS_M:
            self._stationary_warned = True     # it moves; nothing to report
            return
        if now - (self._origin_time or now) < STATIONARY_WARN_S:
            return
        self._stationary_warned = True
        self.get_logger().warn(
            f'LOCAL_POSITION_NED has not moved in {STATIONARY_WARN_S:.0f} s. '
            'ArduSub has no horizontal position source (no DVL/GPS): depth '
            'and attitude are usable, X/Y are not. Any map built against this '
            'pose is wrong as soon as the vehicle translates.')

    def _publish_tick(self) -> None:
        now = self._now()
        if not (self._fresh(self._position_time, now)
                and self._fresh(self._attitude_time, now)):
            if self._publishing:
                self.get_logger().warn(
                    'ArduSub navigation stale — odometry stopped; the safety '
                    'gate will report STALE_ODOMETRY')
                self._publishing = False
            return
        if not self._publishing:
            self.get_logger().info('ArduSub navigation fresh — odometry live')
            self._publishing = True

        position, attitude = self._position, self._attitude
        roll, pitch, yaw = attitude.roll, attitude.pitch, attitude.yaw
        x = position.x if self._require_position else 0.0
        y = position.y if self._require_position else 0.0
        # One depth for the pose, the TF and the SLAM feed. The EKF's own z
        # drifts away from the barometer it is initialised from, so publishing
        # each from its own source put the TF and the pose graph's depth prior
        # metres apart on this vehicle.
        depth = self._depth()
        if depth is None:
            depth = float(position.z)
        self._check_stationary(position.x, position.y, now)

        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
        vx, vy, vz = world_to_body(
            position.vx if self._require_position else 0.0,
            position.vy if self._require_position else 0.0,
            position.vz, roll, pitch, yaw)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._world_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(depth)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Unestimated X/Y are reported as such rather than as a confident zero.
        xy_var = self._position_var if self._require_position else 1e6
        for index, variance in enumerate(
                (xy_var, xy_var, self._depth_var, self._orientation_var,
                 self._orientation_var, self._orientation_var)):
            odom.pose.covariance[index * 7] = variance
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)
        odom.twist.twist.angular.x = float(attitude.rollspeed)
        odom.twist.twist.angular.y = float(attitude.pitchspeed)
        odom.twist.twist.angular.z = float(attitude.yawspeed)
        self._odom_pub.publish(odom)

        self._publish_slam_sensors(stamp, depth, roll, pitch, yaw, vx, vy, vz)

        if self._tf is None:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._world_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(depth)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf.sendTransform(transform)

    def _depth(self) -> float | None:
        """Depth in metres, positive down, or None until a source has arrived."""
        if self._depth_source == 'vfr_hud':
            # VFR_HUD carries altitude; ArduSub's depth is its negation.
            return None if self._vfr_hud is None else -float(self._vfr_hud.alt)
        return float(self._position.z)

    def _publish_slam_sensors(self, stamp, depth, roll, pitch, yaw,
                              vx, vy, vz) -> None:
        """Fill dead_reckoning.py's four inputs from the autopilot's state.

        Body-frame velocity stands in for the DVL: on this vehicle the
        horizontal solution reaching LOCAL_POSITION_NED is bottom track from
        the Nortek Nucleus via VISION_POSITION_DELTA, so it is a velocity
        measurement wearing a different message. It is the driving callback, so
        it is published last.
        """
        if not self._sensor_pubs:
            return

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self._world_frame
        point.point.z = depth
        self._sensor_pubs['depth'].publish(point)

        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
        orientation = QuaternionStamped()
        orientation.header.stamp = stamp
        orientation.header.frame_id = self._base_frame
        (orientation.quaternion.x, orientation.quaternion.y,
         orientation.quaternion.z, orientation.quaternion.w) = qx, qy, qz, qw
        self._sensor_pubs['imu'].publish(orientation)

        heading = Vector3Stamped()
        heading.header.stamp = stamp
        heading.header.frame_id = self._base_frame
        heading.vector.z = float(yaw)
        self._sensor_pubs['compass'].publish(heading)

        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = self._base_frame
        velocity.twist.linear.x = float(vx)
        velocity.twist.linear.y = float(vy)
        velocity.twist.linear.z = float(vz)
        self._sensor_pubs['dvl'].publish(velocity)

    def stop(self) -> None:
        """Close the MAVLink endpoint."""
        try:
            self._link.close()
        except Exception:
            pass


def main(args=None) -> None:
    """Run the ArduSub odometry bridge until shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MavlinkOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
