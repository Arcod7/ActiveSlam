"""Fail-closed ROS 2 gate for normalized vehicle body commands.

Also publishes the vehicle as a state-coloured arrow on /motion/robot_marker
(green = motion enabled, purple = disabled) — the gate is the only node that
knows both the pose it watches and whether motion is armed.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker

from frontier_slam.safety_logic import MotionSafetyState

# Green when the gate passes motion, purple when it is disabled — the same
# reading as the launcher's target-point sphere.
ENABLED_COLOR = ColorRGBA(r=0.16, g=0.86, b=0.16, a=0.95)
DISABLED_COLOR = ColorRGBA(r=0.70, g=0.20, b=0.90, a=0.95)


class MotionSafetyGate(Node):
    """Pass body demand only when explicitly enabled with fresh inputs."""

    def __init__(self) -> None:
        super().__init__('motion_safety_gate')
        self.declare_parameter('command_topic', '/motion/body_command')
        self.declare_parameter('output_topic', '/motion/body_command_safe')
        self.declare_parameter('enable_topic', '/motion/enable')
        self.declare_parameter('status_topic', '/motion/safety_status')
        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('max_abs_command', 1.0)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('marker_topic', '/motion/robot_marker')
        self.declare_parameter('marker_frame', 'world_ned')

        command_topic = str(self.get_parameter('command_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be positive')

        self._state = MotionSafetyState(
            command_timeout_s=float(
                self.get_parameter('command_timeout_s').value),
            odom_timeout_s=float(self.get_parameter('odom_timeout_s').value),
            max_abs_command=float(
                self.get_parameter('max_abs_command').value),
            require_odom=bool(self.get_parameter('require_odom').value),
        )
        self._state.set_enabled(
            bool(self.get_parameter('start_enabled').value))
        self._command_topic = command_topic
        self._last_status: str | None = None
        self._marker_frame = str(self.get_parameter('marker_frame').value)
        self._last_pose = None

        self._output_pub = self.create_publisher(Twist, output_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('marker_topic').value), 1)
        self.create_subscription(
            Twist, command_topic, self._command_cb, 10)
        self.create_subscription(Bool, enable_topic, self._enable_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(1.0 / publish_hz, self._tick)
        self.publish_zero()

        self.get_logger().info(
            f'motion safety gate ready — command={command_topic} '
            f'output={output_topic} enable={enable_topic} odom={odom_topic} '
            f'start_enabled={self._state.enabled}')

    def _now(self) -> float:
        # Safety timeouts must still expire if ROS simulation time pauses.
        return time.monotonic()

    @staticmethod
    def _values(msg: Twist) -> tuple[float, ...]:
        return (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )

    @staticmethod
    def _message(values) -> Twist:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = values[:3]
        msg.angular.x, msg.angular.y, msg.angular.z = values[3:]
        return msg

    def _command_cb(self, msg: Twist) -> None:
        if not self._state.update_command(self._values(msg), self._now()):
            self.get_logger().error(
                'Rejected body command: expected finite values within '
                f'[-{self._state.max_abs_command:.1f}, '
                f'{self._state.max_abs_command:.1f}]')

    def _enable_cb(self, msg: Bool) -> None:
        was_enabled = self._state.enabled
        self._state.set_enabled(msg.data)
        if self._state.enabled != was_enabled:
            action = 'ENABLED' if self._state.enabled else 'DISABLED'
            self.get_logger().warn(f'motion gate {action}')
        if not self._state.enabled:
            self.publish_zero()

    def _odom_cb(self, msg: Odometry) -> None:
        self._state.update_odometry(self._now())
        self._last_pose = msg.pose.pose

    def _publish_robot_marker(self) -> None:
        """The vehicle as an arrow coloured by whether motion is enabled.

        An RViz Odometry display carries a fixed colour, so the state has to
        come from the marker: this replaces the ground-truth arrow in
        demo.rviz. It follows the pose the gate itself watches — ground truth
        under slam:=none, /slam/odometry under slam:=slam — i.e. the pose the
        controller is acting on.
        """
        if self._last_pose is None:
            return
        marker = Marker()
        marker.header.frame_id = self._marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'motion_state'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = self._last_pose
        marker.scale.x = 1.0    # shaft length, along the vehicle's +X
        marker.scale.y = 0.16   # shaft diameter
        marker.scale.z = 0.16   # head diameter
        marker.color = ENABLED_COLOR if self._state.enabled else DISABLED_COLOR
        self._marker_pub.publish(marker)

    def _has_multiple_command_sources(self) -> bool:
        return len(self.get_publishers_info_by_topic(self._command_topic)) > 1

    def _tick(self) -> None:
        decision = self._state.evaluate(
            self._now(), self._has_multiple_command_sources())
        self._output_pub.publish(self._message(decision.output))
        status = String()
        status.data = decision.state
        self._status_pub.publish(status)
        self._publish_robot_marker()
        if decision.state != self._last_status:
            if decision.state == self._state.ACTIVE:
                self.get_logger().info(
                    f'motion safety state: {decision.state}')
            else:
                self.get_logger().warn(
                    f'motion safety state: {decision.state}')
            self._last_status = decision.state

    def publish_zero(self) -> None:
        self._output_pub.publish(self._message(self._state.zero))


def main(args=None) -> None:
    # Keep the context alive when Ctrl-C arrives so the final zero command can
    # be published before shutdown. The default Python SIGINT handler raises
    # KeyboardInterrupt, which is handled below.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionSafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
