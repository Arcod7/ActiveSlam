"""Simulation-only body-command mixer for Stonefish's BlueROV2 Heavy.

Direct Stonefish has no ArduSub attitude controller, so this node supplies the
level roll/pitch hold that ArduSub owns on the physical vehicle.  It must never
be used to command real ESC or servo channels.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String

from frontier_slam.control_utils import (
    attitude_hold_effort,
    mix_thrusters,
    roll_pitch_from_quat,
)


class HeavySimulationMixer(Node):
    """Convert a gated body command to the eight Stonefish actuator values."""

    def __init__(self) -> None:
        super().__init__('heavy_sim_mixer')
        self.declare_parameter('command_topic', '/motion/body_command_safe')
        self.declare_parameter(
            'thruster_topic', '/bluerov2/controller/thruster_setpoints_sim')
        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('safety_status_topic', '/motion/safety_status')
        self.declare_parameter('attitude_stabilization', True)
        self.declare_parameter('attitude_kp', 0.45)
        self.declare_parameter('attitude_kd', 0.18)
        self.declare_parameter('attitude_effort_limit', 0.35)
        self.declare_parameter('safety_status_timeout_s', 0.25)
        command_topic = str(self.get_parameter('command_topic').value)
        thruster_topic = str(self.get_parameter('thruster_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        safety_status_topic = str(
            self.get_parameter('safety_status_topic').value)
        self._stabilize = bool(
            self.get_parameter('attitude_stabilization').value)
        self._attitude_kp = float(self.get_parameter('attitude_kp').value)
        self._attitude_kd = float(self.get_parameter('attitude_kd').value)
        self._attitude_limit = float(
            self.get_parameter('attitude_effort_limit').value)
        self._safety_status_timeout = float(
            self.get_parameter('safety_status_timeout_s').value)
        if self._safety_status_timeout <= 0.0:
            raise ValueError('safety_status_timeout_s must be positive')
        # Validate parameters once at startup.
        attitude_hold_effort(
            0.0, 0.0, 0.0, 0.0,
            self._attitude_kp, self._attitude_kd, self._attitude_limit)

        self._attitude: tuple[float, float, float, float] | None = None
        self._safety_active = False
        self._last_safety_status_time: float | None = None
        self._publisher = self.create_publisher(
            Float64MultiArray, thruster_topic, 10)
        self.create_subscription(Twist, command_topic, self._command_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(
            String, safety_status_topic, self._safety_status_cb, 10)
        self.create_timer(
            min(0.05, self._safety_status_timeout / 2.0),
            self._safety_watchdog_cb)
        self.publish_zero()
        self.get_logger().info(
            f'Heavy simulation mixer ready — command={command_topic} '
            f'thrusters={thruster_topic} odom={odom_topic} '
            f'attitude_stabilization={self._stabilize}')

    def _odom_cb(self, msg: Odometry) -> None:
        roll, pitch = roll_pitch_from_quat(msg.pose.pose.orientation)
        self._attitude = (
            roll, pitch,
            float(msg.twist.twist.angular.x),
            float(msg.twist.twist.angular.y),
        )

    def _safety_status_cb(self, msg: String) -> None:
        was_active = self._safety_active
        self._last_safety_status_time = time.monotonic()
        self._safety_active = msg.data == 'ACTIVE'
        if was_active and not self._safety_active:
            self.publish_zero()

    def _safety_status_is_fresh(self) -> bool:
        return (
            self._last_safety_status_time is not None
            and time.monotonic() - self._last_safety_status_time
            <= self._safety_status_timeout
        )

    def _safety_watchdog_cb(self) -> None:
        if self._safety_active and not self._safety_status_is_fresh():
            self._safety_active = False
            self.get_logger().error(
                'Safety status timed out; forcing Stonefish thrusters to zero')
            self.publish_zero()

    def _command_cb(self, msg: Twist) -> None:
        if not self._safety_active or not self._safety_status_is_fresh():
            self.publish_zero()
            return

        roll_effort = pitch_effort = 0.0
        if self._stabilize and self._attitude is not None:
            roll_effort, pitch_effort = attitude_hold_effort(
                *self._attitude,
                self._attitude_kp,
                self._attitude_kd,
                self._attitude_limit,
            )
        command = Float64MultiArray()
        command.data = [float(value) for value in mix_thrusters(
            msg.linear.x, msg.angular.z, msg.linear.z, msg.linear.y,
            roll_effort, pitch_effort)]
        self._publisher.publish(command)

    def publish_zero(self) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0] * 8
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = HeavySimulationMixer()
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
