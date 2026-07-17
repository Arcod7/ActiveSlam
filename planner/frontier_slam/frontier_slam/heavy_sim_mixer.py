"""Simulation-only body-command mixer for Stonefish's BlueROV2 Heavy."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from frontier_slam.control_utils import mix_thrusters


class HeavySimulationMixer(Node):
    """Convert a gated body command to the eight Stonefish actuator values."""

    def __init__(self) -> None:
        super().__init__('heavy_sim_mixer')
        self.declare_parameter('command_topic', '/motion/body_command_safe')
        self.declare_parameter(
            'thruster_topic', '/bluerov2/controller/thruster_setpoints_sim')
        command_topic = str(self.get_parameter('command_topic').value)
        thruster_topic = str(self.get_parameter('thruster_topic').value)
        self._publisher = self.create_publisher(
            Float64MultiArray, thruster_topic, 10)
        self.create_subscription(Twist, command_topic, self._command_cb, 10)
        self.publish_zero()
        self.get_logger().info(
            f'Heavy simulation mixer ready — command={command_topic} '
            f'thrusters={thruster_topic}')

    def _command_cb(self, msg: Twist) -> None:
        command = Float64MultiArray()
        command.data = [float(value) for value in mix_thrusters(
            msg.linear.x, msg.angular.z, msg.linear.z, msg.linear.y)]
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
