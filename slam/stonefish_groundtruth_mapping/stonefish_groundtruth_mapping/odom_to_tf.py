#!/usr/bin/env python3
"""Republishes /StoneFish/Odometry as a TF transform (world_ned → bluerov2/base_link)."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self._br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, '/StoneFish/Odometry', self._callback, 10)
        self.get_logger().info('odom_to_tf started — listening on /StoneFish/Odometry')

    def _callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world_ned'
        t.child_frame_id = 'bluerov2/base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
