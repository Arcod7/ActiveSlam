#!/usr/bin/env python3
"""Render an organized point cloud back into a range image for RViz.

The noised sonar cloud only exists as a PointCloud2 (/cloud_in), so there is no
depth-camera-style 2D view of what SLAM actually consumes. This node turns any
organized cloud back into a sensor_msgs/Image (32FC1, per-pixel range in metres,
dropouts/invalid = 0), viewable in an RViz Image display exactly like the clean
depth camera. Point it at /cloud_in to see the noise profile (plus whatever
near-field gate / Noise Attenuation is active), or at /cloud_in_raw for the
clean reference.
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


class RangeImageNode(Node):
    def __init__(self, **kwargs):
        super().__init__('range_image', **kwargs)
        self.declare_parameter('input_topic', '/cloud_in')
        self.declare_parameter('output_topic', '/cloud_in/range_image')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(Image, output_topic, 5)
        self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos_profile_sensor_data)
        self.get_logger().info(f"RangeImage started: {input_topic} -> {output_topic}")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if msg.height <= 1 or msg.point_step < 12:
            return   # only organized clouds have an image to render
        floats_per_point = msg.point_step // 4
        xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats_per_point)[:, :3]
        rng = np.linalg.norm(xyz, axis=1).astype(np.float32)
        rng[~np.isfinite(rng)] = 0.0   # dropouts / no-return -> black

        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = '32FC1'
        out.is_bigendian = 0
        out.step = 4 * msg.width
        out.data = rng.tobytes()
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RangeImageNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
