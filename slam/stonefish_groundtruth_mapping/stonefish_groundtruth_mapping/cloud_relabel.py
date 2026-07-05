#!/usr/bin/env python3
"""
Republishes a PointCloud2 with its header.frame_id swapped, unchanged
otherwise.

The depth camera's raw point cloud is identical regardless of which pose
source (ground truth vs SLAM estimate) is active — only the pose used to
place it in world_ned differs. This node lets a second mapper instance
consume the same sensor data through a different TF chain (e.g.
bluerov2/Dcam_gt instead of bluerov2/Dcam) without recomputing anything,
by just relabeling which frame the points are declared to be in.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudRelabel(Node):
    def __init__(self):
        super().__init__('cloud_relabel')
        self.declare_parameter('input_topic', '/cloud_in')
        self.declare_parameter('output_topic', '/gt/cloud_in')
        self.declare_parameter('frame_id', 'bluerov2/Dcam_gt')

        self._frame_id = self.get_parameter('frame_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'cloud_relabel: {input_topic} -> {output_topic} '
            f'(frame_id -> {self._frame_id})')

    def _cloud_cb(self, msg: PointCloud2) -> None:
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRelabel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
