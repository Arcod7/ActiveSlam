#!/usr/bin/env python3
"""
Fix two problems before depth_image_proc sees the depth image:

  1. Zero pixels → NaN
     Stonefish writes 0 for no-return pixels. depth_image_proc converts those to
     (0,0,0) in camera frame, which octomap_server marks as an occupied voxel at
     the sensor origin every frame.

  2. Timestamp correction (GPU render latency)
     Stonefish stamps the depth image *after* GPU readback, so the timestamp is
     T_physics + Δ_render, not T_physics. The odometry is stamped at T_physics
     (CPU-only, negligible latency). When the robot moves, looking up the TF at
     T_physics + Δ_render gives the wrong robot pose.

     Fix: replace the image stamp with the most recent odometry stamp that
     arrived before the image. Because the depth camera fires every 10th physics
     step (100 Hz odometry / 10 Hz camera = 10), there is always an odometry
     sample at exactly T_physics waiting in the buffer.

  Output namespace /depth_cam/
     Both image_rect and camera_info are published under the same namespace so
     depth_image_proc's image_transport::CameraSubscriber auto-derives the
     camera_info path correctly (it strips the image path's last component and
     appends "camera_info" — remappings alone cannot override this behaviour).
"""
import collections
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

_ODOM_BUFFER = 64   # ~0.64 s at 100 Hz


class DepthFix(Node):
    def __init__(self):
        super().__init__('depth_fix')
        self._bridge = CvBridge()
        self._odom_stamps: collections.deque = collections.deque(maxlen=_ODOM_BUFFER)

        self.create_subscription(
            Odometry, '/StoneFish/Odometry', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/sensor_msgs/image_depth', self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/sensor_msgs/camera_info', self._info_cb, qos_profile_sensor_data)

        # Publish under /depth_cam/ so image_transport derives camera_info
        # automatically as /depth_cam/camera_info when given /depth_cam/image_rect.
        self._pub_image = self.create_publisher(Image,      '/depth_cam/image_rect',  10)
        self._pub_info  = self.create_publisher(CameraInfo, '/depth_cam/camera_info', 10)

        self.get_logger().info('depth_fix started (NaN + timestamp correction)')

    def _odom_cb(self, msg: Odometry):
        self._odom_stamps.append(msg.header.stamp)

    def _depth_cb(self, msg: Image):
        depth = self._bridge.imgmsg_to_cv2(msg, '32FC1').copy()
        depth[depth <= 0.0] = float('nan')
        fixed = self._bridge.cv2_to_imgmsg(depth, '32FC1')
        fixed.header = msg.header
        best = self._best_odom_stamp(msg.header.stamp)
        if best is not None:
            fixed.header.stamp = best
        else:
            self.get_logger().warn(
                f'no odom stamp <= image stamp {_to_ns(msg.header.stamp)} — '
                'passing raw stamp through (TF lookup may fail)',
                throttle_duration_sec=2.0)
        self._pub_image.publish(fixed)

    def _info_cb(self, msg: CameraInfo):
        import copy
        fixed = copy.copy(msg)
        best = self._best_odom_stamp(msg.header.stamp)
        if best is not None:
            fixed.header.stamp = best
        self._pub_info.publish(fixed)

    def _best_odom_stamp(self, img_stamp):
        """Return the most recent odometry stamp that is <= img_stamp."""
        if not self._odom_stamps:
            return None
        img_ns = _to_ns(img_stamp)
        best, best_ns = None, -1
        for s in self._odom_stamps:
            s_ns = _to_ns(s)
            if s_ns <= img_ns and s_ns > best_ns:
                best, best_ns = s, s_ns
        return best


def _to_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main(args=None):
    rclpy.init(args=args)
    node = DepthFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
