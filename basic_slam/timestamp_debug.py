#!/usr/bin/env python3
"""
Measures the timestamp lag between Stonefish depth images and odometry.

For each depth image, finds the most recent odometry stamp <= image stamp
and logs:
  - delta_ms : T_img - T_nearest_odom  (our estimate of GPU render latency)
  - robot speed at that moment
  - moving / static label

Prints a rolling summary every 10 frames.

Usage (after rebuild):
  ros2 run basic_slam timestamp_debug

Or without rebuild:
  python3 src/basic_slam/basic_slam/timestamp_debug.py
"""
import collections
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class TimestampDebug(Node):
    def __init__(self):
        super().__init__('timestamp_debug')
        self._odom_buffer: collections.deque = collections.deque(maxlen=200)
        self._deltas_moving: list = []
        self._deltas_static: list = []
        self._n = 0

        self.create_subscription(
            Odometry, '/StoneFish/Odometry', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/sensor_msgs/image_depth', self._depth_cb, qos_profile_sensor_data)

        self.get_logger().info('timestamp_debug ready — move the robot around then check the stats')

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        self._odom_buffer.append((msg.header.stamp, speed))

    def _depth_cb(self, msg: Image):
        if not self._odom_buffer:
            self.get_logger().warn('no odometry yet — is /StoneFish/Odometry publishing?')
            return

        img_ns = _ns(msg.header.stamp)

        # Most recent odometry stamp <= image stamp
        best_stamp, best_speed, best_ns = None, 0.0, -1
        for stamp, speed in self._odom_buffer:
            s_ns = _ns(stamp)
            if s_ns <= img_ns and s_ns > best_ns:
                best_stamp, best_speed, best_ns = stamp, speed, s_ns

        if best_stamp is None:
            self.get_logger().warn('all odometry stamps are AFTER the image stamp — clock issue?')
            return

        delta_ms = (img_ns - best_ns) / 1e6
        moving = best_speed > 0.02
        self._n += 1

        if moving:
            self._deltas_moving.append(delta_ms)
        else:
            self._deltas_static.append(delta_ms)

        self.get_logger().info(
            f'[{self._n:4d}]  delta={delta_ms:8.2f} ms  '
            f'speed={best_speed:.3f} m/s  {"MOVING" if moving else "static"}'
        )

        if self._n % 10 == 0:
            self._print_stats()

    def _print_stats(self):
        lines = [f'========== STATS ({self._n} frames) ==========']
        for label, data in [('static', self._deltas_static), ('moving', self._deltas_moving)]:
            if not data:
                lines.append(f'  {label}: no data yet')
                continue
            mean = sum(data) / len(data)
            std  = math.sqrt(sum((x - mean)**2 for x in data) / len(data))
            lines.append(
                f'  {label:6s} ({len(data):3d} frames): '
                f'mean={mean:7.2f} ms  std={std:6.2f} ms  '
                f'min={min(data):7.2f}  max={max(data):7.2f}'
            )
        lines.append('==========================================')
        self.get_logger().info('\n'.join(lines))


def _ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main(args=None):
    rclpy.init(args=args)
    node = TimestampDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
