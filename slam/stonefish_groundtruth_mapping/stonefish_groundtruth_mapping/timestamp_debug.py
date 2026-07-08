#!/usr/bin/env python3
"""
Measures the timestamp alignment between depth images and odometry.

For each depth image (raw from Stonefish, before depth_fix), finds the
CLOSEST odometry stamp (signed delta) and also the nearest-before delta:

  - delta_closest_ms : T_img - T_closest_odom  (signed, can be negative)
  - delta_before_ms  : T_img - T_nearest_odom_before  (what depth_fix uses)

Expected after the InternalUpdate fix:
  - delta_closest  ≈ 0 ms  (both stamped in the same physics-thread loop)
  - delta_before   ≈ 0..+10 ms  (0 if odom published just before capture,
                                  up to +10 ms if odom fired just after)
  - OLD behaviour was: delta_before ≈ +5 ms mean, max +12 ms

Prints a rolling summary every 10 frames.

Usage:
  ros2 run stonefish_groundtruth_mapping timestamp_debug
"""
import collections
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


class TimestampDebug(Node):
    def __init__(self):
        super().__init__('timestamp_debug')
        self._odom_buffer: collections.deque = collections.deque(maxlen=200)
        self._closest_moving: list = []
        self._closest_static: list = []
        self._before_moving: list = []
        self._before_static: list = []
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
            self.get_logger().warn('no odometry yet')
            return

        img_ns = _ns(msg.header.stamp)

        # Nearest odom <= img (what depth_fix uses)
        before_stamp, before_speed, before_ns = None, 0.0, -1
        # Closest odom overall (signed delta check)
        closest_stamp, closest_speed, closest_dist = None, 0.0, float('inf')

        for stamp, speed in self._odom_buffer:
            s_ns = _ns(stamp)
            if s_ns <= img_ns and s_ns > before_ns:
                before_stamp, before_speed, before_ns = stamp, speed, s_ns
            dist = abs(img_ns - s_ns)
            if dist < closest_dist:
                closest_stamp, closest_speed, closest_dist = stamp, speed, dist

        moving = (before_speed if before_stamp else closest_speed) > 0.02
        self._n += 1

        if before_stamp is not None:
            delta_before = (img_ns - before_ns) / 1e6
            (self._before_moving if moving else self._before_static).append(delta_before)
        else:
            delta_before = float('nan')

        if closest_stamp is not None:
            delta_closest = (img_ns - _ns(closest_stamp)) / 1e6
            (self._closest_moving if moving else self._closest_static).append(delta_closest)
        else:
            delta_closest = float('nan')

        self.get_logger().info(
            f'[{self._n:4d}]  closest={delta_closest:+8.2f} ms  '
            f'before={delta_before:+8.2f} ms  '
            f'speed={closest_speed:.3f} m/s  {"MOVING" if moving else "static"}'
        )

        if self._n % 10 == 0:
            self._print_stats()

    def _print_stats(self):
        lines = [f'========== STATS ({self._n} frames) ==========']
        lines.append('  [closest = signed delta to nearest odom; before = what depth_fix uses]')
        for label, c_data, b_data in [
            ('static', self._closest_static, self._before_static),
            ('moving', self._closest_moving, self._before_moving),
        ]:
            if not c_data:
                lines.append(f'  {label}: no data yet')
                continue
            def fmt(data):
                if not data:
                    return 'no data'
                mean = sum(data) / len(data)
                std = math.sqrt(sum((x - mean)**2 for x in data) / len(data))
                return (f'mean={mean:+7.2f} ms  std={std:5.2f} ms  '
                        f'min={min(data):+7.2f}  max={max(data):+7.2f}')
            lines.append(f'  {label:6s} ({len(c_data):3d} frames):')
            lines.append(f'    closest: {fmt(c_data)}')
            lines.append(f'    before:  {fmt(b_data)}')
        lines.append('==========================================')
        self.get_logger().info('\n'.join(lines))


def _ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main(args=None):
    rclpy.init(args=args)
    node = TimestampDebug()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
