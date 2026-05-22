#!/usr/bin/env python3
"""
Broadcasts world_ned → bluerov2/base_link at the EXACT timestamp of each
depth image by interpolating between buffered odometry messages.

octomap_server will always find a TF hit at the point-cloud timestamp with
no interpolation or extrapolation on its end — eliminating pose drift during
fast motion.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, Pose
import tf2_ros
from scipy.spatial.transform import Rotation, Slerp


def _to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class OdomTfSync(Node):
    _BUFFER_SIZE = 50  # 500 ms at 100 Hz

    def __init__(self):
        super().__init__('odom_tf_sync')
        self._br  = tf2_ros.TransformBroadcaster(self)
        self._buf: list[Odometry] = []

        self.create_subscription(Odometry, '/StoneFish/Odometry',
                                 self._odom_cb, 100)
        self.create_subscription(Image, '/sensor_msgs/image_depth',
                                 self._image_cb, qos_profile_sensor_data)
        self.get_logger().info('odom_tf_sync started')

    # ── buffer incoming odometry ──────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        self._buf.append(msg)
        if len(self._buf) > self._BUFFER_SIZE:
            self._buf.pop(0)

    # ── on each depth frame: interpolate and broadcast ────────────────────
    def _image_cb(self, msg: Image) -> None:
        if len(self._buf) < 2:
            return

        t_img = _to_sec(msg.header.stamp)
        times  = [_to_sec(o.header.stamp) for o in self._buf]

        after = next((i for i, t in enumerate(times) if t >= t_img), None)

        if after is None:
            pose = self._buf[-1].pose.pose          # image is ahead of buffer
        elif after == 0:
            pose = self._buf[0].pose.pose           # image is behind buffer
        else:
            o1, o2 = self._buf[after - 1], self._buf[after]
            t1, t2 = times[after - 1], times[after]
            alpha  = (t_img - t1) / (t2 - t1)

            p1 = o1.pose.pose.position
            p2 = o2.pose.pose.position
            q1 = o1.pose.pose.orientation
            q2 = o2.pose.pose.orientation

            pose = Pose()
            pose.position.x = p1.x + alpha * (p2.x - p1.x)
            pose.position.y = p1.y + alpha * (p2.y - p1.y)
            pose.position.z = p1.z + alpha * (p2.z - p1.z)

            r1 = Rotation.from_quat([q1.x, q1.y, q1.z, q1.w])
            r2 = Rotation.from_quat([q2.x, q2.y, q2.z, q2.w])
            q  = Slerp([0.0, 1.0], Rotation.concatenate([r1, r2]))(alpha).as_quat()
            pose.orientation.x = float(q[0])
            pose.orientation.y = float(q[1])
            pose.orientation.z = float(q[2])
            pose.orientation.w = float(q[3])

        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp   # exact depth-image timestamp
        tf.header.frame_id = 'world_ned'
        tf.child_frame_id  = 'bluerov2/base_link'
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation      = pose.orientation
        self._br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
