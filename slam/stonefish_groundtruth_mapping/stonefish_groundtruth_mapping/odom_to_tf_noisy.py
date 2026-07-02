#!/usr/bin/env python3
"""
Broadcasts two TFs from /StoneFish/Odometry:
  - world_ned → bluerov2/base_link         (ground truth)
  - world_ned → bluerov2/base_link/noisy   (simulated drifting odometry)

The noisy pose integrates body-frame deltas with velocity-proportional Gaussian
noise, mimicking the drift a real DVL/IMU odometry would accumulate over time.
Tune H_COEF and YAW_COEF to control drift rate.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R


def _odom_to_T(msg: Odometry) -> np.ndarray:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _T_to_tf(T: np.ndarray, stamp, frame_id: str, child_frame_id: str) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = float(T[0, 3])
    t.transform.translation.y = float(T[1, 3])
    t.transform.translation.z = float(T[2, 3])
    q = R.from_matrix(T[:3, :3]).as_quat()
    t.transform.rotation.x = float(q[0])
    t.transform.rotation.y = float(q[1])
    t.transform.rotation.z = float(q[2])
    t.transform.rotation.w = float(q[3])
    return t


class OdomToTfNoisy(Node):
    # Drift coefficients: sigma ≈ |v| * coef * dt
    H_COEF   = 4.0   # translational drift (m per m/s per s)
    YAW_COEF = 1.5   # yaw drift (rad per rad/s per s)

    def __init__(self):
        super().__init__('odom_to_tf_noisy')
        self._br = TransformBroadcaster(self)
        self._noisy_odom_pub = self.create_publisher(Odometry, '/StoneFish/Odometry/noisy', 10)
        self._last: Odometry | None = None
        self._T_noisy: np.ndarray | None = None
        self.create_subscription(
            Odometry, '/StoneFish/Odometry', self._callback,
            QoSProfile(depth=10))
        self.get_logger().info(
            'odom_to_tf_noisy started\n'
            '  true  TF:   world_ned → bluerov2/base_link\n'
            '  noisy TF:   world_ned → bluerov2/base_link/noisy\n'
            '  noisy odom: /StoneFish/Odometry/noisy')

    def _callback(self, msg: Odometry):
        stamp = msg.header.stamp

        t_true = TransformStamped()
        t_true.header.stamp = stamp
        t_true.header.frame_id = 'world_ned'
        t_true.child_frame_id = 'bluerov2/base_link'
        t_true.transform.translation.x = msg.pose.pose.position.x
        t_true.transform.translation.y = msg.pose.pose.position.y
        t_true.transform.translation.z = msg.pose.pose.position.z
        t_true.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t_true)

        if self._last is None:
            self._last = msg
            self._T_noisy = _odom_to_T(msg)
            self._br.sendTransform(
                _T_to_tf(self._T_noisy, stamp,
                         'world_ned', 'bluerov2/base_link/noisy'))
            self._publish_noisy_odom(stamp)
            return

        t0 = Time(seconds=self._last.header.stamp.sec,
                  nanoseconds=self._last.header.stamp.nanosec)
        t1 = Time(seconds=msg.header.stamp.sec,
                  nanoseconds=msg.header.stamp.nanosec)
        dt = max((t1 - t0).nanoseconds * 1e-9, 0.0)

        T_delta = np.linalg.inv(_odom_to_T(self._last)) @ _odom_to_T(msg)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        def _noisy(sigma):
            return np.random.normal(0.0, sigma) if sigma > 0.0 else 0.0

        T_delta_noisy = T_delta.copy()
        T_delta_noisy[0, 3] += _noisy(abs(vx) * self.H_COEF * dt)
        T_delta_noisy[1, 3] += _noisy(abs(vy) * self.H_COEF * dt)
        T_delta_noisy[:3, :3] = (
            R.from_matrix(T_delta[:3, :3]) *
            R.from_euler('z', _noisy(abs(wz) * self.YAW_COEF * dt))
        ).as_matrix()

        self._T_noisy = self._T_noisy @ T_delta_noisy
        self._br.sendTransform(
            _T_to_tf(self._T_noisy, stamp,
                     'world_ned', 'bluerov2/base_link/noisy'))
        self._publish_noisy_odom(stamp)
        self._last = msg

    def _publish_noisy_odom(self, stamp) -> None:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world_ned'
        msg.child_frame_id = 'bluerov2/base_link/noisy'
        msg.pose.pose.position.x = float(self._T_noisy[0, 3])
        msg.pose.pose.position.y = float(self._T_noisy[1, 3])
        msg.pose.pose.position.z = float(self._T_noisy[2, 3])
        q = R.from_matrix(self._T_noisy[:3, :3]).as_quat()
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])
        self._noisy_odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNoisy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
