#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R


def odom_to_T(msg: Odometry) -> np.ndarray:
    """4x4 pose matrix from Odometry pose."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    rot = R.from_quat([q.x, q.y, q.z, q.w])
    T = np.eye(4, dtype=float)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def T_to_tf(T: np.ndarray, header, child_frame_id: str) -> TransformStamped:
    """TransformStamped from 4x4 pose matrix."""
    out = TransformStamped()
    out.header = header
    out.child_frame_id = child_frame_id

    out.transform.translation.x = float(T[0, 3])
    out.transform.translation.y = float(T[1, 3])
    out.transform.translation.z = float(T[2, 3])

    q = R.from_matrix(T[:3, :3]).as_quat()  # [x,y,z,w]
    out.transform.rotation.x = float(q[0])
    out.transform.rotation.y = float(q[1])
    out.transform.rotation.z = float(q[2])
    out.transform.rotation.w = float(q[3])
    return out


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf_noise")

        # ---- tune these ----
        # Translation noise std per step: sigma ≈ |v| * h_coef * dt  (meters)
        # Yaw noise std per step:         sigma ≈ |wz| * yaw_coef * dt (radians)
        self.h_coef = 4
        self.yaw_coef = 1.5

        self.last_true: Odometry | None = None
        self.T_noisy: np.ndarray | None = None

        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Odometry, "/bluerov2/odometry", self.cb, qos)
        self.br = TransformBroadcaster(self)

        self.get_logger().info("Publishing true TF: world_ned -> bluerov/base_link")
        self.get_logger().info("Publishing noisy TF: world_ned -> bluerov/base_link/noise")

    def cb(self, msg: Odometry):
        # --- publish TRUE tf (directly from message) ---
        header = msg.header
        header.stamp = Time(seconds=msg.header.stamp.sec,
                            nanoseconds=msg.header.stamp.nanosec).to_msg()

        t_true = TransformStamped()
        t_true.header = header
        t_true.header.frame_id = "world_ned"
        t_true.child_frame_id = "bluerov/base_link"
        t_true.transform.translation.x = msg.pose.pose.position.x
        t_true.transform.translation.y = msg.pose.pose.position.y
        t_true.transform.translation.z = msg.pose.pose.position.z
        t_true.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t_true)

        # --- initialize noisy state on first message ---
        if self.last_true is None:
            self.last_true = msg
            self.T_noisy = odom_to_T(msg)

            t_noisy = T_to_tf(self.T_noisy, header, "bluerov/base_link/noise")
            t_noisy.header.frame_id = "world_ned"
            self.br.sendTransform(t_noisy)
            return

        assert self.T_noisy is not None

        # --- dt ---
        last_time = Time(seconds=self.last_true.header.stamp.sec,
                         nanoseconds=self.last_true.header.stamp.nanosec)
        cur_time = Time(seconds=msg.header.stamp.sec,
                        nanoseconds=msg.header.stamp.nanosec)
        dt = (cur_time - last_time).nanoseconds * 1e-9
        if dt < 0.0:
            dt = 0.0

        # --- true relative motion expressed in PREVIOUS BODY frame ---
        T_prev = odom_to_T(self.last_true)
        T_cur = odom_to_T(msg)
        T_delta = np.linalg.inv(T_prev) @ T_cur  # body-frame delta

        # --- noise scales (your current model) ---
        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        w_z = msg.twist.twist.angular.z

        sigma_dx = abs(v_x) * self.h_coef * dt
        sigma_dy = abs(v_y) * self.h_coef * dt
        sigma_dyaw = abs(w_z) * self.yaw_coef * dt

        ndx = np.random.normal(0.0, sigma_dx) if sigma_dx > 0.0 else 0.0
        ndy = np.random.normal(0.0, sigma_dy) if sigma_dy > 0.0 else 0.0
        dpsi = np.random.normal(0.0, sigma_dyaw) if sigma_dyaw > 0.0 else 0.0

        # --- build noisy delta in BODY frame ---
        T_delta_noisy = T_delta.copy()

        # translation noise on body x/y
        T_delta_noisy[0, 3] += ndx
        T_delta_noisy[1, 3] += ndy
        # optional: add z noise similarly if you want
        # T_delta_noisy[2, 3] += np.random.normal(0.0, sigma_dz)

        # yaw noise about BODY z (intrinsic) => right-multiply delta rotation
        R_delta = R.from_matrix(T_delta[:3, :3])
        R_yaw_noise = R.from_euler("z", dpsi)          # intrinsic body-z
        R_delta_noisy = R_delta * R_yaw_noise          # body-frame yaw perturbation
        T_delta_noisy[:3, :3] = R_delta_noisy.as_matrix()

        # --- propagate noisy pose in world ---
        self.T_noisy = self.T_noisy @ T_delta_noisy

        # --- publish NOISY tf ---
        t_noisy = T_to_tf(self.T_noisy, header, "bluerov/base_link/noise")
        t_noisy.header.frame_id = "world_ned"
        self.br.sendTransform(t_noisy)

        self.last_true = msg


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
