#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import numpy as np
import os

from slam_backend.sensor_models.noise_profiles import load_noise_profile, resolve_seed
from slam_backend.geometry_utils import odom_to_matrix

# Per-sensor offset added to a shared seed so co-launched sims (identical
# profile.seed) don't draw identical RNG streams (imu=+1, pressure=+3, sonar=+4).
SEED_OFFSET = 2


class DVLSimNode(Node):
    """Simulates a DVL: reports noisy body-frame velocity, not a pose.

    Real DVL hardware measures ground-referenced velocity via Doppler shift on
    acoustic beams — it has no notion of position or attitude. Fusing that
    velocity into a navigation solution (using an attitude source to rotate it
    into the world frame, and integrating over time) is a separate concern,
    handled by dead_reckoning.py.
    """

    def __init__(self, **kwargs):
        super().__init__('dvl_sim', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('noise_seed', -1)
        yaml_path = self.get_parameter('noise_profile_path').value

        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            from slam_backend.sensor_models.noise_profiles import NoiseProfile
            self.profile = NoiseProfile().dvl
            profile_seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.dvl
            profile_seed = full_profile.seed

        seed = resolve_seed(profile_seed, self.get_parameter('noise_seed').value, SEED_OFFSET)
        if seed != -1:
            np.random.seed(seed)

        self.sub = self.create_subscription(Odometry, '/StoneFish/Odometry', self.odom_cb, 10)
        self.pub = self.create_publisher(TwistStamped, '/slam/sensors/dvl_velocity', 10)

        self.publish_interval = 1.0 / self.profile.publish_rate_hz
        self.last_pub_time = None
        self.T_last_pub_gt = None

        # Constant per-run scale factor error (e.g. imperfect sound-velocity
        # calibration) — sampled once, not re-drawn every callback.
        self.scale = 1.0 + np.random.normal(0, self.profile.scale_error_pct)

        self.get_logger().info(f"DVLSim started. Rate: {self.profile.publish_rate_hz}Hz, Scale err: {(self.scale-1.0)*100:.3f}%")

    def odom_cb(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        T_curr_gt = odom_to_matrix(msg)

        if self.T_last_pub_gt is None:
            self.T_last_pub_gt = T_curr_gt
            self.last_pub_time = current_time
            return

        dt = current_time - self.last_pub_time
        if dt < 0:
            # Backward stamp jump (sim-time/clock reset) — resync instead of
            # stalling forever or dividing the velocity average by a negative
            # interval below.
            self.T_last_pub_gt = T_curr_gt
            self.last_pub_time = current_time
            return
        if dt < self.publish_interval:
            return

        # Average true body-frame velocity over this DVL ping interval
        # (a real DVL integrates its own beam returns internally and reports
        # one averaged velocity per ping — it does not sample at the GT rate).
        R_prev = self.T_last_pub_gt[:3, :3]
        delta_pos_body = R_prev.T @ (T_curr_gt[:3, 3] - self.T_last_pub_gt[:3, 3])
        v_true = delta_pos_body / dt

        speed = np.linalg.norm(v_true)
        sigma = max(speed * self.profile.sigma_pct, self.profile.sigma_floor_m_s)
        v_noisy = v_true * self.scale + np.random.normal(0, sigma, 3) + self.profile.bias_m_s

        out = TwistStamped()
        out.header = msg.header
        out.header.frame_id = 'bluerov2/base_link'
        out.twist.linear.x = float(v_noisy[0])
        out.twist.linear.y = float(v_noisy[1])
        out.twist.linear.z = float(v_noisy[2])
        self.pub.publish(out)

        self.T_last_pub_gt = T_curr_gt
        self.last_pub_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = DVLSimNode()
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
