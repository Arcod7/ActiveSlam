"""Compass simulator publishing an absolute yaw measurement from Stonefish."""
import math
import os

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from slam_backend.attitude_filter import wrap_angle
from slam_backend.sensor_models.noise_profiles import (
    load_noise_profile,
    NoiseProfile,
    resolve_seed,
)


SEED_OFFSET = 5


class CompassSimNode(Node):
    """Simulate an absolute magnetic-heading observation from ground truth."""

    def __init__(self, **kwargs) -> None:
        super().__init__('compass_sim', **kwargs)
        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('noise_seed', -1)
        yaml_path = self.get_parameter('noise_profile_path').value
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(
                f"Invalid noise profile path: '{yaml_path}', using defaults.")
            self.profile = NoiseProfile().compass
            profile_seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.compass
            profile_seed = full_profile.seed

        seed = resolve_seed(profile_seed, self.get_parameter('noise_seed').value,
                            SEED_OFFSET)
        self._rng = np.random.default_rng(seed if seed != -1 else None)
        self._bias = 0.0
        self._last_time: float | None = None
        self._last_pub_time: float | None = None
        self._publish_interval = 1.0 / self.profile.publish_rate_hz
        self.create_subscription(Odometry, '/StoneFish/Odometry', self._odom_cb, 10)
        self._pub = self.create_publisher(
            Vector3Stamped, '/slam/sensors/compass_heading', 10)
        self.get_logger().info(
            f'CompassSim started. Rate: {self.profile.publish_rate_hz}Hz, '
            f'sigma={math.degrees(self.profile.sigma_yaw_rad):.2f}deg')

    def _odom_cb(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_time is not None:
            dt = stamp - self._last_time
            if dt > 0.0:
                self._bias += self._rng.normal(
                    0.0, self.profile.bias_drift_rad_s * math.sqrt(dt))
        self._last_time = stamp
        if (self._last_pub_time is not None
                and stamp - self._last_pub_time < self._publish_interval):
            return
        self._last_pub_time = stamp

        q = msg.pose.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]
        measured_yaw = wrap_angle(yaw + self._bias + self._rng.normal(
            0.0, self.profile.sigma_yaw_rad))
        out = Vector3Stamped()
        out.header = msg.header
        out.vector.z = float(measured_yaw)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompassSimNode()
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
