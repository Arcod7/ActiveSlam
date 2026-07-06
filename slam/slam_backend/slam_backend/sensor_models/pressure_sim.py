#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np
import os

from slam_backend.sensor_models.noise_profiles import load_noise_profile, resolve_seed

# Per-sensor offset added to a shared seed so co-launched sims (identical
# profile.seed) don't draw identical RNG streams (imu=+1, dvl=+2, sonar=+4).
SEED_OFFSET = 3

class PressureSimNode(Node):
    def __init__(self, **kwargs):
        super().__init__('pressure_sim', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('noise_seed', -1)
        yaml_path = self.get_parameter('noise_profile_path').value

        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            from slam_backend.sensor_models.noise_profiles import NoiseProfile
            self.profile = NoiseProfile().pressure
            profile_seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.pressure
            profile_seed = full_profile.seed

        seed = resolve_seed(profile_seed, self.get_parameter('noise_seed').value, SEED_OFFSET)
        if seed != -1:
            np.random.seed(seed)
            
        self.sub = self.create_subscription(Odometry, '/StoneFish/Odometry', self.odom_cb, 10)
        self.pub = self.create_publisher(PointStamped, '/slam/sensors/pressure_depth', 10)
        
        # Rate limiting
        self.publish_interval = 1.0 / self.profile.publish_rate_hz
        self.last_pub_time = None
        
        self.get_logger().info(f"PressureSim started. Rate: {self.profile.publish_rate_hz}Hz, Sigma: {self.profile.sigma_depth_m}m")

    def odom_cb(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_pub_time is not None:
            elapsed = current_time - self.last_pub_time
            if elapsed < 0:
                # Backward stamp jump (sim-time/clock reset) — resync rather
                # than stall until current_time climbs back past the stale
                # future last_pub_time.
                self.last_pub_time = None
            elif elapsed < self.publish_interval:
                return

        self.last_pub_time = current_time
        
        true_z = msg.pose.pose.position.z
        noisy_z = true_z + np.random.normal(0, self.profile.sigma_depth_m) + self.profile.bias_m
        
        out_msg = PointStamped()
        out_msg.header = msg.header
        out_msg.point.z = float(noisy_z)
        # x and y are not used for depth, but set to 0 explicitly
        out_msg.point.x = 0.0
        out_msg.point.y = 0.0
        
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PressureSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
