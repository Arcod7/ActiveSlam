#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
import numpy as np
import os
from scipy.spatial.transform import Rotation

from slam_backend.sensor_models.noise_profiles import load_noise_profile

class IMUSimNode(Node):
    def __init__(self, **kwargs):
        super().__init__('imu_sim', **kwargs)
        
        self.declare_parameter('noise_profile_path', '')
        yaml_path = self.get_parameter('noise_profile_path').value
        
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            from slam_backend.sensor_models.noise_profiles import NoiseProfile
            self.profile = NoiseProfile().imu
            seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.imu
            seed = full_profile.seed
            
        if seed != -1:
            # We seed globally or use a local generator. Using global for simplicity.
            # In a multi-node setup with the same seed, they will have identical RNG sequences,
            # which is fine if they are in different processes.
            np.random.seed(seed)
            
        self.sub = self.create_subscription(Odometry, '/StoneFish/Odometry', self.odom_cb, 10)
        self.pub = self.create_publisher(QuaternionStamped, '/slam/sensors/imu_orientation', 10)
        
        # Rate limiting
        self.publish_interval = 1.0 / self.profile.publish_rate_hz
        self.last_pub_time = None
        self.last_msg_time = None
        
        self.yaw_bias = 0.0
        
        self.get_logger().info(f"IMUSim started. Rate: {self.profile.publish_rate_hz}Hz")

    def odom_cb(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_msg_time is None:
            self.last_msg_time = current_time
            self.last_pub_time = current_time
            return
            
        dt = current_time - self.last_msg_time
        self.last_msg_time = current_time
        
        # Maintain drifting yaw bias each step (100 Hz from ground truth)
        self.yaw_bias += np.random.normal(0, self.profile.gyro_bias_drift_rad_s * dt)
        
        if current_time - self.last_pub_time < self.publish_interval:
            return
            
        self.last_pub_time = current_time
        
        # Extract quaternion
        q = msg.pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        
        # Convert to Euler (extrinsic xyz is equivalent to intrinsic ZYX -> roll, pitch, yaw)
        roll, pitch, yaw = rot.as_euler('xyz')
        
        # Add noise
        roll += np.random.normal(0, self.profile.sigma_roll_rad)
        pitch += np.random.normal(0, self.profile.sigma_pitch_rad)
        yaw += np.random.normal(0, self.profile.sigma_yaw_rad) + self.yaw_bias
        
        # Recompose quaternion
        noisy_rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
        qx, qy, qz, qw = noisy_rot.as_quat()
        
        out_msg = QuaternionStamped()
        out_msg.header = msg.header
        out_msg.quaternion.x = float(qx)
        out_msg.quaternion.y = float(qy)
        out_msg.quaternion.z = float(qz)
        out_msg.quaternion.w = float(qw)
        
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
