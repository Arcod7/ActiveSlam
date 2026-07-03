#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import os
from scipy.spatial.transform import Rotation

from slam_backend.sensor_models.noise_profiles import load_noise_profile

def odom_to_matrix(msg: Odometry) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = msg.pose.pose.position.x
    T[1, 3] = msg.pose.pose.position.y
    T[2, 3] = msg.pose.pose.position.z
    q = msg.pose.pose.orientation
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return T

def matrix_to_odom(T: np.ndarray, stamp, frame_id: str) -> Odometry:
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = 'bluerov2/base_link'
    
    msg.pose.pose.position.x = float(T[0, 3])
    msg.pose.pose.position.y = float(T[1, 3])
    msg.pose.pose.position.z = float(T[2, 3])
    
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    msg.pose.pose.orientation.x = float(q[0])
    msg.pose.pose.orientation.y = float(q[1])
    msg.pose.pose.orientation.z = float(q[2])
    msg.pose.pose.orientation.w = float(q[3])
    return msg

class DVLSimNode(Node):
    def __init__(self):
        super().__init__('dvl_sim')
        
        self.declare_parameter('noise_profile_path', '')
        yaml_path = self.get_parameter('noise_profile_path').value
        
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            from slam_backend.sensor_models.noise_profiles import NoiseProfile
            self.profile = NoiseProfile().dvl
            seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.dvl
            seed = full_profile.seed
            
        if seed != -1:
            np.random.seed(seed)
            
        self.sub = self.create_subscription(Odometry, '/StoneFish/Odometry', self.odom_cb, 10)
        self.pub = self.create_publisher(Odometry, '/slam/sensors/dvl_odom', 10)
        
        self.publish_interval = 1.0 / self.profile.publish_rate_hz
        self.last_pub_time = None
        self.last_msg_time = None
        
        self.T_prev_gt = None
        self.T_integrated = None
        
        self.scale = 1.0 + np.random.normal(0, self.profile.scale_error_pct)
        
        self.get_logger().info(f"DVLSim started. Rate: {self.profile.publish_rate_hz}Hz, Scale err: {(self.scale-1.0)*100:.3f}%")

    def odom_cb(self, msg: Odometry):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        T_curr_gt = odom_to_matrix(msg)
        
        if self.T_prev_gt is None:
            self.T_prev_gt = T_curr_gt
            self.T_integrated = T_curr_gt
            self.last_msg_time = current_time
            self.last_pub_time = current_time
            # Publish initial pose
            self.pub.publish(matrix_to_odom(self.T_integrated, msg.header.stamp, msg.header.frame_id))
            return
            
        dt = current_time - self.last_msg_time
        self.last_msg_time = current_time
        
        if dt > 0:
            # Compute true body-frame delta
            T_delta = np.linalg.inv(self.T_prev_gt) @ T_curr_gt
            
            # Compute speed from twist / delta
            speed = np.linalg.norm(T_delta[:3, 3]) / dt
            
            # Compute noise sigma
            sigma = max(speed * self.profile.sigma_pct, self.profile.sigma_floor_m_s) * dt
            
            # Noise the translational delta in body frame
            T_delta[:3, 3] *= self.scale
            # Apply white noise and constant bias
            T_delta[:3, 3] += np.random.normal(0, sigma, 3) + self.profile.bias_m_s * dt
            
            # Integrate
            self.T_integrated = self.T_integrated @ T_delta
            
        self.T_prev_gt = T_curr_gt
        
        if current_time - self.last_pub_time >= self.publish_interval:
            self.last_pub_time = current_time
            self.pub.publish(matrix_to_odom(self.T_integrated, msg.header.stamp, msg.header.frame_id))

def main(args=None):
    rclpy.init(args=args)
    node = DVLSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
