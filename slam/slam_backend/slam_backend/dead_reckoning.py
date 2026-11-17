#!/usr/bin/env python3
"""
Fuses pressure + IMU + compass + DVL into a dead-reckoned estimate.

This mirrors what a real AUV's onboard navigation computer does (and what
roller's autopilot-fused `poses.odom` already represents): depth is an
absolute measurement and X/Y position is a relative, DVL-integrated quantity.
Roll/pitch come from the IMU; yaw is propagated from its high-rate increments
then corrected by the lower-rate absolute compass heading in a wrapped-angle
Kalman filter.

This gives the same XYH-relative / ZPR-absolute split used by Suresh et al.
(ICRA 2020) and by roller's axis prior, but resolves it here at the sensor-
fusion level instead of only inside the pose graph's noise models.
"""
import os

from geometry_msgs.msg import PointStamped, QuaternionStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from slam_backend.attitude_filter import YawKalmanFilter
from slam_backend.geometry_utils import matrix_to_odom
from slam_backend.sensor_models.noise_profiles import load_composite_profile


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('world_frame', 'world_ned')
        self.declare_parameter('noise_profile_path', '')
        # Per-sensor overrides matching the sim's: the yaw filter's sigmas must
        # describe the IMU/compass streams actually being fused.
        self.declare_parameter('noise_profile_path_imu', '')
        self.declare_parameter('noise_profile_path_compass', '')

        self.world_frame = self.get_parameter('world_frame').value

        self.latest_depth = None       # float, from pressure_sim
        self.latest_rotation = None    # 3x3 np.ndarray, from fused attitude
        self._roll_pitch: tuple[float, float] | None = None
        self.last_vel_time = None

        yaml_path = self.get_parameter('noise_profile_path').value
        if yaml_path and not os.path.exists(yaml_path):
            self.get_logger().warn(
                f"Invalid noise profile path: '{yaml_path}', using defaults.")
        profile = load_composite_profile(yaml_path, {
            'imu': self.get_parameter('noise_profile_path_imu').value,
            'compass': self.get_parameter('noise_profile_path_compass').value})
        self._imu_yaw_sigma = profile.imu.sigma_yaw_rad
        self._compass_yaw_sigma = profile.compass.sigma_yaw_rad
        self._yaw_filter = YawKalmanFilter(profile.imu.gyro_bias_drift_rad_s)

        self.pos_xy = np.array([
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
        ])

        self.create_subscription(
            PointStamped, '/slam/sensors/pressure_depth', self._pressure_cb, 10)
        self.create_subscription(
            QuaternionStamped, '/slam/sensors/imu_orientation', self._imu_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/slam/sensors/compass_heading', self._compass_cb, 10)
        # DVL velocity is the driving callback: each ping advances the integrated
        # X/Y position and triggers publication of the fused estimate.
        self.create_subscription(
            TwistStamped, '/slam/sensors/dvl_velocity', self._dvl_cb, 10)

        self.pub = self.create_publisher(
            Odometry, '/slam/sensors/dead_reckoned_odom', 10)

        self.get_logger().info(
            'DeadReckoning fusion node started: IMU yaw propagation + compass correction.')

    def _pressure_cb(self, msg: PointStamped):
        self.latest_depth = msg.point.z

    def _imu_cb(self, msg: QuaternionStamped):
        q = msg.quaternion
        roll, pitch, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._roll_pitch = (roll, pitch)
        self._yaw_filter.predict_imu(yaw, stamp, self._imu_yaw_sigma)
        self._update_rotation()

    def _compass_cb(self, msg: Vector3Stamped):
        self._yaw_filter.correct_compass(msg.vector.z, self._compass_yaw_sigma)
        self._update_rotation()

    def _update_rotation(self):
        if self._roll_pitch is None or self._yaw_filter.angle is None:
            return
        roll, pitch = self._roll_pitch
        self.latest_rotation = Rotation.from_euler(
            'xyz', [roll, pitch, self._yaw_filter.angle]).as_matrix()

    def _dvl_cb(self, msg: TwistStamped):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.latest_rotation is None or self.latest_depth is None:
            # Wait for at least one IMU + pressure reading before fusing —
            # avoids integrating velocity with an undefined attitude.
            self.last_vel_time = current_time
            return

        if self.last_vel_time is not None:
            dt = current_time - self.last_vel_time
            if dt > 0:
                v_body = np.array([
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z,
                ])
                # Integrate only the horizontal (X/Y) displacement in the world
                # frame — this is the sole relative/drifting quantity. Z comes
                # from the pressure prior below, not from integrated velocity.
                delta_world = self.latest_rotation @ (v_body * dt)
                self.pos_xy += delta_world[:2]
        self.last_vel_time = current_time

        T = np.eye(4)
        T[:3, :3] = self.latest_rotation
        T[0, 3] = self.pos_xy[0]
        T[1, 3] = self.pos_xy[1]
        T[2, 3] = self.latest_depth

        odom_msg = matrix_to_odom(T, msg.header.stamp, self.world_frame)
        # Yaw slot only. The filter's posterior is the sole uncertainty here that
        # varies at runtime, and the pose graph priors this fused yaw -- so it has
        # to be told how good the fusion actually was, not the input sensor's spec.
        if np.isfinite(self._yaw_filter.variance):
            odom_msg.pose.covariance[35] = float(self._yaw_filter.variance)
        self.pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
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
