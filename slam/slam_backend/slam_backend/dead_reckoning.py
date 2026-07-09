#!/usr/bin/env python3
"""Fuses pressure + IMU + DVL into a single dead-reckoned navigation estimate.

This mirrors what a real AUV's onboard navigation computer does (and what
roller's autopilot-fused `poses.odom` already represents): depth and attitude
are ABSOLUTE, non-drifting measurements, so they are re-applied verbatim on
every fusion step. Only X/Y position is a RELATIVE, integrated quantity (DVL
body-frame velocity rotated into the world frame by the current attitude
estimate) — this is the only part of the trajectory that drifts, and it is
what scan matching in pose_graph.py corrects.

This gives the same XYH-relative / ZPR-absolute split used by Suresh et al.
(ICRA 2020) and by roller's axis prior, but resolves it here at the sensor-
fusion level instead of only inside the pose graph's noise models.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, QuaternionStamped, TwistStamped
from scipy.spatial.transform import Rotation
import numpy as np

from slam_backend.geometry_utils import matrix_to_odom


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('world_frame', 'world_ned')

        self.world_frame = self.get_parameter('world_frame').value

        self.latest_depth = None       # float, from pressure_sim
        self.latest_rotation = None    # 3x3 np.ndarray, from imu_sim
        self.last_vel_time = None

        self.pos_xy = np.array([
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
        ])

        self.create_subscription(PointStamped, '/slam/sensors/pressure_depth',
                                  self._pressure_cb, 10)
        self.create_subscription(QuaternionStamped, '/slam/sensors/imu_orientation',
                                  self._imu_cb, 10)
        # DVL velocity is the driving callback: each ping advances the integrated
        # X/Y position and triggers publication of the fused estimate.
        self.create_subscription(TwistStamped, '/slam/sensors/dvl_velocity',
                                  self._dvl_cb, 10)

        self.pub = self.create_publisher(
            Odometry, '/slam/sensors/dead_reckoned_odom', 10)

        self.get_logger().info('DeadReckoning fusion node started.')

    def _pressure_cb(self, msg: PointStamped):
        self.latest_depth = msg.point.z

    def _imu_cb(self, msg: QuaternionStamped):
        q = msg.quaternion
        self.latest_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

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
                v_body = np.array([msg.twist.linear.x,
                                    msg.twist.linear.y,
                                    msg.twist.linear.z])
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
