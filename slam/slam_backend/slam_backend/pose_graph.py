#!/usr/bin/env python3
"""GTSAM iSAM2 pose-graph SLAM node.

Consumes the fused dead-reckoning odometry (dead_reckoning.py: IMU attitude +
pressure depth + integrated DVL velocity) as the odometry BetweenFactor source,
and depth-camera point clouds (/cloud_in) for scan-matching BetweenFactors that
correct the dead-reckoning drift. Follows roller's single-level factor-graph
pattern (graph.cpp): a Huber-robust ICP noise model for all relative (Between)
factors, and a per-node absolute attitude+depth PriorFactor built directly from
the same fused odometry reading (roll, pitch, yaw, z) with X/Y left unconstrained
(sigma=1e3) since only scan matching / loop closure can correct horizontal drift.
"""
import os
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
import gtsam

from slam_backend.sensor_models.noise_profiles import load_noise_profile, NoiseProfile
from slam_backend.geometry_utils import (
    odom_to_matrix, matrix_to_odom, matrix_to_transform_stamped,
    matrix_to_pose_stamped, transform_msg_to_matrix, orthonormalize)
from slam_backend.scan_matcher import ScanMatcher


@dataclass
class Keyframe:
    index: int
    stamp: object                 # builtin_interfaces/Time (message), for TF/path stamping
    T_odom: np.ndarray            # 4x4 dead-reckoned pose at keyframe time
    T_world: np.ndarray           # 4x4 optimized pose (updated after every iSAM2 solve)
    cloud: np.ndarray             # (N, 3) point cloud in body frame
    symbol: int
    covariance: np.ndarray = None   # 6x6 GTSAM-order marginal, cached at creation time
    loop_closures: list = field(default_factory=list)   # [(target_idx, T_rel, err_per_inlier)]


class PoseGraphNode(Node):
    def __init__(self, **kwargs):
        super().__init__('pose_graph', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('world_frame', 'world_ned')
        self.declare_parameter('base_frame', 'bluerov2/base_link')
        self.declare_parameter('camera_frame', 'bluerov2/Dcam')
        self.declare_parameter('keyframe_dist_m', 1.0)
        self.declare_parameter('keyframe_angle_rad', 0.3)
        self.declare_parameter('loop_closure_radius_m', 5.0)
        self.declare_parameter('loop_closure_min_gap', 10)
        self.declare_parameter('min_inlier_ratio', 0.3)
        self.declare_parameter('min_inlier_count', 50)
        self.declare_parameter('max_error_per_inlier', 0.05)
        self.declare_parameter('icp_sigma_rot', 0.05)
        self.declare_parameter('icp_sigma_trans', 0.05)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('scan_max_correspondence_dist', 0.5)
        self.declare_parameter('cov_ellipsoid_stride', 5)
        self.declare_parameter('cov_ellipsoid_n_sigma', 2.0)

        p = self.get_parameter
        self.world_frame = p('world_frame').value
        self.base_frame = p('base_frame').value
        self.camera_frame = p('camera_frame').value
        self.keyframe_dist_m = p('keyframe_dist_m').value
        self.keyframe_angle_rad = p('keyframe_angle_rad').value
        self.loop_closure_radius_m = p('loop_closure_radius_m').value
        self.loop_closure_min_gap = p('loop_closure_min_gap').value
        self.min_inlier_ratio = p('min_inlier_ratio').value
        self.min_inlier_count = p('min_inlier_count').value
        self.max_error_per_inlier = p('max_error_per_inlier').value
        self.cov_ellipsoid_stride = p('cov_ellipsoid_stride').value
        self.cov_ellipsoid_n_sigma = p('cov_ellipsoid_n_sigma').value

        yaml_path = p('noise_profile_path').value
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            profile = NoiseProfile()
        else:
            profile = load_noise_profile(yaml_path)
        self._profile = profile

        icp_sigma_rot = p('icp_sigma_rot').value
        icp_sigma_trans = p('icp_sigma_trans').value

        self._scanner = ScanMatcher(
            max_correspondence_dist=p('scan_max_correspondence_dist').value,
            downsampling_resolution=p('scan_voxel_size').value)

        self._setup_gtsam(icp_sigma_rot, icp_sigma_trans, profile)

        self._keyframes: list[Keyframe] = []
        self._latest_dead_reckoned_T = None
        self._latest_dead_reckoned_stamp = None
        self._T_base_cam = None   # static extrinsic, resolved lazily via TF

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, '/slam/sensors/dead_reckoned_odom',
                                  self._dead_reckoned_odom_cb, 10)
        self.create_subscription(PointCloud2, '/cloud_in', self._cloud_in_cb, 10)

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/slam/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/slam/odometry', 10)
        self.pub_path_gt = self.create_publisher(Path, '/slam/path_dr', 10)
        self.pub_path_slam = self.create_publisher(Path, '/slam/path_slam', 10)
        self.pub_graph_edges = self.create_publisher(MarkerArray, '/slam/graph_edges', 10)
        self.pub_covariance = self.create_publisher(MarkerArray, '/slam/covariance', 10)
        self.pub_keyframe_count = self.create_publisher(Int32, '/slam/keyframe_count', 10)
        self.pub_loop_closure_count = self.create_publisher(Int32, '/slam/loop_closure_count', 10)
        self.pub_dopt = self.create_publisher(Float64, '/slam/dopt', 10)

        self._path_dr_msgs = []
        self._path_slam_msgs = []
        self._rejected_edges = []   # [(idx_a, idx_b)] for viz only, cleared each keyframe

        self.get_logger().info(
            f"PoseGraph started. noise_profile={profile.name}, "
            f"keyframe_dist={self.keyframe_dist_m}m, keyframe_angle={self.keyframe_angle_rad}rad")

    # ------------------------------------------------------------------
    # GTSAM setup
    # ------------------------------------------------------------------
    def _setup_gtsam(self, icp_sigma_rot, icp_sigma_trans, profile: NoiseProfile):
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.01)
        isam_params.relinearizeSkip = 1
        self._isam = gtsam.ISAM2(isam_params)

        # Single ICP-style noise model for ALL relative (Between) factors —
        # dead-reckoning odometry AND scan-matching registration alike.
        # GTSAM Pose3 tangent order: [rot_x, rot_y, rot_z, trans_x, trans_y, trans_z].
        icp_sigmas = np.array([icp_sigma_rot] * 3 + [icp_sigma_trans] * 3)
        self._icp_noise = gtsam.noiseModel.Diagonal.Sigmas(icp_sigmas)
        self._robust_icp_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), self._icp_noise)

        # Attitude+depth prior, applied on every node from the SAME fused
        # dead-reckoning reading (roller pattern, graph.cpp:44-46): x/y left
        # unconstrained (1e3) since only registration constrains horizontal
        # position; roll/pitch/yaw from IMU, z from pressure.
        prior_sigmas = np.array([
            profile.imu.sigma_roll_rad,
            profile.imu.sigma_pitch_rad,
            profile.imu.sigma_yaw_rad,
            1e3,
            1e3,
            profile.pressure.sigma_depth_m,
        ])
        self._att_depth_noise = gtsam.noiseModel.Diagonal.Sigmas(prior_sigmas)

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def _dead_reckoned_odom_cb(self, msg: Odometry):
        T_odom = odom_to_matrix(msg)
        self._latest_dead_reckoned_T = T_odom
        self._latest_dead_reckoned_stamp = msg.header.stamp

        self._path_dr_msgs.append(matrix_to_pose_stamped(T_odom, msg.header.stamp, self.world_frame))

        if not self._keyframes:
            # No graph yet: broadcast the raw dead-reckoned pose directly.
            self._broadcast_tf(T_odom, msg.header.stamp)
            self._publish_odometry(T_odom, msg.header.stamp)
            return

        last_kf = self._keyframes[-1]
        T_delta = np.linalg.inv(last_kf.T_odom) @ T_odom
        T_corrected = last_kf.T_world @ T_delta
        self._broadcast_tf(T_corrected, msg.header.stamp)
        self._publish_odometry(T_corrected, msg.header.stamp)

    def _cloud_in_cb(self, msg: PointCloud2):
        if self._latest_dead_reckoned_T is None:
            return   # no pose yet to attach this cloud to

        if self._T_base_cam is None and not self._resolve_camera_extrinsic():
            return   # extrinsic TF not available yet

        T_odom = self._latest_dead_reckoned_T
        if self._keyframes and not self._should_create_keyframe(T_odom):
            return

        points_cam = point_cloud2.read_points_numpy(
            msg, field_names=['x', 'y', 'z'], skip_nans=True).astype(np.float64)
        if points_cam.shape[0] < self.min_inlier_count:
            return   # too sparse to be useful for registration

        points_body = (self._T_base_cam[:3, :3] @ points_cam.T).T + self._T_base_cam[:3, 3]

        self._add_keyframe(T_odom, points_body, msg.header.stamp)

    def _resolve_camera_extrinsic(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
        except Exception:
            return False
        self._T_base_cam = transform_msg_to_matrix(tf.transform)
        return True

    def _should_create_keyframe(self, T_odom_new: np.ndarray) -> bool:
        T_delta = np.linalg.inv(self._keyframes[-1].T_odom) @ T_odom_new
        dist = np.linalg.norm(T_delta[:3, 3])
        angle = np.arccos(np.clip((np.trace(T_delta[:3, :3]) - 1) / 2, -1, 1))
        return dist >= self.keyframe_dist_m or angle >= self.keyframe_angle_rad

    # ------------------------------------------------------------------
    # Core graph update
    # ------------------------------------------------------------------
    def _add_keyframe(self, T_odom: np.ndarray, cloud_body: np.ndarray, stamp):
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        n = len(self._keyframes)
        sym = gtsam.symbol('x', n)
        self._rejected_edges = []

        if n == 0:
            pose = gtsam.Pose3(T_odom)
            graph.addPriorPose3(sym, pose, self._icp_noise)
            values.insert(sym, pose)
            T_world_est = T_odom
        else:
            prev = self._keyframes[-1]

            # 1) Dead-reckoning odometry BetweenFactor
            T_delta = np.linalg.inv(prev.T_odom) @ T_odom
            graph.add(gtsam.BetweenFactorPose3(
                prev.symbol, sym, gtsam.Pose3(T_delta), self._robust_icp_noise))

            # 2) Sequential scan-matching BetweenFactor
            result = self._scanner.align(cloud_body, prev.cloud, T_delta)
            if ScanMatcher.is_acceptable(result, self.min_inlier_ratio,
                                          self.min_inlier_count, self.max_error_per_inlier):
                graph.add(gtsam.BetweenFactorPose3(
                    prev.symbol, sym, gtsam.Pose3(result['T_target_source']),
                    self._robust_icp_noise))
            else:
                self._rejected_edges.append((prev.index, n))

            T_world_est = orthonormalize_pose(prev.T_world @ T_delta)

            values.insert(sym, gtsam.Pose3(T_world_est))

        # 3) Attitude+depth prior on every node, read directly off this
        # keyframe's fused dead-reckoning reading (roller: "use odometry
        # attitude as prior").
        T_prior = T_world_est.copy()
        T_prior[:3, :3] = T_odom[:3, :3]
        T_prior[2, 3] = T_odom[2, 3]
        graph.addPriorPose3(sym, gtsam.Pose3(T_prior), self._att_depth_noise)

        # 4) Loop closure detection (against the pre-update world estimate)
        lc_factors = self._detect_loop_closures(n, cloud_body, T_world_est)
        for lc_idx, lc_T, lc_err in lc_factors:
            graph.add(gtsam.BetweenFactorPose3(
                self._keyframes[lc_idx].symbol, sym, gtsam.Pose3(lc_T),
                self._robust_icp_noise))

        self._isam.update(graph, values)
        result_values = self._isam.calculateEstimate()

        for kf in self._keyframes:
            kf.T_world = result_values.atPose3(kf.symbol).matrix()

        kf_new = Keyframe(index=n, stamp=stamp, T_odom=T_odom,
                           T_world=result_values.atPose3(sym).matrix(),
                           cloud=cloud_body, symbol=sym,
                           loop_closures=list(lc_factors))
        self._keyframes.append(kf_new)

        if lc_factors:
            self.get_logger().info(
                f"Loop closure: node {n} <-> {[i for i, _, _ in lc_factors]}")
            moved = self._find_moved_keyframes(result_values)
            if moved:
                self._redetect_and_apply(moved)

        # Marginal covariance is queried only for the node just added — walking
        # every stored keyframe on each update would grow the per-keyframe cost
        # linearly with session length. Older keyframes keep the covariance
        # they were given when THEY were the newest node (see _publish_
        # covariance_ellipsoids); it goes stale (typically shrinks further)
        # after a loop closure, but stays a reasonable upper bound for display.
        kf_new.covariance = self._isam.marginalCovariance(sym)
        self._publish_keyframe_results(kf_new, kf_new.covariance)

    def _detect_loop_closures(self, current_idx, cloud_body, T_world_est):
        # Candidates are filtered by |current_idx - kf.index| rather than a
        # slice on self._keyframes: this method is reused by _redetect_and_apply
        # for an arbitrary historical (possibly early) index, where a slice
        # bound of `idx - min_gap` can go negative — Python silently
        # reinterprets a negative slice bound as "count from the end", which
        # would return the wrong (or entirely backwards) candidate set instead
        # of raising. The symmetric distance check is correct for both the
        # normal forward case (current_idx == len(self._keyframes)) and the
        # redetect case (current_idx already exists in self._keyframes).
        closures = []
        current_pos = T_world_est[:3, 3]
        for kf in self._keyframes:
            if kf.index == current_idx:
                continue
            if abs(current_idx - kf.index) < self.loop_closure_min_gap:
                continue
            dist = np.linalg.norm(current_pos - kf.T_world[:3, 3])
            if dist > self.loop_closure_radius_m:
                continue

            T_init = np.linalg.inv(kf.T_world) @ T_world_est
            result = self._scanner.align(cloud_body, kf.cloud, T_init)
            if ScanMatcher.is_acceptable(result, self.min_inlier_ratio,
                                          self.min_inlier_count, self.max_error_per_inlier):
                closures.append((kf.index, result['T_target_source'], result['error_per_inlier']))
            else:
                self._rejected_edges.append((kf.index, current_idx))
        return closures

    def _find_moved_keyframes(self, result_values, threshold_m=0.1, threshold_rad=0.05):
        moved = []
        for kf in self._keyframes:
            T_new = result_values.atPose3(kf.symbol).matrix()
            T_delta = np.linalg.inv(kf.T_world) @ T_new
            dist = np.linalg.norm(T_delta[:3, 3])
            angle = np.arccos(np.clip((np.trace(T_delta[:3, :3]) - 1) / 2, -1, 1))
            if dist > threshold_m or angle > threshold_rad:
                moved.append(kf.index)
            kf.T_world = T_new
        return moved

    def _redetect_and_apply(self, moved_indices):
        """Re-run loop-closure detection for keyframes that shifted after the
        last optimization, and fold any newly found closures back into iSAM2
        (roller pattern: graph.cpp:120-143)."""
        lc_graph = gtsam.NonlinearFactorGraph()
        for idx in moved_indices:
            kf = self._keyframes[idx]
            already = {i for i, _, _ in kf.loop_closures}
            new_lcs = [
                lc for lc in self._detect_loop_closures(idx, kf.cloud, kf.T_world)
                if lc[0] not in already
            ]
            for lc_idx, lc_T, lc_err in new_lcs:
                lc_graph.add(gtsam.BetweenFactorPose3(
                    self._keyframes[lc_idx].symbol, kf.symbol, gtsam.Pose3(lc_T),
                    self._robust_icp_noise))
                kf.loop_closures.append((lc_idx, lc_T, lc_err))

        if lc_graph.size() > 0:
            self._isam.update(lc_graph, gtsam.Values())
            result_values = self._isam.calculateEstimate()
            for kf in self._keyframes:
                kf.T_world = result_values.atPose3(kf.symbol).matrix()

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _broadcast_tf(self, T: np.ndarray, stamp):
        tf_msg = matrix_to_transform_stamped(T, stamp, self.world_frame, self.base_frame)
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_odometry(self, T: np.ndarray, stamp):
        self.pub_odom.publish(matrix_to_odom(T, stamp, self.world_frame))

    def _gtsam_cov_to_ros(self, cov_gtsam_6x6: np.ndarray) -> np.ndarray:
        """Reorder GTSAM [rot|trans] covariance to ROS [trans|rot]."""
        ros_cov = np.zeros((6, 6))
        ros_cov[:3, :3] = cov_gtsam_6x6[3:6, 3:6]
        ros_cov[:3, 3:] = cov_gtsam_6x6[3:6, 0:3]
        ros_cov[3:, :3] = cov_gtsam_6x6[0:3, 3:6]
        ros_cov[3:, 3:] = cov_gtsam_6x6[0:3, 0:3]
        return ros_cov

    def _publish_keyframe_results(self, kf: Keyframe, cov_6x6: np.ndarray):
        ros_cov = self._gtsam_cov_to_ros(cov_6x6)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = kf.stamp
        pose_msg.header.frame_id = self.world_frame
        pose_stamped = matrix_to_pose_stamped(kf.T_world, kf.stamp, self.world_frame)
        pose_msg.pose.pose = pose_stamped.pose
        pose_msg.pose.covariance = ros_cov.flatten().tolist()
        self.pub_pose.publish(pose_msg)

        self._path_slam_msgs.append(pose_stamped)
        self._publish_path(self.pub_path_slam, self._path_slam_msgs)
        self._publish_path(self.pub_path_gt, self._path_dr_msgs)

        self.pub_keyframe_count.publish(Int32(data=len(self._keyframes)))
        n_closures = sum(len(k.loop_closures) for k in self._keyframes)
        self.pub_loop_closure_count.publish(Int32(data=n_closures))

        cov_pos = cov_6x6[3:6, 3:6]
        dopt = float(np.power(max(np.linalg.det(cov_pos), 0.0), 1.0 / 3.0))
        self.pub_dopt.publish(Float64(data=dopt))

        self._publish_graph_edges()
        self._publish_covariance_ellipsoids()

    def _publish_path(self, publisher, poses: list):
        path = Path()
        if poses:
            path.header = poses[-1].header
        else:
            path.header.frame_id = self.world_frame
        path.poses = poses
        publisher.publish(path)

    def _publish_graph_edges(self):
        markers = MarkerArray()
        stamp = self._keyframes[-1].stamp

        def line_marker(marker_id, ns, color, points):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = marker_id
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color = color
            m.points = points
            return m

        odom_pts, seq_pts, lc_pts, rej_pts = [], [], [], []
        for i, kf in enumerate(self._keyframes):
            if i == 0:
                continue
            prev = self._keyframes[i - 1]
            odom_pts += [Point(x=prev.T_world[0, 3], y=prev.T_world[1, 3], z=prev.T_world[2, 3]),
                         Point(x=kf.T_world[0, 3], y=kf.T_world[1, 3], z=kf.T_world[2, 3])]
        for kf in self._keyframes:
            for lc_idx, _, _ in kf.loop_closures:
                target = self._keyframes[lc_idx]
                lc_pts += [Point(x=target.T_world[0, 3], y=target.T_world[1, 3], z=target.T_world[2, 3]),
                           Point(x=kf.T_world[0, 3], y=kf.T_world[1, 3], z=kf.T_world[2, 3])]
        for a, b in self._rejected_edges:
            if a >= len(self._keyframes) or b >= len(self._keyframes):
                continue
            ka, kb = self._keyframes[a], self._keyframes[b]
            rej_pts += [Point(x=ka.T_world[0, 3], y=ka.T_world[1, 3], z=ka.T_world[2, 3]),
                        Point(x=kb.T_world[0, 3], y=kb.T_world[1, 3], z=kb.T_world[2, 3])]

        markers.markers.append(line_marker(0, 'odometry', ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.8), odom_pts))
        markers.markers.append(line_marker(1, 'loop_closures', ColorRGBA(r=0.0, g=0.8, b=0.0, a=1.0), lc_pts))
        markers.markers.append(line_marker(2, 'rejected', ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.6), rej_pts))
        self.pub_graph_edges.publish(markers)

    def _publish_covariance_ellipsoids(self):
        """Draw an ellipsoid for every Nth keyframe plus the latest one.

        Each keyframe's covariance is whatever was cached when it was the
        newest node (see _add_keyframe) — we never re-query marginalCovariance
        for older nodes, so displayed ellipsoids can be a stale (typically
        conservative/oversized) upper bound after a later loop closure.
        """
        markers = MarkerArray()
        stamp = self._keyframes[-1].stamp
        n_sigma = self.cov_ellipsoid_n_sigma

        display_kfs = list(self._keyframes[::self.cov_ellipsoid_stride])
        if display_kfs[-1] is not self._keyframes[-1]:
            display_kfs.append(self._keyframes[-1])

        for kf in display_kfs:
            if kf.covariance is None:
                continue
            cov_body = kf.covariance[3:6, 3:6]
            R = kf.T_world[:3, :3]
            cov_world = R @ cov_body @ R.T
            evals, evecs = np.linalg.eigh(cov_world)
            evals = np.clip(evals, 0.0, None)

            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = 'covariance'
            m.id = kf.index
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = kf.T_world[0, 3]
            m.pose.position.y = kf.T_world[1, 3]
            m.pose.position.z = kf.T_world[2, 3]
            quat = orthonormalize(evecs)
            q = Rotation.from_matrix(quat).as_quat()
            m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
            m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
            m.scale.x = max(2.0 * n_sigma * np.sqrt(evals[0]), 1e-3)
            m.scale.y = max(2.0 * n_sigma * np.sqrt(evals[1]), 1e-3)
            m.scale.z = max(2.0 * n_sigma * np.sqrt(evals[2]), 1e-3)
            m.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.35)
            markers.markers.append(m)

        self.pub_covariance.publish(markers)


def orthonormalize_pose(T: np.ndarray) -> np.ndarray:
    T = T.copy()
    T[:3, :3] = orthonormalize(T[:3, :3])
    return T


def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
