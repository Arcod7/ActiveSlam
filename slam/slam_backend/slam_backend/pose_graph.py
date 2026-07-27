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
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
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
from slam_backend.odom_noise import odom_trans_sigma
from slam_backend.scan_matcher import ScanMatcher


def pose_delta(T_old: np.ndarray, T_new: np.ndarray) -> tuple:
    """Translation (m) and rotation (rad) between two world poses."""
    T_delta = np.linalg.inv(T_old) @ T_new
    dist = float(np.linalg.norm(T_delta[:3, 3]))
    angle = float(np.arccos(np.clip((np.trace(T_delta[:3, :3]) - 1) / 2, -1, 1)))
    return dist, angle


def worst_pose_drift(pairs) -> tuple:
    """Largest translation/rotation over (T_old, T_new) pairs, and the count.

    Returns (0.0, 0.0, 0) for an empty sequence, so a caller with nothing yet
    corrected reads as "no drift" rather than raising."""
    worst_dist = worst_angle = 0.0
    n = 0
    for T_old, T_new in pairs:
        dist, angle = pose_delta(T_old, T_new)
        worst_dist = max(worst_dist, dist)
        worst_angle = max(worst_angle, angle)
        n += 1
    return worst_dist, worst_angle, n


# Rows/cols of a GTSAM [rot|trans] 6x6 that hold the drifting DoF: x, y, yaw.
XYH_INDICES = [3, 4, 2]

# Yaw variance in a ROS [trans|rot] row-major 6x6 covariance.
YAW_COV_INDEX = 35

# Index of yaw in the GTSAM [rot|trans] prior sigma vector.
PRIOR_YAW_INDEX = 2


def _stamp_seconds(stamp) -> float:
    """builtin_interfaces/Time message -> float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def prior_sigmas_with_yaw(base_sigmas: np.ndarray, yaw_var: float) -> np.ndarray:
    """Swap the attitude filter's own yaw sigma into the attitude+depth prior.

    Roll/pitch/depth are read straight off their sensors, so the profile spec is
    the right sigma for them. Yaw is not -- the prior asserts the FUSED yaw, and
    the fusion is measurably wider than the IMU spec it was built from. Keeps the
    default when no covariance is supplied.
    """
    if not np.isfinite(yaw_var) or yaw_var <= 0.0:
        return base_sigmas
    sigmas = base_sigmas.copy()
    sigmas[PRIOR_YAW_INDEX] = float(np.sqrt(yaw_var))
    return sigmas


def dopt_xyh(cov_gtsam_6x6: np.ndarray) -> float:
    """Kiefer D-optimality det(Sigma)^(1/3) over XYH, per Suresh et al. (2020)
    eq. 4. Depth/pitch/roll are directly observed, so including them would
    deflate the geometric mean rather than report drift."""
    cov_xyh = cov_gtsam_6x6[np.ix_(XYH_INDICES, XYH_INDICES)]
    return float(np.power(max(np.linalg.det(cov_xyh), 0.0), 1.0 / 3.0))


def sigmas_xyh(cov_gtsam_6x6: np.ndarray) -> "tuple[float, float]":
    """The same marginal as dopt_xyh(), split into the two interpretable
    numbers the revisit threshold is stated in: a horizontal sigma in metres
    and a yaw sigma in radians.

    sigma_xy is the 2x2 XY block's det^(1/4), i.e. the radius of the circle of
    equal area to the covariance ellipse, so sigma_xy**4 * sigma_yaw**2 is
    dopt_xyh()**3 up to the XY-yaw cross terms.
    """
    cov_xyh = cov_gtsam_6x6[np.ix_(XYH_INDICES, XYH_INDICES)]
    sigma_xy = np.power(max(np.linalg.det(cov_xyh[:2, :2]), 0.0), 0.25)
    sigma_yaw = np.sqrt(max(cov_xyh[2, 2], 0.0))
    return float(sigma_xy), float(sigma_yaw)


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
    prepared: tuple = None          # ScanMatcher.prepare() result, built on first use as a target


def relative_angle(R_delta: np.ndarray) -> float:
    """Rotation magnitude of a 3x3 relative rotation, in radians."""
    return float(np.arccos(np.clip((np.trace(R_delta) - 1) / 2, -1, 1)))


def select_spread_candidates(candidates: list, max_candidates: int,
                             cluster_radius_m: float) -> list:
    """Thin loop-closure candidates to a bounded, spatially spread subset.

    `candidates` is [(index, distance_m, position_xyz), ...]. Nearest first, then
    greedily skip anything within cluster_radius_m of an already-kept candidate:
    twenty keyframes from one hover collapse to one representative, so the
    registration budget goes to genuinely distinct viewpoints. Returns the kept
    entries in the same tuple form.
    """
    if max_candidates <= 0:
        return []
    kept = []
    for entry in sorted(candidates, key=lambda c: c[1]):
        if cluster_radius_m > 0.0 and any(
                np.linalg.norm(entry[2] - k[2]) < cluster_radius_m for k in kept):
            continue
        kept.append(entry)
        if len(kept) >= max_candidates:
            break
    return kept


class PoseGraphNode(Node):
    def __init__(self, **kwargs):
        super().__init__('pose_graph', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('world_frame', 'world_ned')
        self.declare_parameter('base_frame', 'bluerov2/base_link')
        self.declare_parameter('camera_frame', 'bluerov2/Dcam')
        self.declare_parameter('keyframe_dist_m', 1.0)
        self.declare_parameter('keyframe_angle_rad', 0.3)
        # Station-keeping cap: without it a hover (or a sweep rotating in place)
        # keeps crossing keyframe_angle_rad and piles up co-located keyframes,
        # which is what makes closure candidacy degenerate to all-pairs.
        self.declare_parameter('keyframe_max_per_cell', 3)
        self.declare_parameter('keyframe_cell_radius_m', 0.5)
        self.declare_parameter('keyframe_cell_angle_rad', 0.5)
        self.declare_parameter('loop_closure_enabled', True)
        self.declare_parameter('loop_closure_radius_m', 5.0)
        self.declare_parameter('loop_closure_min_gap', 10)
        # Per-keyframe registration budget. Candidates are thinned to at most
        # max_candidates, spread at least cluster_radius apart, so the budget is
        # not spent on many near-identical targets from one hover.
        self.declare_parameter('loop_closure_max_candidates', 4)
        self.declare_parameter('loop_closure_cluster_radius_m', 1.0)
        # A pair that failed registration is not retried until an endpoint moves.
        self.declare_parameter('loop_closure_retry_move_m', 0.5)
        # Information-gain gate, default off (0.0): skip detection when the pose
        # is already better constrained than this D-optimality floor.
        self.declare_parameter('loop_closure_dopt_floor', 0.0)
        self.declare_parameter('redetect_max_keyframes', 5)
        self.declare_parameter('redetect_min_interval_s', 5.0)
        self.declare_parameter('min_inlier_ratio', 0.3)
        self.declare_parameter('min_inlier_count', 50)
        self.declare_parameter('max_error_per_inlier', 0.05)
        # DVL dead-reckoning is far more accurate than sparse-sonar scan
        # registration, so they get separate noise models. One shared (tight)
        # sigma let noisy scan matches override good odometry -- dragging the
        # estimate via sequential factors and warping it via loop closures.
        # odom_sigma_trans is a FLOOR for near-stationary edges; the real
        # per-edge value is derived from the DVL profile in odom_noise.py.
        self.declare_parameter('odom_sigma_rot', 0.02)
        self.declare_parameter('odom_sigma_trans', 0.002)
        self.declare_parameter('scan_sigma_rot', 0.08)
        self.declare_parameter('scan_sigma_trans', 0.12)
        self.declare_parameter('scan_voxel_size', 0.1)
        self.declare_parameter('scan_max_correspondence_dist', 0.5)
        # Same values iSAM2 was hardcoded to; exposed so the matrix can A/B them.
        self.declare_parameter('isam_relinearize_threshold', 0.01)
        self.declare_parameter('isam_relinearize_skip', 1)
        # Whole-graph chi-square costs O(factors); every keyframe is too often.
        self.declare_parameter('diagnostics_stride', 10)
        self.declare_parameter('viz_period_s', 1.0)
        self.declare_parameter('path_dr_min_move_m', 0.05)
        self.declare_parameter('cov_ellipsoid_stride', 5)
        self.declare_parameter('cov_ellipsoid_n_sigma', 2.0)
        self.declare_parameter('map_rebuild_enabled', False)
        self.declare_parameter('rebuild_min_move_m', 0.3)
        self.declare_parameter('rebuild_min_move_rad', 0.15)
        self.declare_parameter('rebuild_min_interval_s', 30.0)

        p = self.get_parameter
        self.world_frame = p('world_frame').value
        self.base_frame = p('base_frame').value
        self.camera_frame = p('camera_frame').value
        self.keyframe_dist_m = p('keyframe_dist_m').value
        self.keyframe_angle_rad = p('keyframe_angle_rad').value
        self.keyframe_max_per_cell = p('keyframe_max_per_cell').value
        self.keyframe_cell_radius_m = p('keyframe_cell_radius_m').value
        self.keyframe_cell_angle_rad = p('keyframe_cell_angle_rad').value
        self.loop_closure_enabled = p('loop_closure_enabled').value
        self.loop_closure_radius_m = p('loop_closure_radius_m').value
        self.loop_closure_min_gap = p('loop_closure_min_gap').value
        self.loop_closure_max_candidates = p('loop_closure_max_candidates').value
        self.loop_closure_cluster_radius_m = p('loop_closure_cluster_radius_m').value
        self.loop_closure_retry_move_m = p('loop_closure_retry_move_m').value
        self.loop_closure_dopt_floor = p('loop_closure_dopt_floor').value
        self.redetect_max_keyframes = p('redetect_max_keyframes').value
        self.redetect_min_interval_s = p('redetect_min_interval_s').value
        self.min_inlier_ratio = p('min_inlier_ratio').value
        self.min_inlier_count = p('min_inlier_count').value
        self.max_error_per_inlier = p('max_error_per_inlier').value
        self.diagnostics_stride = p('diagnostics_stride').value
        self.path_dr_min_move_m = p('path_dr_min_move_m').value
        self.cov_ellipsoid_stride = p('cov_ellipsoid_stride').value
        self.cov_ellipsoid_n_sigma = p('cov_ellipsoid_n_sigma').value
        self.map_rebuild_enabled = p('map_rebuild_enabled').value
        # rebuild_min_move_m/rad and rebuild_min_interval_s are intentionally
        # NOT cached here -- they're read live via get_parameter() at point of
        # use so `ros2 param set` takes effect without a restart (needed for
        # forced-trigger tests).

        yaml_path = p('noise_profile_path').value
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            profile = NoiseProfile()
        else:
            profile = load_noise_profile(yaml_path)
        self._profile = profile

        odom_sigmas = (p('odom_sigma_rot').value, p('odom_sigma_trans').value)
        scan_sigmas = (p('scan_sigma_rot').value, p('scan_sigma_trans').value)

        self._scanner = ScanMatcher(
            max_correspondence_dist=p('scan_max_correspondence_dist').value,
            downsampling_resolution=p('scan_voxel_size').value)

        self._setup_gtsam(odom_sigmas, scan_sigmas, profile)

        self._keyframes: list[Keyframe] = []
        # (T_odom, T_world) of the newest keyframe, as one tuple the optimiser
        # replaces in a single assignment. The odometry thread reads the pair
        # without a lock; reading the two fields off a live Keyframe could
        # straddle a solve and mix a new T_world with an old T_odom.
        self._anchor: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_dead_reckoned_T = None
        self._latest_dead_reckoned_stamp = None
        self._latest_yaw_var = 0.0    # 0 until odometry arrives -> profile default
        self._T_base_cam = None   # static extrinsic, resolved lazily via TF
        self._icp_calls = 0           # registrations this keyframe, incl. re-detection
        self._last_redetect_time = None
        self._last_path_dr_pos = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Separate groups, run by a MultiThreadedExecutor in main(). Keyframe
        # processing costs O(graph) and grows through a run; on one thread it
        # blocked this node's own 10 Hz /slam/odometry republish, and the
        # controllers — which differentiate that pose — saw it frozen for up to
        # 20 s at a time and drove the vehicle on it. The odometry callback
        # itself is two matrix products against _anchor.
        self.create_subscription(Odometry, '/slam/sensors/dead_reckoned_odom',
                                  self._dead_reckoned_odom_cb, 10,
                                  callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(PointCloud2, '/cloud_in', self._cloud_in_cb, 10,
                                 callback_group=MutuallyExclusiveCallbackGroup())

        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/slam/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/slam/odometry', 10)
        self.pub_path_gt = self.create_publisher(Path, '/slam/path_dr', 10)
        self.pub_path_slam = self.create_publisher(Path, '/slam/path_slam', 10)
        self.pub_graph_edges = self.create_publisher(MarkerArray, '/slam/graph_edges', 10)
        self.pub_covariance = self.create_publisher(MarkerArray, '/slam/covariance', 10)
        self.pub_keyframe_count = self.create_publisher(Int32, '/slam/keyframe_count', 10)
        self.pub_loop_closure_count = self.create_publisher(Int32, '/slam/loop_closure_count', 10)
        self.pub_dopt = self.create_publisher(Float64, '/slam/dopt', 10)
        # The same marginal per axis, so a reader can see which one drove D-opt.
        self.pub_sigma_xy = self.create_publisher(Float64, '/slam/sigma_xy', 10)
        self.pub_sigma_yaw = self.create_publisher(Float64, '/slam/sigma_yaw', 10)
        # Consistency diagnostics: unlike D-optimality these say whether the
        # assumed noise models match the residuals actually observed, and need
        # no ground truth, so they are available on a real vehicle too.
        self.pub_nis = self.create_publisher(Float64, '/slam/nis', 10)
        self.pub_chi2 = self.create_publisher(Float64, '/slam/chi2_normalized', 10)

        # Map rebuild (parameterized, off by default): after a big loop
        # closure moves keyframes, publish only the per-keyframe pose
        # corrections (old + new path, equal length, paired by index) --
        # tsdf_mapper.py keeps its own full scan cache and replays it with
        # these corrections applied, rather than pose_graph streaming every
        # keyframe scan back out (that only replayed keyframes, ~1/m, and
        # discarded everything in between -- see docs/plans/
        # WP0_WP1_WP2_revisit_rebuild.md WP2).
        self.pub_rebuild_begin = self.create_publisher(Int32, '/slam/rebuild/begin', 10)
        self.pub_rebuild_path_old = self.create_publisher(Path, '/slam/rebuild/path_old', 10)
        self.pub_rebuild_path_new = self.create_publisher(Path, '/slam/rebuild/path_new', 10)
        self.pub_rebuild_count = self.create_publisher(Int32, '/slam/rebuild_count', 10)
        self._rebuild_count = 0
        self._last_rebuild_time = None
        self._rebuild_old_poses: dict = {}   # idx -> T_world at last correction, cleared per rebuild

        # Per-keyframe cost, so a hover's growth is measurable rather than felt.
        self.pub_keyframe_ms = self.create_publisher(Float64, '/slam/timing/keyframe_ms', 10)
        self.pub_isam_ms = self.create_publisher(Float64, '/slam/timing/isam_ms', 10)
        self.pub_icp_calls = self.create_publisher(Int32, '/slam/timing/icp_calls', 10)

        self._path_dr_msgs = []
        self._path_slam_msgs = []
        self._rejected_edges = []   # [(idx_a, idx_b)] for viz only, cleared each keyframe
        # frozenset({i, j}) -> (pos_i, pos_j) at the failed attempt. Registration
        # failures were previously retried on every re-detection pass forever;
        # a pair is now reconsidered only once an endpoint has actually moved.
        self._failed_pairs: dict = {}
        self._closed_pairs: set = set()   # frozenset({i, j}) per accepted closure edge --
        # dedupes a pair across initial detection (recorded only under the newer
        # keyframe) and _redetect_and_apply (which re-scans from an older keyframe's
        # side and would otherwise re-add the same physical edge under it too).

        # Paths and markers are O(keyframes)/O(closures) to rebuild and were
        # republished on every keyframe; on a timer they cost the same at 1 Hz
        # however fast keyframes arrive. /slam/pose stays per-keyframe.
        self.create_timer(float(p('viz_period_s').value), self._publish_visualization,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f"PoseGraph started. noise_profile={profile.name}, "
            f"keyframe_dist={self.keyframe_dist_m}m, keyframe_angle={self.keyframe_angle_rad}rad, "
            f"keyframe_max_per_cell={self.keyframe_max_per_cell}, "
            f"loop_closure_enabled={self.loop_closure_enabled}, "
            f"loop_closure_max_candidates={self.loop_closure_max_candidates}, "
            f"loop_closure_dopt_floor={self.loop_closure_dopt_floor}, "
            f"map_rebuild_enabled={self.map_rebuild_enabled}, "
            f"odom_sigma=({odom_sigmas[0]},{odom_sigmas[1]}), "
            f"scan_sigma=({scan_sigmas[0]},{scan_sigmas[1]})")

    # ------------------------------------------------------------------
    # GTSAM setup
    # ------------------------------------------------------------------
    def _setup_gtsam(self, odom_sigmas, scan_sigmas, profile: NoiseProfile):
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(
            float(self.get_parameter('isam_relinearize_threshold').value))
        # relinearizeSkip is a plain property in gtsam 4.2.1, not a setter.
        isam_params.relinearizeSkip = int(self.get_parameter('isam_relinearize_skip').value)
        self._isam = gtsam.ISAM2(isam_params)

        # Two relative-factor noise models. DVL dead-reckoning is accurate
        # (~cm/keyframe) and NON-robust so a bad scan/loop factor cannot
        # down-weight it; sonar scan-matching is decimeter-level and robustified
        # so outlier registrations are suppressed. GTSAM Pose3 tangent order:
        # [rot_x, rot_y, rot_z, trans_x, trans_y, trans_z].
        # odom_trans is now a FLOOR, not the sigma: the per-edge value comes
        # from the DVL profile via odom_trans_sigma(), because a constant cannot
        # scale with edge length or respond to the noise profile at all. The
        # static model below is kept for the anchor prior on keyframe 0.
        odom_rot, odom_trans = odom_sigmas
        self._odom_rot = odom_rot
        self._odom_trans_floor = odom_trans
        self._odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([odom_rot] * 3 + [odom_trans] * 3))
        scan_rot, scan_trans = scan_sigmas
        self._scan_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([scan_rot] * 3 + [scan_trans] * 3))
        self._robust_scan_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), self._scan_noise)

        # Attitude+depth prior, applied on every node from the SAME fused
        # dead-reckoning reading (roller pattern, graph.cpp:44-46): x/y left
        # unconstrained (1e3) since only registration constrains horizontal
        # position; roll/pitch/yaw from IMU, z from pressure.
        # The yaw entry is a fallback only -- prior_sigmas_with_yaw replaces it
        # with the attitude filter's live posterior when odometry carries one.
        self._prior_sigmas = np.array([
            profile.imu.sigma_roll_rad,
            profile.imu.sigma_pitch_rad,
            profile.imu.sigma_yaw_rad,
            1e3,
            1e3,
            profile.pressure.sigma_depth_m,
        ])

        # Scratch keys for the throwaway factor _closure_nis scores; never
        # inserted into iSAM2, so they cannot collide with keyframe symbols.
        self._sym_a = gtsam.symbol('n', 0)
        self._sym_b = gtsam.symbol('n', 1)

    def _odom_noise_for(self, T_delta: np.ndarray, prev_stamp, stamp):
        """Dead-reckoning noise for one edge, scaled by how far and how long
        the vehicle travelled. Rotation keeps its constant: relative attitude
        between two keyframes does not accumulate the way position does."""
        sigma = odom_trans_sigma(
            float(np.linalg.norm(T_delta[:3, 3])),
            _stamp_seconds(stamp) - _stamp_seconds(prev_stamp),
            self._profile.dvl, self._odom_trans_floor)
        return gtsam.noiseModel.Diagonal.Sigmas(
            np.array([self._odom_rot] * 3 + [sigma] * 3))

    def _att_depth_noise_for(self, yaw_var: float):
        """Per-keyframe attitude+depth prior; rebuilt because yaw varies."""
        return gtsam.noiseModel.Diagonal.Sigmas(
            prior_sigmas_with_yaw(self._prior_sigmas, yaw_var))

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def _dead_reckoned_odom_cb(self, msg: Odometry):
        T_odom = odom_to_matrix(msg)
        self._latest_dead_reckoned_T = T_odom
        self._latest_dead_reckoned_stamp = msg.header.stamp
        self._latest_yaw_var = msg.pose.covariance[YAW_COV_INDEX]

        # Appended by distance, not by tick: this list used to grow with elapsed
        # time, so a stationary vehicle inflated it and every republish with it.
        pos = T_odom[:3, 3]
        if (self._last_path_dr_pos is None
                or np.linalg.norm(pos - self._last_path_dr_pos) >= self.path_dr_min_move_m):
            self._last_path_dr_pos = pos.copy()
            self._path_dr_msgs.append(
                matrix_to_pose_stamped(T_odom, msg.header.stamp, self.world_frame))

        anchor = self._anchor
        if anchor is None:
            # No graph yet: broadcast the raw dead-reckoned pose directly.
            self._broadcast_tf(T_odom, msg.header.stamp)
            self._publish_odometry(T_odom, msg.header.stamp)
            return

        anchor_odom, anchor_world = anchor
        T_delta = np.linalg.inv(anchor_odom) @ T_odom
        T_corrected = anchor_world @ T_delta
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
        prev = self._keyframes[-1]
        T_delta = np.linalg.inv(prev.T_odom) @ T_odom_new
        dist = np.linalg.norm(T_delta[:3, 3])
        angle = relative_angle(T_delta[:3, :3])
        if dist < self.keyframe_dist_m and angle < self.keyframe_angle_rad:
            return False
        return not self._cell_is_saturated(prev.T_world @ T_delta)

    def _cell_is_saturated(self, T_world_pred: np.ndarray) -> bool:
        """True once this pose already has keyframe_max_per_cell near-identical
        neighbours. A hovering vehicle keeps crossing the distance/angle gates on
        dead-reckoning drift alone, and each extra keyframe at one viewpoint adds
        no coverage while making every later closure search longer.

        The long odometry edge that follows a saturated stretch is handled: the
        BetweenFactor sigma from odom_trans_sigma() scales with both edge length
        and elapsed time, so a skipped keyframe is not a silently tightened edge.
        """
        if self.keyframe_max_per_cell <= 0:
            return False
        pos = T_world_pred[:3, 3]
        R = T_world_pred[:3, :3]
        n_near = 0
        for kf in self._keyframes:
            if np.linalg.norm(pos - kf.T_world[:3, 3]) > self.keyframe_cell_radius_m:
                continue
            if relative_angle(kf.T_world[:3, :3].T @ R) > self.keyframe_cell_angle_rad:
                continue
            n_near += 1
            if n_near >= self.keyframe_max_per_cell:
                return True
        return False

    # ------------------------------------------------------------------
    # Core graph update
    # ------------------------------------------------------------------
    def _add_keyframe(self, T_odom: np.ndarray, cloud_body: np.ndarray, stamp):
        t_start = time.perf_counter()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        n = len(self._keyframes)
        sym = gtsam.symbol('x', n)
        self._rejected_edges = []
        self._icp_calls = 0
        # Prepared once here and reused as the source for the sequential match and
        # every closure candidate; also cached on the keyframe as a future target.
        prepared_new = self._scanner.prepare(cloud_body)

        if n == 0:
            pose = gtsam.Pose3(T_odom)
            graph.addPriorPose3(sym, pose, self._odom_noise)
            values.insert(sym, pose)
            T_world_est = T_odom
        else:
            prev = self._keyframes[-1]

            # 1) Dead-reckoning odometry BetweenFactor (tight, non-robust)
            T_delta = np.linalg.inv(prev.T_odom) @ T_odom
            graph.add(gtsam.BetweenFactorPose3(
                prev.symbol, sym, gtsam.Pose3(T_delta),
                self._odom_noise_for(T_delta, prev.stamp, stamp)))

            # 2) Sequential scan-matching BetweenFactor (looser, robustified)
            result = self._align(cloud_body, prev, T_delta, prepared_new)
            if ScanMatcher.is_acceptable(result, self.min_inlier_ratio,
                                          self.min_inlier_count, self.max_error_per_inlier):
                graph.add(gtsam.BetweenFactorPose3(
                    prev.symbol, sym, gtsam.Pose3(result['T_target_source']),
                    self._robust_scan_noise))
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
        graph.addPriorPose3(sym, gtsam.Pose3(T_prior),
                            self._att_depth_noise_for(self._latest_yaw_var))

        # 4) Loop closure detection (against the pre-update world estimate).
        # Disabled entirely (A/B benchmarking) still lets _find_moved_keyframes
        # run below -- poses stay current for path/viz even without closures.
        lc_factors = (self._detect_loop_closures(n, cloud_body, T_world_est, prepared_new)
                      if self.loop_closure_enabled and not self._well_constrained() else [])
        for lc_idx, lc_T, lc_err in lc_factors:
            graph.add(gtsam.BetweenFactorPose3(
                self._keyframes[lc_idx].symbol, sym, gtsam.Pose3(lc_T),
                self._robust_scan_noise))

        # Innovations must be read off the pre-update estimate: after the solve
        # the measurement has already been absorbed into the poses.
        nis_samples = [
            self._closure_nis(np.linalg.inv(self._keyframes[lc_idx].T_world) @ T_world_est,
                              lc_T)
            for lc_idx, lc_T, _ in lc_factors]

        t_isam = time.perf_counter()
        self._isam.update(graph, values)
        result_values = self._isam.calculateEstimate()
        isam_ms = (time.perf_counter() - t_isam) * 1e3

        # Refreshes every existing keyframe's T_world against this update AND
        # returns which ones shifted enough (>0.1 m / 0.05 rad) to warrant
        # re-running loop-closure detection from their new position — must run
        # unconditionally (not just when lc_factors fired) so poses stay
        # current for path publishing/covariance viz/future proximity checks.
        moved = self._find_moved_keyframes(result_values)

        kf_new = Keyframe(index=n, stamp=stamp, T_odom=T_odom,
                           T_world=result_values.atPose3(sym).matrix(),
                           cloud=cloud_body, symbol=sym,
                           loop_closures=list(lc_factors), prepared=prepared_new)
        self._keyframes.append(kf_new)
        self._set_anchor(kf_new)

        if lc_factors:
            self.get_logger().info(
                f"Loop closure: node {n} <-> {[i for i, _, _ in lc_factors]}")
            redetect = self._select_redetect_targets(moved)
            if redetect:
                self._redetect_and_apply(redetect)

        # Measured against the last rebuild's baseline, not against this
        # update: the correction the rebuild publishes is cumulative, so the
        # threshold deciding whether to publish it has to be too. Gated on
        # `moved` only as a cheap early-out — nothing moved, nothing changed.
        if self.map_rebuild_enabled and moved:
            max_dist, max_angle, n_drifted = self._drift_since_rebuild()
            rebuild_min_move_m = float(self.get_parameter('rebuild_min_move_m').value)
            rebuild_min_move_rad = float(self.get_parameter('rebuild_min_move_rad').value)
            if max_dist > rebuild_min_move_m or max_angle > rebuild_min_move_rad:
                self._maybe_trigger_rebuild(max_dist, max_angle, n_drifted)

        # Marginal covariance is queried only for the node just added — walking
        # every stored keyframe on each update would grow the per-keyframe cost
        # linearly with session length. Older keyframes keep the covariance
        # they were given when THEY were the newest node (see _publish_
        # covariance_ellipsoids); it goes stale (typically shrinks further)
        # after a loop closure, but stays a reasonable upper bound for display.
        kf_new.covariance = self._isam.marginalCovariance(sym)
        for nis in nis_samples:
            self.pub_nis.publish(Float64(data=nis))
        # Whole-graph chi-square walks every factor, so it runs on a stride and
        # reuses the estimate already solved for above rather than re-deriving it.
        if self.diagnostics_stride > 0 and n % self.diagnostics_stride == 0:
            self.pub_chi2.publish(Float64(data=self._normalized_chi2(result_values)))
        self._publish_keyframe_results(kf_new, kf_new.covariance)

        self.pub_isam_ms.publish(Float64(data=isam_ms))
        self.pub_icp_calls.publish(Int32(data=self._icp_calls))
        self.pub_keyframe_ms.publish(
            Float64(data=(time.perf_counter() - t_start) * 1e3))

    def _closure_nis(self, T_predicted: np.ndarray, T_measured: np.ndarray) -> float:
        """Normalised innovation squared for one loop closure, chi-square with
        6 DoF against the scan-matching noise model alone.

        The estimate's own covariance is deliberately NOT in the denominator,
        so a single spike is not evidence of a bad noise model: a correct
        closure after real drift carries that drift in its innovation and will
        read high. It is the distribution over a run that is diagnostic — a
        median far below 6 means scan_sigma_* is looser than the matches need.
        Scored on the non-robust model, since the Huber weight would flatten
        exactly the large residuals worth seeing."""
        factor = gtsam.BetweenFactorPose3(
            self._sym_a, self._sym_b, gtsam.Pose3(T_measured), self._scan_noise)
        values = gtsam.Values()
        values.insert(self._sym_a, gtsam.Pose3())
        values.insert(self._sym_b, gtsam.Pose3(orthonormalize_pose(T_predicted)))
        return float(2.0 * factor.error(values))

    def _align(self, source_cloud, target_kf, T_init, source_prepared):
        """One registration against a keyframe, reusing both sides' preprocessing.

        The target's downsample and KD-tree are built once per keyframe and cached
        on it; previously every candidate rebuilt the target tree from scratch.
        """
        if target_kf.prepared is None:
            target_kf.prepared = self._scanner.prepare(target_kf.cloud)
        self._icp_calls += 1
        return self._scanner.align(
            source_cloud, target_kf.cloud, T_init,
            source_prepared=source_prepared, target_prepared=target_kf.prepared)

    def _well_constrained(self) -> bool:
        """True when the newest pose is already tighter than loop_closure_dopt_floor.

        The first closure on returning to a known place absorbs the accumulated
        drift; once the marginal has collapsed, further closures to the same place
        re-use correlated evidence, so they cost registrations and bias the
        covariance downward without adding information. Off by default (floor 0.0).
        """
        if self.loop_closure_dopt_floor <= 0.0 or not self._keyframes:
            return False
        cov = self._keyframes[-1].covariance
        return cov is not None and dopt_xyh(cov) < self.loop_closure_dopt_floor

    def _select_redetect_targets(self, moved: list) -> list:
        """Most-displaced keyframes only, rate-limited.

        Re-detection used to run a full candidate scan for every moved keyframe,
        so one closure that shifted the whole graph cost O(n) scans in a single
        callback. The largest shifts are the ones likely to expose a new closure.
        """
        if not moved or self.redetect_max_keyframes <= 0:
            return []
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self._last_redetect_time is not None
                and now - self._last_redetect_time < self.redetect_min_interval_s):
            return []
        self._last_redetect_time = now
        ranked = sorted(moved, key=lambda m: m[1], reverse=True)
        return [idx for idx, _, _ in ranked[:self.redetect_max_keyframes]]

    def _normalized_chi2(self, values=None) -> float:
        """Whole-graph chi-square per degree of freedom. Every factor here is
        6-dimensional, so the DoF is 6*(factors - nodes). Lands near 1.0 when
        the assumed sigmas match the residuals; well below means the noise
        models are looser than the data needs. Scan and closure factors are
        counted through their Huber kernel, which caps what an outlier edge can
        contribute."""
        factors = self._isam.getFactorsUnsafe()
        dof = 6 * (factors.size() - len(self._keyframes))
        if dof <= 0:
            return float('nan')
        if values is None:
            values = self._isam.calculateEstimate()
        return float(2.0 * factors.error(values) / dof)

    def _detect_loop_closures(self, current_idx, cloud_body, T_world_est,
                              source_prepared=None):
        # Candidates are filtered by |current_idx - kf.index| rather than a
        # slice on self._keyframes: this method is reused by _redetect_and_apply
        # for an arbitrary historical (possibly early) index, where a slice
        # bound of `idx - min_gap` can go negative — Python silently
        # reinterprets a negative slice bound as "count from the end", which
        # would return the wrong (or entirely backwards) candidate set instead
        # of raising. The symmetric distance check is correct for both the
        # normal forward case (current_idx == len(self._keyframes)) and the
        # redetect case (current_idx already exists in self._keyframes).
        #
        # Two phases: the cheap index/proximity/history gates build a candidate
        # list, which is then thinned before any registration runs. Registering
        # every in-radius keyframe is what made a hover quadratic — inside one
        # the whole graph is in radius, and most of it is the same viewpoint.
        current_pos = T_world_est[:3, 3]
        if source_prepared is None:
            source_prepared = self._scanner.prepare(cloud_body)

        candidates = []
        for kf in self._keyframes:
            if kf.index == current_idx:
                continue
            if abs(current_idx - kf.index) < self.loop_closure_min_gap:
                continue
            pair = frozenset((current_idx, kf.index))
            if pair in self._closed_pairs:
                continue
            dist = np.linalg.norm(current_pos - kf.T_world[:3, 3])
            if dist > self.loop_closure_radius_m:
                continue
            if self._retry_suppressed(pair, current_pos, kf.T_world[:3, 3]):
                continue
            candidates.append((kf.index, dist, kf.T_world[:3, 3].copy()))

        closures = []
        for kf_index, _, _ in select_spread_candidates(
                candidates, self.loop_closure_max_candidates,
                self.loop_closure_cluster_radius_m):
            kf = self._keyframes[kf_index]
            pair = frozenset((current_idx, kf.index))
            T_init = np.linalg.inv(kf.T_world) @ T_world_est
            result = self._align(cloud_body, kf, T_init, source_prepared)
            if ScanMatcher.is_acceptable(result, self.min_inlier_ratio,
                                          self.min_inlier_count, self.max_error_per_inlier):
                closures.append((kf.index, result['T_target_source'], result['error_per_inlier']))
                self._closed_pairs.add(pair)
                self._failed_pairs.pop(pair, None)
            else:
                self._rejected_edges.append((kf.index, current_idx))
                self._failed_pairs[pair] = (current_pos.copy(), kf.T_world[:3, 3].copy())
        return closures

    def _retry_suppressed(self, pair, pos_a, pos_b) -> bool:
        """True while a previously failed pair sits where it failed.

        Rejected pairs carried no memory, so every re-detection pass paid full
        registration cost to fail them again. Optimization moves poses, so a
        retry is worth it once either endpoint has shifted materially.
        """
        previous = self._failed_pairs.get(pair)
        if previous is None:
            return False
        moved = max(np.linalg.norm(pos_a - previous[0]),
                    np.linalg.norm(pos_b - previous[1]))
        return bool(moved < self.loop_closure_retry_move_m)

    def _refresh_pose(self, kf, T_new: np.ndarray) -> None:
        """Overwrite kf.T_world, first recording the pose it had as of the
        last rebuild baseline (or, absent one yet, the first time this
        keyframe is refreshed here) into _rebuild_old_poses -- the rebuild
        trigger needs both endpoints to interpolate a correction from. This
        is the ONLY place kf.T_world may be assigned outside keyframe
        creation: _find_moved_keyframes and _redetect_and_apply's bulk
        iSAM2 refresh both funnel through it, since either one silently
        moving a keyframe without this capture would leave a later rebuild
        computing its correction against a stale baseline."""
        self._rebuild_old_poses.setdefault(kf.index, kf.T_world.copy())
        kf.T_world = T_new
        if kf is self._keyframes[-1]:
            self._set_anchor(kf)

    def _set_anchor(self, kf) -> None:
        """Republish basis for the odometry thread — one atomic assignment."""
        self._anchor = (kf.T_odom, kf.T_world)

    def _find_moved_keyframes(self, result_values, threshold_m=0.1, threshold_rad=0.05):
        """Returns [(index, dist, angle), ...] for keyframes that shifted more
        than the threshold *in this update* — the trigger for re-detecting
        loop closures on keyframes whose pose just changed.

        Deliberately NOT the map-rebuild trigger: see _drift_since_rebuild."""
        moved = []
        for kf in self._keyframes:
            T_new = result_values.atPose3(kf.symbol).matrix()
            dist, angle = pose_delta(kf.T_world, T_new)
            if dist > threshold_m or angle > threshold_rad:
                moved.append((kf.index, dist, angle))
            self._refresh_pose(kf, T_new)
        return moved

    def _drift_since_rebuild(self):
        """Worst keyframe correction accumulated since the last re-integration.

        This, not the per-update movement, is what the map is stale by: the
        rebuild publishes its corrections against _rebuild_old_poses, so the
        trigger has to be measured against the same baseline. With loop
        closure running continuously a metre of accumulated correction
        arrives in centimetre increments, and no single update ever crosses
        a 0.3 m threshold while the map drifts metres behind the graph."""
        return worst_pose_drift(
            (old, kf.T_world) for kf in self._keyframes
            if (old := self._rebuild_old_poses.get(kf.index)) is not None)

    def _redetect_and_apply(self, moved_indices):
        """Re-run loop-closure detection for keyframes that shifted after the
        last optimization, and fold any newly found closures back into iSAM2
        (roller pattern: graph.cpp:120-143)."""
        self.get_logger().info(f"Re-detecting loop closures for moved keyframes {moved_indices}")
        lc_graph = gtsam.NonlinearFactorGraph()
        for idx in moved_indices:
            kf = self._keyframes[idx]
            if kf.prepared is None:
                kf.prepared = self._scanner.prepare(kf.cloud)
            # _detect_loop_closures already excludes pairs in self._closed_pairs,
            # so no separate "already found" filter is needed here.
            new_lcs = self._detect_loop_closures(idx, kf.cloud, kf.T_world, kf.prepared)
            for lc_idx, lc_T, lc_err in new_lcs:
                lc_graph.add(gtsam.BetweenFactorPose3(
                    self._keyframes[lc_idx].symbol, kf.symbol, gtsam.Pose3(lc_T),
                    self._robust_scan_noise))
                kf.loop_closures.append((lc_idx, lc_T, lc_err))

        if lc_graph.size() > 0:
            self._isam.update(lc_graph, gtsam.Values())
            result_values = self._isam.calculateEstimate()
            for kf in self._keyframes:
                self._refresh_pose(kf, result_values.atPose3(kf.symbol).matrix())

    # ------------------------------------------------------------------
    # Map rebuild (parameterized; see module docstring / STATE.md)
    # ------------------------------------------------------------------
    def _maybe_trigger_rebuild(self, max_dist: float, max_angle: float, n_moved: int):
        """Publish the per-keyframe pose correction (old world pose -> new
        corrected world pose) as two paired nav_msgs/Path messages, equal
        length, index-for-index. tsdf_mapper.py keeps its own cache of every
        integrated scan and re-integrates it with these corrections applied --
        pose_graph no longer streams scans itself, so every scan between
        keyframes is preserved on rebuild, not just the keyframe ones."""
        now = self.get_clock().now().nanoseconds * 1e-9
        rebuild_min_interval_s = float(self.get_parameter('rebuild_min_interval_s').value)
        if (self._last_rebuild_time is not None
                and now - self._last_rebuild_time < rebuild_min_interval_s):
            return
        self._last_rebuild_time = now
        self._rebuild_count += 1
        self.pub_rebuild_count.publish(Int32(data=self._rebuild_count))

        stamp = self._keyframes[-1].stamp
        path_old, path_new = Path(), Path()
        path_old.header.stamp = path_new.header.stamp = stamp
        path_old.header.frame_id = path_new.header.frame_id = self.world_frame
        for kf in self._keyframes:
            T_old = self._rebuild_old_poses.get(kf.index, kf.T_world)
            path_old.poses.append(matrix_to_pose_stamped(T_old, kf.stamp, self.world_frame))
            path_new.poses.append(matrix_to_pose_stamped(kf.T_world, kf.stamp, self.world_frame))

        self.pub_rebuild_begin.publish(Int32(data=len(self._keyframes)))
        self.pub_rebuild_path_old.publish(path_old)
        self.pub_rebuild_path_new.publish(path_new)
        self._rebuild_old_poses.clear()   # next rebuild's corrections compose on top of this one

        self.get_logger().info(
            f"Map rebuild #{self._rebuild_count}: {len(self._keyframes)} keyframe corrections "
            f"({n_moved} moved, max move {max_dist:.2f}m / {np.degrees(max_angle):.1f}deg)")

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

        self.pub_keyframe_count.publish(Int32(data=len(self._keyframes)))
        n_closures = sum(len(k.loop_closures) for k in self._keyframes)
        self.pub_loop_closure_count.publish(Int32(data=n_closures))

        self.pub_dopt.publish(Float64(data=dopt_xyh(cov_6x6)))
        sigma_xy, sigma_yaw = sigmas_xyh(cov_6x6)
        self.pub_sigma_xy.publish(Float64(data=sigma_xy))
        self.pub_sigma_yaw.publish(Float64(data=sigma_yaw))

    def _publish_visualization(self):
        """Paths and markers, on a timer rather than per keyframe.

        These rebuild O(keyframes) path messages and an O(closures) LINE_LIST
        every call, which is why they no longer sit on the optimization path.
        """
        if not self._keyframes:
            return
        self._publish_path(self.pub_path_slam, self._path_slam_msgs)
        self._publish_path(self.pub_path_gt, self._path_dr_msgs)
        self._publish_graph_edges()
        self._publish_covariance_ellipsoids()

    def _publish_path(self, publisher, poses: list):
        path = Path()
        if poses:
            path.header = poses[-1].header
        else:
            path.header.frame_id = self.world_frame
        # Snapshot: the odometry thread appends to _path_dr_msgs while this
        # serialises, and a list must not grow under the serialiser.
        path.poses = list(poses)
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

        odom_pts, lc_pts, rej_pts = [], [], []
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
        # RViz keeps markers by (ns, id) forever, incl. across a restart -- drop ids we no longer draw.
        markers.markers.append(Marker(action=Marker.DELETEALL))
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
    # One thread per callback group: keyframe optimisation must not stall the
    # odometry republish the controllers steer on, nor the visualization timer.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
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
