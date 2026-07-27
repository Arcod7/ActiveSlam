#!/usr/bin/env python3
"""Real-time trajectory evaluation: compares ground truth against the SLAM
estimate (and, for reference, the raw dead-reckoning baseline). The scored
estimate is ``/slam/odometry``, the continuously corrected pose published at
the dead-reckoning rate; scoring sparse ``/slam/pose`` keyframes would weight
turns more heavily than straight travel and bias ATE/RPE between runs.

Publishes running scalars for RViz/PlotJuggler and writes TUM trajectory files
plus a metrics.csv for offline analysis (`evo_ape`, `evo_rpe`,
plot_results.py). RPE uses a fixed temporal delta (``rpe_delta`` seconds), not
a fixed number of samples.

Terminology: "abs_error"/"ate" are computed WITHOUT SE3 (Umeyama) alignment —
in simulation, ground truth and the SLAM estimate already share the world_ned
frame, so no alignment is needed. This differs from the usual offline `evo_ape
--align` convention; state this when reporting numbers.

ATE says how wrong the estimate is; NEES says whether the filter knows it.
NEES pairs each keyframe's reported covariance with the true error over the
same XYH DoF that feed D-optimality, so it measures whether the revisit
trigger's input is calibrated. A consistent estimator averages NEES ~= 3
(chi-square, 3 DoF); ANEES >> 3 means overconfident, << 3 conservative.

Two ANEES columns are reported. ``anees`` is the textbook mean; ``anees_robust``
is the median-based estimator of the same quantity, and is what the RViz HUD
shows and what the end-of-run verdict is taken from, because the mean is not
robust to a single degenerate reported covariance.
"""
import os
import bisect
import time as pytime
from collections import namedtuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Float64, Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation

from eval_tools.run_paths import new_run_dir
from eval_tools.tum_writer import TUMWriter
from eval_tools.consistency import (
    XYH_ROS_INDICES, NEES_DOF, xyh_tangent_error, normalised_squared_error,
    anees_bounds, classify_anees, covariance_rejection, robust_anees)

PoseSample = namedtuple('PoseSample', ['t', 'pos', 'quat'])

# Above this a single sample dominates the running mean, so record what produced
# it rather than leaving the spike to be reconstructed from the CSV afterwards.
NEES_OUTLIER_LOG_THRESHOLD = 1e3


def _stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _fmt(value, digits: int = 6) -> str:
    """CSV cell: empty when the signal has not been published yet."""
    return '' if value is None else f'{value:.{digits}f}'


def _odom_to_sample(msg: Odometry) -> PoseSample:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return PoseSample(_stamp_to_float(msg.header.stamp),
                       np.array([p.x, p.y, p.z]),
                       np.array([q.x, q.y, q.z, q.w]))


class TimestampBuffer:
    """Time-sorted ring buffer supporting nearest-timestamp lookup, used to
    pair asynchronous GT / estimate messages arriving at different rates."""

    def __init__(self, max_len: int = 2000, max_dt: float = 0.05):
        self._times = []
        self._samples = []
        self._max_len = max_len
        self._max_dt = max_dt

    def add(self, sample: PoseSample):
        self._times.append(sample.t)
        self._samples.append(sample)
        if len(self._times) > self._max_len:
            self._times.pop(0)
            self._samples.pop(0)

    def nearest(self, t_query: float):
        if not self._times:
            return None
        i = bisect.bisect_left(self._times, t_query)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(self._times)]
        if not candidates:
            return None
        best = min(candidates, key=lambda j: abs(self._times[j] - t_query))
        if abs(self._times[best] - t_query) > self._max_dt:
            return None
        return self._samples[best]


class BenchmarkNode(Node):
    LIVE_DRIFT_HZ = 10.0   # /slam/odometry updates every dead-reckoning tick, not just per keyframe

    def __init__(self, **kwargs):
        super().__init__('benchmark', **kwargs)

        self.declare_parameter('output_dir', '')
        self.declare_parameter('rpe_delta', 1.0)
        self.declare_parameter('gt_topic', '/StoneFish/Odometry')

        out_dir = self.get_parameter('output_dir').value
        if not out_dir:
            out_dir = new_run_dir(pytime.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(out_dir, exist_ok=True)
        self._out_dir = out_dir
        self._rpe_delta_s = float(self.get_parameter('rpe_delta').value)
        if self._rpe_delta_s <= 0.0:
            raise ValueError('rpe_delta must be positive seconds')

        self._gt_buffer = TimestampBuffer()
        self._gt_tum = TUMWriter(os.path.join(out_dir, 'gt_traj.tum'))
        self._slam_tum = TUMWriter(os.path.join(out_dir, 'slam_traj.tum'))
        self._odom_tum = TUMWriter(os.path.join(out_dir, 'odom_traj.tum'))

        # Consistency columns update at keyframe rate and are carried forward on
        # the intervening odometry-rate rows, the same way dopt already is.
        self._metrics_file = open(os.path.join(out_dir, 'metrics.csv'), 'w')
        self._metrics_file.write(
            't,abs_error,ate,rpe_trans,rpe_rot_deg,dopt,nees,anees,nis,chi2_norm,'
            'lc_count,rebuild_count,revisit_count,anees_robust,nees_rejected,'
            'sigma_xy,sigma_yaw,u_ratio\n')

        self._matched_pairs = []   # time-ordered [(gt_sample, est_sample), ...] for RPE
        self._sq_errors = []       # running ATE accumulator
        self._nees_samples = []    # running ANEES accumulator, one per keyframe
        self._nees_rejected = 0    # keyframes whose covariance failed the degeneracy gate
        self._latest_nees = None
        self._latest_anees = None
        self._latest_anees_robust = None
        self._latest_nis = None         # cached from /slam/nis, per loop closure
        self._latest_chi2_norm = None   # cached from /slam/chi2_normalized
        self._latest_dopt = None   # cached from pose_graph.py's /slam/dopt
        # The same marginal per axis, plus the ratio the revisit trigger reads:
        # D-opt alone says how uncertain, not in which DoF nor against what.
        self._latest_sigma_xy = None
        self._latest_sigma_yaw = None
        self._latest_u_ratio = None
        self._latest_kf_count = 0
        self._latest_lc_count = 0
        self._latest_rebuild_count = 0
        self._latest_revisit_count = 0
        self._latest_slam_odom: PoseSample | None = None
        self._last_scored_t: float | None = None

        gt_topic = self.get_parameter('gt_topic').value
        self.create_subscription(Odometry, gt_topic, self._gt_cb, 50)
        self.create_subscription(Odometry, '/slam/sensors/dead_reckoned_odom', self._dr_cb, 10)
        self.create_subscription(Odometry, '/slam/odometry', self._slam_odom_cb, 10)
        self.create_subscription(Float64, '/slam/dopt', self._dopt_cb, 10)
        self.create_subscription(Float64, '/slam/sigma_xy', self._sigma_xy_cb, 10)
        self.create_subscription(Float64, '/slam/sigma_yaw', self._sigma_yaw_cb, 10)
        self.create_subscription(Float64, '/frontier_slam/uncertainty_ratio',
                                 self._u_ratio_cb, 10)
        self.create_subscription(Float64, '/slam/nis', self._nis_cb, 10)
        self.create_subscription(Float64, '/slam/chi2_normalized', self._chi2_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/slam/pose', self._slam_pose_cb, 10)
        self.create_subscription(Int32, '/slam/keyframe_count', self._kf_count_cb, 10)
        self.create_subscription(Int32, '/slam/loop_closure_count', self._lc_count_cb, 10)
        self.create_subscription(Int32, '/slam/rebuild_count', self._rebuild_count_cb, 10)
        self.create_subscription(Int32, '/frontier_slam/revisit_count', self._revisit_count_cb, 10)

        self.pub_abs_error = self.create_publisher(Float64, '/eval/abs_error', 10)
        self.pub_ate = self.create_publisher(Float64, '/eval/ate', 10)
        self.pub_rpe_trans = self.create_publisher(Float64, '/eval/rpe_trans', 10)
        self.pub_rpe_rot = self.create_publisher(Float64, '/eval/rpe_rot', 10)
        self.pub_dr_error = self.create_publisher(Float64, '/eval/dr_error', 10)
        self.pub_nees = self.create_publisher(Float64, '/eval/nees', 10)
        self.pub_anees = self.create_publisher(Float64, '/eval/anees', 10)
        self.pub_anees_robust = self.create_publisher(Float64, '/eval/anees_robust', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/eval/markers', 10)
        self.pub_markers_live = self.create_publisher(MarkerArray, '/eval/markers_live', 10)

        self.create_timer(1.0 / self.LIVE_DRIFT_HZ, self._publish_live_drift)

        self.get_logger().info(f'Benchmark node started. Writing to {out_dir}')

    def _dopt_cb(self, msg: Float64):
        self._latest_dopt = msg.data

    def _sigma_xy_cb(self, msg: Float64):
        self._latest_sigma_xy = msg.data

    def _sigma_yaw_cb(self, msg: Float64):
        self._latest_sigma_yaw = msg.data

    def _u_ratio_cb(self, msg: Float64):
        self._latest_u_ratio = msg.data

    def _nis_cb(self, msg: Float64):
        self._latest_nis = msg.data

    def _chi2_cb(self, msg: Float64):
        self._latest_chi2_norm = msg.data

    def _slam_pose_cb(self, msg: PoseWithCovarianceStamped):
        """Score one keyframe's reported covariance against the true error."""
        t = _stamp_to_float(msg.header.stamp)
        gt = self._gt_buffer.nearest(t)
        if gt is None:
            return

        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        cov = np.asarray(msg.pose.covariance).reshape(6, 6)
        cov_xyh = cov[np.ix_(XYH_ROS_INDICES, XYH_ROS_INDICES)]

        rejection = covariance_rejection(cov_xyh)
        if rejection is not None:
            self._nees_rejected += 1
            self.get_logger().warn(
                f'NEES sample dropped at t={t:.3f}, degenerate covariance: {rejection}',
                throttle_duration_sec=10.0)
            return

        err = xyh_tangent_error(gt.pos, gt.quat,
                                np.array([p.x, p.y, p.z]),
                                np.array([q.x, q.y, q.z, q.w]))
        sample = normalised_squared_error(err, cov_xyh)
        if sample is None:
            return

        if sample > NEES_OUTLIER_LOG_THRESHOLD:
            self.get_logger().warn(
                f'NEES outlier {sample:.3e} at t={t:.3f}: '
                f'err_xyh={np.array2string(err, precision=5)} '
                f'cov_xyh={np.array2string(cov_xyh.ravel(), precision=9)}')

        self._nees_samples.append(sample)
        self._latest_nees = sample
        self._latest_anees = float(np.mean(self._nees_samples))
        self._latest_anees_robust = robust_anees(self._nees_samples)
        self.pub_nees.publish(Float64(data=self._latest_nees))
        self.pub_anees.publish(Float64(data=self._latest_anees))
        self.pub_anees_robust.publish(Float64(data=self._latest_anees_robust))

    def _kf_count_cb(self, msg: Int32):
        self._latest_kf_count = msg.data

    def _lc_count_cb(self, msg: Int32):
        self._latest_lc_count = msg.data

    def _rebuild_count_cb(self, msg: Int32):
        self._latest_rebuild_count = msg.data

    def _revisit_count_cb(self, msg: Int32):
        self._latest_revisit_count = msg.data

    def _gt_cb(self, msg: Odometry):
        sample = _odom_to_sample(msg)
        self._gt_buffer.add(sample)
        self._gt_tum.write_pose(sample.t, sample.pos, sample.quat)

    def _slam_odom_cb(self, msg: Odometry):
        sample = _odom_to_sample(msg)
        self._latest_slam_odom = sample

        # A simulator reset can briefly replay an old timestamp. Do not append
        # duplicate/backward samples to the trajectory or cumulative metrics.
        if self._last_scored_t is not None and sample.t <= self._last_scored_t:
            return
        self._last_scored_t = sample.t
        self._slam_tum.write_pose(sample.t, sample.pos, sample.quat)

        gt = self._gt_buffer.nearest(sample.t)
        if gt is not None:
            self._score_slam_sample(gt, sample)

    def _dr_cb(self, msg: Odometry):
        sample = _odom_to_sample(msg)
        self._odom_tum.write_pose(sample.t, sample.pos, sample.quat)
        gt = self._gt_buffer.nearest(sample.t)
        if gt is not None:
            err = float(np.linalg.norm(sample.pos - gt.pos))
            self.pub_dr_error.publish(Float64(data=err))

    def _score_slam_sample(self, gt: PoseSample, sample: PoseSample):
        """Score one continuous corrected-odometry sample against timestamped GT."""
        abs_error = float(np.linalg.norm(sample.pos - gt.pos))
        self._sq_errors.append(abs_error ** 2)
        ate = float(np.sqrt(np.mean(self._sq_errors)))

        self.pub_abs_error.publish(Float64(data=abs_error))
        self.pub_ate.publish(Float64(data=ate))

        rpe_trans, rpe_rot_deg = self._update_rpe(gt, sample)

        self._metrics_file.write(
            f'{sample.t:.6f},{abs_error:.6f},{ate:.6f},'
            f'{_fmt(rpe_trans)},{_fmt(rpe_rot_deg)},{_fmt(self._latest_dopt, 8)},'
            f'{_fmt(self._latest_nees)},{_fmt(self._latest_anees)},'
            f'{_fmt(self._latest_nis)},{_fmt(self._latest_chi2_norm)},'
            f'{self._latest_lc_count},{self._latest_rebuild_count},{self._latest_revisit_count},'
            f'{_fmt(self._latest_anees_robust)},{self._nees_rejected},'
            f'{_fmt(self._latest_sigma_xy, 6)},{_fmt(self._latest_sigma_yaw, 6)},'
            f'{_fmt(self._latest_u_ratio, 4)}\n')
        self._metrics_file.flush()

        self._publish_eval_markers(gt, sample, abs_error, ate, rpe_trans, rpe_rot_deg)

    def _publish_eval_markers(self, gt: PoseSample, est: PoseSample, abs_error: float,
                               ate: float, rpe_trans, rpe_rot_deg):
        """Drift arrow (GT -> SLAM estimate) + a live text HUD, both in
        world_ned — the direct answer to "how far off, right now" that a
        Path-only view can't show without eyeballing gaps between lines."""
        markers = MarkerArray()

        arrow = Marker()
        arrow.header.frame_id = 'world_ned'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'eval_drift'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [Point(x=gt.pos[0], y=gt.pos[1], z=gt.pos[2]),
                        Point(x=est.pos[0], y=est.pos[1], z=est.pos[2])]
        arrow.scale.x = 0.05   # shaft diameter
        arrow.scale.y = 0.12   # head diameter
        arrow.scale.z = 0.15   # head length
        arrow.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)
        markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = 'world_ned'
        text.header.stamp = arrow.header.stamp
        text.ns = 'eval_hud'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = est.pos[0]
        text.pose.position.y = est.pos[1]
        text.pose.position.z = est.pos[2] - 2.0   # NED: -z is up, so this floats above the robot
        text.scale.z = 0.4
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        rpe_t_str = f'{rpe_trans:.2f} m' if rpe_trans is not None else 'n/a'
        rpe_r_str = f'{rpe_rot_deg:.1f} deg' if rpe_rot_deg is not None else 'n/a'
        dopt_str = f'{self._latest_dopt:.4f}' if self._latest_dopt is not None else 'n/a'
        # The robust estimator, unlabelled: on a HUD the number has to be the one
        # worth acting on. The raw mean stays on /eval/anees and in the CSV.
        anees_str = (f'{self._latest_anees_robust:.1f}'
                     if self._nees_samples else 'n/a')
        rejected_str = (f'  |  NEES rej {self._nees_rejected}'
                        if self._nees_rejected else '')
        # D-opt is the two sigmas rolled into one scalar, so it sits next to
        # them: the sigmas say which DoF, U_r says how close that is to firing
        # a revisit.
        sigma_xy_str = (f'{self._latest_sigma_xy:.3f} m'
                        if self._latest_sigma_xy is not None else 'n/a')
        sigma_yaw_str = (f'{self._latest_sigma_yaw:.3f} rad'
                         if self._latest_sigma_yaw is not None else 'n/a')
        u_ratio_str = (f'{self._latest_u_ratio:.2f}'
                       if self._latest_u_ratio is not None else 'n/a')
        # Spaced pipes and spaced units: the HUD panel renders this monospaced.
        # Two lines, not three — the panel sits in a dock whose height comes
        # from the saved RViz geometry, and the width is what there is to spare.
        text.text = (
            f'err {abs_error:.2f} m  |  ATE {ate:.2f} m  |  '
            f'RPE {rpe_t_str} / {rpe_r_str}  |  '
            f'KF {self._latest_kf_count}  |  LC {self._latest_lc_count}\n'
            f'sigma xy {sigma_xy_str}  |  sigma yaw {sigma_yaw_str}  |  '
            f'D-opt {dopt_str}  |  U_r {u_ratio_str}  |  '
            f'ANEES {anees_str}{rejected_str}'
        )
        markers.markers.append(text)

        self.pub_markers.publish(markers)

    def _publish_live_drift(self):
        """Continuously-updating GT-vs-SLAM-belief line, sourced from
        /slam/odometry (published on every dead-reckoning tick, already
        loop-closure-corrected) rather than /slam/pose (only published per
        keyframe, i.e. roughly every keyframe_dist_m of travel) — the
        eval_drift arrow above is precise but visibly jumps in steps; this
        one is for watching the correction happen in real time. A plain
        segment, not an arrow: at small drift the head dominated the shaft
        and read as a blob rather than a distance.

        GT is looked up by nearest-timestamp to the estimate sample (like
        _slam_cb does), not just "whatever GT sample is freshest right now"
        — the two topics publish on independent, unsynchronized clocks, so
        pairing the latest of each independently left the arrow pointing
        along a stale phase-lag offset instead of the true instantaneous
        drift, especially visible while the robot is moving."""
        if self._latest_slam_odom is None:
            return
        est = self._latest_slam_odom
        gt = self._gt_buffer.nearest(est.t)
        if gt is None:
            return

        line = Marker()
        line.header.frame_id = 'world_ned'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'eval_drift_live'
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.points = [Point(x=gt.pos[0], y=gt.pos[1], z=gt.pos[2]),
                       Point(x=est.pos[0], y=est.pos[1], z=est.pos[2])]
        line.scale.x = 0.08   # line width
        line.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)

        # Same number the line length shows, in metres, so the segment is
        # readable without measuring it against the grid.
        label = Marker()
        label.header.frame_id = 'world_ned'
        label.header.stamp = line.header.stamp
        label.ns = 'eval_drift_live'
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        mid = 0.5 * (gt.pos + est.pos)
        label.pose.position.x = float(mid[0])
        label.pose.position.y = float(mid[1])
        label.pose.position.z = float(mid[2]) - 0.4   # NED: -z is up
        label.scale.z = 0.35
        label.color = line.color
        label.text = f'error {float(np.linalg.norm(est.pos - gt.pos)):.2f} m'

        markers = MarkerArray()
        markers.markers.append(line)
        markers.markers.append(label)
        self.pub_markers_live.publish(markers)

    def _update_rpe(self, gt_sample: PoseSample, est_sample: PoseSample):
        self._matched_pairs.append((gt_sample, est_sample))
        if len(self._matched_pairs) < 2:
            return None, None

        target_t = est_sample.t - self._rpe_delta_s
        prior = self._matched_pairs[:-1]
        prior_times = [pair[1].t for pair in prior]
        i = bisect.bisect_left(prior_times, target_t)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(prior)]
        if not candidates:
            return None, None
        best = min(candidates, key=lambda j: abs(prior_times[j] - target_t))
        # Continuous odometry is normally ~10 Hz. A large hole should yield no
        # RPE sample, not silently change the requested temporal baseline.
        tolerance_s = max(0.25, 0.25 * self._rpe_delta_s)
        if abs(prior_times[best] - target_t) > tolerance_s:
            return None, None

        gt_prev, est_prev = prior[best]
        gt_curr, est_curr = self._matched_pairs[-1]

        R_gt_prev = Rotation.from_quat(gt_prev.quat)
        R_est_prev = Rotation.from_quat(est_prev.quat)

        delta_gt_trans = R_gt_prev.inv().apply(gt_curr.pos - gt_prev.pos)
        delta_est_trans = R_est_prev.inv().apply(est_curr.pos - est_prev.pos)
        rpe_trans = float(np.linalg.norm(delta_est_trans - delta_gt_trans))

        delta_gt_rot = R_gt_prev.inv() * Rotation.from_quat(gt_curr.quat)
        delta_est_rot = R_est_prev.inv() * Rotation.from_quat(est_curr.quat)
        rpe_rot_deg = float(np.degrees(np.linalg.norm(
            (delta_gt_rot.inv() * delta_est_rot).as_rotvec())))

        self.pub_rpe_trans.publish(Float64(data=rpe_trans))
        self.pub_rpe_rot.publish(Float64(data=rpe_rot_deg))
        return rpe_trans, rpe_rot_deg

    def _log_consistency_summary(self):
        n = len(self._nees_samples)
        if n == 0:
            self.get_logger().info('No NEES samples: /slam/pose never paired with GT.')
            return
        anees = float(np.mean(self._nees_samples))
        robust = robust_anees(self._nees_samples)
        lo, hi = anees_bounds(n)
        # Classified on the robust value: the mean is the statistic a single
        # degenerate covariance destroys, so it is reported but not judged on.
        self.get_logger().info(
            f'ANEES {robust:.2f} (robust) over {n} keyframes, {NEES_DOF} DoF — '
            f'95% acceptance (mean-based, narrower than the median estimator warrants) '
            f'[{lo:.2f}, {hi:.2f}] — {classify_anees(robust, n)}')
        self.get_logger().info(f'ANEES {anees:.2f} (raw mean, outlier-sensitive)')
        if self._nees_rejected:
            self.get_logger().warn(
                f'{self._nees_rejected} keyframes excluded from ANEES on a '
                f'degenerate reported covariance.')

    def destroy_node(self):
        self._log_consistency_summary()
        self._gt_tum.close()
        self._slam_tum.close()
        self._odom_tum.close()
        self._metrics_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkNode()
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
