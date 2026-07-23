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
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int32, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation

from eval_tools.run_paths import new_run_dir
from eval_tools.tum_writer import TUMWriter

PoseSample = namedtuple('PoseSample', ['t', 'pos', 'quat'])


def _stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


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
    LIVE_ARROW_HZ = 10.0   # /slam/odometry updates every dead-reckoning tick, not just per keyframe

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

        self._metrics_file = open(os.path.join(out_dir, 'metrics.csv'), 'w')
        self._metrics_file.write(
            't,abs_error,ate,rpe_trans,rpe_rot_deg,dopt,lc_count,rebuild_count,revisit_count\n')

        self._matched_pairs = []   # time-ordered [(gt_sample, est_sample), ...] for RPE
        self._sq_errors = []       # running ATE accumulator
        self._latest_dopt = None   # cached from pose_graph.py's /slam/dopt
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
        self.create_subscription(Int32, '/slam/keyframe_count', self._kf_count_cb, 10)
        self.create_subscription(Int32, '/slam/loop_closure_count', self._lc_count_cb, 10)
        self.create_subscription(Int32, '/slam/rebuild_count', self._rebuild_count_cb, 10)
        self.create_subscription(Int32, '/frontier_slam/revisit_count', self._revisit_count_cb, 10)

        self.pub_abs_error = self.create_publisher(Float64, '/eval/abs_error', 10)
        self.pub_ate = self.create_publisher(Float64, '/eval/ate', 10)
        self.pub_rpe_trans = self.create_publisher(Float64, '/eval/rpe_trans', 10)
        self.pub_rpe_rot = self.create_publisher(Float64, '/eval/rpe_rot', 10)
        self.pub_dr_error = self.create_publisher(Float64, '/eval/dr_error', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/eval/markers', 10)
        self.pub_markers_live = self.create_publisher(MarkerArray, '/eval/markers_live', 10)

        self.create_timer(1.0 / self.LIVE_ARROW_HZ, self._publish_live_arrow)

        self.get_logger().info(f'Benchmark node started. Writing to {out_dir}')

    def _dopt_cb(self, msg: Float64):
        self._latest_dopt = msg.data

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

        rpe_trans_str = f'{rpe_trans:.6f}' if rpe_trans is not None else ''
        rpe_rot_str = f'{rpe_rot_deg:.6f}' if rpe_rot_deg is not None else ''
        dopt_str = f'{self._latest_dopt:.8f}' if self._latest_dopt is not None else ''
        self._metrics_file.write(
            f'{sample.t:.6f},{abs_error:.6f},{ate:.6f},{rpe_trans_str},{rpe_rot_str},{dopt_str},'
            f'{self._latest_lc_count},{self._latest_rebuild_count},{self._latest_revisit_count}\n')
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
        rpe_t_str = f'{rpe_trans:.2f}m' if rpe_trans is not None else 'n/a'
        rpe_r_str = f'{rpe_rot_deg:.1f}deg' if rpe_rot_deg is not None else 'n/a'
        dopt_str = f'{self._latest_dopt:.4f}' if self._latest_dopt is not None else 'n/a'
        text.text = (
            f'err {abs_error:.2f}m | ATE {ate:.2f}m | RPE {rpe_t_str}/{rpe_r_str}\n'
            f'KF {self._latest_kf_count} | LC {self._latest_lc_count} | D-opt {dopt_str}'
        )
        markers.markers.append(text)

        self.pub_markers.publish(markers)

    def _publish_live_arrow(self):
        """Continuously-updating GT-vs-SLAM-belief arrow, sourced from
        /slam/odometry (published on every dead-reckoning tick, already
        loop-closure-corrected) rather than /slam/pose (only published per
        keyframe, i.e. roughly every keyframe_dist_m of travel) — the
        eval_drift arrow above is precise but visibly jumps in steps; this
        one is for watching the correction happen in real time.

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

        arrow = Marker()
        arrow.header.frame_id = 'world_ned'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'eval_drift_live'
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [Point(x=gt.pos[0], y=gt.pos[1], z=gt.pos[2]),
                        Point(x=est.pos[0], y=est.pos[1], z=est.pos[2])]
        arrow.scale.x = 0.08   # shaft diameter
        arrow.scale.y = 0.16   # head diameter
        arrow.scale.z = 0.20   # head length
        arrow.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.9)

        markers = MarkerArray()
        markers.markers.append(arrow)
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

    def destroy_node(self):
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
