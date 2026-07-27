#!/usr/bin/env python3
"""Uncertainty-driven explore-vs-revisit decision node (Week 3 active-SLAM
contribution, v1 — see docs/ROADMAP.md).

Loop closure in this project is opportunistic-only: pose_graph.py only closes
a loop when the robot happens to pass within loop_closure_radius_m of an old
keyframe. This node breaks off frontier exploration on purpose when pose
uncertainty grows, drives the robot back toward a geometrically-promising old
area to try to force a closure, then resumes exploration.

State machine (RevisitStateMachine, plain Python — no rclpy, directly
unit-testable): EXPLORING -> REVISITING -> COOLDOWN -> EXPLORING.
  - Trigger source: /slam/dopt, the only live-correct uncertainty signal
    (per-keyframe covariances go stale after later closures — see
    pose_graph.py's own comments). It is compared as Suresh et al. (2020)
    eq. 5 does — the ratio U_r = D(Sigma)/D(Sigma_allow) against a maximum
    allowable covariance stated in metres and radians — rather than against a
    bare determinant threshold, which has no interpretable scale.
  - Candidate scoring v1 is plain geometric distinctiveness (the fallback
    ROADMAP explicitly allows): keyframes old enough (index gap) and far
    enough from the robot, scored by how many other old keyframes cluster
    nearby (reward) minus travel distance (penalty) — a dense old
    neighbourhood is likely to yield a closure once the robot is within
    pose_graph's own loop_closure_radius_m of it.
  - Suspends frontier_extractor's goal publication (/frontier_slam/suspend)
    while revisiting and publishes the revisit goal directly onto
    /frontier_slam/goal; waypoint_controller's own >1m goal-change hysteresis
    preempts cleanly.

Explicitly out of scope for v1 (see docs/ROADMAP.md Week 3 — future work):
FPFH/saliency-based candidate scoring, and mirror-graph virtual-factor
covariance propagation along candidate paths — this node consumes the
already-published scalar D-optimality instead of propagating anything itself.
"""
import os
from dataclasses import dataclass
from enum import Enum

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, Int32, String

from frontier_slam.session_log import open_session_log


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'state', 'dopt', 'u_ratio', 'sigma_xy', 'sigma_yaw', 'cause',
    'lc_count', 'n_kf', 'tgt_x', 'tgt_y', 'dist_m', 'revisit_count', 'event',
]

# Which axis pushed U_r over the trigger. Published verbatim on
# /frontier_slam/revisit_cause; the RViz HUD and the launcher each render it in
# their own house style, so these strings are the shared contract between them.
CAUSE_POSITION = 'position'
CAUSE_HEADING = 'heading'


def dopt_allowable(sigma_xy_m: float, sigma_yaw_rad: float) -> float:
    """D-optimality of the largest pose covariance the mission tolerates,
    built from per-axis sigmas so the threshold can be stated in metres and
    radians rather than as a bare determinant."""
    return float(np.power((sigma_xy_m ** 2) ** 2 * sigma_yaw_rad ** 2, 1.0 / 3.0))


def uncertainty_ratio(dopt, dopt_allow: float) -> "float | None":
    """Suresh et al. (2020) eq. 5: U_r = D(Sigma) / D(Sigma_allow). Revisit
    when it exceeds 1, i.e. when the estimate is less certain than the mission
    allows. Dimensionless, so the mixed metre/radian units cancel."""
    if dopt is None or dopt_allow <= 0.0:
        return None
    return float(dopt) / dopt_allow


def revisit_cause(sigma_xy, sigma_yaw, sigma_allow_xy_m: float,
                  sigma_allow_yaw_rad: float) -> "str | None":
    """Which axis is responsible for U_r, or None without both sigmas.

    U_r factorises exactly: with r_xy = sigma_xy/sigma_allow_xy and
    r_yaw = sigma_yaw/sigma_allow_yaw, U_r = r_xy**(4/3) * r_yaw**(2/3). Those
    two exponents are the comparison — position carries twice the weight
    because XY is two axes, so the larger raw sigma is not the larger cause.
    """
    values = (sigma_xy, sigma_yaw, sigma_allow_xy_m, sigma_allow_yaw_rad)
    if any(v is None or not np.isfinite(v) or v <= 0.0 for v in values):
        return None
    weight_xy = (4.0 / 3.0) * np.log(float(sigma_xy) / sigma_allow_xy_m)
    weight_yaw = (2.0 / 3.0) * np.log(float(sigma_yaw) / sigma_allow_yaw_rad)
    return CAUSE_POSITION if weight_xy >= weight_yaw else CAUSE_HEADING


class RevisitState(Enum):
    EXPLORING = 'exploring'
    REVISITING = 'revisiting'
    COOLDOWN = 'cooldown'


@dataclass
class RevisitConfig:
    # Largest pose uncertainty the mission tolerates, as per-axis sigmas.
    sigma_allow_xy_m: float = 0.045
    sigma_allow_yaw_rad: float = 0.045
    ratio_trigger: float = 1.0
    ratio_resume: float = 0.5
    min_keyframes: int = 15
    min_index_gap: int = 10
    candidate_radius_m: float = 5.0
    min_target_dist_m: float = 3.0
    w_density: float = 1.0
    w_travel: float = 0.2
    revisit_timeout_s: float = 120.0
    arrival_radius_m: float = 2.5
    arrival_dwell_s: float = 30.0
    cooldown_s: float = 60.0


def select_revisit_target(kf_xyz: np.ndarray, robot_xy: np.ndarray,
                           min_index_gap: int, candidate_radius_m: float,
                           min_target_dist_m: float, w_density: float,
                           w_travel: float) -> "int | None":
    """Pick the keyframe index most likely to yield a loop closure if
    revisited, or None if nothing qualifies.

    Eligible = old enough (index <= latest - min_index_gap). Density is
    computed over ALL eligible keyframes (how many other old keyframes
    cluster within candidate_radius_m of each candidate — irrespective of
    the candidate's own distance from the robot), then candidates closer
    than min_target_dist_m to the robot right now are excluded (a closure
    there would already have fired). score = w_density*density -
    w_travel*distance_to_robot; returns the argmax.
    """
    n = len(kf_xyz)
    if n == 0:
        return None
    latest_idx = n - 1
    idx = np.arange(n)
    eligible_mask = idx <= (latest_idx - min_index_gap)
    if not np.any(eligible_mask):
        return None

    eligible_idx = idx[eligible_mask]
    eligible_xy = np.asarray(kf_xyz)[eligible_mask][:, :2]
    robot_xy = np.asarray(robot_xy)[:2]

    dist_to_robot = np.linalg.norm(eligible_xy - robot_xy, axis=1)
    far_mask = dist_to_robot >= min_target_dist_m
    if not np.any(far_mask):
        return None

    pairwise = np.linalg.norm(
        eligible_xy[:, None, :] - eligible_xy[None, :, :], axis=2)
    density = np.sum(pairwise < candidate_radius_m, axis=1) - 1   # exclude self

    score = w_density * density - w_travel * dist_to_robot
    score = np.where(far_mask, score, -np.inf)
    best = int(np.argmax(score))
    return int(eligible_idx[best])


class RevisitStateMachine:
    """Pure decision logic, no rclpy — directly unit-testable."""

    def __init__(self, cfg: RevisitConfig):
        self.cfg = cfg
        self.state = RevisitState.EXPLORING
        self.revisit_count = 0
        self.target_idx = None
        # Latched at TRIGGER: the reason it fired, not whichever axis happens
        # to dominate later once the detour has already changed the marginal.
        self.cause = None
        self._lc_at_start = 0
        self._t_start = None
        self._arrived_since = None
        self._last_revisit_end = None

    @property
    def suspended(self) -> bool:
        return self.state == RevisitState.REVISITING

    def dopt_allow(self) -> float:
        return dopt_allowable(self.cfg.sigma_allow_xy_m, self.cfg.sigma_allow_yaw_rad)

    def ratio(self, dopt) -> "float | None":
        return uncertainty_ratio(dopt, self.dopt_allow())

    def tick(self, now: float, dopt, lc_count: int,
             kf_xyz: np.ndarray, robot_xy: np.ndarray,
             sigma_xy=None, sigma_yaw=None) -> "str | None":
        """Advance one tick. Returns an event string ('TRIGGER', 'CLOSED',
        'RESUMED_DOPT', 'TIMEOUT', 'ARRIVED_STERILE', 'COOLDOWN_DONE') or
        None if nothing changed this tick.

        The sigmas only label a trigger with its cause; without them the
        machine behaves exactly as before and the cause stays None.
        """
        if self.state == RevisitState.EXPLORING:
            return self._try_trigger(now, dopt, lc_count, kf_xyz, robot_xy,
                                     sigma_xy, sigma_yaw)
        if self.state == RevisitState.REVISITING:
            return self._tick_revisiting(now, dopt, lc_count, kf_xyz, robot_xy)
        return self._tick_cooldown(now)

    def _try_trigger(self, now, dopt, lc_count, kf_xyz, robot_xy,
                     sigma_xy=None, sigma_yaw=None):
        cfg = self.cfg
        u_ratio = self.ratio(dopt)
        if u_ratio is None or u_ratio <= cfg.ratio_trigger:
            return None
        if len(kf_xyz) < cfg.min_keyframes:
            return None
        target = select_revisit_target(
            kf_xyz, robot_xy, cfg.min_index_gap, cfg.candidate_radius_m,
            cfg.min_target_dist_m, cfg.w_density, cfg.w_travel)
        if target is None:
            return None

        self.state = RevisitState.REVISITING
        self.revisit_count += 1
        self.target_idx = target
        self.cause = revisit_cause(sigma_xy, sigma_yaw, cfg.sigma_allow_xy_m,
                                   cfg.sigma_allow_yaw_rad)
        self._lc_at_start = lc_count
        self._t_start = now
        self._arrived_since = None
        return 'TRIGGER'

    def _tick_revisiting(self, now, dopt, lc_count, kf_xyz, robot_xy):
        cfg = self.cfg
        if lc_count > self._lc_at_start:
            return self._end_revisit(now, 'CLOSED')
        u_ratio = self.ratio(dopt)
        if u_ratio is not None and u_ratio < cfg.ratio_resume:
            return self._end_revisit(now, 'RESUMED_DOPT')
        if now - self._t_start > cfg.revisit_timeout_s:
            return self._end_revisit(now, 'TIMEOUT')

        if self.target_idx is not None and self.target_idx < len(kf_xyz):
            dist = float(np.linalg.norm(
                np.asarray(kf_xyz)[self.target_idx][:2] - np.asarray(robot_xy)[:2]))
            if dist < cfg.arrival_radius_m:
                if self._arrived_since is None:
                    self._arrived_since = now
            else:
                self._arrived_since = None
            if (self._arrived_since is not None
                    and now - self._arrived_since > cfg.arrival_dwell_s):
                return self._end_revisit(now, 'ARRIVED_STERILE')
        return None

    def _end_revisit(self, now, event):
        self.state = RevisitState.COOLDOWN
        self._last_revisit_end = now
        self.target_idx = None
        self.cause = None
        return event

    def _tick_cooldown(self, now):
        if now - self._last_revisit_end >= self.cfg.cooldown_s:
            self.state = RevisitState.EXPLORING
            return 'COOLDOWN_DONE'
        return None


class RevisitPlanner(Node):
    TICK_HZ = 1.0
    GOAL_REPUBLISH_S = 2.0   # matches frontier_extractor's own republish cadence

    def __init__(self):
        super().__init__('revisit_planner')

        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('sigma_allow_xy_m', 0.045)
        self.declare_parameter('sigma_allow_yaw_rad', 0.045)
        self.declare_parameter('ratio_trigger', 1.0)
        self.declare_parameter('ratio_resume', 0.5)
        self.declare_parameter('min_keyframes', 15)
        self.declare_parameter('min_index_gap', 10)
        self.declare_parameter('candidate_radius_m', 5.0)
        self.declare_parameter('min_target_dist_m', 3.0)
        self.declare_parameter('w_density', 1.0)
        self.declare_parameter('w_travel', 0.2)
        self.declare_parameter('revisit_timeout_s', 120.0)
        self.declare_parameter('arrival_radius_m', 2.5)
        self.declare_parameter('arrival_dwell_s', 30.0)
        self.declare_parameter('cooldown_s', 60.0)

        odom_topic = str(self.get_parameter('odom_topic').value)

        cfg = RevisitConfig(
            sigma_allow_xy_m=float(self.get_parameter('sigma_allow_xy_m').value),
            sigma_allow_yaw_rad=float(self.get_parameter('sigma_allow_yaw_rad').value),
            ratio_trigger=float(self.get_parameter('ratio_trigger').value),
            ratio_resume=float(self.get_parameter('ratio_resume').value),
            min_keyframes=int(self.get_parameter('min_keyframes').value),
            min_index_gap=int(self.get_parameter('min_index_gap').value),
            candidate_radius_m=float(self.get_parameter('candidate_radius_m').value),
            min_target_dist_m=float(self.get_parameter('min_target_dist_m').value),
            w_density=float(self.get_parameter('w_density').value),
            w_travel=float(self.get_parameter('w_travel').value),
            arrival_radius_m=float(self.get_parameter('arrival_radius_m').value),
            arrival_dwell_s=float(self.get_parameter('arrival_dwell_s').value),
            cooldown_s=float(self.get_parameter('cooldown_s').value),
        )
        self._sm = RevisitStateMachine(cfg)

        self._dopt = None
        self._sigma_xy = None
        self._sigma_yaw = None
        self._lc_count = 0
        self._kf_xyz = np.empty((0, 3))
        self._robot_xy = None
        self._last_goal_pub_time = None

        self._log = open_session_log('revisit', CSV_COLUMNS, _LOG_DIR,
                                     precision={'dopt': 8, 'u_ratio': 4,
                                                'sigma_xy': 6, 'sigma_yaw': 6})

        self.create_subscription(Float64, '/slam/dopt', self._dopt_cb, 10)
        # Diagnostic only — the same marginal split per axis, to label a
        # trigger with its cause. The trigger itself is the combined D-opt.
        self.create_subscription(Float64, '/slam/sigma_xy', self._sigma_xy_cb, 10)
        self.create_subscription(Float64, '/slam/sigma_yaw', self._sigma_yaw_cb, 10)
        self.create_subscription(Path, '/slam/path_slam', self._path_cb, 10)
        self.create_subscription(Int32, '/slam/loop_closure_count', self._lc_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self._suspend_pub = self.create_publisher(Bool, '/frontier_slam/suspend', 1)
        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal', 1)
        self._count_pub = self.create_publisher(Int32, '/frontier_slam/revisit_count', 10)
        self._state_pub = self.create_publisher(String, '/frontier_slam/revisit_state', 10)
        self._cause_pub = self.create_publisher(String, '/frontier_slam/revisit_cause', 10)
        self._ratio_pub = self.create_publisher(Float64, '/frontier_slam/uncertainty_ratio', 10)

        self.create_timer(1.0 / self.TICK_HZ, self._tick)
        self.get_logger().info(
            f'revisit_planner ready — revisit above U_r {cfg.ratio_trigger:g} '
            f'(D_allow {self._sm.dopt_allow():.5f} from sigma_xy '
            f'{cfg.sigma_allow_xy_m:g} m, sigma_yaw {cfg.sigma_allow_yaw_rad:g} rad) '
            f'— logging to {self._log.path}')

    # ------------------------------------------------------------------
    # ROS callbacks
    def _dopt_cb(self, msg: Float64) -> None:
        self._dopt = msg.data

    def _sigma_xy_cb(self, msg: Float64) -> None:
        self._sigma_xy = msg.data

    def _sigma_yaw_cb(self, msg: Float64) -> None:
        self._sigma_yaw = msg.data

    def _lc_cb(self, msg: Int32) -> None:
        self._lc_count = msg.data

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_xy = np.array([p.x, p.y])

    def _path_cb(self, msg: Path) -> None:
        if not msg.poses:
            self._kf_xyz = np.empty((0, 3))
            return
        self._kf_xyz = np.array([
            [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
            for ps in msg.poses])

    # ------------------------------------------------------------------
    def _refresh_live_params(self) -> None:
        """Live-tunable thresholds, re-read every tick so `ros2 param set`
        takes effect immediately (needed for forced-trigger testing)."""
        cfg = self._sm.cfg
        cfg.sigma_allow_xy_m = float(self.get_parameter('sigma_allow_xy_m').value)
        cfg.sigma_allow_yaw_rad = float(self.get_parameter('sigma_allow_yaw_rad').value)
        cfg.ratio_trigger = float(self.get_parameter('ratio_trigger').value)
        cfg.ratio_resume = float(self.get_parameter('ratio_resume').value)
        cfg.revisit_timeout_s = float(self.get_parameter('revisit_timeout_s').value)

    def _tick(self) -> None:
        self._refresh_live_params()
        if self._robot_xy is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        prev_state = self._sm.state
        event = self._sm.tick(now, self._dopt, self._lc_count, self._kf_xyz,
                              self._robot_xy, self._sigma_xy, self._sigma_yaw)

        suspended = self._sm.suspended
        self._suspend_pub.publish(Bool(data=suspended))
        self._state_pub.publish(String(data=self._sm.state.value))
        self._cause_pub.publish(String(data=self._sm.cause or ''))
        self._count_pub.publish(Int32(data=self._sm.revisit_count))

        tgt_x = tgt_y = dist_m = float('nan')
        if suspended and self._sm.target_idx is not None and self._sm.target_idx < len(self._kf_xyz):
            tgt = self._kf_xyz[self._sm.target_idx]
            tgt_x, tgt_y = float(tgt[0]), float(tgt[1])
            dist_m = float(np.linalg.norm(tgt[:2] - self._robot_xy))
            if (self._last_goal_pub_time is None
                    or now - self._last_goal_pub_time >= self.GOAL_REPUBLISH_S):
                self._publish_goal(tgt)
                self._last_goal_pub_time = now
        else:
            self._last_goal_pub_time = None

        if event is not None or prev_state != self._sm.state:
            cause = (f', driven by {self._sm.cause}' if event == 'TRIGGER'
                     and self._sm.cause else '')
            self.get_logger().info(
                f'{prev_state.value} -> {self._sm.state.value}'
                + (f' ({event}{cause})' if event else ''))

        u_ratio = self._sm.ratio(self._dopt)
        self._ratio_pub.publish(Float64(
            data=u_ratio if u_ratio is not None else float('nan')))

        self._log.write([
            now, self._sm.state.value,
            self._dopt if self._dopt is not None else float('nan'),
            u_ratio if u_ratio is not None else float('nan'),
            self._sigma_xy if self._sigma_xy is not None else float('nan'),
            self._sigma_yaw if self._sigma_yaw is not None else float('nan'),
            self._sm.cause or '',
            self._lc_count, len(self._kf_xyz),
            tgt_x, tgt_y, dist_m, self._sm.revisit_count, event or '',
        ])

    def _publish_goal(self, tgt_xyz: np.ndarray) -> None:
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x = float(tgt_xyz[0])
        goal.point.y = float(tgt_xyz[1])
        goal.point.z = float(tgt_xyz[2])
        self._goal_pub.publish(goal)

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RevisitPlanner()
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
