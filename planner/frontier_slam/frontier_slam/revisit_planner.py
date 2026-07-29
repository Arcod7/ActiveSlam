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
    pose_graph's own loop_closure_radius_m of it. Both terms are scaled to
    0..1 across the candidates in contention, so w_density and w_travel are a
    plain preference ratio rather than a count traded against a length.
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
    # Fire on distance travelled since the last revisit instead of on U_r.
    # 0 = off, which leaves the uncertainty trigger in charge.
    #
    # This exists to be compared against, not to be used. The uncertainty signal
    # is a weak estimator of true error (AUC 0.65 as a detector of a run's worst
    # quartile), so it is fair to ask whether triggering on it beats triggering
    # on a clock. Matching a fixed schedule would mean the D-optimality
    # machinery earns nothing here; beating it would mean the signal is poorly
    # calibrated in magnitude yet still usefully ordered in time.
    schedule_every_m: float = 0.0
    # A single tick's motion above this is a graph correction, not travel.
    max_travel_step_m: float = 1.0
    # Closures that must fire before a revisit is allowed to end on closure
    # count alone. 0 = not taken into account, which leaves U_r the only
    # uncertainty-based exit. Default 0: ending on the first closure pre-empts
    # the U_r test, and one closure rarely brings the covariance back under
    # ratio_resume -- the vehicle resumed exploring with sigma_xy still just
    # under sigma_allow. The trigger is stated in uncertainty
    # (U_r = D(Sigma)/D(Sigma_allow), Suresh et al. 2020 eq. 5), so the
    # symmetric exit is "uncertainty restored", not "a closure happened".
    min_closures: int = 0
    min_keyframes: int = 30
    min_index_gap: int = 20
    candidate_radius_m: float = 5.0
    min_target_dist_m: float = 3.0
    # Weights over 0..1 terms, so this is a preference ratio. Keep in step with
    # the declare_parameter default, which is what the node runs on.
    w_density: float = 1.0
    w_travel: float = 1.0
    # Transit budget only, measured from the trigger: give up if the target
    # cannot even be reached. Once there, arrival_dwell_s owns the clock.
    revisit_timeout_s: float = 120.0
    arrival_radius_m: float = 2.5
    # How long to sit at the target waiting for the uncertainty to come back
    # down before declaring the detour sterile.
    arrival_dwell_s: float = 30.0
    # Leave sooner once the graph saturates pose_graph's per-cell keyframe cap:
    # with no keyframe and no closure, no covariance change is possible. 0 = off.
    stall_exit_s: float = 5.0
    cooldown_s: float = 60.0


def _unit_scale(values: np.ndarray) -> np.ndarray:
    """0..1 against the largest value present; an all-zero input stays zero."""
    peak = float(np.max(values)) if values.size else 0.0
    return values / peak if peak > 0.0 else np.zeros(values.shape)


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
    there would already have fired). What remains is in contention.

    score = w_density*d - w_travel*t, where d and t are the density and the
    travel distance each divided by their largest value among the contenders.
    Scaling both matters: a raw density is an unbounded count that keeps
    growing as the mission lays down keyframes, so trading it against a raw
    length made travel negligible by the end of a run. Against 0..1 terms, the
    furthest-but-densest neighbourhood ties one at the robot's feet whose scaled
    density is 1 - w_travel: at 1.0 only a candidate the scoring calls maximally
    dense can justify the longest drive in contention, and every partial one
    loses to a nearer candidate. Travel is weighted as heavily as density
    because the drive out is where a detour spends its time, and it adds drift
    on the way — a 17 m outbound leg raised U_r 1.03 -> 1.51 before the target
    was even reached.
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

    d = _unit_scale(density[far_mask].astype(float))
    t = _unit_scale(dist_to_robot[far_mask])
    score = w_density * d - w_travel * t
    return int(eligible_idx[far_mask][int(np.argmax(score))])


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
        self._graph_state = None
        self._graph_changed_at = None
        self._last_revisit_end = None
        # Path length since the last revisit ended, for schedule_every_m.
        # Path length, not displacement: a vehicle orbiting one piece of
        # structure accumulates exposure to drift without ever getting far from
        # where it started, and that is exactly when a revisit is due.
        self._travelled_m = 0.0
        self._last_xy = None

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
        'RESUMED_DOPT', 'TIMEOUT', 'ARRIVED_SATURATED', 'ARRIVED_STERILE',
        'COOLDOWN_DONE') or None if nothing changed this tick.

        The sigmas only label a trigger with its cause; without them the
        machine behaves exactly as before and the cause stays None.
        """
        self._accumulate_travel(robot_xy)
        if self.state == RevisitState.EXPLORING:
            return self._try_trigger(now, dopt, lc_count, kf_xyz, robot_xy,
                                     sigma_xy, sigma_yaw)
        if self.state == RevisitState.REVISITING:
            return self._tick_revisiting(now, dopt, lc_count, kf_xyz, robot_xy)
        return self._tick_cooldown(now)

    def _accumulate_travel(self, robot_xy) -> None:
        xy = np.asarray(robot_xy, dtype=float)[:2]
        if self._last_xy is not None:
            step = float(np.linalg.norm(xy - self._last_xy))
            # A pose jump on loop closure is a correction, not travel, so it
            # must not be banked as progress toward the next scheduled revisit.
            if step < self.cfg.max_travel_step_m:
                self._travelled_m += step
        self._last_xy = xy

    def _try_trigger(self, now, dopt, lc_count, kf_xyz, robot_xy,
                     sigma_xy=None, sigma_yaw=None):
        cfg = self.cfg
        if cfg.schedule_every_m > 0.0:
            # Scheduled mode ignores U_r entirely -- that is the point of it.
            if self._travelled_m < cfg.schedule_every_m:
                return None
        else:
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
        self._graph_state = None
        self._graph_changed_at = None
        return 'TRIGGER'

    def _tick_revisiting(self, now, dopt, lc_count, kf_xyz, robot_xy):
        cfg = self.cfg
        # min_closures = 0 disables this exit entirely, leaving U_r to decide.
        # ARRIVED_STERILE and revisit_timeout_s remain the backstops either
        # way, so a revisit cannot run forever.
        if (cfg.min_closures > 0
                and lc_count - self._lc_at_start >= cfg.min_closures):
            return self._end_revisit(now, 'CLOSED')
        # Scheduled mode entered without consulting U_r, so it must not leave on
        # U_r either. The schedule fires precisely when uncertainty is low, so a
        # ratio_resume exit would end every scheduled revisit on its first tick
        # and the arm would measure nothing. Arrival, closures and the timeout
        # remain, so it still cannot run forever.
        if cfg.schedule_every_m <= 0.0:
            u_ratio = self.ratio(dopt)
            if u_ratio is not None and u_ratio < cfg.ratio_resume:
                return self._end_revisit(now, 'RESUMED_DOPT')

        self._track_arrival(now, kf_xyz, robot_xy, cfg.arrival_radius_m)
        if self._arrived_since is not None:
            if self._graph_has_stalled(now, len(kf_xyz), lc_count):
                return self._end_revisit(now, 'ARRIVED_SATURATED')
            if now - self._arrived_since > cfg.arrival_dwell_s:
                return self._end_revisit(now, 'ARRIVED_STERILE')
        # The two clocks are separate on purpose: revisit_timeout_s budgets the
        # transit, so a long drive can never eat into the recovery window the
        # dwell grants once the robot is actually at the target.
        if (self._arrived_since is None
                and now - self._t_start > cfg.revisit_timeout_s):
            return self._end_revisit(now, 'TIMEOUT')
        return None

    def _graph_has_stalled(self, now, n_kf: int, lc_count: int) -> bool:
        """True once neither the keyframe count nor the closure count has moved
        for stall_exit_s. Nothing else feeds the covariance, so the rest of the
        dwell cannot change U_r — the vehicle may as well go back to exploring.
        """
        state = (n_kf, lc_count)
        if state != self._graph_state:
            self._graph_state = state
            self._graph_changed_at = now
            return False
        return (self.cfg.stall_exit_s > 0.0
                and now - self._graph_changed_at > self.cfg.stall_exit_s)

    def _track_arrival(self, now, kf_xyz, robot_xy, arrival_radius_m) -> None:
        if self.target_idx is None or self.target_idx >= len(kf_xyz):
            return
        dist = float(np.linalg.norm(
            np.asarray(kf_xyz)[self.target_idx][:2] - np.asarray(robot_xy)[:2]))
        if dist >= arrival_radius_m:
            self._arrived_since = None
            self._graph_state = None
            self._graph_changed_at = None
        elif self._arrived_since is None:
            self._arrived_since = now

    def _end_revisit(self, now, event):
        self.state = RevisitState.COOLDOWN
        self._last_revisit_end = now
        self._travelled_m = 0.0
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
        self.declare_parameter('revisit_min_closures', 0)
        self.declare_parameter('revisit_schedule_every_m', 0.0)
        # Both counted in keyframes; scaled with keyframe_dist_m to hold their metres.
        self.declare_parameter('min_keyframes', 30)
        self.declare_parameter('min_index_gap', 20)
        self.declare_parameter('candidate_radius_m', 5.0)
        self.declare_parameter('min_target_dist_m', 3.0)
        self.declare_parameter('w_density', 1.0)
        self.declare_parameter('w_travel', 1.0)
        self.declare_parameter('revisit_timeout_s', 120.0)
        self.declare_parameter('arrival_radius_m', 2.5)
        self.declare_parameter('arrival_dwell_s', 30.0)
        self.declare_parameter('stall_exit_s', 5.0)
        self.declare_parameter('cooldown_s', 60.0)

        odom_topic = str(self.get_parameter('odom_topic').value)

        cfg = RevisitConfig(
            sigma_allow_xy_m=float(self.get_parameter('sigma_allow_xy_m').value),
            sigma_allow_yaw_rad=float(self.get_parameter('sigma_allow_yaw_rad').value),
            ratio_trigger=float(self.get_parameter('ratio_trigger').value),
            ratio_resume=float(self.get_parameter('ratio_resume').value),
            min_closures=int(self.get_parameter('revisit_min_closures').value),
            schedule_every_m=float(
                self.get_parameter('revisit_schedule_every_m').value),
            min_keyframes=int(self.get_parameter('min_keyframes').value),
            min_index_gap=int(self.get_parameter('min_index_gap').value),
            candidate_radius_m=float(self.get_parameter('candidate_radius_m').value),
            min_target_dist_m=float(self.get_parameter('min_target_dist_m').value),
            w_density=float(self.get_parameter('w_density').value),
            w_travel=float(self.get_parameter('w_travel').value),
            revisit_timeout_s=float(self.get_parameter('revisit_timeout_s').value),
            arrival_radius_m=float(self.get_parameter('arrival_radius_m').value),
            arrival_dwell_s=float(self.get_parameter('arrival_dwell_s').value),
            stall_exit_s=float(self.get_parameter('stall_exit_s').value),
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
        self._last_exit = ''

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
        # How the last revisit ended. Latched, so a subscriber that samples at
        # its own rate still sees it: the exit is one tick wide and TIMEOUT vs
        # RESUMED_DOPT is the difference between a revisit that worked and one
        # that merely ran out of budget.
        self._exit_pub = self.create_publisher(String, '/frontier_slam/revisit_exit', 10)
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
        cfg.min_closures = int(self.get_parameter('revisit_min_closures').value)
        cfg.schedule_every_m = float(
            self.get_parameter('revisit_schedule_every_m').value)
        cfg.revisit_timeout_s = float(self.get_parameter('revisit_timeout_s').value)
        cfg.arrival_dwell_s = float(self.get_parameter('arrival_dwell_s').value)
        cfg.stall_exit_s = float(self.get_parameter('stall_exit_s').value)

    def _tick(self) -> None:
        self._refresh_live_params()
        if self._robot_xy is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        prev_state = self._sm.state
        event = self._sm.tick(now, self._dopt, self._lc_count, self._kf_xyz,
                              self._robot_xy, self._sigma_xy, self._sigma_yaw)

        if event in ('CLOSED', 'RESUMED_DOPT', 'TIMEOUT', 'ARRIVED_SATURATED',
                     'ARRIVED_STERILE'):
            self._last_exit = event
        self._exit_pub.publish(String(data=self._last_exit))

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
