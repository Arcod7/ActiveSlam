"""Pure-logic tests for revisit_planner.py — no rclpy, no ROS init.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import math

import numpy as np
import pytest

from frontier_slam.revisit_planner import (
    RevisitConfig, RevisitState, RevisitStateMachine, dopt_allowable,
    select_revisit_target, uncertainty_ratio,
)


# ----------------------------------------------------------------------
# select_revisit_target
# ----------------------------------------------------------------------

def _line_keyframes(n, spacing=1.0):
    """n keyframes at (0,0), (spacing,0), (2*spacing,0), ... along +x."""
    return np.array([[i * spacing, 0.0, -1.0] for i in range(n)])


def test_select_target_too_few_keyframes_returns_none():
    kf = _line_keyframes(5)   # latest_idx=4, min_index_gap=10 -> nothing eligible
    assert select_revisit_target(kf, robot_xy=np.array([4.0, 0.0]),
                                  min_index_gap=10, candidate_radius_m=5.0,
                                  min_target_dist_m=3.0, w_density=1.0, w_travel=0.2) is None


def test_select_target_all_too_close_returns_none():
    kf = _line_keyframes(20, spacing=1.0)   # eligible: idx 0..9
    robot_xy = np.array([5.0, 0.0])   # within 3m of every eligible keyframe (2..9)... check closest
    # eligible keyframes are at x=0..9; robot at x=5 -> distances 0..5, several < 3
    # use min_target_dist_m large enough that ALL eligible are excluded
    assert select_revisit_target(kf, robot_xy=robot_xy,
                                  min_index_gap=10, candidate_radius_m=5.0,
                                  min_target_dist_m=100.0, w_density=1.0, w_travel=0.2) is None


def test_select_target_dense_cluster_beats_nearer_isolated():
    # Dense cluster of 4 old keyframes near (0,0); one isolated old keyframe
    # near (10,0), closer to the robot. Recent (ineligible) keyframes pad
    # the index so both clusters satisfy min_index_gap.
    cluster = [[0.0, 0.0, -1.0], [0.5, 0.0, -1.0], [0.0, 0.5, -1.0], [0.5, 0.5, -1.0]]
    isolated = [[10.0, 0.0, -1.0]]
    recent_padding = [[20.0 + i, 0.0, -1.0] for i in range(10)]   # indices 5..14
    kf = np.array(cluster + isolated + recent_padding)
    # latest_idx = 14, min_index_gap=10 -> eligible idx <= 4 (the 5 old ones: 0-3 cluster, 4 isolated)
    robot_xy = np.array([10.0, 5.0])   # closer to isolated (idx 4, dist~5) than cluster (idx 0-3, dist~11)
    target = select_revisit_target(kf, robot_xy, min_index_gap=10, candidate_radius_m=1.0,
                                    min_target_dist_m=3.0, w_density=1.0, w_travel=0.2)
    # isolated (idx 4) is closer (lower travel penalty) but has density 0;
    # cluster members have density 3 each. density reward (3.0) should beat
    # the extra ~6m*0.2=1.2 travel penalty.
    assert target in (0, 1, 2, 3), f"expected a dense-cluster member, got {target}"


def test_select_target_recent_keyframes_never_picked():
    kf = _line_keyframes(20, spacing=1.0)
    robot_xy = np.array([19.0, 0.0])
    target = select_revisit_target(kf, robot_xy, min_index_gap=10, candidate_radius_m=5.0,
                                    min_target_dist_m=3.0, w_density=1.0, w_travel=0.2)
    assert target is not None
    assert target <= 19 - 10   # index <= latest - min_index_gap


# ----------------------------------------------------------------------
# RevisitStateMachine
# ----------------------------------------------------------------------

# Equal per-axis sigmas make D(Sigma_allow) exactly sigma^2, so this config
# trips at dopt 0.02 and resumes at 0.01 — the thresholds these tests were
# written against, now expressed as an allowable covariance.
_SIGMA_ALLOW = math.sqrt(0.02)


def _cfg(**overrides):
    base = dict(sigma_allow_xy_m=_SIGMA_ALLOW, sigma_allow_yaw_rad=_SIGMA_ALLOW,
                ratio_trigger=1.0, ratio_resume=0.5, min_keyframes=15,
                min_index_gap=10, candidate_radius_m=5.0, min_target_dist_m=3.0,
                w_density=1.0, w_travel=0.2, revisit_timeout_s=120.0,
                arrival_radius_m=2.5, arrival_dwell_s=30.0, cooldown_s=60.0)
    base.update(overrides)
    return RevisitConfig(**base)


def _kf_for_trigger(n=20):
    return _line_keyframes(n, spacing=1.0)


def test_no_trigger_below_trigger_ratio():
    sm = RevisitStateMachine(_cfg())
    kf = _kf_for_trigger()
    event = sm.tick(now=0.0, dopt=0.01, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    assert event is None
    assert sm.state == RevisitState.EXPLORING


def test_no_trigger_below_min_keyframes():
    sm = RevisitStateMachine(_cfg(min_keyframes=50))
    kf = _kf_for_trigger(n=20)
    event = sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    assert event is None
    assert sm.state == RevisitState.EXPLORING


def test_trigger_fires_and_sets_target_and_suspend():
    sm = RevisitStateMachine(_cfg())
    kf = _kf_for_trigger()
    event = sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    assert event == 'TRIGGER'
    assert sm.state == RevisitState.REVISITING
    assert sm.suspended is True
    assert sm.target_idx is not None
    assert sm.revisit_count == 1


def test_no_retrigger_during_cooldown():
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=1.0))
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert sm.state == RevisitState.REVISITING
    # force timeout -> COOLDOWN
    event = sm.tick(now=10.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert event == 'TIMEOUT'
    assert sm.state == RevisitState.COOLDOWN
    # still high dopt, but cooldown not elapsed -> no retrigger
    event = sm.tick(now=10.5, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert event is None
    assert sm.state == RevisitState.COOLDOWN
    assert sm.revisit_count == 1


def test_exit_on_loop_closure_success():
    sm = RevisitStateMachine(_cfg())
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert sm.state == RevisitState.REVISITING
    event = sm.tick(now=5.0, dopt=0.05, lc_count=1, kf_xyz=kf, robot_xy=robot_xy)
    assert event == 'CLOSED'
    assert sm.state == RevisitState.COOLDOWN
    assert sm.suspended is False


def test_exit_on_resume_ratio():
    sm = RevisitStateMachine(_cfg())
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    event = sm.tick(now=5.0, dopt=0.005, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert event == 'RESUMED_DOPT'
    assert sm.state == RevisitState.COOLDOWN


def test_exit_on_timeout():
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=10.0))
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    event = sm.tick(now=11.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert event == 'TIMEOUT'
    assert sm.state == RevisitState.COOLDOWN


def test_exit_on_arrival_dwell_sterile():
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=10.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    target_xy = kf[sm.target_idx][:2]
    # arrive within arrival_radius_m
    at_target = target_xy + np.array([0.1, 0.0])
    event = sm.tick(now=1.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=at_target)
    assert event is None
    assert sm.state == RevisitState.REVISITING
    # still there past the dwell window, no closure -> sterile give-up
    event = sm.tick(now=12.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=at_target)
    assert event == 'ARRIVED_STERILE'
    assert sm.state == RevisitState.COOLDOWN


def test_arrival_timer_resets_if_robot_leaves():
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=10.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    target_xy = kf[sm.target_idx][:2]
    at_target = target_xy + np.array([0.1, 0.0])
    far_away = target_xy + np.array([50.0, 0.0])
    sm.tick(now=1.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=at_target)
    sm.tick(now=5.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=far_away)   # leaves before dwell elapses
    event = sm.tick(now=12.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=at_target)
    # arrival timer should have reset at t=5, so at t=12 (7s since re-arrival) not yet sterile
    assert event is None
    assert sm.state == RevisitState.REVISITING


def test_cooldown_expires_and_returns_to_exploring():
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=1.0, cooldown_s=5.0))
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    sm.tick(now=2.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)   # -> COOLDOWN (timeout)
    assert sm.state == RevisitState.COOLDOWN
    event = sm.tick(now=6.9, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert event is None
    assert sm.state == RevisitState.COOLDOWN
    event = sm.tick(now=7.1, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert event == 'COOLDOWN_DONE'
    assert sm.state == RevisitState.EXPLORING


def test_revisit_count_increments_once_per_trigger_not_per_tick():
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=1.0, cooldown_s=1.0))
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    sm.tick(now=0.5, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)   # still REVISITING
    assert sm.revisit_count == 1
    sm.tick(now=2.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)   # -> COOLDOWN (timeout)
    sm.tick(now=3.5, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)   # -> EXPLORING
    sm.tick(now=3.6, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)   # -> REVISITING again
    assert sm.revisit_count == 2


@pytest.mark.parametrize("state,expected_suspended", [
    (RevisitState.EXPLORING, False),
    (RevisitState.REVISITING, True),
    (RevisitState.COOLDOWN, False),
])
def test_suspended_flag_per_state(state, expected_suspended):
    sm = RevisitStateMachine(_cfg())
    sm.state = state
    assert sm.suspended is expected_suspended


# ----------------------------------------------------------------------
# Uncertainty ratio (Suresh et al. 2020 eq. 5)
# ----------------------------------------------------------------------

def test_dopt_allowable_is_sigma_squared_for_equal_axes():
    # D-opt is the geometric mean of the eigenvalues, so equal sigmas collapse
    # to sigma^2 — the property _cfg relies on to state thresholds directly.
    assert dopt_allowable(0.2, 0.2) == pytest.approx(0.04)


def test_dopt_allowable_weights_all_three_axes():
    # x and y both count, so the yaw term cannot dominate on its own.
    assert dopt_allowable(0.1, 0.4) == pytest.approx((0.01 * 0.01 * 0.16) ** (1 / 3))


def test_uncertainty_ratio_is_one_at_the_allowable_covariance():
    allow = dopt_allowable(0.1, 0.1)
    assert uncertainty_ratio(allow, allow) == pytest.approx(1.0)


def test_uncertainty_ratio_none_without_a_reading():
    assert uncertainty_ratio(None, 0.02) is None


def test_uncertainty_ratio_none_for_degenerate_allowance():
    assert uncertainty_ratio(0.05, 0.0) is None


def test_state_machine_ratio_matches_its_config():
    sm = RevisitStateMachine(_cfg())
    assert sm.dopt_allow() == pytest.approx(0.02)
    assert sm.ratio(0.05) == pytest.approx(2.5)


def test_trigger_scales_with_the_allowable_covariance():
    # The same D-opt reading is tolerable under a loose allowance and not
    # under a tight one — the point of expressing the threshold as a ratio.
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    loose = RevisitStateMachine(_cfg(sigma_allow_xy_m=0.5, sigma_allow_yaw_rad=0.5))
    assert loose.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy) is None
    tight = RevisitStateMachine(_cfg(sigma_allow_xy_m=0.05, sigma_allow_yaw_rad=0.05))
    assert tight.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy) == 'TRIGGER'
