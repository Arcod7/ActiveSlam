"""Pure-logic tests for revisit_planner.py — no rclpy, no ROS init.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import math

import numpy as np
import pytest

from frontier_slam.revisit_planner import (
    CAUSE_HEADING, CAUSE_POSITION, RevisitConfig, RevisitState,
    RevisitStateMachine, dopt_allowable, revisit_cause, select_revisit_target,
    uncertainty_ratio,
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


def _cluster(cx, cy, k, step=0.4):
    """k keyframes packed tight enough that every member counts the other k-1
    as neighbours at candidate_radius_m=3.0 — so density == k-1 exactly."""
    return [[cx + (i % 5) * step, cy + (i // 5) * step, -1.0] for i in range(k)]


def _two_cluster_map(near_k, far_k, far_x=40.0):
    """A near cluster at the origin, a far one at far_x, and recent padding so
    both clear min_index_gap. Indices 0..near_k-1 are the near cluster."""
    padding = [[100.0 + i, 100.0, -1.0] for i in range(10)]
    return np.array(_cluster(0.0, 0.0, near_k) + _cluster(far_x, 0.0, far_k)
                    + padding)


_SELECT_KW = dict(min_index_gap=10, candidate_radius_m=3.0,
                  min_target_dist_m=3.0, w_density=1.0, w_travel=0.5)


def test_select_target_proximity_breaks_a_density_tie():
    # Two equally dense clusters: the nearer one must win. Under a pure
    # density score this is a coin toss decided by index order.
    kf = _two_cluster_map(near_k=4, far_k=4)
    target = select_revisit_target(kf, robot_xy=np.array([-5.0, 0.0]), **_SELECT_KW)
    assert target < 4, f"expected the near cluster, got {target}"


def test_select_target_proximity_beats_a_modest_density_edge():
    # 4 neighbours at the robot's feet vs 6 forty metres away: at w_travel=0.5
    # the near candidate wins. (Halve its density and it ties instead — the
    # exchange rate the weight is chosen to state.)
    kf = _two_cluster_map(near_k=5, far_k=7)
    target = select_revisit_target(kf, robot_xy=np.array([-4.0, 0.0]), **_SELECT_KW)
    assert target < 5, f"expected the near cluster, got {target}"


def test_select_target_depends_on_the_density_ratio_not_the_raw_counts():
    # The regression this guards: raw counts traded against a raw length made
    # travel negligible as keyframes accumulated, so the same map at a later
    # point in the mission silently changed answer. Both maps hold the same
    # near:far density ratio (4:9 and 8:18) and the same geometry, so the
    # decision must not move; scoring on raw counts flips it (gap 5 -> 10).
    sparse = _two_cluster_map(near_k=5, far_k=10)
    dense = _two_cluster_map(near_k=9, far_k=19)
    robot_xy = np.array([-5.0, 0.0])
    assert (select_revisit_target(sparse, robot_xy, **_SELECT_KW) < 5) == \
           (select_revisit_target(dense, robot_xy, **_SELECT_KW) < 9)


def test_select_target_picks_the_nearest_when_no_candidate_has_neighbours():
    # All densities zero — the travel term is then the only signal, and must
    # not collapse into "pick whichever came first".
    isolated = [[0.0, 0.0, -1.0], [30.0, 0.0, -1.0], [8.0, 0.0, -1.0]]
    padding = [[100.0 + i, 100.0, -1.0] for i in range(10)]
    kf = np.array(isolated + padding)
    target = select_revisit_target(kf, robot_xy=np.array([9.0, 0.0]),
                                    min_index_gap=10, candidate_radius_m=1.0,
                                    min_target_dist_m=3.0, w_density=1.0, w_travel=0.5)
    assert target == 0, f"expected the nearest isolated keyframe, got {target}"


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
    # min_closures is opt-in now: 0 (the default) ignores the closure count
    # entirely and leaves U_r as the uncertainty-based exit.
    sm = RevisitStateMachine(_cfg(min_closures=1))
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)
    assert sm.state == RevisitState.REVISITING
    event = sm.tick(now=5.0, dopt=0.05, lc_count=1, kf_xyz=kf, robot_xy=robot_xy)
    assert event == 'CLOSED'
    assert sm.state == RevisitState.COOLDOWN
    assert sm.suspended is False


def test_closures_alone_do_not_end_a_revisit_by_default():
    # The regression this guards: ending on the first closure pre-empted the
    # U_r test, so the vehicle resumed exploring with sigma still near the
    # allowance -- one closure rarely restores the covariance.
    sm = RevisitStateMachine(_cfg())            # min_closures = 0
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)

    # Several closures fire, but U_r stays above ratio_resume.
    for i, t in enumerate((5.0, 6.0, 7.0), start=1):
        assert sm.tick(now=t, dopt=0.05, lc_count=i, kf_xyz=kf,
                       robot_xy=robot_xy) is None
    assert sm.state == RevisitState.REVISITING

    # It ends when the uncertainty actually comes down.
    assert sm.tick(now=8.0, dopt=0.005, lc_count=3, kf_xyz=kf,
                   robot_xy=robot_xy) == 'RESUMED_DOPT'


def test_two_closures_required_when_min_closures_is_two():
    sm = RevisitStateMachine(_cfg(min_closures=2))
    kf = _kf_for_trigger()
    robot_xy = np.array([19.0, 0.0])
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy)

    assert sm.tick(now=5.0, dopt=0.05, lc_count=1, kf_xyz=kf,
                   robot_xy=robot_xy) is None
    assert sm.tick(now=6.0, dopt=0.05, lc_count=2, kf_xyz=kf,
                   robot_xy=robot_xy) == 'CLOSED'


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
    # lc_count keeps moving, so the stall exit stays out of the way: this is
    # the case where the graph is alive but the uncertainty never recovers.
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=10.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    target_xy = kf[sm.target_idx][:2]
    # arrive within arrival_radius_m
    at_target = target_xy + np.array([0.1, 0.0])
    event = sm.tick(now=1.0, dopt=0.05, lc_count=1, kf_xyz=kf, robot_xy=at_target)
    assert event is None
    assert sm.state == RevisitState.REVISITING
    # still there past the dwell window, uncertainty never recovered -> give up
    for i, t in enumerate((5.0, 9.0), start=2):
        assert sm.tick(now=t, dopt=0.05, lc_count=i, kf_xyz=kf,
                       robot_xy=at_target) is None
    event = sm.tick(now=12.0, dopt=0.05, lc_count=4, kf_xyz=kf, robot_xy=at_target)
    assert event == 'ARRIVED_STERILE'
    assert sm.state == RevisitState.COOLDOWN


def test_arrival_dwell_is_not_cut_short_by_the_transit_timeout():
    # revisit_timeout_s budgets the drive out, not the recovery window: a
    # target reached late must still get its full dwell, or a long detour
    # silently buys less time to close a loop than a short one.
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=10.0, arrival_dwell_s=30.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    at_target = kf[sm.target_idx][:2] + np.array([0.1, 0.0])

    # Arrives at t=9, one second before the transit budget runs out. lc_count
    # keeps moving so the stall exit stays out of this test's way.
    assert sm.tick(now=9.0, dopt=0.05, lc_count=1, kf_xyz=kf,
                   robot_xy=at_target) is None
    # Well past the transit timeout, still inside the dwell -> keeps waiting.
    for i, t in enumerate((20.0, 30.0), start=2):
        assert sm.tick(now=t, dopt=0.05, lc_count=i, kf_xyz=kf,
                       robot_xy=at_target) is None
    assert sm.state == RevisitState.REVISITING
    # And the dwell, not the timeout, is what finally ends it.
    assert sm.tick(now=40.0, dopt=0.05, lc_count=4, kf_xyz=kf,
                   robot_xy=at_target) == 'ARRIVED_STERILE'


def test_stall_exit_ends_the_dwell_once_the_graph_stops_moving():
    # Neither the keyframe count nor the closure count moves, so nothing can
    # change U_r — the rest of arrival_dwell_s would be dead time.
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=30.0,
                                  stall_exit_s=5.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    at_target = kf[sm.target_idx][:2] + np.array([0.1, 0.0])

    assert sm.tick(now=1.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) is None
    assert sm.tick(now=5.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) is None
    event = sm.tick(now=7.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=at_target)
    assert event == 'ARRIVED_SATURATED'
    assert sm.state == RevisitState.COOLDOWN


def test_stall_clock_restarts_on_every_new_keyframe_or_closure():
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=30.0,
                                  stall_exit_s=5.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    at_target = kf[sm.target_idx][:2] + np.array([0.1, 0.0])

    # A closure every 4 s keeps the graph alive past several stall windows.
    for i, t in enumerate((1.0, 5.0, 9.0, 13.0, 17.0)):
        assert sm.tick(now=t, dopt=0.05, lc_count=i, kf_xyz=kf,
                       robot_xy=at_target) is None
    assert sm.state == RevisitState.REVISITING
    # It ends only once the closures stop.
    assert sm.tick(now=23.0, dopt=0.05, lc_count=4, kf_xyz=kf,
                   robot_xy=at_target) == 'ARRIVED_SATURATED'


def test_stall_exit_disabled_leaves_the_dwell_in_charge():
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=10.0,
                                  stall_exit_s=0.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    at_target = kf[sm.target_idx][:2] + np.array([0.1, 0.0])

    assert sm.tick(now=1.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) is None
    assert sm.tick(now=8.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) is None
    assert sm.tick(now=12.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) == 'ARRIVED_STERILE'


def test_stall_exit_does_not_fire_while_still_driving_out():
    # In transit the graph can legitimately sit still (saturated cell, paused
    # optimiser); revisit_timeout_s owns that case, not the stall exit.
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=30.0, stall_exit_s=5.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    for t in (5.0, 10.0, 20.0):
        assert sm.tick(now=t, dopt=0.05, lc_count=0, kf_xyz=kf,
                       robot_xy=np.array([19.0, 0.0])) is None
    assert sm.state == RevisitState.REVISITING


def test_stall_clock_resets_when_the_robot_drifts_back_out():
    sm = RevisitStateMachine(_cfg(arrival_radius_m=2.5, arrival_dwell_s=60.0,
                                  revisit_timeout_s=1000.0, stall_exit_s=5.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    target_xy = kf[sm.target_idx][:2]
    at_target = target_xy + np.array([0.1, 0.0])
    far_away = target_xy + np.array([50.0, 0.0])

    sm.tick(now=1.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=at_target)
    sm.tick(now=4.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=far_away)
    # Re-arrives at t=10 with the graph still frozen: the clock starts again
    # there, so t=13 is only 3 s in and too early to give up.
    assert sm.tick(now=10.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) is None
    assert sm.tick(now=13.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) is None
    assert sm.tick(now=17.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                   robot_xy=at_target) == 'ARRIVED_SATURATED'


def test_transit_timeout_still_fires_once_the_robot_leaves_the_target():
    # The dwell must not become an open-ended reprieve: drift back out and the
    # transit budget applies again.
    sm = RevisitStateMachine(_cfg(revisit_timeout_s=10.0, arrival_dwell_s=30.0))
    kf = _kf_for_trigger()
    sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=np.array([19.0, 0.0]))
    target_xy = kf[sm.target_idx][:2]
    sm.tick(now=9.0, dopt=0.05, lc_count=0, kf_xyz=kf,
            robot_xy=target_xy + np.array([0.1, 0.0]))

    event = sm.tick(now=20.0, dopt=0.05, lc_count=0, kf_xyz=kf,
                    robot_xy=target_xy + np.array([50.0, 0.0]))

    assert event == 'TIMEOUT'


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


# ----------------------------------------------------------------------
# revisit_cause — attribution of an already-fired trigger, not a threshold
# ----------------------------------------------------------------------

def test_cause_is_the_axis_furthest_past_its_own_allowance():
    assert revisit_cause(0.20, 0.05, 0.1, 0.1) == CAUSE_POSITION
    assert revisit_cause(0.05, 0.20, 0.1, 0.1) == CAUSE_HEADING


def test_cause_is_relative_to_each_allowance_not_the_raw_sigma():
    # Yaw is numerically the smaller sigma yet the larger exceedance: 4x its
    # allowance against 1.5x for XY.
    assert revisit_cause(0.15, 0.04, 0.1, 0.01) == CAUSE_HEADING


def test_cause_counts_position_twice_for_two_axes():
    # Equal exceedance on both (2x each): XY carries exponent 4/3 against
    # yaw's 2/3, so two drifting axes outweigh one.
    assert revisit_cause(0.2, 0.2, 0.1, 0.1) == CAUSE_POSITION
    # Yaw only wins once its exceedance passes the square of XY's.
    assert revisit_cause(0.2, 0.5, 0.1, 0.1) == CAUSE_HEADING


def test_cause_factorises_the_ratio_it_explains():
    # U_r == r_xy**(4/3) * r_yaw**(2/3) is the identity the weighting rests on.
    sigma_xy, sigma_yaw, allow_xy, allow_yaw = 0.3, 0.08, 0.1, 0.05
    u_ratio = uncertainty_ratio(dopt_allowable(sigma_xy, sigma_yaw),
                                dopt_allowable(allow_xy, allow_yaw))
    assert u_ratio == pytest.approx(
        (sigma_xy / allow_xy) ** (4 / 3) * (sigma_yaw / allow_yaw) ** (2 / 3))


def test_cause_is_none_without_both_sigmas():
    assert revisit_cause(None, 0.05, 0.1, 0.1) is None
    assert revisit_cause(0.05, None, 0.1, 0.1) is None
    assert revisit_cause(0.0, 0.05, 0.1, 0.1) is None
    assert revisit_cause(0.05, 0.05, 0.1, 0.0) is None
    assert revisit_cause(float('nan'), 0.05, 0.1, 0.1) is None


# ----------------------------------------------------------------------
# The state machine latches the cause at the trigger
# ----------------------------------------------------------------------

def test_trigger_latches_the_cause_and_clears_it_on_exit():
    sm = RevisitStateMachine(_cfg(min_closures=1))
    kf, robot_xy = _kf_for_trigger(), np.array([19.0, 0.0])
    assert sm.cause is None
    assert sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy,
                   sigma_xy=0.05, sigma_yaw=0.9) == 'TRIGGER'
    assert sm.cause == CAUSE_HEADING
    # Held for the duration, even as the sigmas move during the detour.
    assert sm.tick(now=1.0, dopt=0.05, lc_count=0, kf_xyz=kf, robot_xy=robot_xy,
                   sigma_xy=0.9, sigma_yaw=0.05) is None
    assert sm.cause == CAUSE_HEADING
    assert sm.tick(now=2.0, dopt=0.05, lc_count=1, kf_xyz=kf,
                   robot_xy=robot_xy) == 'CLOSED'
    assert sm.cause is None


def test_trigger_without_sigmas_still_fires_with_no_cause():
    # The sigmas are diagnostic: the machine must behave identically without
    # them, so a missing /slam/sigma_* can never suppress a revisit.
    sm = RevisitStateMachine(_cfg())
    assert sm.tick(now=0.0, dopt=0.05, lc_count=0, kf_xyz=_kf_for_trigger(),
                   robot_xy=np.array([19.0, 0.0])) == 'TRIGGER'
    assert sm.state == RevisitState.REVISITING
    assert sm.cause is None
