"""Distance-scheduled revisit triggering.

The uncertainty trigger is a weak estimator of true error, so the question is
whether triggering on it beats triggering on a clock. That comparison needs a
scheduled arm that ignores U_r entirely; these tests pin its behaviour, and in
particular that switching it off leaves the uncertainty path byte-identical.
"""
import numpy as np
import pytest

from frontier_slam.revisit_planner import (
    RevisitConfig, RevisitState, RevisitStateMachine, dopt_allowable)


def _cfg(**kw):
    base = dict(sigma_allow_xy_m=0.1, sigma_allow_yaw_rad=0.05,
                min_keyframes=0, min_index_gap=0, min_target_dist_m=1.0,
                candidate_radius_m=5.0, cooldown_s=1.0)
    base.update(kw)
    return RevisitConfig(**base)


def _kf(n=40):
    """Keyframes strung out along +x so a far target always qualifies."""
    return np.array([[float(i), 0.0, 0.0] for i in range(n)])


def _calm(cfg):
    """A D-optimality well under the trigger, so U_r can never fire."""
    return 0.01 * dopt_allowable(cfg.sigma_allow_xy_m, cfg.sigma_allow_yaw_rad)


def _walk(sm, cfg, metres, step=0.5, t0=0.0):
    """Drive the vehicle `metres` in `step` increments, returning events."""
    events, t = [], t0
    for i in range(int(metres / step)):
        t += 1.0
        ev = sm.tick(t, _calm(cfg), 0, _kf(), np.array([i * step, 0.0]))
        if ev:
            events.append((t, ev))
    return events


def test_schedule_off_leaves_the_uncertainty_trigger_alone():
    cfg = _cfg(schedule_every_m=0.0)
    sm = RevisitStateMachine(cfg)
    # Calm graph, plenty of travel: nothing should fire without U_r.
    assert _walk(sm, cfg, 50.0) == []
    assert sm.state is RevisitState.EXPLORING


def test_schedule_fires_after_the_configured_distance():
    cfg = _cfg(schedule_every_m=10.0)
    sm = RevisitStateMachine(cfg)
    events = _walk(sm, cfg, 30.0)
    assert events, 'a scheduled revisit must fire on travel alone'
    assert events[0][1] == 'TRIGGER'
    assert sm.state is RevisitState.REVISITING


def test_schedule_ignores_uncertainty_entirely():
    """Fires on a calm graph; the point is that U_r has no say."""
    cfg = _cfg(schedule_every_m=5.0)
    sm = RevisitStateMachine(cfg)
    events = _walk(sm, cfg, 20.0)
    assert any(e == 'TRIGGER' for _, e in events)


def test_travel_is_not_banked_before_the_threshold():
    cfg = _cfg(schedule_every_m=25.0)
    sm = RevisitStateMachine(cfg)
    assert _walk(sm, cfg, 12.0) == [], 'fired early'


def test_a_pose_jump_is_not_counted_as_travel():
    """A loop closure teleports the estimate; that is a correction, not progress.

    Without the guard a single 30 m correction would satisfy a 25 m schedule
    outright and trigger a revisit the vehicle never earned.
    """
    cfg = _cfg(schedule_every_m=25.0, max_travel_step_m=1.0)
    sm = RevisitStateMachine(cfg)
    calm = _calm(cfg)
    sm.tick(1.0, calm, 0, _kf(), np.array([0.0, 0.0]))
    ev = sm.tick(2.0, calm, 0, _kf(), np.array([30.0, 0.0]))   # the jump
    assert ev is None
    assert sm.state is RevisitState.EXPLORING


def test_a_scheduled_revisit_does_not_exit_on_low_uncertainty():
    """It entered on distance, so ratio_resume must not end it.

    The schedule fires when U_r is low by construction; if the U_r exit stayed
    live the revisit would end on its first tick and measure nothing.
    """
    cfg = _cfg(schedule_every_m=10.0)
    sm = RevisitStateMachine(cfg)
    _walk(sm, cfg, 20.0)
    assert sm.state is RevisitState.REVISITING
    # Several more calm ticks: still revisiting, not resumed.
    for i in range(5):
        sm.tick(200.0 + i, _calm(cfg), 0, _kf(), np.array([10.0, 0.0]))
    assert sm.state is RevisitState.REVISITING


def test_the_odometer_resets_after_a_revisit_ends():
    cfg = _cfg(schedule_every_m=10.0, min_closures=1)
    sm = RevisitStateMachine(cfg)
    _walk(sm, cfg, 20.0)
    assert sm.state is RevisitState.REVISITING
    # One closure ends it, then cooldown expires.
    sm.tick(500.0, _calm(cfg), 1, _kf(), np.array([10.0, 0.0]))
    assert sm.state is RevisitState.COOLDOWN
    sm.tick(600.0, _calm(cfg), 1, _kf(), np.array([10.0, 0.0]))
    assert sm.state is RevisitState.EXPLORING
    # Fresh odometer: a short hop must not immediately re-trigger.
    ev = sm.tick(601.0, _calm(cfg), 1, _kf(), np.array([10.5, 0.0]))
    assert ev is None


def test_scheduled_and_uncertainty_modes_are_mutually_exclusive():
    """With a schedule set, a wildly over-threshold U_r still waits for distance."""
    cfg = _cfg(schedule_every_m=40.0)
    sm = RevisitStateMachine(cfg)
    hot = 1000.0 * dopt_allowable(cfg.sigma_allow_xy_m, cfg.sigma_allow_yaw_rad)
    for i in range(10):
        ev = sm.tick(float(i + 1), hot, 0, _kf(), np.array([i * 0.5, 0.0]))
        assert ev is None, 'U_r must not fire while a schedule is in charge'


# --- D-opt median filter (default off, see dopt_median_window) ---

def test_median_window_of_one_is_a_no_op():
    """The default must return the live sample untouched, or every recorded
    comparison made before the filter existed becomes incomparable."""
    from frontier_slam.revisit_planner import median_of
    import collections
    w = collections.deque(maxlen=1)
    for v in (0.1, 0.9, 0.2):
        w.append(v)
        assert median_of(w) == v


def test_median_window_rejects_a_single_dip():
    """The artefact is one relinearised recovery dipping ~2x. A median of three
    must ignore it; a mean would still be dragged down."""
    from frontier_slam.revisit_planner import median_of
    import collections
    w = collections.deque(maxlen=3)
    for v in (0.50, 0.52, 0.24):
        w.append(v)
    assert median_of(w) == 0.50


def test_median_of_empty_is_none():
    from frontier_slam.revisit_planner import median_of
    assert median_of([]) is None


def test_median_window_still_follows_a_real_rise():
    """Suppressing dips must not blind the trigger to genuine growth."""
    from frontier_slam.revisit_planner import median_of
    import collections
    w = collections.deque(maxlen=3)
    out = []
    for v in (0.10, 0.20, 0.30, 0.40, 0.50):
        w.append(v)
        out.append(median_of(w))
    assert out[-1] > out[0]
    assert out == sorted(out)
