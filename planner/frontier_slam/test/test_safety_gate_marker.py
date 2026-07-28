"""Label and colour rules for the vehicle marker the motion safety gate
publishes — the arrow in RViz and the state row of the eval HUD panel."""

from types import SimpleNamespace

from frontier_slam.safety_gate import (
    DISABLED_COLOR, ENABLED_COLOR, INIT_SCAN_COLOR, MotionSafetyGate,
    REVISIT_COLOR,
)


def _gate(enabled=True, revisit_state=None, revisit_age_s=0.0,
          revisit_cause=None, cause_age_s=0.0,
          activity=None, activity_age_s=0.0):
    """A stand-in with just the fields _marker_state reads — no ROS node."""
    gate = SimpleNamespace(
        _state=SimpleNamespace(enabled=enabled),
        _revisit_state=revisit_state,
        _revisit_state_time=100.0 - revisit_age_s,
        _revisit_cause=revisit_cause,
        _revisit_cause_time=100.0 - cause_age_s,
        _activity=activity,
        _activity_time=100.0 - activity_age_s,
        _marker_state_timeout_s=3.0,
        _now=lambda: 100.0,
    )
    gate._fresh = lambda *a: MotionSafetyGate._fresh(gate, *a)
    return gate


def _state(**kwargs):
    return MotionSafetyGate._marker_state(_gate(**kwargs))


def test_enabled_without_planner_state_is_green():
    assert _state() == ('MOTION ENABLED', ENABLED_COLOR)


def test_revisiting_is_cyan():
    assert _state(revisit_state='revisiting') == ('REVISITING', REVISIT_COLOR)


def test_revisiting_names_the_axis_that_drove_the_trigger():
    assert _state(revisit_state='revisiting', revisit_cause='position') == (
        'REVISITING — POSITION DRIFT', REVISIT_COLOR)
    assert _state(revisit_state='revisiting', revisit_cause='heading') == (
        'REVISITING — HEADING DRIFT', REVISIT_COLOR)


def test_revisiting_without_a_usable_cause_stays_unqualified():
    # Empty string is what the planner publishes outside a revisit, and an
    # unknown value must not reach the HUD raw.
    assert _state(revisit_state='revisiting', revisit_cause='') == (
        'REVISITING', REVISIT_COLOR)
    assert _state(revisit_state='revisiting', revisit_cause='something_new') == (
        'REVISITING', REVISIT_COLOR)
    assert _state(revisit_state='revisiting', revisit_cause='position',
                  cause_age_s=4.0) == ('REVISITING', REVISIT_COLOR)


def test_initial_scan_is_white():
    assert _state(activity='INIT_SCAN') == ('INITIAL SCAN', INIT_SCAN_COLOR)


def test_other_activities_stay_green_with_their_own_label():
    assert _state(activity='FOLLOW_PATH') == ('DRIVING TO WAYPOINT', ENABLED_COLOR)
    assert _state(revisit_state='cooldown', activity='SCAN') == (
        'SCANNING FOR FRONTIERS', ENABLED_COLOR)


def test_unmapped_activity_falls_back_to_its_own_name():
    assert _state(activity='SOME_NEW_MODE') == ('SOME NEW MODE', ENABLED_COLOR)


def test_stale_states_fall_back_to_plain_enabled():
    assert _state(revisit_state='revisiting', revisit_age_s=4.0) == (
        'MOTION ENABLED', ENABLED_COLOR)
    assert _state(activity='INIT_SCAN', activity_age_s=4.0) == (
        'MOTION ENABLED', ENABLED_COLOR)


def test_revisit_outranks_initial_scan():
    assert _state(revisit_state='revisiting', activity='INIT_SCAN') == (
        'REVISITING', REVISIT_COLOR)


def test_disabled_gate_stays_purple_whatever_the_planner_says():
    assert _state(enabled=False, revisit_state='revisiting') == (
        'MOTION DISABLED', DISABLED_COLOR)
    assert _state(enabled=False, activity='INIT_SCAN') == (
        'MOTION DISABLED', DISABLED_COLOR)
