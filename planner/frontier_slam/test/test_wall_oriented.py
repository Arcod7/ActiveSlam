"""Pure geometry tests for wall-oriented path execution."""
import math
from types import SimpleNamespace

from frontier_slam.control_utils import HeadingReference
from frontier_slam.wall_oriented_controller import (
    choose_wall_side,
    lookahead_path_heading,
    offset_heading,
    wall_side_distances,
    WallOrientedController,
)
import numpy as np
import pytest


def test_right_wall_turns_heading_right_by_offset():
    assert offset_heading(0.0, 1, 30.0) == pytest.approx(math.radians(30.0))


def test_left_wall_turns_heading_left_by_offset():
    assert offset_heading(0.0, -1, 30.0) == pytest.approx(math.radians(-30.0))


def test_no_wall_keeps_forward_route_heading():
    assert offset_heading(1.2, 0, 30.0) == pytest.approx(1.2)


def test_offset_wraps_across_pi():
    result = offset_heading(math.radians(170.0), 1, 30.0)
    assert result == pytest.approx(math.radians(-160.0))


def test_nearest_wall_side_is_measured_relative_to_route():
    pose = np.array([0.0, 0.0, 5.0])
    points = np.array([
        [0.0, -2.0, 5.0],  # left at 2 m
        [0.0, 1.0, 5.0],   # right at 1 m
        [0.0, 0.5, 9.0],   # closer in XY, but outside the depth band
    ])
    left, right = wall_side_distances(points, pose, 0.0, 1.5, 8.0)
    assert left == pytest.approx(2.0)
    assert right == pytest.approx(1.0)
    assert choose_wall_side(left, right) == 1


def test_side_classification_rotates_with_route_heading():
    pose = np.array([0.0, 0.0, 0.0])
    # When travelling East (+Y), South/-X is starboard/right.
    points = np.array([[-1.0, 0.0, 0.0]])
    left, right = wall_side_distances(
        points, pose, math.pi / 2.0, 1.0, 8.0)
    assert math.isinf(left)
    assert right == pytest.approx(1.0)


def test_current_side_is_retained_until_other_wall_is_clearly_closer():
    assert choose_wall_side(2.0, 1.8, current_side=-1,
                            switch_margin_m=0.3) == -1
    assert choose_wall_side(2.0, 1.6, current_side=-1,
                            switch_margin_m=0.3) == 1


def test_no_usable_wall_returns_no_side():
    assert choose_wall_side(float('inf'), float('inf')) == 0


def test_zero_lookahead_keeps_the_current_route_heading():
    point, heading = lookahead_path_heading(
        [(2.0, 0.0), (2.0, 2.0)], 0, np.array([0.0, 0.0]), 0.0, 0.4)
    assert np.allclose(point, [0.0, 0.0])
    assert heading == pytest.approx(0.4)


def test_lookahead_uses_the_path_tangent_at_the_circle_intersection():
    path = [(2.0, 0.0), (2.0, 4.0)]
    point, heading = lookahead_path_heading(
        path, 0, np.array([0.0, 0.0]), 3.0, 0.0)
    assert np.allclose(point, [2.0, math.sqrt(5.0)])
    assert heading == pytest.approx(math.pi / 2.0)


def test_lookahead_falls_back_when_the_remaining_path_does_not_reach_circle():
    point, heading = lookahead_path_heading(
        [(1.0, 0.0)], 0, np.array([0.0, 0.0]), 2.0, 0.7)
    assert np.allclose(point, [0.0, 0.0])
    assert heading == pytest.approx(0.7)









def _bare_controller(**attrs):
    """A controller with only the attributes under test — no rclpy, no Node init."""
    controller = object.__new__(WallOrientedController)
    for name, value in attrs.items():
        setattr(controller, name, value)
    return controller


def _hold(route_heading, wall_side, yaw, offset_deg=45.0, yaw_rate=None,
          ticks=1):
    """yaw_rate=None models an absent gyro, i.e. the pre-cascade fallback law.

    The commanded heading is a shaped reference seeded at the vehicle's own
    yaw, so the first tick always reads on-heading; `ticks` > 1 lets the
    reference glide toward the raw target (0.9 s apart, under the reseed gap).
    """
    controller = _bare_controller(
        _last_route_heading=route_heading,
        _look_offset_deg=offset_deg,
        _wall_side=wall_side,
        _yaw=yaw,
        _gyro_at=None if yaw_rate is None else 0.0,
        _yaw_ctrl=None,
        _yaw_rate=SimpleNamespace(value=yaw_rate or 0.0),
        _look_ref=HeadingReference(
            WallOrientedController.LOOK_REF_TAU,
            WallOrientedController.LOOK_REF_RATE),
    )
    controller._select_wall_side = lambda *_: None   # side is fixed by the caller
    for i in range(ticks):
        if yaw_rate is not None:
            controller._gyro_at = 0.9 * i   # a live gyro keeps its stamp fresh
        result = controller._hold_yaw_cmd(0.9 * i)
    return result


def test_waiting_on_the_viewing_heading_commands_no_yaw():
    yaw_cmd, _, look_heading, error = _hold(0.0, 1, math.radians(45.0))
    assert yaw_cmd == pytest.approx(0.0)
    assert look_heading == pytest.approx(math.radians(45.0))
    assert error == pytest.approx(0.0)


def test_the_first_closed_loop_tick_never_demands_a_turn():
    """The reference seeds at the vehicle's yaw, whatever the raw target."""
    yaw_cmd, _, look_heading, error = _hold(0.0, 1, 0.0, ticks=1)
    assert yaw_cmd == pytest.approx(0.0)
    assert look_heading == pytest.approx(0.0)


def test_waiting_off_the_viewing_heading_glides_back_towards_it():
    yaw_cmd, _, look_heading, error = _hold(0.0, 1, 0.0, ticks=2)
    # The reference has moved toward the 45 deg target but not jumped to it.
    assert 0.0 < look_heading < math.radians(45.0)
    assert error == pytest.approx(look_heading)      # yaw is 0
    assert 0.0 < yaw_cmd <= WallOrientedController.YAW_EFFORT_LIMIT


def test_waiting_yaw_effort_is_bounded():
    yaw_cmd, _, _, _ = _hold(0.0, -1, math.radians(179.0), ticks=2)
    assert abs(yaw_cmd) <= WallOrientedController.YAW_EFFORT_LIMIT


def test_turning_towards_the_heading_reduces_the_effort_asked_for():
    """The damping term: the same error commands less while already turning.

    A 10 deg offset keeps both cases inside YAW_EFFORT_LIMIT — at the shipped
    45 deg both saturate and the comparison says nothing.
    """
    still, _, _, _ = _hold(0.0, 1, 0.0, offset_deg=10.0, yaw_rate=0.0, ticks=2)
    turning, _, _, _ = _hold(0.0, 1, 0.0, offset_deg=10.0, yaw_rate=0.1, ticks=2)
    assert 0.0 < turning < still
    assert still < WallOrientedController.YAW_EFFORT_LIMIT


def test_turning_past_the_requested_rate_commands_a_brake():
    """Overrunning the rate cap reverses the effort rather than easing it."""
    yaw_cmd, _, _, error = _hold(0.0, 1, 0.0, yaw_rate=1.2, ticks=2)
    assert error > 0.0        # still short of the heading...
    assert yaw_cmd < 0.0      # ...but arriving too fast to keep pushing


def test_absent_gyro_falls_back_to_the_pre_cascade_gain():
    """Without a rate the cascade would run at 4x the gain it replaced."""
    yaw_cmd, _, _, error = _hold(0.0, 1, 0.0, yaw_rate=None, ticks=2)
    expected = WallOrientedController.KP_YAW_FALLBACK * error
    assert yaw_cmd == pytest.approx(expected)
    cascade_gain = (WallOrientedController.KP_HEADING
                    * WallOrientedController.KP_YAW_RATE)
    assert WallOrientedController.KP_YAW_FALLBACK < cascade_gain


def test_waiting_without_a_route_heading_holds_still():
    yaw_cmd, route_heading, look_heading, error = _hold(None, 0, 0.0)
    assert yaw_cmd == 0.0
    assert all(math.isnan(v) for v in (route_heading, look_heading, error))


def test_a_new_goal_keeps_the_current_path():
    controller = _bare_controller(
        _goal=np.array([0.0, 0.0, 0.0]),
        _path=[(1.0, 1.0), (2.0, 2.0)],
        _path_at=10.0,
        _wp_idx=1,
        _goal_reached_at=5.0,
        _stuck_ref_pos=np.zeros(3),
        _stuck_ref_t=5.0,
    )
    controller._goal_cb(SimpleNamespace(
        point=SimpleNamespace(x=20.0, y=0.0, z=0.0)))
    assert controller._path == [(1.0, 1.0), (2.0, 2.0)]
    assert controller._wp_idx == 1
    assert controller._goal_reached_at is None


def _brake(goal_dist):
    controller = object.__new__(WallOrientedController)
    return controller._arrival_brake(goal_dist)


def test_approach_runs_at_full_speed_beyond_the_brake_ramp():
    assert _brake(WallOrientedController.ARRIVAL_BRAKE_M + 5.0) == pytest.approx(1.0)


def test_approach_speed_slows_to_the_floor_but_never_stops():
    """Braking to zero at the radius is an asymptote: the vehicle stalled just
    outside GOAL_RADIUS and never registered arrival."""
    floor = WallOrientedController.ARRIVAL_MIN_SCALE
    assert floor > 0.0
    assert _brake(WallOrientedController.GOAL_RADIUS) == pytest.approx(floor)
    assert _brake(WallOrientedController.GOAL_RADIUS - 1.0) == pytest.approx(floor)


def test_approach_speed_tapers_linearly_across_the_ramp():
    mid = 0.5 * (WallOrientedController.ARRIVAL_BRAKE_M
                 + WallOrientedController.GOAL_RADIUS)
    assert _brake(mid) == pytest.approx(0.5)


def test_misalignment_slows_travel_but_never_stops_it():
    """A target dead astern puts the raw look error past 90 deg; the vehicle
    must creep through the re-aim, not freeze for the length of it."""
    controller = _bare_controller(
        _pose=np.array([0.0, 0.0, 5.0]),
        _path=[], _wp_idx=0, _lookahead_m=0.0,
        _yaw=0.0, _yaw_ctrl=None, _gyro_at=None,
        _yaw_rate=SimpleNamespace(value=0.0),
        _look_ref=HeadingReference(
            WallOrientedController.LOOK_REF_TAU,
            WallOrientedController.LOOK_REF_RATE),
        _wall_side=1, _look_offset_deg=45.0,
        _last_heading_error=0.0,
    )
    controller._select_wall_side = lambda *_: None
    surge, sway, *_ = controller._drive(np.array([-3.0, 0.0]), 0.0)
    expected = (WallOrientedController.KP_SPEED * 3.0
                * WallOrientedController.ALIGN_MIN_SCALE)
    assert math.hypot(surge, sway) == pytest.approx(expected)


def _resigning(side, memory, route=0.0):
    controller = _bare_controller(
        _pose=np.array([0.0, 0.0, 5.0]),
        _wall_side=side,
        _wall_memory=None if memory is None else np.array(memory, dtype=float),
        _side_candidate=side,
        _side_candidate_t=0.0,
    )
    controller._resign_side_from_memory(route, 1.0)
    return controller


def test_a_route_reversal_resigns_the_side_at_once():
    """Same wall, new direction: right becomes left with no dwell to earn."""
    controller = _resigning(1, [0.0, 5.0, 5.0], route=math.pi)
    assert controller._wall_side == -1
    assert controller._side_candidate == -1


def test_the_side_holds_while_the_wall_stays_on_it():
    assert _resigning(1, [0.0, 5.0, 5.0], route=0.0)._wall_side == 1


def test_no_memory_or_no_side_never_resigns():
    assert _resigning(0, [0.0, 5.0, 5.0], route=math.pi)._wall_side == 0
    assert _resigning(1, None, route=math.pi)._wall_side == 1


def test_a_wall_on_the_route_axis_cannot_sign_a_side():
    controller = _resigning(1, [5.0, -0.1, 5.0], route=0.0)  # nearly dead ahead
    assert controller._wall_side == 1


def _side_controller(wall_side, points):
    """Controller wired just enough to run _select_wall_side."""
    controller = _bare_controller(
        _map_points=np.asarray(points, dtype=float),
        _map_received_at=0.0,
        _pose=np.array([0.0, 0.0, 5.0]),
        _max_wall_distance=20.0,
        _wall_z_band=2.0,
        _side_switch_margin=0.3,
        _wall_side=wall_side,
        _wall_distance=float("nan"),
        _wall_memory=None,
        _side_candidate=0,
        _side_candidate_t=0.0,
        _last_heading_error=math.radians(90.0),
        _settled_since=None,
    )
    controller.get_parameter = lambda _: SimpleNamespace(value=2.0)
    controller._publish_selected_wall = lambda _: None
    return controller


def _tick(controller, t):
    """One _select_wall_side call on a route heading of north, map kept fresh."""
    controller._map_received_at = t
    controller._select_wall_side(0.0, t)


def test_a_side_switch_waits_for_the_heading_to_settle_and_stay_settled():
    # Holding the left wall; the map now shows the only wall on the right.
    controller = _side_controller(-1, [[0.0, 1.0, 5.0]])

    # Dwell alone is not enough while the vehicle is still swinging.
    for t in (0.0, 5.0, 9.0):
        _tick(controller, t)
        assert controller._wall_side == -1

    # The error enters the settled band, but has not yet held there.
    controller._last_heading_error = math.radians(10.0)
    _tick(controller, 10.0)
    assert controller._wall_side == -1
    _tick(controller, 11.0)
    assert controller._wall_side == -1

    # Held for SIDE_SWITCH_SETTLED_S — only now may the side change.
    _tick(controller, 12.1)
    assert controller._wall_side == 1


def test_a_momentary_dip_into_the_settled_band_does_not_authorise_a_switch():
    controller = _side_controller(-1, [[0.0, 1.0, 5.0]])
    _tick(controller, 0.0)

    controller._last_heading_error = math.radians(10.0)
    _tick(controller, 10.0)
    controller._last_heading_error = math.radians(90.0)   # swings back out
    _tick(controller, 10.5)
    controller._last_heading_error = math.radians(10.0)   # and back in
    _tick(controller, 11.0)

    # The dwell restarted with the dip, so 1 s of settling is not enough.
    assert controller._wall_side == -1
    _tick(controller, 13.1)
    assert controller._wall_side == 1
