"""Pure geometry tests for wall-oriented path execution."""
import math
from types import SimpleNamespace

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


def _hold(route_heading, wall_side, yaw, offset_deg=45.0):
    controller = _bare_controller(
        _last_route_heading=route_heading,
        _look_offset_deg=offset_deg,
        _wall_side=wall_side,
        _yaw=yaw,
    )
    controller._select_wall_side = lambda *_: None   # side is fixed by the caller
    return controller._hold_yaw_cmd(0.0)


def test_waiting_on_the_viewing_heading_commands_no_yaw():
    yaw_cmd, _, look_heading, error = _hold(0.0, 1, math.radians(45.0))
    assert yaw_cmd == pytest.approx(0.0)
    assert look_heading == pytest.approx(math.radians(45.0))
    assert error == pytest.approx(0.0)


def test_waiting_off_the_viewing_heading_turns_back_towards_it():
    yaw_cmd, _, _, error = _hold(0.0, 1, 0.0)
    assert error == pytest.approx(math.radians(45.0))
    assert 0.0 < yaw_cmd <= WallOrientedController.YAW_EFFORT_LIMIT


def test_waiting_yaw_effort_is_bounded():
    yaw_cmd, _, _, _ = _hold(0.0, -1, math.radians(179.0))
    assert abs(yaw_cmd) <= WallOrientedController.YAW_EFFORT_LIMIT


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
