"""Pure geometry tests for wall-oriented path execution."""
import math

from frontier_slam.wall_oriented_controller import (
    choose_wall_side,
    lookahead_path_heading,
    offset_heading,
    sweep_yaw_effort,
    wall_side_distances,
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


AMPLITUDE = math.radians(10.0)


def test_sweep_holds_its_direction_inside_the_window():
    effort, direction = sweep_yaw_effort(0.0, 1, AMPLITUDE, 0.03, 0.08)
    assert (effort, direction) == (0.03, 1)
    effort, direction = sweep_yaw_effort(0.0, -1, AMPLITUDE, 0.03, 0.08)
    assert (effort, direction) == (-0.03, -1)


def test_sweep_reverses_at_each_edge_of_the_window():
    # heading_error = look - yaw, so a yaw above the viewing heading is negative.
    effort, direction = sweep_yaw_effort(-AMPLITUDE, 1, AMPLITUDE, 0.03, 0.08)
    assert (effort, direction) == (-0.03, -1)
    effort, direction = sweep_yaw_effort(AMPLITUDE, -1, AMPLITUDE, 0.03, 0.08)
    assert (effort, direction) == (0.03, 1)


def test_sweep_never_commands_the_faster_effort_inside_the_window():
    for error in np.linspace(-2.0 * AMPLITUDE, 2.0 * AMPLITUDE, 41):
        effort, _ = sweep_yaw_effort(float(error), 1, AMPLITUDE, 0.03, 0.08)
        assert abs(effort) == pytest.approx(0.03)


def test_a_large_error_is_closed_toward_the_viewing_heading_at_acquire_effort():
    effort, direction = sweep_yaw_effort(1.0, -1, AMPLITUDE, 0.03, 0.08)
    assert (effort, direction) == (0.08, 1)
    effort, direction = sweep_yaw_effort(-1.0, 1, AMPLITUDE, 0.03, 0.08)
    assert (effort, direction) == (-0.08, -1)
