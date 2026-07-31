"""Pure geometry tests for wall-looking's planner-influenced heading rule."""
import math

import numpy as np
import pytest

from frontier_slam.control_utils import wrap_angle
from frontier_slam.wall_looking import (
    _blended_heading, _offset_heading_towards, _travel_direction)


def test_zero_path_influence_keeps_wall_heading():
    assert _blended_heading(math.pi / 2.0, 0.0, 0.0) == pytest.approx(math.pi / 2.0)


def test_full_path_influence_faces_along_route():
    assert _blended_heading(math.pi / 2.0, 0.0, 1.0) == pytest.approx(0.0)


def test_half_path_influence_is_halfway_between_wall_and_route():
    assert _blended_heading(math.pi / 2.0, 0.0, 0.5) == pytest.approx(math.pi / 4.0)


def test_blend_uses_the_shortest_angular_arc():
    heading = _blended_heading(math.radians(170.0), math.radians(-170.0), 0.5)
    assert abs(wrap_angle(heading)) == pytest.approx(math.pi)


def test_path_offset_turns_toward_wall_on_the_left():
    heading = _offset_heading_towards(0.0, math.pi / 2.0, 30.0)
    assert heading == pytest.approx(math.radians(30.0))


def test_path_offset_turns_toward_wall_on_the_right():
    heading = _offset_heading_towards(0.0, -math.pi / 2.0, 30.0)
    assert heading == pytest.approx(math.radians(-30.0))


def test_offset_stops_at_target_when_requested_offset_is_larger():
    heading = _offset_heading_towards(0.0, math.radians(20.0), 45.0)
    assert heading == pytest.approx(math.radians(20.0))


def test_candidate_offsets_are_applied_before_heading_blend():
    route_heading = 0.0
    wall_heading = math.pi / 2.0
    path_look = _offset_heading_towards(route_heading, wall_heading, 30.0)
    wall_look = _offset_heading_towards(wall_heading, route_heading, 0.0)

    assert _blended_heading(wall_look, path_look, 0.5) == pytest.approx(
        math.radians(60.0))


def test_yaw_only_travel_ignores_the_wall_and_follows_the_route():
    # Wall normal along +Y, route due -Y: the tangent is perpendicular to the
    # route, so the blended rule has no route component to work with.
    n = np.array([0.0, 1.0])
    travel = _travel_direction(n, np.array([0.0, -2.0]), 1, True, 0.1)
    assert travel == pytest.approx(np.array([0.0, -1.0]))


def test_blended_travel_chatters_when_the_route_is_perpendicular():
    """The stall mode yaw_only exists to avoid: a sign that flips on noise."""
    n = np.array([0.0, 1.0])
    left = _travel_direction(n, np.array([-1e-3, -2.0]), 1, False, 0.1)
    right = _travel_direction(n, np.array([+1e-3, -2.0]), 1, False, 0.1)
    assert left[0] * right[0] < 0.0            # travel reverses on noise
    assert abs(left[1]) < 0.2                  # and barely advances the route


def test_blended_travel_still_follows_the_wall_tangent_when_enabled():
    n = np.array([0.0, 1.0])
    travel = _travel_direction(n, np.array([2.0, 0.0]), 1, False, 0.1)
    assert travel == pytest.approx(np.array([1.0, 0.0]))
