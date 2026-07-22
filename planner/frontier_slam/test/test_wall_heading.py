"""Pure geometry tests for wall-looking's planner-influenced heading rule."""
import math

import pytest

from frontier_slam.control_utils import wrap_angle
from frontier_slam.wall_follower import _blended_heading, _offset_heading_towards


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
