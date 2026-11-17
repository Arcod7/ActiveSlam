"""Corner-cutting on the A* path, and the obstacle check that bounds it."""
import math
from types import SimpleNamespace

from frontier_slam.path_planner import smooth_path
import numpy as np

RES, OX, OY = 0.25, -25.0, -25.0


def _grid(blocked_world=()):
    grid = np.zeros((400, 400), bool)
    for wx, wy in blocked_world:
        grid[int((wy - OY) / RES), int((wx - OX) / RES)] = True
    return SimpleNamespace(hard_blocked=grid, res=RES, ox=OX, oy=OY)


def _turns_deg(points):
    out = []
    for a, b, c in zip(points, points[1:], points[2:]):
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        out.append(abs(math.degrees((h2 - h1 + math.pi) % (2 * math.pi) - math.pi)))
    return out


CORNER = [(0.0, 0.0), (0.0, 2.0), (0.0, 4.0), (2.0, 4.0), (4.0, 4.0)]


def test_a_right_angle_becomes_a_curve():
    smoothed = smooth_path(CORNER, _grid())
    assert max(_turns_deg(CORNER)) == 90.0
    assert max(_turns_deg(smoothed)) < 30.0


def test_endpoints_survive_smoothing():
    """The first point is the robot and the last is the goal."""
    smoothed = smooth_path(CORNER, _grid())
    assert smoothed[0] == CORNER[0]
    assert smoothed[-1] == CORNER[-1]


def test_a_cut_that_would_clip_an_obstacle_is_refused():
    """Corners are cut toward the inside of the turn — where obstacles are.

    The first pass shortcuts (0, 3.5) -> (0.5, 4.0); blocking the diagonal it
    crosses must leave the original corner standing.
    """
    on_the_cut = _grid([(0.25, 3.75)])
    assert smooth_path(CORNER, on_the_cut) == CORNER


def test_smoothing_never_returns_a_blocked_segment():
    blocked = _grid([(0.4, 3.6), (1.0, 4.4)])
    for path in (CORNER, list(reversed(CORNER))):
        for point in smooth_path(path, blocked):
            r = int((point[1] - OY) / RES)
            c = int((point[0] - OX) / RES)
            assert not blocked.hard_blocked[r, c]


def test_a_straight_path_stays_straight():
    """Subdivision adds points on a straight run; it must not bend it."""
    straight = [(0.0, 0.0), (0.0, 2.0), (0.0, 4.0)]
    smoothed = smooth_path(straight, _grid())
    assert len(smoothed) > len(straight)
    assert all(turn == 0.0 for turn in _turns_deg(smoothed))
    assert all(x == 0.0 for x, _ in smoothed)


def test_two_point_paths_are_returned_unchanged():
    assert smooth_path([(0.0, 0.0), (1.0, 1.0)], _grid()) == [(0.0, 0.0), (1.0, 1.0)]
    assert smooth_path([], _grid()) == []
