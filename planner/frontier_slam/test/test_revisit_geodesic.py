"""Geodesic revisit-target scoring.

The case that matters is the one measured on a007 `lc_revisit_s101 ep2`: the
candidate nearest in a straight line sat the far side of the hull, so its true
route was 3.5x its Euclidean score and it was never reached. A selector scoring
straight-line distance cannot see that; one scoring the cost field can.
"""
import numpy as np
import pytest

from frontier_slam.path_planner import CostGrid, PAD_CELLS
from frontier_slam.revisit_geodesic import (
    select_revisit_target_geodesic, travel_cost_field)

RES = 0.5


def make_grid(blocked_cells=()):
    """20x20 m of free space at 0.5 m, origin at (0,0), with an optional wall."""
    n = 40
    cost = np.ones((n, n), dtype=float)
    hard = np.zeros((n, n), dtype=bool)
    for r, c in blocked_cells:
        cost[r, c] = np.inf
        hard[r, c] = True
    return CostGrid(cost_grid=cost, ox=0.0, oy=0.0, res=RES, hard_blocked=hard,
                    soft_zone=np.zeros((n, n), bool),
                    plan_zone=np.zeros((n, n), bool),
                    raw=np.zeros((n, n), np.int8))


def test_cost_field_matches_straight_line_in_open_water():
    cg = make_grid()
    field = travel_cost_field(cg, np.array([1.0, 1.0]))
    assert field is not None
    # 10 m due east of the robot, no obstacles: cost should be ~10 m
    r, c = int(1.0 / RES), int(11.0 / RES)
    assert field[r, c] == pytest.approx(10.0, abs=0.6)


def test_wall_makes_a_near_candidate_expensive():
    """A wall between robot and target: geodesic cost >> Euclidean distance."""
    wall = [(r, 20) for r in range(0, 38)]     # vertical wall, gap at the bottom
    cg = make_grid(wall)
    field = travel_cost_field(cg, np.array([5.0, 5.0]))
    r, c = int(5.0 / RES), int(15.0 / RES)      # 10 m east, but behind the wall
    assert np.isfinite(field[r, c])
    assert field[r, c] > 20.0, 'route round the wall must cost far more than 10 m'


def test_picks_the_reachable_candidate_over_the_nearer_walled_one():
    """The measured failure, reproduced: nearest by Euclid, farthest to swim."""
    wall = [(r, 20) for r in range(0, 38)]
    cg = make_grid(wall)
    robot = np.array([5.0, 5.0])
    # index 0: 10 m east but the far side of the wall (nearest by Euclid)
    # index 1: 14 m north, open water
    kf = np.array([[15.0, 5.0, 0.0], [5.0, 19.0, 0.0]] + [[5.0, 5.0, 0.0]] * 30)
    pick, info = select_revisit_target_geodesic(
        kf, robot, cg, min_index_gap=0, candidate_radius_m=5.0,
        min_target_dist_m=3.0, w_density=0.0, w_travel=1.0)
    assert not info['fallback']
    assert pick == 1, 'must reject the walled-off candidate despite it being nearer'


def test_falls_back_when_no_grid():
    kf = np.array([[10.0, 0.0, 0.0]] * 40)
    pick, info = select_revisit_target_geodesic(
        kf, np.array([0.0, 0.0]), None, min_index_gap=0, candidate_radius_m=5.0,
        min_target_dist_m=3.0, w_density=1.0, w_travel=1.0)
    assert pick is None and info['fallback'] is True


def test_max_travel_excludes_unreachable_in_budget():
    cg = make_grid()
    kf = np.array([[18.0, 18.0, 0.0], [8.0, 1.0, 0.0]] + [[1.0, 1.0, 0.0]] * 30)
    pick, _ = select_revisit_target_geodesic(
        kf, np.array([1.0, 1.0]), cg, min_index_gap=0, candidate_radius_m=5.0,
        min_target_dist_m=3.0, w_density=0.0, w_travel=0.0, max_travel_m=12.0)
    assert pick == 1, 'the 24 m-away candidate exceeds the 12 m budget'


def test_age_breaks_ties_between_similar_range():
    cg = make_grid()
    # two candidates at the same distance; index 0 is much older than index 20
    kf = np.zeros((40, 3))
    kf[:] = [1.0, 1.0, 0.0]
    kf[0] = [11.0, 1.0, 0.0]
    kf[20] = [1.0, 11.0, 0.0]
    pick, _ = select_revisit_target_geodesic(
        kf, np.array([1.0, 1.0]), cg, min_index_gap=0, candidate_radius_m=1.0,
        min_target_dist_m=3.0, w_density=0.0, w_travel=1.0, w_age=0.5)
    assert pick == 0, 'at equal range the older keyframe should win'
