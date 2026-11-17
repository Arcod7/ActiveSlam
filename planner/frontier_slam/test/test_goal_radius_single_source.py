"""The arrival radius is one number, and every consumer reads that one number.

Split thresholds deadlocked the stack once already: the executor parked at 4 m
while the planner would only accept 2 m, so no goal was ever reached by
arriving — 0 arrivals against 3 stuck-blacklists in a 510 s run — and the
vehicle only moved on when the 30 s stuck timer fired.
"""
from dataclasses import dataclass

from frontier_slam.goal_manager import GoalManager
from frontier_slam.mission_params import GOAL_RADIUS_M
import numpy as np
import pytest


@dataclass
class _Cluster:
    wx: float
    wy: float
    size: int = 40
    distance: float = 10.0


def _cluster(wx, wy, robot_xy, size=40):
    return _Cluster(wx=wx, wy=wy, size=size,
                    distance=float(np.hypot(wx - robot_xy[0],
                                            wy - robot_xy[1])))


def _manager(**kwargs):
    return GoalManager(min_explore_dist=1.0, arrival_blacklist_duration=60.0,
                       **kwargs)


def test_the_goal_manager_defaults_to_the_shared_radius():
    assert _manager().goal_radius == pytest.approx(GOAL_RADIUS_M)


def test_arrival_fires_while_the_cluster_is_still_visible():
    """The deadlock: the cluster keeps being re-detected, because the frontier
    does not stop existing just because the vehicle is parked at it."""
    manager = _manager(goal_radius=4.0)
    robot = np.array([0.0, 0.0])
    far = _cluster(30.0, 0.0, robot)
    assert manager.select([far], robot, 0.0).gx == pytest.approx(30.0)

    # Parked just inside the radius, with the cluster still in the map.
    robot = np.array([26.5, 0.0])
    still_there = _cluster(30.0, 0.0, robot)
    other = _cluster(-20.0, 0.0, robot)
    result = manager.select([still_there, other], robot, 5.0)

    assert result.gx == pytest.approx(-20.0), "should have moved on"
    assert manager.is_blacklisted(30.0, 0.0)


def test_a_goal_still_out_of_reach_is_not_abandoned():
    manager = _manager(goal_radius=4.0)
    robot = np.array([0.0, 0.0])
    target = _cluster(30.0, 0.0, robot)
    manager.select([target], robot, 0.0)

    robot = np.array([20.0, 0.0])          # 10 m short of the radius
    result = manager.select([_cluster(30.0, 0.0, robot),
                             _cluster(-20.0, 0.0, robot)], robot, 5.0)
    assert result.gx == pytest.approx(30.0)
    assert not manager.is_blacklisted(30.0, 0.0)


def test_a_wider_radius_retires_the_goal_sooner():
    """The parameter has to actually move the arrival boundary."""
    robot_far = np.array([0.0, 0.0])
    outcomes = {}
    for radius in (4.0, 12.0):
        manager = _manager(goal_radius=radius)
        manager.select([_cluster(30.0, 0.0, robot_far)], robot_far, 0.0)
        robot = np.array([20.0, 0.0])      # 10 m from the goal
        outcomes[radius] = manager.select(
            [_cluster(30.0, 0.0, robot), _cluster(-20.0, 0.0, robot)],
            robot, 5.0).gx
    assert outcomes[4.0] == pytest.approx(30.0)    # not yet arrived
    assert outcomes[12.0] == pytest.approx(-20.0)  # arrived, moved on
