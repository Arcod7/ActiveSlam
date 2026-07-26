"""Pure-logic tests for trajectory_mission.py — no rclpy, no ROS init.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np
import pytest

from frontier_slam.trajectory_mission import (
    MissionConfig, MissionState, TrajectoryMissionStateMachine,
    parse_mission_waypoints,
)


# ----------------------------------------------------------------------
# parse_mission_waypoints
# ----------------------------------------------------------------------

def test_parse_single_point():
    wp = parse_mission_waypoints([1.0, 2.0, -3.0])
    assert wp.shape == (1, 3)
    np.testing.assert_allclose(wp[0], [1.0, 2.0, -3.0])


def test_parse_rectangle():
    flat = [0.0, 0.0, -1.0, 10.0, 0.0, -1.0, 10.0, 10.0, -1.0, 0.0, 10.0, -1.0]
    wp = parse_mission_waypoints(flat)
    assert wp.shape == (4, 3)


def test_parse_empty_raises():
    with pytest.raises(ValueError):
        parse_mission_waypoints([])


def test_parse_malformed_length_raises():
    with pytest.raises(ValueError):
        parse_mission_waypoints([1.0, 2.0, 3.0, 4.0])   # length 4, not a multiple of 3


# ----------------------------------------------------------------------
# TrajectoryMissionStateMachine
# ----------------------------------------------------------------------

def _cfg(**overrides):
    base = dict(arrival_radius_m=2.0, release_dwell_s=5.0)
    base.update(overrides)
    return MissionConfig(**base)


def _rect():
    return np.array([
        [0.0, 0.0, -1.0], [10.0, 0.0, -1.0], [10.0, 10.0, -1.0], [0.0, 10.0, -1.0],
    ])


def test_on_odom_transitions_from_wait_to_following():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    assert sm.state == MissionState.WAIT_ODOM
    event = sm.on_odom()
    assert event == 'STARTED'
    assert sm.state == MissionState.FOLLOWING
    assert sm.index == 0


def test_tick_before_odom_is_noop():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    event = sm.tick(now=0.0, robot_xyz=np.array([0.0, 0.0, -1.0]), revisit_state=None)
    assert event is None
    assert sm.state == MissionState.WAIT_ODOM


def test_arrival_advances_to_next_waypoint():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.on_odom()
    event = sm.tick(now=1.0, robot_xyz=np.array([0.1, 0.0, -1.0]), revisit_state='exploring')
    assert event == 'WAYPOINT_REACHED'
    assert sm.index == 1
    assert sm.state == MissionState.FOLLOWING


def test_no_advance_outside_arrival_radius():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.on_odom()
    event = sm.tick(now=1.0, robot_xyz=np.array([5.0, 0.0, -1.0]), revisit_state='exploring')
    assert event is None
    assert sm.index == 0


def test_arrival_at_last_waypoint_without_loop_enters_done():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.on_odom()
    sm.index = 3   # jump to last waypoint
    event = sm.tick(now=1.0, robot_xyz=np.array([0.1, 10.0, -1.0]), revisit_state='exploring')
    assert event == 'MISSION_DONE'
    assert sm.state == MissionState.DONE
    assert sm.suspended is True


def test_arrival_at_last_waypoint_with_loop_wraps_to_start():
    sm = TrajectoryMissionStateMachine(_rect(), loop=True, cfg=_cfg())
    sm.on_odom()
    sm.index = 3
    event = sm.tick(now=1.0, robot_xyz=np.array([0.1, 10.0, -1.0]), revisit_state='exploring')
    assert event == 'WAYPOINT_REACHED_LOOP'
    assert sm.index == 0
    assert sm.state == MissionState.FOLLOWING


def test_single_point_mission_reaches_done():
    wp = np.array([[5.0, 5.0, -2.0]])
    sm = TrajectoryMissionStateMachine(wp, loop=False, cfg=_cfg())
    sm.on_odom()
    event = sm.tick(now=1.0, robot_xyz=np.array([5.1, 5.0, -2.0]), revisit_state='exploring')
    assert event == 'MISSION_DONE'
    assert sm.state == MissionState.DONE


def test_done_releases_after_dwell():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg(release_dwell_s=5.0))
    sm.on_odom()
    sm.index = 3
    sm.tick(now=0.0, robot_xyz=np.array([0.1, 10.0, -1.0]), revisit_state='exploring')
    assert sm.state == MissionState.DONE
    event = sm.tick(now=3.0, robot_xyz=np.array([0.1, 10.0, -1.0]), revisit_state='exploring')
    assert event is None
    assert sm.state == MissionState.DONE
    event = sm.tick(now=5.1, robot_xyz=np.array([0.1, 10.0, -1.0]), revisit_state='exploring')
    assert event == 'RELEASED'
    assert sm.state == MissionState.RELEASED
    assert sm.suspended is False


# ----------------------------------------------------------------------
# Revisit-planner arbitration (yield / resume)
# ----------------------------------------------------------------------

def test_yield_on_non_exploring_revisit_state():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.on_odom()
    event = sm.tick(now=1.0, robot_xyz=np.array([5.0, 0.0, -1.0]), revisit_state='revisiting')
    assert event == 'YIELDED'
    assert sm.state == MissionState.YIELDED
    assert sm.suspended is True
    assert sm.holding_goal is False


def test_yielded_ignores_arrival_and_holds_index():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.on_odom()
    sm.tick(now=1.0, robot_xyz=np.array([5.0, 0.0, -1.0]), revisit_state='revisiting')
    assert sm.state == MissionState.YIELDED
    # even sitting right on top of the held waypoint, a yielded mission must
    # not advance or publish a goal
    event = sm.tick(now=2.0, robot_xyz=np.array([0.0, 0.0, -1.0]), revisit_state='revisiting')
    assert event is None
    assert sm.index == 0
    assert sm.state == MissionState.YIELDED


def test_resume_preserves_index_after_yield_round_trip():
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.on_odom()
    sm.tick(now=1.0, robot_xyz=np.array([0.1, 0.0, -1.0]), revisit_state='exploring')
    assert sm.index == 1
    sm.tick(now=2.0, robot_xyz=np.array([5.0, 0.0, -1.0]), revisit_state='revisiting')
    assert sm.state == MissionState.YIELDED
    event = sm.tick(now=3.0, robot_xyz=np.array([5.0, 0.0, -1.0]), revisit_state='cooldown')
    assert event is None
    assert sm.state == MissionState.YIELDED   # cooldown is not the idle value either
    event = sm.tick(now=4.0, robot_xyz=np.array([5.0, 0.0, -1.0]), revisit_state='exploring')
    assert event == 'RESUMED'
    assert sm.state == MissionState.FOLLOWING
    assert sm.index == 1   # held across the whole round trip


@pytest.mark.parametrize("state,expected_suspended,expected_holding_goal", [
    (MissionState.WAIT_ODOM, False, False),
    (MissionState.FOLLOWING, True, True),
    (MissionState.YIELDED, True, False),
    (MissionState.DONE, True, True),
    (MissionState.RELEASED, False, False),
])
def test_flags_per_state(state, expected_suspended, expected_holding_goal):
    sm = TrajectoryMissionStateMachine(_rect(), loop=False, cfg=_cfg())
    sm.state = state
    assert sm.suspended is expected_suspended
    assert sm.holding_goal is expected_holding_goal
