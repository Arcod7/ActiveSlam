"""Tests for the loop-closure compute budget: candidate thinning, keyframe
density saturation, failed-pair cooldown and bounded re-detection.

All of these exist because a station-keeping vehicle makes closure candidacy
degenerate to all-pairs — every co-located keyframe is inside
loop_closure_radius_m of every other. The behaviour under test is what keeps the
per-keyframe cost bounded while a genuine revisit still closes.
"""
import numpy as np
import pytest

from slam_backend.pose_graph import (
    Keyframe, PoseGraphNode, relative_angle, select_spread_candidates)


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T[:3, 3] = [x, y, z]
    return T


def keyframe(index, T_world):
    return Keyframe(index=index, stamp=None, T_odom=T_world.copy(),
                    T_world=T_world.copy(), cloud=np.zeros((1, 3)), symbol=index)


def bare_node(**attrs):
    """A PoseGraphNode with only the attributes under test, no ROS init."""
    node = object.__new__(PoseGraphNode)
    for key, value in attrs.items():
        setattr(node, key, value)
    return node


# ---------------------------------------------------------------- candidates

def test_candidate_count_is_capped():
    """Twenty in-radius keyframes must not mean twenty registrations."""
    candidates = [(i, float(i), np.array([float(i) * 3.0, 0.0, 0.0])) for i in range(20)]
    assert len(select_spread_candidates(candidates, 4, 1.0)) == 4


def test_nearest_candidate_is_kept_first():
    """Proximity is the best cheap predictor of a registration that will pass."""
    candidates = [(7, 4.0, np.array([4.0, 0.0, 0.0])),
                  (3, 1.0, np.array([1.0, 0.0, 0.0])),
                  (5, 2.5, np.array([2.5, 0.0, 0.0]))]
    assert [c[0] for c in select_spread_candidates(candidates, 3, 0.5)] == [3, 5, 7]


def test_clustered_candidates_collapse_to_one_representative():
    """The hover case: many candidates at one spot are one viewpoint, not many."""
    hover = [(i, 2.0 + i * 0.01, np.array([2.0, 0.0, 0.0])) for i in range(15)]
    distinct = (30, 4.0, np.array([4.0, 0.0, 0.0]))
    kept = select_spread_candidates(hover + [distinct], 4, 1.0)
    assert len(kept) == 2
    assert 30 in [c[0] for c in kept]


def test_spread_candidates_are_all_kept_when_below_the_cap():
    """Genuinely distinct revisit targets must survive thinning."""
    candidates = [(i, float(i) * 2.0, np.array([float(i) * 2.0, 0.0, 0.0]))
                  for i in range(1, 4)]
    assert len(select_spread_candidates(candidates, 4, 1.0)) == 3


def test_zero_cluster_radius_disables_spreading_only():
    """Radius 0 must still respect the cap, just not deduplicate."""
    candidates = [(i, 1.0, np.array([1.0, 0.0, 0.0])) for i in range(10)]
    assert len(select_spread_candidates(candidates, 3, 0.0)) == 3


@pytest.mark.parametrize('max_candidates', [0, -1])
def test_non_positive_cap_selects_nothing(max_candidates):
    candidates = [(1, 1.0, np.zeros(3))]
    assert select_spread_candidates(candidates, max_candidates, 1.0) == []


def test_empty_candidate_list_is_safe():
    assert select_spread_candidates([], 4, 1.0) == []


# ------------------------------------------------------------ density cap

def saturation_node(keyframes, max_per_cell=3):
    return bare_node(_keyframes=keyframes, keyframe_max_per_cell=max_per_cell,
                     keyframe_cell_radius_m=0.5, keyframe_cell_angle_rad=0.5)


def test_hover_saturates_after_the_cap():
    """Dead-reckoning drift keeps crossing the distance/angle gates; the cell cap
    is what stops that from growing the graph without adding coverage."""
    kfs = [keyframe(i, pose(0.0, 0.0)) for i in range(3)]
    assert saturation_node(kfs)._cell_is_saturated(pose(0.1, 0.1)) is True


def test_below_the_cap_is_not_saturated():
    kfs = [keyframe(i, pose(0.0, 0.0)) for i in range(2)]
    assert saturation_node(kfs)._cell_is_saturated(pose(0.1, 0.0)) is False


def test_moving_away_resumes_keyframe_creation():
    """The cap is positional, so leaving the saturated cell must free it."""
    kfs = [keyframe(i, pose(0.0, 0.0)) for i in range(5)]
    assert saturation_node(kfs)._cell_is_saturated(pose(3.0, 0.0)) is False


def test_a_new_heading_at_the_same_position_is_a_new_cell():
    """Rotating in place changes what the sonar sees, so it is not redundant."""
    kfs = [keyframe(i, pose(0.0, 0.0, yaw=0.0)) for i in range(5)]
    assert saturation_node(kfs)._cell_is_saturated(pose(0.0, 0.0, yaw=1.5)) is False


def test_zero_cap_disables_the_gate():
    """Escape hatch for A/B runs against the pre-change behaviour."""
    kfs = [keyframe(i, pose(0.0, 0.0)) for i in range(20)]
    assert saturation_node(kfs, max_per_cell=0)._cell_is_saturated(pose(0.0, 0.0)) is False


# --------------------------------------------------------- retry cooldown

def test_failed_pair_is_not_retried_in_place():
    """Rejections used to cost a full registration on every re-detection pass."""
    pair = frozenset((0, 12))
    node = bare_node(_failed_pairs={pair: (np.zeros(3), np.array([2.0, 0.0, 0.0]))},
                     loop_closure_retry_move_m=0.5)
    assert node._retry_suppressed(pair, np.zeros(3), np.array([2.0, 0.0, 0.0])) is True


def test_failed_pair_is_retried_once_an_endpoint_moves():
    """Optimization moves poses, so a stale rejection deserves one more attempt."""
    pair = frozenset((0, 12))
    node = bare_node(_failed_pairs={pair: (np.zeros(3), np.array([2.0, 0.0, 0.0]))},
                     loop_closure_retry_move_m=0.5)
    assert node._retry_suppressed(pair, np.array([1.0, 0.0, 0.0]),
                                  np.array([2.0, 0.0, 0.0])) is False


def test_unseen_pair_is_never_suppressed():
    node = bare_node(_failed_pairs={}, loop_closure_retry_move_m=0.5)
    assert node._retry_suppressed(frozenset((0, 12)), np.zeros(3), np.zeros(3)) is False


# ------------------------------------------------------- bounded redetect

class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)


class FakeClock:
    """Mirrors the get_clock().now().nanoseconds chain the node actually calls."""
    def __init__(self, seconds):
        self._time = FakeTime(seconds)

    def now(self):
        return self._time


def redetect_node(now_s=100.0, last=None, max_kf=5, interval=5.0):
    node = bare_node(redetect_max_keyframes=max_kf, redetect_min_interval_s=interval,
                     _last_redetect_time=last)
    node.get_clock = lambda: FakeClock(now_s)
    return node


def test_redetection_takes_the_most_displaced_keyframes():
    """One closure can shift the whole graph; the largest shifts are the ones
    most likely to expose a closure that was not visible before."""
    moved = [(idx, float(idx) * 0.1, 0.0) for idx in range(10)]
    assert redetect_node(max_kf=3)._select_redetect_targets(moved) == [9, 8, 7]


def test_redetection_is_rate_limited():
    """Back-to-back closures must not each trigger a full re-detection sweep."""
    moved = [(1, 0.5, 0.0)]
    assert redetect_node(now_s=100.0, last=98.0)._select_redetect_targets(moved) == []


def test_redetection_resumes_after_the_interval():
    moved = [(1, 0.5, 0.0)]
    assert redetect_node(now_s=100.0, last=90.0)._select_redetect_targets(moved) == [1]


def test_no_movement_means_no_redetection():
    assert redetect_node()._select_redetect_targets([]) == []


# ------------------------------------------------------------------ angles

def test_relative_angle_of_identity_is_zero():
    assert relative_angle(np.eye(3)) == pytest.approx(0.0)


def test_relative_angle_recovers_a_known_rotation():
    R = pose(yaw=0.7)[:3, :3]
    assert relative_angle(R) == pytest.approx(0.7)


def test_relative_angle_clamps_numerical_overshoot():
    """A trace slightly outside [-1, 1] after float error must not become NaN."""
    assert not np.isnan(relative_angle(np.eye(3) * (1.0 + 1e-12)))
