"""Pure-logic tests for tsdf_mapper.py's map-rebuild correction math — no
rclpy, no ROS init, no VDBFusion.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
from collections import OrderedDict

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from frontier_slam.tsdf_mapper import (
    _compute_corrections, _interpolate_correction, _evict_fifo,
)


def _T(xyz=(0.0, 0.0, 0.0), yaw_deg=0.0) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('z', yaw_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


# ----------------------------------------------------------------------
# _compute_corrections
# ----------------------------------------------------------------------

def test_compute_corrections_identity_when_unmoved():
    T0 = _T((1.0, 2.0, 3.0), yaw_deg=30.0)
    A = _compute_corrections([T0], [T0])
    assert np.allclose(A[0], np.eye(4), atol=1e-9)


def test_compute_corrections_pure_translation():
    T_old = _T((0.0, 0.0, 0.0))
    T_new = _T((1.0, 0.0, 0.0))
    A = _compute_corrections([T_old], [T_new])
    assert np.allclose(A[0][:3, 3], [1.0, 0.0, 0.0])
    assert np.allclose(A[0][:3, :3], np.eye(3))
    # Applying the correction to the old pose must reproduce the new one.
    assert np.allclose(A[0] @ T_old, T_new)


# ----------------------------------------------------------------------
# _interpolate_correction
# ----------------------------------------------------------------------

def test_interpolate_translation_midpoint_lerp():
    keyframe_ts = np.array([0.0, 10.0])
    corrections = _compute_corrections(
        [_T((0.0, 0.0, 0.0)), _T((0.0, 0.0, 0.0))],
        [_T((0.0, 0.0, 0.0)), _T((2.0, 0.0, 0.0))])
    A_mid = _interpolate_correction(5.0, keyframe_ts, corrections)
    assert np.allclose(A_mid[:3, 3], [1.0, 0.0, 0.0])


def test_interpolate_yaw_slerp_midpoint_is_45_degrees():
    # A matrix-lerp (instead of quaternion/rotation Slerp) of a 0deg and a
    # 90deg rotation matrix at u=0.5 is NOT a valid rotation at all (it isn't
    # orthonormal) -- Slerp is required to get exactly 45deg here.
    keyframe_ts = np.array([0.0, 10.0])
    corrections = _compute_corrections(
        [_T(), _T()],
        [_T(yaw_deg=0.0), _T(yaw_deg=90.0)])
    A_mid = _interpolate_correction(5.0, keyframe_ts, corrections)
    yaw = Rotation.from_matrix(A_mid[:3, :3]).as_euler('xyz', degrees=True)[2]
    assert yaw == pytest.approx(45.0, abs=1e-6)
    # A genuine rotation matrix is orthonormal: R @ R.T == I.
    assert np.allclose(A_mid[:3, :3] @ A_mid[:3, :3].T, np.eye(3), atol=1e-9)


def test_interpolate_clamps_outside_keyframe_range():
    keyframe_ts = np.array([10.0, 20.0])
    corrections = _compute_corrections(
        [_T(), _T()],
        [_T((1.0, 0.0, 0.0)), _T((5.0, 0.0, 0.0))])
    before = _interpolate_correction(0.0, keyframe_ts, corrections)
    after = _interpolate_correction(100.0, keyframe_ts, corrections)
    assert np.allclose(before, corrections[0])
    assert np.allclose(after, corrections[-1])


def test_interpolate_single_keyframe_returns_its_correction():
    keyframe_ts = np.array([5.0])
    corrections = _compute_corrections([_T()], [_T((3.0, 0.0, 0.0))])
    A = _interpolate_correction(999.0, keyframe_ts, corrections)
    assert np.allclose(A, corrections[0])


# ----------------------------------------------------------------------
# Two-pass composition invariance
# ----------------------------------------------------------------------

def test_two_pass_correction_composes_with_single_pass():
    """Applying correction A1 (T0->T1) then, on a fresh baseline, A2 (T1->T2)
    to the same cached pose must land on exactly T2 -- the same place a
    single direct correction (T0->T2) would put it. This is the invariant
    that lets successive rebuilds compose on top of the write-back instead
    of needing to replay from the very first baseline every time."""
    T0 = _T((0.0, 0.0, 0.0), yaw_deg=0.0)
    T1 = _T((1.0, 0.0, 0.0), yaw_deg=20.0)
    T2 = _T((1.5, 0.5, 0.0), yaw_deg=50.0)

    A1 = _compute_corrections([T0], [T1])[0]
    cached_after_1 = A1 @ T0
    assert np.allclose(cached_after_1, T1)

    A2 = _compute_corrections([T1], [T2])[0]   # fresh baseline = T1, not T0
    cached_after_2 = A2 @ cached_after_1

    A_direct = _compute_corrections([T0], [T2])[0]
    direct = A_direct @ T0

    assert np.allclose(cached_after_2, T2)
    assert np.allclose(cached_after_2, direct)


# ----------------------------------------------------------------------
# _evict_fifo
# ----------------------------------------------------------------------

def test_evict_fifo_drops_oldest_first():
    cache = OrderedDict((i, f'scan{i}') for i in range(5))
    _evict_fifo(cache, max_size=3)
    assert list(cache.keys()) == [2, 3, 4]


def test_evict_fifo_noop_under_limit():
    cache = OrderedDict((i, f'scan{i}') for i in range(3))
    _evict_fifo(cache, max_size=10)
    assert list(cache.keys()) == [0, 1, 2]
