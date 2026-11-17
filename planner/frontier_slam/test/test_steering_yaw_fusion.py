"""Complementary fusion of the steering yaw: gyro carries, estimate corrects."""
import math

from frontier_slam.wall_oriented_controller import WallOrientedController
import pytest


def _fusing(yaw_est, yaw_ctrl, gyro_fresh=True, now=10.0):
    c = object.__new__(WallOrientedController)
    c._yaw = yaw_est
    c._yaw_ctrl = yaw_ctrl
    c._gyro_at = now if gyro_fresh else now - 5.0
    return c


def test_first_estimate_seeds_the_steering_yaw():
    c = _fusing(0.7, None)
    c._fuse_steering_yaw(10.0, None)
    assert c._yaw_ctrl == pytest.approx(0.7)


def test_estimate_jitter_is_heavily_attenuated():
    """A 3 deg estimate wobble moves the steering yaw ~6% per 0.1 s sample."""
    c = _fusing(math.radians(3.0), 0.0)
    c._fuse_steering_yaw(10.0, 0.1)
    alpha = 0.1 / (WallOrientedController.YAW_FUSE_TAU_S + 0.1)
    assert c._yaw_ctrl == pytest.approx(alpha * math.radians(3.0))
    assert c._yaw_ctrl < math.radians(0.2)


def test_the_estimate_wins_over_time():
    c = _fusing(1.0, 0.0)
    t = 10.0
    for _ in range(200):
        c._fuse_steering_yaw(t, 0.1)
        c._gyro_at = t                    # gyro stream stays live
        t += 0.1
    assert c._yaw_ctrl == pytest.approx(1.0, abs=1e-3)


def test_a_graph_correction_is_followed_at_once():
    c = _fusing(1.0, 0.0)                 # 57 deg innovation > snap threshold
    c._fuse_steering_yaw(10.0, 0.1)
    assert c._yaw_ctrl == pytest.approx(1.0)


def test_a_stale_gyro_reverts_to_the_raw_estimate():
    """No gyro means nothing carries the yaw between corrections."""
    c = _fusing(0.4, 0.0, gyro_fresh=False)
    c._fuse_steering_yaw(10.0, 0.1)
    assert c._yaw_ctrl == pytest.approx(0.4)
