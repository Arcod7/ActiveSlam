"""Pure tests for rate-damped depth hold."""
import pytest

from frontier_slam.control_utils import depth_hold_effort, LowPassRate


def test_pure_proportional_is_unchanged_when_stationary():
    """With no vertical motion the law reduces to the original P controller."""
    assert depth_hold_effort(0.5, 0.0, kp=0.35, kd=0.5) == pytest.approx(-0.175)


def test_descending_toward_setpoint_reduces_the_command():
    """Closing on the setpoint from above must back the thrust off early --
    this is the term whose absence sustained the limit cycle."""
    undamped = depth_hold_effort(-0.5, 0.0, kp=0.35, kd=0.5)
    damped = depth_hold_effort(-0.5, 0.4, kp=0.35, kd=0.5)
    assert damped < undamped


def test_effort_is_bounded():
    assert depth_hold_effort(-99.0, -99.0, kp=0.35, kd=0.5) == 1.0
    assert depth_hold_effort(99.0, 99.0, kp=0.35, kd=0.5) == -1.0


@pytest.mark.parametrize('kp, kd, limit', [(-0.1, 0.5, 1.0), (0.35, -0.1, 1.0),
                                           (0.35, 0.5, 0.0), (0.35, 0.5, 1.5)])
def test_invalid_gains_rejected(kp, kd, limit):
    with pytest.raises(ValueError):
        depth_hold_effort(0.0, 0.0, kp, kd, limit)


def test_rate_is_zero_until_a_second_sample():
    r = LowPassRate(0.2)
    assert r.update(5.0, 0.0) == 0.0


def test_rate_converges_on_a_constant_slope():
    """A steady 0.5 m/s descent must be tracked, not merely detected."""
    r = LowPassRate(0.2)
    for i in range(200):
        r.update(0.5 * i * 0.01, i * 0.01)
    assert r.value == pytest.approx(0.5, abs=1e-3)


def test_rate_ignores_non_advancing_time():
    """Duplicate stamps must not divide by zero."""
    r = LowPassRate(0.2)
    r.update(1.0, 1.0)
    assert r.update(2.0, 1.0) == 0.0


def test_reset_clears_history():
    r = LowPassRate(0.2)
    r.update(0.0, 0.0)
    r.update(1.0, 1.0)
    r.reset()
    assert r.value == 0.0 and r.update(9.0, 2.0) == 0.0
