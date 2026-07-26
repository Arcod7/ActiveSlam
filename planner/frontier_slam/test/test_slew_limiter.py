"""Tests for the yaw command slew limiter."""

import pytest

from frontier_slam.control_utils import SlewLimiter


def test_a_command_step_is_spread_over_the_ramp_time():
    slew = SlewLimiter(0.30)
    slew.update(0.0, 0.0)

    # 0.1 s of a 0.30/s ramp reaches 0.03, not the 0.30 asked for.
    assert slew.update(0.30, 0.1) == pytest.approx(0.03)
    assert slew.update(0.30, 0.2) == pytest.approx(0.06)


def test_full_scale_reversal_takes_two_ramp_times():
    slew = SlewLimiter(0.30)
    slew.update(0.30, 0.0)
    for tick in range(1, 11):
        slew.update(0.30, tick * 0.1)
    assert slew.value == pytest.approx(0.30)

    # +0.30 -> -0.30 is 0.60 of travel: 2 s, not one 0.1 s tick.
    for tick in range(11, 21):
        slew.update(-0.30, tick * 0.1)
    assert slew.value == pytest.approx(0.0, abs=1e-9)
    for tick in range(21, 31):
        slew.update(-0.30, tick * 0.1)
    assert slew.value == pytest.approx(-0.30)


def test_a_small_command_passes_through_within_one_tick():
    slew = SlewLimiter(0.30)
    slew.update(0.0, 0.0)

    assert slew.update(0.02, 0.1) == pytest.approx(0.02)


def test_a_stalled_caller_does_not_bank_a_large_step():
    # A 30 s gap must not authorise a jump: the vehicle did not ramp during it.
    slew = SlewLimiter(0.30, max_gap_s=0.5)
    slew.update(0.0, 0.0)

    assert slew.update(1.0, 30.0) == pytest.approx(0.15)


def test_time_going_backwards_holds_the_last_value():
    slew = SlewLimiter(0.30)
    slew.update(0.0, 10.0)
    slew.update(0.30, 10.5)
    held = slew.value

    assert slew.update(0.30, 9.0) == pytest.approx(held)


def test_reset_clears_the_ramp_state():
    slew = SlewLimiter(0.30)
    slew.update(0.0, 0.0)
    slew.update(0.30, 1.0)
    slew.reset()

    assert slew.value == 0.0
    assert slew.update(0.30, 2.0) == 0.0   # first sample after reset has no dt


def test_a_non_positive_rate_is_rejected():
    with pytest.raises(ValueError):
        SlewLimiter(0.0)
