"""Pure tests for rate-damped depth hold."""
import math

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


# --- rate feedback off a pose estimate -----------------------------------
# The sampled channel is /slam/odometry, which steps when the graph is
# optimised and stalls while the optimiser runs. Neither is vehicle motion.

def test_a_publisher_stall_reports_no_rate():
    """alpha = dt/(tau+dt) tends to 1 as the gap grows, so an unguarded filter
    passes the whole accumulated quotient exactly when it is least trustworthy."""
    r = LowPassRate(0.15, max_gap_s=0.5)
    r.update(0.0, 0.0)
    assert r.update(3.0, 2.0) == 0.0


def test_a_graph_correction_is_not_read_as_rotation():
    """The measured fault: a fed-back 9.44 rad/s where the hull never exceeded
    1.26 held yaw_cmd at its limit and flipped its sign."""
    r = LowPassRate(0.15, wrap=True, max_rate=1.5)
    for i in range(40):
        r.update(0.5 * i * 0.1, i * 0.1)     # steady 0.5 rad/s
    settled = r.value
    r.update(0.5 * 40 * 0.1 + 0.94, 4.0)     # 0.94 rad step in one 0.1 s tick
    assert r.value == pytest.approx(settled)


def test_the_sample_after_a_correction_measures_from_the_new_pose():
    """Rejecting the step must not also reject the motion that follows it."""
    r = LowPassRate(0.0, max_rate=1.5)
    r.update(0.0, 0.0)
    r.update(9.0, 1.0)              # rejected: 9 m/s
    assert r.update(9.5, 2.0) == pytest.approx(0.5)


def test_motion_inside_the_bound_still_passes():
    r = LowPassRate(0.0, max_rate=1.5)
    r.update(0.0, 0.0)
    assert r.update(1.2, 1.0) == pytest.approx(1.2)


def test_guards_are_off_by_default():
    """Existing callers must be unaffected."""
    r = LowPassRate(0.0)
    r.update(0.0, 0.0)
    assert r.update(50.0, 1.0) == pytest.approx(50.0)


# --- yaw authority -------------------------------------------------------

from frontier_slam.control_utils import (
    allocate_horizontal, mix_thrusters, yaw_rate_command)


def test_yaw_survives_a_saturating_surge():
    """The bug: at speed_factor 4 surge reached ~1.0 and whole-group
    normalisation scaled yaw down exactly when heading error was largest."""
    surge, sway, yaw = allocate_horizontal(1.0, 0.0, 0.5)
    assert yaw == pytest.approx(0.5)
    assert surge < 1.0


def test_translation_is_untouched_when_it_already_fits():
    assert allocate_horizontal(0.3, 0.1, 0.2) == pytest.approx((0.3, 0.1, 0.2))


def test_pure_rotation_keeps_full_authority():
    """Nothing to starve when translation is zero -- and scan/escape rely on
    this, with the rate loop bounding how fast it actually turns."""
    assert allocate_horizontal(0.0, 0.0, 1.0) == pytest.approx((0.0, 0.0, 1.0))


def test_yaw_cannot_claim_the_whole_budget():
    """Letting yaw take everything zeroed translation, tripped the stuck
    detector, and the escape manoeuvre span the vehicle in place."""
    surge, sway, yaw = allocate_horizontal(1.0, 1.0, 1.0)
    assert yaw == pytest.approx(0.6)
    assert surge > 0.0 and sway > 0.0


def test_translation_always_keeps_some_authority():
    for yaw in (0.5, 0.9, 1.0, -1.0):
        surge, sway, _ = allocate_horizontal(1.0, 0.0, yaw)
        assert surge > 0.0


def test_allocated_commands_never_saturate_the_horizontal_mixer():
    """If allocation is right, no horizontal thruster is clipped, so the
    normaliser never rescales and yaw is delivered as commanded."""
    for surge, sway, yaw in [(1.0, 0.0, 0.5), (0.8, 0.8, 0.3), (1.0, 1.0, 0.9)]:
        s, w, y = allocate_horizontal(surge, sway, yaw)
        for v in (s - w - y, s + w + y, -s - w + y, -s + w - y):
            assert abs(v) <= 1.0 + 1e-9


def test_mixer_still_returns_eight_setpoints():
    assert len(mix_thrusters(1.0, 0.5, 0.2, sway=0.3)) == 8


def test_wrapped_rate_does_not_spike_across_pi():
    """A naive difference reads ~-2pi when yaw crosses +pi, which would slam
    the damping term hard over."""
    r = LowPassRate(0.0, wrap=True)
    r.update(math.pi - 0.05, 0.0)
    assert r.update(-math.pi + 0.05, 1.0) == pytest.approx(0.1, abs=1e-6)


# --- yaw rate limiting ---------------------------------------------------

RATE_GAINS = dict(kp_heading=0.7, kp_rate=1.2, max_rate=0.5)


def test_large_heading_error_is_capped_at_the_rate_limit():
    """The 400 deg/s spin: a big heading error must not ask for unbounded rate.
    At the cap and already turning at it, no further command is needed."""
    assert yaw_rate_command(math.pi, 0.5, **RATE_GAINS) == pytest.approx(0.0)


def test_command_opposes_overspeed_rotation():
    """Turning faster than requested must brake, not keep pushing."""
    assert yaw_rate_command(0.0, 1.5, **RATE_GAINS) < 0.0


def test_small_error_still_produces_a_correction():
    """The old failure was under-turning: modest error must still command."""
    assert yaw_rate_command(0.3, 0.0, **RATE_GAINS) > 0.2


def test_rate_cap_is_independent_of_heading_error_size():
    """Doubling an already-saturating error changes nothing -- that is what
    makes the limit hold regardless of thrust ceiling."""
    a = yaw_rate_command(1.5, 0.0, **RATE_GAINS)
    b = yaw_rate_command(3.0, 0.0, **RATE_GAINS)
    assert a == pytest.approx(b)


def test_output_is_bounded():
    assert abs(yaw_rate_command(math.pi, -9.0, **RATE_GAINS)) <= 1.0


@pytest.mark.parametrize('bad', [dict(kp_heading=-1.0), dict(kp_rate=-1.0),
                                 dict(max_rate=0.0)])
def test_invalid_yaw_gains_rejected(bad):
    with pytest.raises(ValueError):
        yaw_rate_command(0.0, 0.0, **{**RATE_GAINS, **bad})
