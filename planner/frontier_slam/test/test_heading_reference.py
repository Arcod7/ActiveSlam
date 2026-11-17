"""Reference shaping for the commanded heading."""
import math

from frontier_slam.control_utils import HeadingReference
import pytest


def _ref():
    return HeadingReference(0.4, 0.6, max_gap_s=1.0)


def test_seeds_at_the_vehicle_heading_not_the_target():
    """Entering closed-loop control must never demand an instant turn."""
    r = _ref()
    assert r.update(math.radians(90.0), 0.0, current=math.radians(10.0)) == (
        pytest.approx(math.radians(10.0)))


def test_the_reference_moves_no_faster_than_its_rate_cap():
    r = _ref()
    r.update(2.0, 0.0, current=0.0)
    moved = r.update(2.0, 0.1, current=0.0)
    assert moved == pytest.approx(0.6 * 0.1)


def test_small_jitter_is_pulled_not_jumped():
    """Below the rate cap the first-order pull applies, attenuating jitter."""
    r = _ref()
    r.update(0.0, 0.0, current=0.0)
    step = math.radians(4.0)
    value = r.update(step, 0.1, current=0.0)
    alpha = 0.1 / (0.4 + 0.1)
    assert value == pytest.approx(alpha * step)
    assert value < step


def test_wrap_crossing_takes_the_short_way():
    r = _ref()
    r.update(math.radians(170.0), 0.0, current=math.radians(170.0))
    value = r.update(math.radians(-170.0), 0.1, current=math.radians(170.0))
    # Short way is +20 deg through pi, so the value grows past 170.
    assert value > math.radians(170.0) or value < math.radians(-170.0)


def test_a_gap_reseeds_at_the_current_heading():
    """After an open-loop spin the glide restarts from where the hull points."""
    r = _ref()
    r.update(1.0, 0.0, current=0.0)
    value = r.update(1.0, 5.0, current=2.0)   # 5 s gap > max_gap
    assert value == pytest.approx(2.0)


def test_converges_to_a_held_target():
    r = _ref()
    t, value = 0.0, r.update(1.0, 0.0, current=0.0)
    for _ in range(200):
        t += 0.1
        value = r.update(1.0, t, current=0.0)
    assert value == pytest.approx(1.0, abs=1e-3)


def test_a_non_finite_target_is_ignored():
    r = _ref()
    r.update(0.5, 0.0, current=0.5)
    assert r.update(float("nan"), 0.1, current=0.0) == pytest.approx(0.5)
