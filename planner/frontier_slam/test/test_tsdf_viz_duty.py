"""Pure tests for tsdf_mapper.py's viz duty gate and normal-sampling stride —
no rclpy, no ROS init, no VDBFusion.

These guard the fix for the freeze where marching cubes and the VDB grid walk
grew past one cloud period, starved /cloud_in and the TF callbacks, and every
deferred scan expired.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import pytest

from frontier_slam.tsdf_mapper import TSDFMapper, _DutyGate


class _StrideOnly:
    """TSDFMapper._normal_stride without constructing the node."""

    def __init__(self, normal_every, max_normals):
        self._normal_every = normal_every
        self._max_normals = max_normals

    stride = TSDFMapper._normal_stride


def test_gate_is_open_before_any_run():
    assert _DutyGate(0.25).ready(0.0)


def test_gate_blocks_for_the_configured_duty_ratio():
    gate = _DutyGate(0.25)          # at most 25% of wall clock
    gate.record(now=4.0, elapsed=4.0)
    # 4s of work at 25% duty must be followed by 12s of quiet.
    assert not gate.ready(15.9)
    assert gate.ready(16.0)
    assert gate.last_s == pytest.approx(4.0)


def test_full_duty_never_blocks():
    gate = _DutyGate(1.0)
    gate.record(now=4.0, elapsed=4.0)
    assert gate.ready(4.0)


def test_duty_is_clamped_into_range():
    # >1 clamps to 1 (no blocking); <=0 clamps to the 1e-3 floor, not a crash.
    fast = _DutyGate(5.0)
    fast.record(now=1.0, elapsed=1.0)
    assert fast.ready(1.0)

    slow = _DutyGate(0.0)
    slow.record(now=1.0, elapsed=1.0)
    assert not slow.ready(500.0)


def test_cheap_run_barely_delays_the_next_one():
    gate = _DutyGate(0.5)
    gate.record(now=0.01, elapsed=0.01)
    assert gate.ready(0.02)


def test_stride_falls_back_to_normal_every_when_uncapped():
    assert _StrideOnly(normal_every=10, max_normals=0).stride(91683) == 10


def test_stride_widens_to_honour_the_cap():
    s = _StrideOnly(normal_every=10, max_normals=4000).stride(91683)
    assert s > 10
    assert -(-91683 // s) <= 4000


def test_stride_never_drops_below_normal_every():
    # Small mesh: the cap is not binding, so normal_every still rules.
    assert _StrideOnly(normal_every=10, max_normals=4000).stride(500) == 10


def test_stride_is_at_least_one():
    assert _StrideOnly(normal_every=0, max_normals=4000).stride(10) == 1
