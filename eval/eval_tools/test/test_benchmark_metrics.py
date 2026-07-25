"""Regression tests for time-based benchmark pairing."""
from types import SimpleNamespace

import numpy as np
import pytest

from eval_tools.benchmark import BenchmarkNode, PoseSample, TimestampBuffer


IDENTITY_Q = np.array([0.0, 0.0, 0.0, 1.0])


def sample(t, x=0.0):
    return PoseSample(float(t), np.array([float(x), 0.0, 0.0]), IDENTITY_Q)


class Publisher:
    def __init__(self):
        self.values = []

    def publish(self, msg):
        self.values.append(msg.data)


def rpe_harness(delta_s=1.0):
    return SimpleNamespace(
        _matched_pairs=[],
        _rpe_delta_s=delta_s,
        pub_rpe_trans=Publisher(),
        pub_rpe_rot=Publisher(),
    )


def update(node, gt, est):
    return BenchmarkNode._update_rpe(node, gt, est)


def test_rpe_uses_fixed_time_not_previous_sample():
    node = rpe_harness(delta_s=1.0)
    assert update(node, sample(0.0), sample(0.0)) == (None, None)
    # An irregular intermediate sample must not become the one-second baseline.
    assert update(node, sample(0.1, 0.1), sample(0.1, 0.1)) == (None, None)
    trans, rot = update(node, sample(1.0, 1.0), sample(1.0, 2.0))
    assert trans == pytest.approx(1.0)
    assert rot == pytest.approx(0.0)


def test_rpe_refuses_to_change_baseline_across_a_large_sample_gap():
    node = rpe_harness(delta_s=1.0)
    update(node, sample(0.0), sample(0.0))
    assert update(node, sample(2.0, 2.0), sample(2.0, 2.0)) == (None, None)
    assert node.pub_rpe_trans.values == []


def test_timestamp_buffer_enforces_pairing_tolerance():
    buf = TimestampBuffer(max_dt=0.05)
    buf.add(sample(1.0))
    assert buf.nearest(1.04).t == pytest.approx(1.0)
    assert buf.nearest(1.06) is None
