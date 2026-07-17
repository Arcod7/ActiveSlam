"""Tests for the simulation-only BlueROV2 Heavy thruster mixer."""

import pytest

from frontier_slam.control_utils import mix_thrusters


def test_heavy_mixer_returns_eight_commands():
    assert len(mix_thrusters(0.0, 0.0, 0.0)) == 8


@pytest.mark.parametrize('args, expected', [
    ((1.0, 0.0, 0.0, 0.0), [1.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
    ((0.0, 0.0, 0.0, 1.0), [-1.0, 1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 0.0]),
    ((0.0, 1.0, 0.0, 0.0), [-1.0, 1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]),
    ((0.0, 0.0, 1.0, 0.0), [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]),
])
def test_heavy_mixer_axis_conventions(args, expected):
    assert mix_thrusters(*args) == pytest.approx(expected)


def test_heavy_mixer_normalizes_without_changing_ratios():
    mixed = mix_thrusters(1.0, 1.0, 0.5)
    assert max(abs(value) for value in mixed) == pytest.approx(1.0)
    assert mixed[4:] == pytest.approx([0.25] * 4)
