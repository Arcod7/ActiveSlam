"""Tests for the simulation-only BlueROV2 Heavy thruster mixer."""

import pytest

from frontier_slam.control_utils import attitude_hold_effort, mix_thrusters


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


def test_heavy_mixer_normalizes_disjoint_motor_groups():
    mixed = mix_thrusters(1.0, 1.0, 0.5)
    assert max(abs(value) for value in mixed) == pytest.approx(1.0)
    # Horizontal saturation cannot consume vertical motor authority.
    assert mixed[4:] == pytest.approx([0.5 ** 0.5] * 4)


def test_heavy_mixer_converts_thrust_effort_to_angular_speed():
    mixed = mix_thrusters(0.25, 0.0, 0.0)
    assert mixed[:4] == pytest.approx([0.5, 0.5, -0.5, -0.5])


@pytest.mark.parametrize('kwargs, expected_vertical', [
    ({'roll': 1.0}, [1.0, -1.0, 1.0, -1.0]),
    ({'pitch': 1.0}, [-1.0, -1.0, 1.0, 1.0]),
])
def test_heavy_mixer_attitude_axes(kwargs, expected_vertical):
    mixed = mix_thrusters(0.0, 0.0, 0.0, **kwargs)
    assert mixed[4:] == pytest.approx(expected_vertical)


def test_attitude_hold_is_restorative_and_bounded():
    roll, pitch = attitude_hold_effort(
        roll=0.4, pitch=-0.3,
        roll_rate=0.2, pitch_rate=-0.1,
        kp=0.45, kd=0.18, limit=0.2)
    assert roll == pytest.approx(-0.2)
    assert pitch == pytest.approx(0.153)
