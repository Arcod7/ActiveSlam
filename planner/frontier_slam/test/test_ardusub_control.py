"""Tests for fail-closed ArduSub command conversion and encoding."""

import math

import pytest

from frontier_slam.ardusub_control import (
    ArduSubCommandSender,
    BACKEND_LOCAL_NED_VELOCITY,
    BACKEND_MANUAL_CONTROL,
    BodyDemand,
    ReauthorizationLatch,
    VELOCITY_YAW_RATE_TYPE_MASK,
    adapter_readiness,
    local_ned_velocity,
    manual_control_axes,
    validate_body_command,
)


class FakeMav:
    """Record the encoder call without a network or vehicle."""

    def __init__(self):
        self.manual_calls = []
        self.local_ned_calls = []

    def manual_control_send(self, *args):
        self.manual_calls.append(args)

    def set_position_target_local_ned_send(self, *args):
        self.local_ned_calls.append(args)


class FakeConnection:
    def __init__(self):
        self.mav = FakeMav()


def test_body_command_validation_rejects_unsafe_or_unsupported_values():
    assert validate_body_command((0.2, -0.1, 0.0, 0.0, 0.0, 0.3)) == BodyDemand(
        0.2, -0.1, 0.0, 0.3)
    assert validate_body_command((0.0,) * 5) is None
    assert validate_body_command((1.01, 0.0, 0.0, 0.0, 0.0, 0.0)) is None
    assert validate_body_command((math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)) is None
    assert validate_body_command((0.0, 0.0, 0.0, 0.1, 0.0, 0.0)) is None


def test_manual_control_mapping_is_limited_and_vertical_defaults_neutral():
    demand = BodyDemand(surge=1.0, sway=-0.5, heave=1.0, yaw_rate=0.25)
    assert manual_control_axes(demand, 0.15) == (150, -75, 500, 38)
    assert manual_control_axes(demand, 0.20, enable_vertical=True) == (
        200, -100, 600, 50)
    with pytest.raises(ValueError):
        manual_control_axes(demand, 1.01)


def test_local_ned_mapping_uses_metric_limits_and_ned_vertical_sign():
    demand = BodyDemand(surge=0.5, sway=-0.5, heave=1.0, yaw_rate=-0.25)
    result = local_ned_velocity(demand, 0.2, 0.1, 0.08, 0.4)
    assert result.vx == pytest.approx(0.1)
    assert result.vy == pytest.approx(-0.05)
    assert result.vz == 0.0
    assert result.yaw_rate == pytest.approx(-0.1)
    vertical = local_ned_velocity(
        demand, 0.2, 0.1, 0.08, 0.4, enable_vertical=True)
    assert vertical.vz == pytest.approx(0.08)


def test_manual_sender_encodes_ardusub_legacy_axes():
    connection = FakeConnection()
    sender = ArduSubCommandSender(
        connection, BACKEND_MANUAL_CONTROL, manual_authority=0.2)
    sender.send(BodyDemand(0.5, -0.25, 1.0, 0.1), 1, 1, 123)
    assert connection.mav.manual_calls == [(1, 100, -50, 500, 20, 0)]


def test_local_ned_sender_encodes_velocity_and_yaw_rate_only():
    connection = FakeConnection()
    sender = ArduSubCommandSender(
        connection,
        BACKEND_LOCAL_NED_VELOCITY,
        max_surge_mps=0.2,
        max_sway_mps=0.1,
        max_vertical_mps=0.08,
        max_yaw_rate_rps=0.4,
    )
    sender.send(BodyDemand(0.5, -0.5, 1.0, -0.25), 2, 3, 1234)
    call = connection.mav.local_ned_calls[0]
    assert call[:5] == (1234, 2, 3, 8, VELOCITY_YAW_RATE_TYPE_MASK)
    assert VELOCITY_YAW_RATE_TYPE_MASK == 1479
    assert call[5:8] == (0.0, 0.0, 0.0)
    assert call[8:11] == pytest.approx((0.1, -0.05, 0.0))
    assert call[11:15] == (0.0, 0.0, 0.0, 0.0)
    assert call[15] == pytest.approx(-0.1)


def _ready_inputs():
    return {
        'now': 10.0,
        'heartbeat_time': 9.9,
        'heartbeat_timeout_s': 2.0,
        'gate_status': 'ACTIVE',
        'gate_status_time': 9.9,
        'gate_status_timeout_s': 0.5,
        'command': BodyDemand(0.1, 0.0, 0.0, 0.0),
        'command_time': 9.9,
        'command_timeout_s': 0.5,
        'invalid_command': False,
        'armed': True,
        'require_armed': True,
        'vehicle_mode': 'ALT_HOLD',
        'required_mode': 'ALT_HOLD',
    }


@pytest.mark.parametrize(
    ('change', 'expected'),
    [
        ({'heartbeat_time': None}, 'NO_HEARTBEAT'),
        ({'heartbeat_time': 7.0}, 'STALE_HEARTBEAT'),
        ({'gate_status': None}, 'NO_GATE_STATUS'),
        ({'gate_status_time': 9.0}, 'STALE_GATE_STATUS'),
        ({'gate_status': 'DISABLED'}, 'GATE_DISABLED'),
        ({'invalid_command': True}, 'INVALID_COMMAND'),
        ({'command': None}, 'NO_COMMAND'),
        ({'command_time': 9.0}, 'STALE_COMMAND'),
        ({'armed': False}, 'VEHICLE_DISARMED'),
        ({'vehicle_mode': 'STABILIZE'}, 'MODE_MISMATCH:STABILIZE'),
    ],
)
def test_adapter_readiness_is_fail_closed(change, expected):
    inputs = _ready_inputs()
    inputs.update(change)
    assert adapter_readiness(**inputs) == expected


def test_adapter_readiness_requires_every_condition():
    assert adapter_readiness(**_ready_inputs()) == 'ACTIVE'


def test_active_fault_requires_explicit_disable_before_resume():
    latch = ReauthorizationLatch()
    assert latch.evaluate('ACTIVE', had_authority=False) == 'ACTIVE'
    assert latch.evaluate('STALE_HEARTBEAT', had_authority=True) == (
        'REAUTHORIZE_REQUIRED:STALE_HEARTBEAT')
    assert latch.evaluate('ACTIVE', had_authority=False) == (
        'REAUTHORIZE_REQUIRED:STALE_HEARTBEAT')
    assert latch.evaluate('GATE_DISABLED', had_authority=False) == (
        'GATE_DISABLED')
    assert latch.evaluate('ACTIVE', had_authority=False) == 'ACTIVE'


def test_pre_authority_mismatch_does_not_create_a_hidden_latch():
    latch = ReauthorizationLatch()
    assert latch.evaluate('MODE_MISMATCH:STABILIZE', False) == (
        'MODE_MISMATCH:STABILIZE')
    assert latch.evaluate('ACTIVE', False) == 'ACTIVE'
