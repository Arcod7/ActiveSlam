"""Pure-logic tests for the fail-closed body-command safety gate."""

import math

from frontier_slam.safety_logic import MotionSafetyState


# Twist field order: linear x/y/z, angular x/y/z.
COMMAND = [0.1, -0.1, 0.05, 0.0, 0.0, 0.2]
ZERO = (0.0,) * 6


def _ready_gate(now=10.0):
    gate = MotionSafetyState(command_timeout_s=0.5, odom_timeout_s=0.5)
    gate.update_odometry(now)
    gate.set_enabled(True)
    assert gate.update_command(COMMAND, now)
    return gate


def test_gate_starts_disabled_and_outputs_zero():
    gate = MotionSafetyState()
    gate.update_odometry(1.0)
    gate.update_command(COMMAND, 1.0)
    decision = gate.evaluate(1.0)
    assert decision.state == gate.DISABLED
    assert decision.output == ZERO


def test_gate_passes_only_fresh_command_and_odometry():
    gate = _ready_gate()
    decision = gate.evaluate(10.1)
    assert decision.state == gate.ACTIVE
    assert decision.output == tuple(COMMAND)


def test_enable_transition_requires_a_new_command():
    gate = MotionSafetyState()
    gate.update_odometry(2.0)
    gate.update_command(COMMAND, 2.0)
    gate.set_enabled(True)
    decision = gate.evaluate(2.0)
    assert decision.state == gate.NO_COMMAND
    assert decision.output == ZERO


def test_disable_clears_command_and_requires_new_command_after_reenable():
    gate = _ready_gate()
    gate.set_enabled(False)
    gate.set_enabled(True)
    assert gate.evaluate(10.1).state == gate.NO_COMMAND


def test_stale_command_forces_zero():
    gate = _ready_gate()
    decision = gate.evaluate(10.51)
    assert decision.state == gate.STALE_COMMAND
    assert decision.output == ZERO


def test_stale_odometry_forces_zero():
    gate = _ready_gate()
    gate.update_command(COMMAND, 10.5)
    decision = gate.evaluate(10.51)
    assert decision.state == gate.STALE_ODOMETRY
    assert decision.output == ZERO


def test_missing_odometry_forces_zero():
    gate = MotionSafetyState()
    gate.set_enabled(True)
    gate.update_command(COMMAND, 4.0)
    decision = gate.evaluate(4.0)
    assert decision.state == gate.NO_ODOMETRY
    assert decision.output == ZERO


def test_odometry_requirement_can_be_disabled_for_manual_control():
    gate = MotionSafetyState(require_odom=False)
    gate.set_enabled(True)
    gate.update_command(COMMAND, 4.0)
    assert gate.evaluate(4.0).state == gate.ACTIVE


def test_wrong_command_length_is_rejected():
    gate = _ready_gate()
    assert not gate.update_command([0.0] * 5, 10.1)
    assert gate.evaluate(10.1).state == gate.INVALID_COMMAND


def test_nonfinite_command_is_rejected():
    gate = _ready_gate()
    bad = COMMAND.copy()
    bad[2] = math.nan
    assert not gate.update_command(bad, 10.1)
    assert gate.evaluate(10.1).state == gate.INVALID_COMMAND


def test_out_of_range_command_is_rejected_instead_of_clamped():
    gate = _ready_gate()
    bad = COMMAND.copy()
    bad[0] = 1.01
    assert not gate.update_command(bad, 10.1)
    decision = gate.evaluate(10.1)
    assert decision.state == gate.INVALID_COMMAND
    assert decision.output == ZERO


def test_valid_command_recovers_from_invalid_command():
    gate = _ready_gate()
    gate.update_command([2.0] * 6, 10.1)
    assert gate.update_command(COMMAND, 10.2)
    assert gate.evaluate(10.2).state == gate.ACTIVE


def test_multiple_publishers_force_zero():
    gate = _ready_gate()
    decision = gate.evaluate(10.1, multiple_command_sources=True)
    assert decision.state == gate.MULTIPLE_COMMAND_SOURCES
    assert decision.output == ZERO
