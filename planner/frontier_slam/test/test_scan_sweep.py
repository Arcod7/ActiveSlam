"""Pure-logic tests for the cable-safe sweep scan (right half, left full, return)."""

import itertools
import math

import pytest

from frontier_slam.control_utils import wrap_angle
from frontier_slam.scan_sweep import DONE, LEFT, RETURN, RIGHT, SweepScan, scan_yaw_command


def _simulate(sweep: SweepScan, start_yaw: float, rate: float, dt: float,
             max_ticks: int = 100000):
    """Drive a vehicle that turns at exactly `rate` in the commanded sign each tick.

    Returns (end_yaw, end_time, phase_sequence).
    """
    now = 0.0
    yaw = start_yaw
    sweep.start(yaw, now)
    phases = [sweep.phase]
    for _ in range(max_ticks):
        now += dt
        cmd = sweep.update(yaw, now)
        phases.append(cmd.phase)
        if cmd.done:
            return yaw, now, phases
        yaw = wrap_angle(yaw + cmd.yaw_sign * rate * dt)
    raise AssertionError('sweep never completed')


def test_full_cycle_phase_sequence():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    _, _, phases = _simulate(sweep, start_yaw=0.0, rate=0.5, dt=0.05)
    collapsed = [phase for phase, _ in itertools.groupby(phases)]
    assert collapsed == [RIGHT, LEFT, RETURN, DONE]


def test_net_zero_cumulative_yaw_over_a_completed_cycle():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    start_yaw = 0.3
    end_yaw, _, _ = _simulate(sweep, start_yaw=start_yaw, rate=0.5, dt=0.05)
    assert abs(wrap_angle(end_yaw - start_yaw)) <= SweepScan.TOLERANCE_RAD


def test_net_zero_cumulative_yaw_handles_wraparound_start_heading():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    start_yaw = math.radians(175.0)   # near the +-pi seam
    end_yaw, _, _ = _simulate(sweep, start_yaw=start_yaw, rate=0.5, dt=0.05)
    assert abs(wrap_angle(end_yaw - start_yaw)) <= SweepScan.TOLERANCE_RAD


def test_right_phase_reaches_half_sweep_before_turning_left():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    now = 0.0
    yaw = 0.0
    sweep.start(yaw, now)
    cmd = sweep.update(yaw, now)
    while cmd.phase == RIGHT:
        now += 0.05
        yaw = wrap_angle(yaw + cmd.yaw_sign * 0.5 * 0.05)
        cmd = sweep.update(yaw, now)
    # Just crossed into LEFT: heading should be at (approximately) -half sweep.
    assert yaw == pytest.approx(-math.pi / 2.0, abs=0.05)


def test_timeout_guard_forces_early_return_when_turning_too_slowly():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=1.0, timeout_multiplier=1.0)
    # timeout_s = 1.0 * (2*pi rad) / 1.0 rad/s ~= 6.28s
    slow_rate = 0.05   # much slower than the 1.0 rad/s assumed for the timeout
    end_yaw, end_time, phases = _simulate(sweep, start_yaw=0.0, rate=slow_rate, dt=0.05)
    assert RETURN in phases
    # At slow_rate it would take ~2*pi/0.05 ~= 126s to complete a full untimed
    # cycle; RIGHT/LEFT are bounded by timeout_s and RETURN by its own
    # same-length deadline, so total time can never exceed 2x timeout_s.
    assert end_time <= 2 * sweep.timeout_s + 1.0


def test_timeout_guard_ends_a_fully_jammed_cycle_immediately():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5, timeout_multiplier=1.0)
    sweep.start(0.0, now=0.0)
    # Heading never actually moves (e.g. a jammed thruster) despite commands.
    cmd = sweep.update(0.0, now=sweep.timeout_s + 0.01)
    # Already at the start heading once the guard fires, so the cycle just ends.
    assert cmd.phase == DONE
    assert cmd.done


def test_return_phase_has_its_own_bounded_deadline_when_jammed_mid_return():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=1.0, timeout_multiplier=1.0)
    sweep.start(0.0, now=0.0)
    # Turned partway into RIGHT (short of the -half target) before the
    # timeout forces an early RETURN, same shape as the slow-turning case.
    cmd = sweep.update(-0.3, now=sweep.timeout_s + 0.01)
    assert cmd.phase == RETURN
    assert not cmd.done
    # Now fully jammed for the rest of the run: heading never moves again,
    # for far longer than a second timeout_s would allow if RETURN were
    # unbounded (this is the bug the RETURN deadline fixes).
    cmd = sweep.update(-0.3, now=2 * sweep.timeout_s + 1000.0)
    assert cmd.phase == DONE
    assert cmd.done


def test_reset_discards_an_in_progress_cycle():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    sweep.start(0.0, now=0.0)
    sweep.update(0.0, now=0.05)
    assert sweep.active
    sweep.reset()
    assert not sweep.active
    assert sweep.phase == DONE


def test_invalid_construction_arguments_are_rejected():
    with pytest.raises(ValueError):
        SweepScan(sweep_deg=0.0, yaw_rate_rad_s=0.5)
    with pytest.raises(ValueError):
        SweepScan(sweep_deg=400.0, yaw_rate_rad_s=0.5)
    with pytest.raises(ValueError):
        SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.0)
    with pytest.raises(ValueError):
        SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5, timeout_multiplier=0.0)


def test_spin_style_bypasses_the_sweep_engine():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    yaw_cmd = scan_yaw_command(sweep, 'spin', yaw=0.0, now=0.0,
                               scan_yaw_speed=0.08, repeat=True)
    assert yaw_cmd == pytest.approx(0.08)
    # Bypassed entirely: the sweep object is never touched.
    assert sweep.phase == DONE
    assert not sweep.active


def test_sweep_style_drives_the_engine_and_scales_by_scan_yaw_speed():
    sweep = SweepScan(sweep_deg=180.0, yaw_rate_rad_s=0.5)
    yaw_cmd = scan_yaw_command(sweep, 'sweep', yaw=0.0, now=0.0,
                               scan_yaw_speed=0.08, repeat=True)
    assert sweep.active
    assert yaw_cmd == pytest.approx(-0.08)   # RIGHT phase, first tick


def test_scan_yaw_command_restarts_when_repeat_true_and_stays_idle_when_false():
    sweep = SweepScan(sweep_deg=60.0, yaw_rate_rad_s=1.0)
    _simulate(sweep, start_yaw=0.0, rate=1.0, dt=0.02)
    assert not sweep.active

    idle_cmd = scan_yaw_command(sweep, 'sweep', yaw=0.0, now=1000.0,
                                scan_yaw_speed=0.5, repeat=False)
    assert idle_cmd == 0.0
    assert not sweep.active

    restart_cmd = scan_yaw_command(sweep, 'sweep', yaw=0.0, now=1000.0,
                                   scan_yaw_speed=0.5, repeat=True)
    assert sweep.active
    assert restart_cmd != 0.0
