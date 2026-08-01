"""The revisit patrol axis, and that it no longer feeds back into itself."""
import math
from types import SimpleNamespace

from frontier_slam.control_utils import HeadingReference
from frontier_slam.wall_oriented_controller import WallOrientedController
import numpy as np
import pytest


def _patrolling(**attrs):
    controller = object.__new__(WallOrientedController)
    controller._pose = np.array([0.0, 0.0, 5.0])
    controller._patrol_anchor = None
    controller._patrol_tangent = None
    controller._patrol_sign = 1.0
    controller._wall_side = 1
    controller._look_offset_deg = 45.0
    controller._last_route_heading = 0.0
    controller._wall_memory = np.array([0.0, 5.0, 5.0])   # wall due +y
    controller._patrol_look_delta = None
    controller._last_look_heading = None
    for name, value in attrs.items():
        setattr(controller, name, value)
    return controller


def test_the_patrol_axis_runs_along_the_wall_not_at_it():
    """Bearing to the wall is +y, so the beat must walk +/-x."""
    c = _patrolling()
    c._revisit_patrol_target(0.0)
    assert abs(math.cos(c._patrol_tangent)) > 0.99


def test_the_axis_is_fixed_once_the_dwell_starts():
    """The route heading may swing freely; the beat must not follow it."""
    c = _patrolling()
    c._revisit_patrol_target(0.0)
    first = c._patrol_tangent
    for heading in (1.0, -2.0, 3.0, 0.5):
        c._last_route_heading = heading
        c._revisit_patrol_target(0.0)
        assert c._patrol_tangent == first


def test_the_target_stays_on_one_axis_across_ticks():
    """The 2026-08-01 failure: targets cycling through bearings 90 deg apart."""
    c = _patrolling()
    bearings = []
    for heading in (0.0, 1.5, -1.5, 3.0, 0.0):
        c._last_route_heading = heading      # as _drive would rewrite it
        target = c._revisit_patrol_target(0.0)
        delta = target - c._patrol_anchor
        if np.hypot(*delta) > 1e-9:
            bearings.append(math.atan2(delta[1], delta[0]))
    # Every target lies on one line through the anchor: bearings are equal or
    # exactly opposed (the beat reverses), never 90 deg apart.
    for b in bearings[1:]:
        offset = abs((b - bearings[0] + math.pi) % (2 * math.pi) - math.pi)
        assert offset < 1e-6 or abs(offset - math.pi) < 1e-6


def test_without_a_remembered_wall_the_vehicle_holds_the_anchor():
    c = _patrolling(_wall_memory=None)
    target = c._revisit_patrol_target(0.0)
    assert np.array_equal(target, c._patrol_anchor)


def test_a_new_dwell_takes_a_fresh_axis():
    c = _patrolling()
    c._revisit_patrol_target(0.0)
    first = c._patrol_tangent
    c._patrol_anchor = None                       # as _revisit_state_cb clears it
    c._pose = np.array([10.0, 0.0, 5.0])
    c._wall_memory = np.array([15.0, 0.0, 5.0])   # wall now due +x
    c._revisit_patrol_target(0.0)
    assert c._patrol_tangent != first
    assert abs(math.sin(c._patrol_tangent)) > 0.99


def _driving(c):
    """Wire the drive-path attributes onto a patrol controller."""
    c._path, c._wp_idx, c._lookahead_m = [], 0, 0.0
    c._yaw, c._gyro_at = 0.0, None
    c._yaw_ctrl = None
    c._yaw_rate = SimpleNamespace(value=0.0)
    c._look_ref = HeadingReference(
        WallOrientedController.LOOK_REF_TAU,
        WallOrientedController.LOOK_REF_RATE)
    c._select_wall_side = lambda *_: None
    c._last_heading_error = 0.0
    return c


def _settled_drive(c, target_xy, look_at_wall, t0=0.0, ticks=200):
    """Tick until the shaped reference has converged on the raw target."""
    t = t0
    for _ in range(ticks):
        result = c._drive(np.asarray(target_xy, dtype=float), t,
                          look_at_wall=look_at_wall)
        t += 0.1
    return result, t


def test_the_dwell_heading_does_not_flip_when_the_beat_reverses():
    """A route-relative heading would swing 180 deg at each end of the beat."""
    c = _driving(_patrolling())
    result, t = _settled_drive(c, [3.0, 0.0], True)
    inbound = c._drive(np.array([-3.0, 0.0]), t, look_at_wall=True)[5]
    assert inbound == pytest.approx(result[5], abs=1e-3)


def test_the_dwell_heading_points_at_the_wall():
    c = _driving(_patrolling())
    result, _ = _settled_drive(c, [3.0, 0.0], True)
    # No delta captured, so it converges on the wall bearing (due +y).
    assert result[5] == pytest.approx(math.pi / 2.0, abs=1e-3)


def test_entering_the_dwell_is_continuous():
    """The 45 deg jolt on arrival: the commanded heading must not step when
    transit hands over to the patrol."""
    c = _driving(_patrolling())
    result, t = _settled_drive(c, [3.0, 0.0], False)   # transit: route + offset
    transit_look = result[5]
    c._last_look_heading = transit_look
    c._revisit_patrol_target(t)                        # captures the delta
    entering = c._drive(np.array([3.0, 0.0]), t + 0.1, look_at_wall=True)[5]
    assert entering == pytest.approx(transit_look, abs=1e-3)


def test_the_held_heading_stays_wall_relative_as_the_vehicle_moves():
    c = _patrolling()
    c._last_look_heading = math.radians(45.0)
    c._revisit_patrol_target(0.0)
    c._pose = np.array([2.0, 0.0, 5.0])            # strafed along the beat
    bearing = math.atan2(5.0 - 0.0, 0.0 - 2.0)
    assert c._wall_look_heading() == pytest.approx(
        (bearing + c._patrol_look_delta + math.pi) % (2 * math.pi) - math.pi)


def test_transit_still_uses_the_route_relative_offset():
    c = _driving(_patrolling())
    result, _ = _settled_drive(c, [3.0, 0.0], False)
    # Travel +x with the wall to starboard: converges on route + 45 deg.
    assert result[5] == pytest.approx(math.radians(45.0), abs=1e-3)
