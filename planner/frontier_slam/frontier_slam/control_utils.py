"""Stateless helpers for the motion controllers.

Thruster mixing convention (BlueROV2 Heavy, NED body frame, scn-file order):
  Index  Name        Formula
  0      FrontRight   surge - sway - yaw
  1      FrontLeft    surge + sway + yaw
  2      BackRight   -surge - sway + yaw
  3      BackLeft    -surge + sway - yaw
  4      DiveFrontRight  heave + roll - pitch
  5      DiveFrontLeft   heave - roll - pitch
  6      DiveBackRight   heave + roll + pitch
  7      DiveBackLeft    heave - roll + pitch

Sway sign: positive = starboard (strafe right), matching teleop's E key.
Verified against bluerov2_unphy.scn geometry: the four 45° horizontal thrusters
at [-s, +s, -s, +s] sum to a pure +Y (East-when-facing-North) body force.

A note on the heave sign:
  In NED the Z axis points DOWN. Positive heave pushes the robot further down.
  The depth-hold law therefore reads `heave = -KP * (pose_z - setpoint_z)`:
  when the robot is too deep (pose_z > setpoint_z), heave < 0, and the robot
  rises.
"""
import math


def yaw_from_quat(q) -> float:
    """Extract the yaw angle (radians, ROS REP-103) from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def roll_pitch_from_quat(q) -> tuple[float, float]:
    """Extract body roll and pitch in radians from a geometry_msgs Quaternion."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = (math.copysign(math.pi / 2.0, sinp)
             if abs(sinp) >= 1.0 else math.asin(sinp))
    return roll, pitch


def wrap_angle(a: float) -> float:
    """Wrap any angle into [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_from_rpy(roll: float, pitch: float,
                  yaw: float) -> tuple[float, float, float, float]:
    """ZYX intrinsic roll/pitch/yaw to a quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def world_to_body(vx: float, vy: float, vz: float, roll: float, pitch: float,
                  yaw: float) -> tuple[float, float, float]:
    """Rotate a world NED vector into the body frame (R transpose, ZYX)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cp * cy * vx + cp * sy * vy - sp * vz,
        (sr * sp * cy - cr * sy) * vx + (sr * sp * sy + cr * cy) * vy
        + sr * cp * vz,
        (cr * sp * cy + sr * sy) * vx + (cr * sp * sy - sr * cy) * vy
        + cr * cp * vz,
    )


def attitude_hold_effort(roll: float, pitch: float,
                         roll_rate: float, pitch_rate: float,
                         kp: float, kd: float,
                         limit: float) -> tuple[float, float]:
    """Return bounded restorative roll/pitch torque efforts for level hold."""
    if kp < 0.0 or kd < 0.0 or not 0.0 < limit <= 1.0:
        raise ValueError('attitude gains must be non-negative and limit in (0, 1]')
    roll_effort = max(-limit, min(limit, -kp * roll - kd * roll_rate))
    pitch_effort = max(-limit, min(limit, -kp * pitch - kd * pitch_rate))
    return roll_effort, pitch_effort


class LowPass:
    """Exponential low-pass on a channel that is already the quantity wanted.

    LowPassRate's sibling for a measured rate rather than a differentiated one:
    a gyro needs smoothing, not differencing. max_gap_s treats a stall as a
    dropout and holds the last value rather than jumping on the stale sample.
    """

    def __init__(self, tau_s: float, max_gap_s: float = 0.0) -> None:
        self._tau = max(0.0, tau_s)
        self._max_gap = max(0.0, max_gap_s)
        self._last_t: float | None = None
        self.value = 0.0

    def update(self, sample: float, t: float) -> float:
        sample = float(sample)
        if self._last_t is None:
            self.value = sample
        else:
            dt = t - self._last_t
            if dt > 0.0 and not (self._max_gap and dt > self._max_gap):
                alpha = dt / (self._tau + dt) if self._tau > 0.0 else 1.0
                self.value += alpha * (sample - self.value)
        self._last_t = float(t)
        return self.value

    def reset(self) -> None:
        self._last_t = None
        self.value = 0.0


class HeadingReference:
    """Rate-limited, wrap-aware first-order shaping for a heading setpoint.

    The raw viewing target is a staircase: path geometry rebuilt on ~1 Hz map
    updates, jittered by the SLAM pose, re-signed by tens of degrees at mode
    changes. A heading loop tight enough to track well chases every edge of
    it, and smoothing the actuator command instead puts a rate limiter inside
    the loop -- amplitude-dependent phase lag, which is what sustains a limit
    cycle. Shaping belongs on the reference: the loop then follows a target
    that moves like the hull does.

    First-order pull toward the target, capped at max_rate. After max_gap_s
    without an update the next call re-seeds at `current` (the vehicle's own
    heading), so re-entering closed-loop control never demands an instant
    turn, whatever mode it comes from. A non-finite target is ignored.
    """

    def __init__(self, tau_s: float, max_rate: float,
                 max_gap_s: float = 1.0) -> None:
        if max_rate <= 0.0:
            raise ValueError('max_rate must be positive')
        self._tau = max(0.0, tau_s)
        self._max_rate = max_rate
        self._max_gap = max(0.0, max_gap_s)
        self._last_t: float | None = None
        self.value = 0.0

    def update(self, target: float, t: float, current: float) -> float:
        if not math.isfinite(target):
            return self.value
        if self._last_t is None or (self._max_gap
                                    and t - self._last_t > self._max_gap):
            self.value = wrap_angle(current)
        else:
            dt = t - self._last_t
            if dt > 0.0:
                alpha = dt / (self._tau + dt) if self._tau > 0.0 else 1.0
                step = alpha * wrap_angle(target - self.value)
                cap = self._max_rate * dt
                self.value = wrap_angle(
                    self.value + max(-cap, min(cap, step)))
        self._last_t = float(t)
        return self.value

    def reset(self) -> None:
        self._last_t = None


class LowPassRate:
    """Low-pass-filtered finite difference of a sampled channel.

    The channel is a pose estimate, not a measurement of motion, so two things
    that are not rate reach this filter: a pose-graph correction, which is a
    step, and a stalled publisher, across which alpha = dt/(tau+dt) tends to 1
    and passes whatever arrives straight through. max_rate rejects the first,
    max_gap_s the second.
    """

    def __init__(self, tau_s: float, wrap: bool = False,
                 max_rate: float | None = None,
                 max_gap_s: float = 0.0) -> None:
        self._tau = max(0.0, tau_s)
        self._wrap = wrap      # for angles: a raw difference jumps 2pi at +/-pi
        self._max_rate = max_rate
        self._max_gap = max(0.0, max_gap_s)
        self._last: float | None = None
        self._last_t: float | None = None
        self.value = 0.0

    def update(self, sample: float, t: float) -> float:
        if self._last is not None and t > self._last_t:
            dt = t - self._last_t
            if self._max_gap and dt > self._max_gap:
                self.value = 0.0   # the gap moved the sample, not the vehicle
            else:
                delta = sample - self._last
                raw = (wrap_angle(delta) if self._wrap else delta) / dt
                # Past what the hull can do is a correction; re-baseline on it
                # rather than filtering it, so one step cannot swamp the loop.
                if self._max_rate is None or abs(raw) <= self._max_rate:
                    alpha = dt / (self._tau + dt) if self._tau > 0.0 else 1.0
                    self.value += alpha * (raw - self.value)
        self._last, self._last_t = float(sample), float(t)
        return self.value

    def reset(self) -> None:
        self._last = self._last_t = None
        self.value = 0.0


class SlewLimiter:
    """Rate limit on a command channel, so a setpoint step cannot reach the
    actuators as a step.

    Yaw is inertia plus a little drag, so an open-loop effort delivered in one
    tick is one impulse of angular acceleration. A heading setpoint that jumps
    -- a new goal, a wall-side change -- therefore whipped the vehicle to
    1.37 rad/s on a hull whose steady scan rate is 0.22. An effort limit bounds
    only the rate that is eventually reached; bounding how fast the effort may
    change bounds the acceleration used to get there, whatever issued it.

    `release_rate_per_s` may raise the rate allowed while the command shrinks
    toward zero. Applying torque is what whips the hull, so that stays slow;
    removing it only lets the hull coast, and rate-limiting it is pure lag. At
    a symmetric 0.25/s the yaw command needed 1.2 s to fall from full
    authority, still pushing the old way long after the heading error changed
    sign -- measured 2026-07-31 at a median 52 deg of overshoot. Defaults to
    the apply rate, i.e. the symmetric behaviour every other caller expects.
    """

    def __init__(self, max_rate_per_s: float, max_gap_s: float = 0.5,
                 release_rate_per_s: float | None = None) -> None:
        if max_rate_per_s <= 0.0:
            raise ValueError('max_rate_per_s must be positive')
        if release_rate_per_s is not None and release_rate_per_s <= 0.0:
            raise ValueError('release_rate_per_s must be positive')
        self._max_rate = max_rate_per_s
        self._release_rate = (max_rate_per_s if release_rate_per_s is None
                              else release_rate_per_s)
        self._max_gap = max(0.0, max_gap_s)   # a longer gap is a stall, not a ramp
        self._last_t: float | None = None
        self.value = 0.0

    def update(self, target: float, t: float) -> float:
        dt = (0.0 if self._last_t is None
              else min(max(0.0, t - self._last_t), self._max_gap))
        self._last_t = float(t)
        delta = float(target) - self.value
        if delta == 0.0 or dt == 0.0:
            return self.value

        direction = math.copysign(1.0, delta)
        # Shrinking |value| releases torque; growing it applies fresh torque. A
        # step that crosses zero does both, in that order, splitting the tick.
        shrink = (min(abs(delta), abs(self.value))
                  if self.value * direction < 0.0 else 0.0)
        released = min(shrink, self._release_rate * dt)
        self.value += direction * released
        if released < shrink:
            return self.value
        apply_dt = max(0.0, dt - released / self._release_rate)
        self.value += direction * min(
            abs(delta) - released, self._max_rate * apply_dt)
        return self.value

    def reset(self) -> None:
        self._last_t = None
        self.value = 0.0


def depth_hold_effort(depth_error: float, depth_rate: float,
                      kp: float, kd: float, limit: float = 1.0) -> float:
    """Bounded depth-hold effort with rate damping.

    Proportional-only depth hold limit-cycles: the vehicle is effectively a
    double integrator, the loop runs at 10 Hz, and once the thrusters have their
    real authority there is no phase margin left. The rate term supplies it.
    """
    if kp < 0.0 or kd < 0.0 or not 0.0 < limit <= 1.0:
        raise ValueError('depth gains must be non-negative and limit in (0, 1]')
    return max(-limit, min(limit, -kp * depth_error - kd * depth_rate))


def yaw_rate_command(heading_error: float, measured_rate: float,
                     kp_heading: float, kp_rate: float, max_rate: float,
                     limit: float = 1.0) -> float:
    """Cascaded heading hold: ask for a turn RATE, then drive the measured rate.

    A direct heading->thrust law knows nothing about how fast the vehicle is
    already turning. Raising its gain to cure under-turning therefore bought
    400 deg/s spins once the thrusters had boost authority -- an angular
    acceleration no hull of this size reaches in water. Capping the requested
    rate bounds the achieved motion instead of the command, so the limit holds
    whatever the thrust ceiling is. The rate error term also supplies the
    damping a separate KD used to.
    """
    if kp_heading < 0.0 or kp_rate < 0.0 or max_rate <= 0.0:
        raise ValueError('yaw gains must be non-negative and max_rate positive')
    desired = max(-max_rate, min(max_rate, kp_heading * heading_error))
    return max(-limit, min(limit, kp_rate * (desired - measured_rate)))


def allocate_horizontal(surge: float, sway: float, yaw: float,
                        yaw_share: float = 0.6) -> tuple[float, float, float]:
    """Fit surge/sway/yaw into the horizontal thrusters, yaw first.

    The four horizontal motors serve translation and rotation at once, so a
    large surge can consume the authority yaw needs. Normalising the group as a
    whole silently scales yaw down exactly when heading error is largest.
    Translation yields instead: the vehicle slows to turn rather than turning
    weakly at speed. The vertical group is already protected this way.

    yaw_share caps what rotation may claim *when translation is also wanted*.
    Letting yaw take the whole budget stopped travel dead, which tripped the
    controller's stuck detector, whose escape manoeuvre yaws -- so the vehicle
    span in place and never recovered. Turning slows travel; it must not stop
    it. A pure rotation command keeps full authority: there is nothing to
    starve, and the rate loop upstream bounds how fast it actually turns.
    """
    yaw = max(-1.0, min(1.0, yaw))
    translation = abs(surge) + abs(sway)
    if translation == 0.0:
        return surge, sway, yaw
    yaw_share = min(1.0, max(0.0, yaw_share))
    yaw = max(-yaw_share, min(yaw_share, yaw))
    headroom = 1.0 - abs(yaw)
    if translation > headroom:
        scale = max(0.0, headroom) / translation if translation > 0.0 else 0.0
        surge *= scale
        sway *= scale
    return surge, sway, yaw


def _normalise_group(values: list[float]) -> list[float]:
    peak = max(abs(value) for value in values)
    return [value / peak for value in values] if peak > 1.0 else values


def _effort_to_omega(value: float) -> float:
    """Convert signed thrust effort to normalized propeller angular velocity."""
    return math.copysign(math.sqrt(abs(value)), value) if value else 0.0


def mix_thrusters(surge: float, yaw: float, heave: float, sway: float = 0.0,
                  roll: float = 0.0, pitch: float = 0.0) -> list:
    """Map body commands to the 8 Stonefish BlueROV2 Heavy thrusters.

    Inputs are normalized *thrust efforts*. Stonefish's fluid-dynamics model
    produces thrust proportional to propeller angular velocity squared, so the
    returned actuator setpoints use a signed square root. Horizontal and
    vertical groups are normalized independently because they use disjoint
    motors. Positive sway = starboard (strafe right).
    """
    surge, sway, yaw = allocate_horizontal(surge, sway, yaw)
    horizontal = [
        surge - sway - yaw,    # 0 FrontRight
        surge + sway + yaw,    # 1 FrontLeft
       -surge - sway + yaw,    # 2 BackRight
       -surge + sway - yaw,    # 3 BackLeft
    ]
    vertical = [
        heave + roll - pitch,  # 4 DiveFrontRight
        heave - roll - pitch,  # 5 DiveFrontLeft
        heave + roll + pitch,  # 6 DiveBackRight
        heave - roll + pitch,  # 7 DiveBackLeft
    ]
    efforts = _normalise_group(horizontal) + _normalise_group(vertical)
    return [_effort_to_omega(value) for value in efforts]
