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
