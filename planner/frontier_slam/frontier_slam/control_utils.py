"""Stateless helpers for the motion controllers.

Thruster mixing convention (BlueROV2 Heavy, NED body frame, scn-file order):
  Index  Name        Formula
  0      FrontRight   surge - sway - yaw
  1      FrontLeft    surge + sway + yaw
  2      BackRight   -surge - sway + yaw
  3      BackLeft    -surge + sway - yaw
  4      DiveFrontRight  heave   (negative = upward thrust in Stonefish)
  5      DiveFrontLeft   heave
  6      DiveBackRight   heave
  7      DiveBackLeft    heave

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


def wrap_angle(a: float) -> float:
    """Wrap any angle into [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def mix_thrusters(surge: float, yaw: float, heave: float, sway: float = 0.0) -> list:
    """Map body commands to the 8 Stonefish BlueROV2 Heavy thrusters.

    Output values are normalised so the peak magnitude never exceeds 1.0 — this
    preserves the requested command ratios when they would otherwise clip.
    Positive sway = starboard (strafe right); defaults to 0 so existing
    surge/yaw/heave call sites are unchanged.
    """
    raw = [
        surge - sway - yaw,    # 0 FrontRight
        surge + sway + yaw,    # 1 FrontLeft
       -surge - sway + yaw,    # 2 BackRight
       -surge + sway - yaw,    # 3 BackLeft
        heave,                 # 4 DiveFrontRight
        heave,                 # 5 DiveFrontLeft
        heave,                 # 6 DiveBackRight
        heave,                 # 7 DiveBackLeft
    ]
    peak = max(abs(v) for v in raw)
    if peak > 1.0:
        raw = [v / peak for v in raw]
    return raw
