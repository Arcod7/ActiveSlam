"""Stateless helpers for the waypoint controller.

Thruster mixing convention (BlueROV2 vectored, NED body frame, scn-file order):
  Index  Name        Formula
  0      FrontRight   surge - yaw
  1      FrontLeft    surge + yaw
  2      BackRight   -surge + yaw
  3      BackLeft    -surge - yaw
  4      VertFront    heave   (negative = upward thrust in Stonefish)
  5      VertBack     heave

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


def mix_thrusters(surge: float, yaw: float, heave: float) -> list:
    """Map (surge, yaw, heave) body-frame commands to 6 BlueROV2 thrusters.

    Output values are normalised so the peak magnitude never exceeds 1.0 — this
    preserves the requested surge/yaw ratio when commands would otherwise clip.
    """
    raw = [
        surge - yaw,    # 0 FrontRight
        surge + yaw,    # 1 FrontLeft
       -surge + yaw,    # 2 BackRight
       -surge - yaw,    # 3 BackLeft
        heave,          # 4 VertFront
        heave,          # 5 VertBack
    ]
    peak = max(abs(v) for v in raw)
    if peak > 1.0:
        raw = [v / peak for v in raw]
    return raw
