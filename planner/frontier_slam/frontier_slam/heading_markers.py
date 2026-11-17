"""RViz arrows for the heading a motion controller is asking the vehicle to hold.

Every other display shows where the vehicle went. This shows what it was told:
the commanded viewing heading next to the heading actually held, anchored at the
vehicle so the gap between them is the tracking error, live.

Only the closed-loop heading law has a setpoint to draw. An open-loop spin (the
initial scan) asks for a rate, not a heading, so it passes look_heading=None and
the commanded arrow is deleted rather than left pointing at a stale setpoint.
"""
import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

TOPIC = '/motion/commanded_heading'
# Amber reads as the request, grey as the state, against the blue the map and
# /motion/selected_wall already occupy.
COMMANDED_RGBA = (1.00, 0.55, 0.00, 0.90)
CURRENT_RGBA = (0.90, 0.90, 0.90, 0.70)
# Both arrows share one length so the angle between them is the only difference
# the eye has to read.
ARROW_LEN_M = 3.0
SHAFT_D_M = 0.12
HEAD_D_M = 0.28
HEAD_LEN_M = 0.45


def _arrow(ns: str, rgba: tuple, stamp, frame: str) -> Marker:
    r, g, b, a = rgba
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame
    m.ns = ns
    m.id = 0
    m.type = Marker.ARROW
    m.lifetime = Duration(sec=1)
    m.color = ColorRGBA(r=r, g=g, b=b, a=a)
    m.pose.orientation.w = 1.0
    m.scale.x = SHAFT_D_M
    m.scale.y = HEAD_D_M
    m.scale.z = HEAD_LEN_M
    return m


def _lay(marker: Marker, pose, heading: float) -> None:
    """Point the arrow along `heading` in the horizontal plane through the pose."""
    marker.action = Marker.ADD
    marker.points = [
        Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])),
        Point(
            x=float(pose[0] + ARROW_LEN_M * math.cos(heading)),
            y=float(pose[1] + ARROW_LEN_M * math.sin(heading)),
            z=float(pose[2]),
        ),
    ]


def heading_markers(pose, look_heading, yaw, stamp,
                    frame: str = 'world_ned') -> MarkerArray:
    """Commanded viewing heading and held heading, both from the vehicle.

    `look_heading` is the setpoint the yaw law is closing on and `yaw` the
    heading the estimate reports; both in radians, ENU/REP-103 like the rest of
    the controller. A missing pose deletes both, so neither arrow can outlive
    the decision it explains.
    """
    commanded = _arrow('commanded_heading', COMMANDED_RGBA, stamp, frame)
    current = _arrow('current_heading', CURRENT_RGBA, stamp, frame)

    if pose is None:
        commanded.action = current.action = Marker.DELETE
        return MarkerArray(markers=[commanded, current])

    if look_heading is None or not math.isfinite(look_heading):
        commanded.action = Marker.DELETE
    else:
        _lay(commanded, pose, look_heading)

    if yaw is None or not math.isfinite(yaw):
        current.action = Marker.DELETE
    else:
        _lay(current, pose, yaw)

    return MarkerArray(markers=[commanded, current])
