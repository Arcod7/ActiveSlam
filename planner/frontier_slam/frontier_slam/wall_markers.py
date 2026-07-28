"""RViz marker for the wall a motion controller is steering by.

Shared by wall_looking (nearest surface sample) and wall_oriented_controller
(nearest point on the chosen side) so one /motion/selected_wall display answers
the same question whichever executor is running: every other display says where
the vehicle went, this says which piece of map it is pointing at.
"""
import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

TOPIC = '/motion/selected_wall'
# Transparent, so the voxel it marks stays visible underneath.
SELECTED_WALL_RGBA = (0.20, 0.55, 1.00, 0.45)
LINK_WIDTH_M = 0.04


def selected_wall_markers(pose, wall_point, voxel_size: float, stamp,
                          frame: str = 'world_ned') -> MarkerArray:
    """Link the vehicle to the voxel its heading is taken from, plus that voxel.

    The wall point is a cloud sample, so it is snapped to the mapper's grid: the
    highlight then covers the cube visible in /tsdf/voxels rather than floating
    on its face. A missing pose or wall yields DELETEs, so the link cannot
    outlive the decision it explains.
    """
    r, g, b, a = SELECTED_WALL_RGBA
    link, cube = Marker(), Marker()
    for m, ns in ((link, 'selected_wall_link'), (cube, 'selected_wall_cube')):
        m.header.stamp    = stamp
        m.header.frame_id = frame
        m.ns       = ns
        m.id       = 0
        m.lifetime = Duration(sec=1)
        m.color    = ColorRGBA(r=r, g=g, b=b, a=a)
        m.pose.orientation.w = 1.0
    link.type = Marker.LINE_LIST
    cube.type = Marker.CUBE

    if wall_point is None or pose is None:
        link.action = cube.action = Marker.DELETE
        return MarkerArray(markers=[link, cube])

    centre = (np.floor(np.asarray(wall_point, dtype=np.float64) / voxel_size)
              + 0.5) * voxel_size
    link.action = cube.action = Marker.ADD
    link.scale.x = LINK_WIDTH_M
    link.points = [
        Point(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])),
        Point(x=float(centre[0]), y=float(centre[1]), z=float(centre[2])),
    ]
    cube.pose.position.x = float(centre[0])
    cube.pose.position.y = float(centre[1])
    cube.pose.position.z = float(centre[2])
    cube.scale.x = cube.scale.y = cube.scale.z = voxel_size
    return MarkerArray(markers=[link, cube])
