"""Tests for the selected-wall marker both motion controllers publish — the
link from the vehicle to the map point its heading is taken from — and for
wall_oriented's nearest-point lookup that feeds it.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from frontier_slam.wall_markers import selected_wall_markers
from frontier_slam.wall_oriented_controller import (
    nearest_wall_point,
    wall_side_distances,
)


def _markers(wall_point, pose=(1.0, 2.0, 3.0), voxel_size=0.2):
    msg = selected_wall_markers(
        None if pose is None else np.array(pose, dtype=float),
        wall_point, voxel_size, Time())
    link, cube = msg.markers
    return link, cube


_WALL = np.array([4.37, -2.02, 3.11])


def test_the_link_runs_from_the_vehicle_to_the_chosen_voxel():
    link, _ = _markers(_WALL)

    assert link.type == Marker.LINE_LIST
    assert link.action == Marker.ADD
    start, end = link.points
    assert (start.x, start.y, start.z) == (1.0, 2.0, 3.0)
    # (4.37, -2.02, 3.11) snapped to the 0.2 m grid centre.
    assert (round(end.x, 6), round(end.y, 6), round(end.z, 6)) == (4.3, -2.1, 3.1)


def test_the_highlight_covers_the_whole_voxel_not_the_sample_point():
    _, cube = _markers(_WALL)

    assert cube.type == Marker.CUBE
    assert (cube.scale.x, cube.scale.y, cube.scale.z) == (0.2, 0.2, 0.2)
    assert (round(cube.pose.position.x, 6),
            round(cube.pose.position.y, 6),
            round(cube.pose.position.z, 6)) == (4.3, -2.1, 3.1)


def test_the_highlight_follows_the_configured_voxel_size():
    _, cube = _markers(_WALL, voxel_size=0.5)

    assert cube.scale.x == 0.5
    assert round(cube.pose.position.x, 6) == 4.25   # floor(4.37/0.5)+0.5 -> 8.5*0.5


def test_both_markers_are_transparent_so_the_voxel_stays_visible():
    link, cube = _markers(_WALL)

    assert 0.0 < link.color.a < 1.0
    assert link.color.b > link.color.r   # blue-dominant
    assert (cube.color.r, cube.color.g, cube.color.b, cube.color.a) == (
        link.color.r, link.color.g, link.color.b, link.color.a)


def test_no_wall_deletes_the_link_instead_of_leaving_it_standing():
    link, cube = _markers(None)

    assert link.action == cube.action == Marker.DELETE


def test_no_pose_also_deletes_it():
    link, cube = _markers(_WALL, pose=None)

    assert link.action == cube.action == Marker.DELETE


def test_the_two_markers_use_separate_namespaces():
    link, cube = _markers(_WALL)

    assert link.ns != cube.ns
    assert link.header.frame_id == cube.header.frame_id == 'world_ned'


# ----------------------------------------------------------------------
# wall_oriented: the point behind the side distance it steers on.
# Route heading 0 (+X/north), so +Y is starboard/right and -Y is left.
_POSE = np.array([0.0, 0.0, 0.0])
_CLOUD = np.array([
    [1.0, -3.0, 0.0],   # left, 3.16 m
    [2.0, -8.0, 0.0],   # left, further
    [1.0,  5.0, 0.0],   # right, 5.10 m
    [0.5,  0.0, 0.0],   # inside the deadband — no usable side
    [1.0, -1.0, 9.0],   # left but far outside the depth band
])


def _nearest(side, **kwargs):
    return nearest_wall_point(_CLOUD, _POSE, 0.0, 1.5, 20.0, side, **kwargs)


def test_the_marked_point_is_the_one_the_steered_distance_came_from():
    left, right = wall_side_distances(_CLOUD, _POSE, 0.0, 1.5, 20.0)

    assert np.allclose(_nearest(-1), [1.0, -3.0, 0.0])
    assert np.hypot(*_nearest(-1)[:2]) == left
    assert np.hypot(*_nearest(1)[:2]) == right


def test_the_depth_band_excludes_the_same_points_for_both():
    # The 9 m-deep left point is nearer in XY than the one that is picked.
    assert _nearest(-1)[2] == 0.0


def test_centreline_points_give_no_side_and_are_never_marked():
    assert not np.allclose(_nearest(-1), _CLOUD[3])
    assert not np.allclose(_nearest(1), _CLOUD[3])


def test_holding_no_side_marks_nothing():
    assert _nearest(0) is None


def test_an_empty_side_marks_nothing():
    only_left = _CLOUD[:2]

    assert nearest_wall_point(only_left, _POSE, 0.0, 1.5, 20.0, 1) is None


def test_out_of_range_walls_mark_nothing():
    assert nearest_wall_point(_CLOUD, _POSE, 0.0, 1.5, 1.0, -1) is None


def test_no_cloud_marks_nothing():
    assert nearest_wall_point(None, _POSE, 0.0, 1.5, 20.0, -1) is None
