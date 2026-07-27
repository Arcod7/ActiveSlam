"""Tests for tsdf_mapper.py's capped-view banner — no rclpy init, no VDBFusion.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from frontier_slam.tsdf_mapper import _cap_banner

HEADER = Header(frame_id='world_ned')
PTS = np.array([[0.0, 0.0, 2.0],
                [4.0, 2.0, 6.0],
                [2.0, 4.0, 4.0]], dtype=np.float32)


def test_deletes_itself_when_nothing_dropped():
    m = _cap_banner(len(PTS), len(PTS), PTS, HEADER)
    assert m.action == Marker.DELETE


def test_deletes_itself_on_empty_map():
    m = _cap_banner(0, 0, None, HEADER)
    assert m.action == Marker.DELETE


def test_adds_banner_when_thinned():
    m = _cap_banner(40, 100, PTS, HEADER)
    assert m.action == Marker.ADD
    assert m.type == Marker.TEXT_VIEW_FACING
    assert '40 of 100' in m.text
    assert '40%' in m.text


def test_text_says_the_map_is_complete():
    """A thinned view must not read as a mapping failure."""
    text = _cap_banner(40, 100, PTS, HEADER).text.lower()
    assert 'complete' in text
    assert 'display limit' in text


def test_own_namespace_so_it_never_clobbers_the_cube_list():
    assert _cap_banner(40, 100, PTS, HEADER).ns != 'tsdf_voxels'


def test_sits_above_the_map_centroid_in_ned():
    """world_ned is +Z down, so the banner goes below min Z to render on top."""
    m = _cap_banner(40, 100, PTS, HEADER)
    assert m.pose.position.x == np.float32(PTS[:, 0].mean())
    assert m.pose.position.y == np.float32(PTS[:, 1].mean())
    assert m.pose.position.z < float(PTS[:, 2].min())


def test_carries_the_caller_frame():
    m = _cap_banner(40, 100, PTS, HEADER)
    assert m.header.frame_id == 'world_ned'
