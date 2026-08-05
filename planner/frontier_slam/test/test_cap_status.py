"""Tests for tsdf_mapper.py's capped-view HUD line — no rclpy init, no VDBFusion.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
from frontier_slam.tsdf_mapper import _cap_status


def test_empty_when_nothing_dropped():
    assert _cap_status(100, 100) == ''


def test_empty_on_empty_map():
    assert _cap_status(0, 0) == ''


def test_reports_the_counts_when_thinned():
    text = _cap_status(40, 100)
    assert '40 of 100' in text
    assert '40%' in text


def test_text_says_the_map_is_complete():
    """A thinned view must not read as a mapping failure."""
    text = _cap_status(40, 100).lower()
    assert 'complete' in text
    assert 'display limit' in text


def test_single_line_so_the_hud_row_stays_one_row():
    assert '\n' not in _cap_status(40, 100)
