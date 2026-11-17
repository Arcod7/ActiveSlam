"""Tests for the blue planning-band tint on the TSDF voxel view — no rclpy, no
VDBFusion: _planning_band_voxel_mask is called unbound on a stub carrying only
the band settings it reads.

The tint marks the Z slice /projected_map is built from, so its boundary has to
stay identical to the one _build_projected_map applies; a tint that disagreed
would claim the planner sees geometry it does not.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np

from frontier_slam.tsdf_mapper import TSDFMapper


class _Mapper:
    """Only the three attributes _planning_band_voxel_mask reaches for."""

    def __init__(self, z=5.0, band=1.0, publishing=True):
        self._z = z
        self._projected_map_band = band
        self._projected_map_pub = object() if publishing else None

    def _band_center_z(self):
        return self._z


def _mask(pts, **kwargs):
    return TSDFMapper._planning_band_voxel_mask(
        _Mapper(**kwargs), np.asarray(pts, dtype=float))


def test_a_voxel_at_the_band_centre_is_tinted():
    assert _mask([[3.0, -7.0, 5.0]]).tolist() == [True]


def test_the_tint_covers_the_full_band_not_just_the_centre_plane():
    mask = _mask([[0.0, 0.0, 4.05], [0.0, 0.0, 5.0], [0.0, 0.0, 5.95]],
                 z=5.0, band=1.0)

    assert mask.tolist() == [True, True, True]


def test_the_band_edges_are_inclusive_like_the_projected_map():
    mask = _mask([[0.0, 0.0, 4.0], [0.0, 0.0, 6.0]], z=5.0, band=1.0)

    assert mask.tolist() == [True, True]


def test_voxels_above_or_below_the_band_are_left_alone():
    mask = _mask([[0.0, 0.0, 3.9], [0.0, 0.0, 5.0], [0.0, 0.0, 6.1]],
                 z=5.0, band=1.0)

    assert mask.tolist() == [False, True, False]


def test_xy_is_irrelevant_the_band_is_a_slab_not_a_column():
    mask = _mask([[-400.0, 900.0, 5.0], [0.0, 0.0, 5.0]])

    assert mask.tolist() == [True, True]


def test_a_wider_band_tints_more_of_the_map():
    pts = [[0.0, 0.0, 7.5]]

    assert _mask(pts, z=5.0, band=1.0) is None
    assert _mask(pts, z=5.0, band=3.0).tolist() == [True]


def test_nothing_is_tinted_without_a_projected_map():
    # No projected map means no planning band — the tint would be inventing one.
    assert _mask([[0.0, 0.0, 5.0]], publishing=False) is None


def test_nothing_is_tinted_before_the_band_centre_resolves():
    assert _mask([[0.0, 0.0, 5.0]], z=None) is None


def test_an_empty_voxel_view_reports_no_mask():
    assert TSDFMapper._planning_band_voxel_mask(
        _Mapper(), np.empty((0, 3))) is None
