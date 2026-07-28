"""Pure-math tests for tsdf_mapper.py's directional-TSDF binning and merge —
no rclpy, no ROS init, no VDBFusion.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np
import pytest

from frontier_slam.tsdf_mapper import _direction_bins, _merge_voxel_parts

PLUS_X, MINUS_X, PLUS_Y, MINUS_Y, PLUS_Z, MINUS_Z = range(6)


def test_axis_aligned_rays_land_in_their_own_bin():
    dirs = np.array([[5., 0, 0], [-5, 0, 0], [0, 5, 0],
                     [0, -5, 0], [0, 0, 5], [0, 0, -5]])
    assert _direction_bins(dirs).tolist() == [
        PLUS_X, MINUS_X, PLUS_Y, MINUS_Y, PLUS_Z, MINUS_Z]


def test_bin_follows_the_dominant_axis_not_the_largest_coordinate():
    # Mostly -Y despite non-zero X and Z components.
    assert _direction_bins(np.array([[2., -7, 1]])).tolist() == [MINUS_Y]


def test_opposite_views_of_one_surface_never_share_a_bin():
    """The whole point: two faces of a sheet must not average together."""
    surface = np.array([[10., 4, -2]])
    from_west = _direction_bins(surface - np.array([0., 4, -2]))
    from_east = _direction_bins(surface - np.array([20., 4, -2]))
    assert from_west[0] != from_east[0]


def test_merge_keeps_the_most_solid_bin_for_a_disputed_voxel():
    free = (np.array([[1, 2, 3]]), np.float32([0.5]), np.float32([9]))
    solid = (np.array([[1, 2, 3]]), np.float32([-0.4]), np.float32([3]))
    keys, d_vals, w_vals = _merge_voxel_parts([free, solid])
    assert keys.tolist() == [[1, 2, 3]]
    assert d_vals.tolist() == pytest.approx([-0.4])
    # The winning bin's weight travels with its d, not the loser's.
    assert w_vals.tolist() == [3.0]


def test_merge_unions_voxels_only_one_bin_saw():
    a = (np.array([[0, 0, 0]]), np.float32([-0.1]), np.float32([4]))
    b = (np.array([[9, 9, 9]]), np.float32([-0.2]), np.float32([5]))
    keys, _d, _w = _merge_voxel_parts([a, b])
    assert sorted(keys.tolist()) == [[0, 0, 0], [9, 9, 9]]


def test_merge_skips_empty_bins_and_passes_a_lone_bin_through():
    empty = (None, None, None)
    only = (np.array([[7, 7, 7]]), np.float32([-0.3]), np.float32([2]))
    assert _merge_voxel_parts([empty, empty]) == (None, None, None)
    keys, d_vals, _w = _merge_voxel_parts([empty, only, empty])
    assert keys.tolist() == [[7, 7, 7]]
    assert d_vals.tolist() == pytest.approx([-0.3])


def test_merge_accepts_float_world_centres_as_keys():
    """_extract_voxels returns world centres, not index coords; both merge."""
    a = (np.float32([[0.1, 0.2, 0.3]]), np.float32([0.2]), np.float32([6]))
    b = (np.float32([[0.1, 0.2, 0.3]]), np.float32([-0.5]), np.float32([1]))
    keys, d_vals, _w = _merge_voxel_parts([a, b])
    assert len(keys) == 1
    assert d_vals.tolist() == pytest.approx([-0.5])
