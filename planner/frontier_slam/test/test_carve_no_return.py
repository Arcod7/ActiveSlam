"""Pure-math tests for tsdf_mapper.py's no-return free-space carving — no
rclpy, no ROS init, no VDBFusion.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np
import pytest

from frontier_slam.tsdf_mapper import (
    fit_pinhole_intrinsics, free_ray_voxels, no_return_ray_dirs,
)

WIDTH, HEIGHT = 32, 16
FX, CX, FY, CY = 24.0, 15.5, 20.0, 7.5


def _organized_cloud(depth=5.0, invalid_pixels=()):
    """(H*W, 3) organized cloud from the reference intrinsics above."""
    v, u = np.meshgrid(np.arange(HEIGHT), np.arange(WIDTH), indexing='ij')
    u, v = u.ravel().astype(float), v.ravel().astype(float)
    xyz = np.column_stack([(u - CX) / FX * depth, (v - CY) / FY * depth,
                           np.full(u.shape, depth)])
    for px in invalid_pixels:
        xyz[px] = np.nan
    return xyz


def _pixel(u, v):
    return v * WIDTH + u


def test_intrinsics_recovered_from_valid_pixels():
    fx, cx, fy, cy = fit_pinhole_intrinsics(_organized_cloud(), WIDTH)
    assert fx == pytest.approx(FX, rel=1e-6)
    assert cx == pytest.approx(CX, rel=1e-6)
    assert fy == pytest.approx(FY, rel=1e-6)
    assert cy == pytest.approx(CY, rel=1e-6)


def test_centre_pixel_ray_points_straight_ahead():
    # (16, 8) is half a pixel off centre in both axes; its ray is the closest
    # thing this even-sized grid has to boresight.
    px = _pixel(16, 8)
    dirs = no_return_ray_dirs(_organized_cloud(invalid_pixels=[px]), WIDTH)
    assert len(dirs) == 1
    assert np.linalg.norm(dirs[0]) == pytest.approx(1.0)
    assert dirs[0][2] > 0.99
    assert abs(dirs[0][0]) < 0.05 and abs(dirs[0][1]) < 0.05


def test_corner_pixel_rays_match_the_intrinsics():
    corners = [_pixel(0, 0), _pixel(WIDTH - 1, 0),
               _pixel(0, HEIGHT - 1), _pixel(WIDTH - 1, HEIGHT - 1)]
    dirs = no_return_ray_dirs(_organized_cloud(invalid_pixels=corners), WIDTH)
    assert len(dirs) == 4
    for idx, d in zip(corners, dirs):
        u, v = idx % WIDTH, idx // WIDTH
        expected = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
        expected /= np.linalg.norm(expected)
        np.testing.assert_allclose(d, expected, rtol=1e-6)


def test_all_pixels_valid_yields_no_rays():
    assert no_return_ray_dirs(_organized_cloud(), WIDTH).shape == (0, 3)


def test_nan_only_input_yields_no_rays():
    xyz = np.full((HEIGHT * WIDTH, 3), np.nan)
    assert no_return_ray_dirs(xyz, WIDTH).shape == (0, 3)
    assert fit_pinhole_intrinsics(xyz, WIDTH) is None


def test_too_few_valid_pixels_to_fit_is_not_an_error():
    xyz = np.full((HEIGHT * WIDTH, 3), np.nan)
    xyz[_pixel(4, 4)] = [0.1, 0.2, 5.0]
    assert no_return_ray_dirs(xyz, WIDTH).shape == (0, 3)


def test_empty_cloud_yields_no_rays():
    assert no_return_ray_dirs(np.empty((0, 3)), WIDTH).shape == (0, 3)
    assert no_return_ray_dirs(None, WIDTH).shape == (0, 3)


def test_stride_decimates_but_keeps_directions():
    px = [_pixel(u, v) for u in (2, 3) for v in (2, 3)]
    dense = no_return_ray_dirs(_organized_cloud(invalid_pixels=px), WIDTH)
    sparse = no_return_ray_dirs(_organized_cloud(invalid_pixels=px), WIDTH, 2)
    assert len(dense) == 4
    # Only (2, 2) survives u % 2 == 0 and v % 2 == 0.
    assert len(sparse) == 1
    np.testing.assert_allclose(sparse[0], dense[0], rtol=1e-12)


# ── free_ray_voxels ─────────────────────────────────────────────────────

VS = 0.2


def test_axis_ray_carves_every_cell_up_to_max_range():
    dirs = np.array([[1.0, 0.0, 0.0]])
    voxels = free_ray_voxels(dirs, np.zeros(3), VS, 2.0)
    xs = np.unique(voxels[:, 0])
    np.testing.assert_array_equal(xs, np.arange(0, 10))
    # A straight +X ray from the origin stays in one row of cells.
    assert set(voxels[:, 1]) == {0} and set(voxels[:, 2]) == {0}


def test_nothing_is_carved_beyond_max_range():
    dirs = np.array([[1.0, 0.0, 0.0]])
    voxels = free_ray_voxels(dirs, np.zeros(3), VS, 3.0)
    assert voxels[:, 0].max() * VS < 3.0


def test_carve_is_relative_to_the_sensor_origin():
    dirs = np.array([[0.0, 0.0, 1.0]])
    origin = np.array([5.0, -3.0, 0.0])
    voxels = free_ray_voxels(dirs, origin, VS, 1.0)
    assert set(voxels[:, 0]) == {25} and set(voxels[:, 1]) == {-15}
    np.testing.assert_array_equal(np.unique(voxels[:, 2]), np.arange(0, 5))


@pytest.mark.parametrize('vec', [[1.0, 1.0, 1.0], [3.0, -1.0, 0.4]])
def test_no_cell_is_stepped_over_along_a_ray(vec):
    d = np.array([vec]) / np.linalg.norm(vec)
    voxels = free_ray_voxels(d, np.zeros(3), VS, 5.0)
    order = np.argsort(voxels @ d[0])          # walk the ray, not the axes
    # Half-voxel sampling moves <= vs/2 per step, so no index can jump by 2.
    assert np.abs(np.diff(voxels[order], axis=0)).max() == 1


def test_rows_are_unique():
    dirs = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    voxels = free_ray_voxels(dirs, np.zeros(3), VS, 2.0)
    assert len(np.unique(voxels, axis=0)) == len(voxels)


def test_degenerate_inputs_yield_no_voxels():
    assert free_ray_voxels(np.empty((0, 3)), np.zeros(3), VS, 5.0).shape == (0, 3)
    assert free_ray_voxels(np.array([[1.0, 0.0, 0.0]]),
                           np.zeros(3), VS, 0.0).shape == (0, 3)
