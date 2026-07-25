"""Pure-math tests for tsdf_mapper.py's no-return free-space carving — no
rclpy, no ROS init, no VDBFusion.

Run: python3 -m pytest planner/frontier_slam/test/ -q
"""
import numpy as np
import pytest

from frontier_slam.tsdf_mapper import (
    fit_pinhole_intrinsics, synth_no_return_points,
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
    pts = synth_no_return_points(_organized_cloud(invalid_pixels=[px]), WIDTH, 16.0)
    assert len(pts) == 1
    assert np.linalg.norm(pts[0]) == pytest.approx(16.0)
    d = pts[0] / np.linalg.norm(pts[0])
    assert d[2] > 0.99
    assert abs(d[0]) < 0.05 and abs(d[1]) < 0.05


def test_corner_pixel_rays_match_the_intrinsics():
    corners = [_pixel(0, 0), _pixel(WIDTH - 1, 0),
               _pixel(0, HEIGHT - 1), _pixel(WIDTH - 1, HEIGHT - 1)]
    pts = synth_no_return_points(
        _organized_cloud(invalid_pixels=corners), WIDTH, 16.0)
    assert len(pts) == 4
    for idx, p in zip(corners, pts):
        u, v = idx % WIDTH, idx // WIDTH
        expected = np.array([(u - CX) / FX, (v - CY) / FY, 1.0])
        expected *= 16.0 / np.linalg.norm(expected)
        np.testing.assert_allclose(p, expected, rtol=1e-6)
        assert np.linalg.norm(p) == pytest.approx(16.0)


def test_all_pixels_valid_yields_no_pseudo_points():
    assert synth_no_return_points(_organized_cloud(), WIDTH, 16.0).shape == (0, 3)


def test_nan_only_input_yields_no_pseudo_points():
    xyz = np.full((HEIGHT * WIDTH, 3), np.nan)
    assert synth_no_return_points(xyz, WIDTH, 16.0).shape == (0, 3)
    assert fit_pinhole_intrinsics(xyz, WIDTH) is None


def test_too_few_valid_pixels_to_fit_is_not_an_error():
    xyz = np.full((HEIGHT * WIDTH, 3), np.nan)
    xyz[_pixel(4, 4)] = [0.1, 0.2, 5.0]
    assert synth_no_return_points(xyz, WIDTH, 16.0).shape == (0, 3)


def test_empty_cloud_yields_no_pseudo_points():
    assert synth_no_return_points(np.empty((0, 3)), WIDTH, 16.0).shape == (0, 3)
    assert synth_no_return_points(None, WIDTH, 16.0).shape == (0, 3)


def test_carve_range_scales_every_ray():
    px = [_pixel(3, 3), _pixel(28, 12)]
    near = synth_no_return_points(_organized_cloud(invalid_pixels=px), WIDTH, 8.0)
    far = synth_no_return_points(_organized_cloud(invalid_pixels=px), WIDTH, 16.0)
    np.testing.assert_allclose(far, near * 2.0, rtol=1e-9)
