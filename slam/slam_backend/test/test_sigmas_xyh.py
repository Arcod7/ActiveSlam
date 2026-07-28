"""The per-axis split of the same XYH marginal D-optimality scores."""
import numpy as np
import pytest

from slam_backend.pose_graph import XYH_INDICES, dopt_xyh, sigmas_xyh


def _cov(sigma_x, sigma_y, sigma_yaw, xy_corr=0.0):
    """A GTSAM-order 6x6 whose XYH block carries the given sigmas."""
    cov = np.eye(6) * 1e-6
    block = np.array([
        [sigma_x ** 2, xy_corr * sigma_x * sigma_y, 0.0],
        [xy_corr * sigma_x * sigma_y, sigma_y ** 2, 0.0],
        [0.0, 0.0, sigma_yaw ** 2],
    ])
    cov[np.ix_(XYH_INDICES, XYH_INDICES)] = block
    return cov


def test_isotropic_covariance_returns_its_own_sigmas():
    sigma_xy, sigma_yaw = sigmas_xyh(_cov(0.3, 0.3, 0.07))
    assert sigma_xy == pytest.approx(0.3)
    assert sigma_yaw == pytest.approx(0.07)


def test_anisotropic_xy_collapses_to_the_equal_area_radius():
    """det^(1/4) of the 2x2, i.e. the geometric mean of the two axes."""
    sigma_xy, _ = sigmas_xyh(_cov(0.4, 0.1, 0.05))
    assert sigma_xy == pytest.approx(np.sqrt(0.4 * 0.1))


def test_the_split_reproduces_the_dopt_it_came_from():
    """sigma_xy**4 * sigma_yaw**2 == dopt**3, so the two numbers on the HUD
    and the scalar the trigger reads cannot disagree."""
    cov = _cov(0.4, 0.1, 0.05)
    sigma_xy, sigma_yaw = sigmas_xyh(cov)
    assert sigma_xy ** 4 * sigma_yaw ** 2 == pytest.approx(dopt_xyh(cov) ** 3)


def test_correlated_xy_shrinks_the_reported_sigma():
    """A correlated ellipse encloses less area than its per-axis sigmas
    suggest, and the equal-area radius follows it down."""
    independent, _ = sigmas_xyh(_cov(0.3, 0.3, 0.05))
    correlated, _ = sigmas_xyh(_cov(0.3, 0.3, 0.05, xy_corr=0.8))
    assert correlated < independent
    assert correlated == pytest.approx(0.3 * (1.0 - 0.8 ** 2) ** 0.25)


def test_degenerate_covariance_is_zero_not_nan():
    sigma_xy, sigma_yaw = sigmas_xyh(_cov(0.0, 0.0, 0.0))
    assert sigma_xy == 0.0
    assert sigma_yaw == 0.0
