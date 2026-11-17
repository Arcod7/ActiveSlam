"""Consistency statistics, in particular the guard that keeps a degenerate
reported covariance out of the ANEES accumulator."""
import numpy as np
import pytest

from eval_tools.consistency import (
    MIN_XYH_VARIANCE, NEES_DOF, covariance_rejection, normalised_squared_error,
    robust_anees)

HEALTHY_COV = np.diag([2.0e-3, 2.0e-3, 4.0e-4])


def test_healthy_covariance_is_accepted():
    assert covariance_rejection(HEALTHY_COV) is None


def test_collapse_along_one_axis_is_rejected():
    # det stays within a factor of the healthy case, so a determinant-based
    # gate would pass this; NEES along the collapsed axis would not.
    cov = np.diag([2.0e-3, 2.0e-3, 2.2e-9])
    assert covariance_rejection(cov) is not None


def test_non_positive_definite_is_rejected():
    assert covariance_rejection(np.diag([2.0e-3, 2.0e-3, -1.0e-5])) is not None
    assert covariance_rejection(np.zeros((3, 3))) is not None


def test_non_finite_is_rejected():
    cov = HEALTHY_COV.copy()
    cov[0, 0] = np.nan
    assert covariance_rejection(cov) is not None


def test_variance_just_above_the_floor_is_accepted():
    cov = np.diag([MIN_XYH_VARIANCE * 10, MIN_XYH_VARIANCE * 10,
                   MIN_XYH_VARIANCE * 10])
    assert covariance_rejection(cov) is None


def test_normalised_squared_error_refuses_a_degenerate_covariance():
    err = np.array([0.116, 0.0, 0.0])
    collapsed = np.diag([2.2e-9, 2.0e-3, 4.0e-4])
    # Invertible, so the old LinAlgError guard let this through at ~6e6.
    assert np.linalg.det(collapsed) > 0.0
    assert normalised_squared_error(err, collapsed) is None
    assert normalised_squared_error(err, HEALTHY_COV) == pytest.approx(6.728, rel=1e-3)


def test_robust_anees_recovers_three_for_chi_square_samples():
    rng = np.random.default_rng(0)
    samples = rng.chisquare(NEES_DOF, size=4000)
    assert robust_anees(samples) == pytest.approx(3.0, abs=0.15)


def test_robust_anees_survives_an_outlier_the_mean_does_not():
    rng = np.random.default_rng(0)
    samples = list(rng.chisquare(NEES_DOF, size=2400))
    samples[400] = 6.09e6          # the collapse observed in run 20260726_205928
    assert float(np.mean(samples)) > 2000.0
    assert robust_anees(samples) == pytest.approx(3.0, abs=0.2)


def test_robust_anees_is_none_without_samples():
    assert robust_anees([]) is None
