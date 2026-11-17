#!/usr/bin/env python3
"""Estimator-consistency metrics: does the SLAM covariance match the error?

ATE says how wrong the estimate is; these say whether the filter knows it.
Scoring is over the same XYH degrees of freedom that feed D-optimality
(pose_graph.dopt_xyh), so what is measured here is exactly the signal the
revisit trigger consumes.

  NEES  e^T Sigma^-1 e against ground truth. A consistent 3-DoF estimator
        averages 3; ANEES far above means overconfident, far below means
        conservative. Needs ground truth, so it is a simulation-only
        diagnostic and can never be an online trigger input.
  NIS   the same idea using a measurement residual instead of the true
        error, so it needs no ground truth and does work online.

Kept separate from benchmark.py so the statistics stay unit-testable without
a ROS graph.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

# Rows/cols of a ROS-order [trans|rot] 6x6 covariance holding x, y and yaw.
XYH_ROS_INDICES = [0, 1, 5]
NEES_DOF = 3

# Floor below which a reported marginal is numerical debris rather than a
# confident estimate: 1e-6 is a 1 mm / 1 mrad sigma, orders below anything the
# sonar, DVL or compass can justify.
MIN_XYH_VARIANCE = 1e-6
MAX_XYH_CONDITION = 1e8


def xyh_tangent_error(gt_pos, gt_quat, est_pos, est_quat) -> np.ndarray:
    """Estimate-to-truth error as [dx, dy, dyaw], expressed in the estimate's
    own body frame — the frame GTSAM reports its marginal covariance in."""
    R_est = Rotation.from_quat(est_quat)
    trans_body = R_est.inv().apply(np.asarray(gt_pos) - np.asarray(est_pos))
    yaw_err = (R_est.inv() * Rotation.from_quat(gt_quat)).as_rotvec()[2]
    return np.array([trans_body[0], trans_body[1], yaw_err])


def covariance_rejection(cov, min_variance: float = MIN_XYH_VARIANCE,
                         max_condition: float = MAX_XYH_CONDITION) -> "str | None":
    """Why this covariance must not produce a NEES sample, or None if it may.

    Gated on the smallest eigenvalue, not the determinant: a collapse along one
    axis leaves det almost intact while NEES, which divides by the variance
    along the error direction, blows up by the full collapse factor.
    """
    cov = np.asarray(cov, dtype=float)
    if not np.all(np.isfinite(cov)):
        return 'non-finite entries'
    eigenvalues = np.linalg.eigvalsh(0.5 * (cov + cov.T))
    smallest, largest = float(eigenvalues[0]), float(eigenvalues[-1])
    if smallest <= 0.0:
        return f'not positive definite (min eigenvalue {smallest:.3e})'
    if smallest < min_variance:
        return f'variance below floor ({smallest:.3e} < {min_variance:.3e})'
    if largest / smallest > max_condition:
        return f'ill-conditioned (condition number {largest / smallest:.3e})'
    return None


def normalised_squared_error(error, cov) -> "float | None":
    """e^T Sigma^-1 e. None when the covariance cannot support a sample: either
    singular, the normal state before the first keyframe is solved, or
    degenerate per covariance_rejection()."""
    if covariance_rejection(cov) is not None:
        return None
    error = np.asarray(error, dtype=float)
    try:
        return float(error @ np.linalg.solve(np.asarray(cov, dtype=float), error))
    except np.linalg.LinAlgError:
        return None


def robust_anees(samples, dof: int = NEES_DOF) -> "float | None":
    """ANEES estimated from the sample median rather than the mean.

    NEES is unbounded above, so the mean is not robust: one degenerate
    covariance contributes a term no realistic number of good samples can
    dilute. Rescaling the median by the chi-square median returns the same
    quantity for well-behaved samples while ignoring that tail.
    """
    samples = np.asarray(samples, dtype=float)
    if samples.size == 0:
        return None
    return float(np.median(samples) * dof / chi2.ppf(0.5, dof))


def anees_bounds(n_samples: int, dof: int = NEES_DOF,
                 alpha: float = 0.05) -> "tuple[float, float]":
    """Two-sided chi-square acceptance region for an ANEES over n samples.
    A consistent estimator lands inside; above the upper bound is
    overconfident, below the lower bound is conservative."""
    if n_samples < 1:
        return (float('nan'), float('nan'))
    total_dof = dof * n_samples
    return (chi2.ppf(alpha / 2.0, total_dof) / n_samples,
            chi2.ppf(1.0 - alpha / 2.0, total_dof) / n_samples)


def classify_anees(anees: float, n_samples: int, dof: int = NEES_DOF) -> str:
    lo, hi = anees_bounds(n_samples, dof)
    if anees > hi:
        return 'OVERCONFIDENT (reported covariance too small)'
    if anees < lo:
        return 'CONSERVATIVE (reported covariance too large)'
    return 'consistent'
