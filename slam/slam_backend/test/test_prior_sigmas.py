"""Pure tests for the attitude+depth prior's yaw sigma substitution."""
import math

import numpy as np
import pytest

from slam_backend.pose_graph import PRIOR_YAW_INDEX, prior_sigmas_with_yaw

BASE = np.array([0.02, 0.02, 0.01, 1e3, 1e3, 0.02])


def test_reported_variance_replaces_the_profile_yaw_sigma():
    """The filter's posterior, not the IMU spec, ends up in the prior."""
    out = prior_sigmas_with_yaw(BASE, 0.0292 ** 2)
    assert out[PRIOR_YAW_INDEX] == pytest.approx(0.0292)


def test_other_axes_are_untouched():
    """Roll/pitch/depth are direct measurements and keep their profile sigmas."""
    out = prior_sigmas_with_yaw(BASE, 0.04 ** 2)
    kept = [i for i in range(6) if i != PRIOR_YAW_INDEX]
    assert np.array_equal(out[kept], BASE[kept])


@pytest.mark.parametrize('yaw_var', [0.0, -1.0, math.inf, math.nan])
def test_missing_covariance_falls_back_to_the_profile_default(yaw_var):
    """Odometry without a covariance must not zero out or blow up the prior."""
    assert np.array_equal(prior_sigmas_with_yaw(BASE, yaw_var), BASE)


def test_base_sigmas_are_not_mutated():
    """The node reuses one base array for every keyframe."""
    before = BASE.copy()
    prior_sigmas_with_yaw(BASE, 0.05 ** 2)
    assert np.array_equal(BASE, before)
