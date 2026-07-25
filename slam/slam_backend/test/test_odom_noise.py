"""Pure tests for the profile-derived per-edge odometry sigma."""
import pytest

from slam_backend.odom_noise import DEFAULT_FLOOR_M, odom_trans_sigma
from slam_backend.sensor_models.noise_profiles import load_noise_profile, NoiseProfile

REALISTIC = 'config/noise_realistic.yaml'
DEGRADED = 'config/noise_degraded.yaml'


def dvl(path):
    return load_noise_profile(path).dvl


def test_sigma_grows_with_edge_length():
    """A longer edge accumulates more DVL error -- the constant could not say so."""
    d = dvl(REALISTIC)
    assert odom_trans_sigma(3.0, 3.0, d) > odom_trans_sigma(1.0, 1.0, d)


def test_sigma_grows_with_dwell_time_at_fixed_distance():
    """Bias is linear in time, so a slow edge is worse than a fast one."""
    d = dvl(REALISTIC)
    assert odom_trans_sigma(1.0, 8.0, d) > odom_trans_sigma(1.0, 1.0, d)


def test_degraded_profile_raises_sigma():
    """The whole point: the model must respond to the profile. The constant
    it replaces gave an identical sigma for both."""
    fast = odom_trans_sigma(1.0, 1.0, dvl(REALISTIC))
    slow = odom_trans_sigma(1.0, 1.0, dvl(DEGRADED))
    assert slow > 3.0 * fast


def test_typical_edge_is_tighter_than_the_old_constant():
    """Measured chi2/dof of 0.06-0.22 says 0.02 m was 2-4x too loose."""
    assert odom_trans_sigma(1.0, 1.0, dvl(REALISTIC)) < 0.02 / 2.0


@pytest.mark.parametrize('dist, dt', [(0.0, 1.0), (1.0, 0.0), (1.0, -1.0),
                                      (float('nan'), 1.0), (-1.0, 1.0)])
def test_degenerate_edges_fall_back_to_the_floor(dist, dt):
    """A zero-length or zero-duration edge must not produce a zero sigma:
    gtsam treats that as an exact constraint."""
    assert odom_trans_sigma(dist, dt, dvl(REALISTIC)) == DEFAULT_FLOOR_M


def test_ideal_profile_still_has_a_positive_sigma():
    """Ideal zeroes every DVL term; the factor still must not be exact."""
    assert odom_trans_sigma(1.0, 1.0, NoiseProfile().dvl) > 0.0
