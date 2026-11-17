"""Pure tests for the profile-derived per-edge odometry sigma."""
import pytest

from slam_backend.odom_noise import (
    DEFAULT_FLOOR_M, fused_yaw_stats, odom_trans_sigma)
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


def test_coherent_mode_matches_default_on_the_first_edge():
    """With both cumulative arms starting at zero, (before, after) = (0, dist/dt)
    -- the coherent formula must reduce to the plain per-edge one."""
    d = dvl(DEGRADED)
    default = odom_trans_sigma(2.0, 1.5, d)
    coherent = odom_trans_sigma(2.0, 1.5, d, cum_dist_m=(0.0, 2.0), cum_time_s=(0.0, 1.5))
    assert coherent == pytest.approx(default)


def test_coherent_total_variance_over_a_run_matches_the_closed_form():
    """The whole point: summing every edge's coherent variance across a run
    must telescope to scale_error_pct^2 * D_total^2 (and bias_m_s^2 *
    T_total^2), not the much smaller sum of independent per-edge terms."""
    d = dvl(DEGRADED)
    n_edges = 50
    step_dist, step_dt = 1.0, 1.0
    cum_d = cum_t = 0.0
    total_var = 0.0
    for _ in range(n_edges):
        d0, t0 = cum_d, cum_t
        cum_d += step_dist
        cum_t += step_dt
        sigma = odom_trans_sigma(step_dist, step_dt, d,
                                 cum_dist_m=(d0, cum_d), cum_time_s=(t0, cum_t))
        # Random-walk term is independent per edge and small at this rate;
        # isolate the coherent contribution by subtracting it back out.
        rw_var = (max(d.sigma_floor_m_s, d.sigma_pct * (step_dist / step_dt))
                  * (step_dt / d.publish_rate_hz) ** 0.5) ** 2
        total_var += sigma ** 2 - rw_var
    expected_scale_var = (d.scale_error_pct * cum_d) ** 2
    expected_bias_var = (d.bias_m_s * cum_t) ** 2
    assert total_var == pytest.approx(expected_scale_var + expected_bias_var, rel=1e-9)


def test_coherent_mode_reports_larger_sigma_deep_into_a_run():
    """This is the fix for fig3_trigger.png: independent per-edge accounting
    (the default) barely grows over a run because it treats one run-long
    error as N small unrelated ones; the coherent accounting must not."""
    d = dvl(DEGRADED)
    cum_d = cum_t = 0.0
    default_sigma = coherent_sigma = None
    for _ in range(200):
        d0, t0 = cum_d, cum_t
        cum_d += 1.0
        cum_t += 1.0
        default_sigma = odom_trans_sigma(1.0, 1.0, d)
        coherent_sigma = odom_trans_sigma(1.0, 1.0, d,
                                          cum_dist_m=(d0, cum_d), cum_time_s=(t0, cum_t))
    assert coherent_sigma > 5.0 * default_sigma


# --- heading-driven cross-track, the term the DVL-only model omitted ---

def profile(path):
    return load_noise_profile(path)


def test_fused_yaw_stats_matches_the_filter_fixed_point():
    """Solved from the profile, not fitted: a 300-trial Monte Carlo of the
    YawKalmanFilter on the degraded profile measured 0.1333 rad and 0.140 s."""
    p = profile(DEGRADED)
    sigma, tau = fused_yaw_stats(p.imu, p.compass)
    assert sigma == pytest.approx(0.1333, rel=0.15)
    assert tau == pytest.approx(0.140, rel=0.20)


def test_fused_yaw_stats_responds_to_the_profile():
    realistic, _ = fused_yaw_stats(profile(REALISTIC).imu, profile(REALISTIC).compass)
    degraded, _ = fused_yaw_stats(profile(DEGRADED).imu, profile(DEGRADED).compass)
    assert degraded > 3.0 * realistic


def test_attitude_term_dominates_a_typical_edge():
    """The finding that motivated it: on the degraded profile the heading term
    is the largest contributor, not any of the three DVL terms."""
    p = profile(DEGRADED)
    sigma, tau = fused_yaw_stats(p.imu, p.compass)
    dvl_only = odom_trans_sigma(1.0, 2.0, p.dvl)
    with_yaw = odom_trans_sigma(1.0, 2.0, p.dvl, yaw_sigma_rad=sigma,
                                yaw_corr_time_s=tau)
    assert with_yaw > 2.0 * dvl_only


def test_yaw_walk_is_additive_across_a_split_edge():
    """The cross-track walk carries no cumulative arm, so splitting an edge in
    two must conserve its variance -- that is what makes it safe per edge."""
    p = profile(DEGRADED)
    sigma, tau = fused_yaw_stats(p.imu, p.compass)
    kw = dict(yaw_sigma_rad=sigma, yaw_corr_time_s=tau)
    whole = odom_trans_sigma(2.0, 2.0, p.dvl, **kw) ** 2
    halves = 2 * odom_trans_sigma(1.0, 1.0, p.dvl, **kw) ** 2
    rw = 2 * odom_trans_sigma(1.0, 1.0, p.dvl) ** 2 - odom_trans_sigma(2.0, 2.0, p.dvl) ** 2
    assert whole == pytest.approx(halves - rw, rel=1e-9)


# --- each systematic term against the arm it actually accumulates on ---

def test_scale_term_uses_displacement_not_arc_length():
    """A path that doubles back has far more arc than displacement; charging the
    scale error against arc overstates it by the ratio (5.9x on the shipwreck
    survey)."""
    d = dvl(DEGRADED)
    by_arc = odom_trans_sigma(1.0, 1.0, d, cum_dist_m=(59.0, 60.0))
    by_disp = odom_trans_sigma(1.0, 1.0, d, cum_dist_m=(59.0, 60.0),
                               cum_disp_m=(9.0, 10.0))
    assert by_disp < by_arc


def test_bias_term_uses_the_rotation_integrated_arm():
    """A body-frame velocity bias partly cancels as the vehicle turns, so the
    arm is ||integral R dt||, always <= elapsed time."""
    d = dvl(DEGRADED)
    by_time = odom_trans_sigma(1.0, 1.0, d, cum_time_s=(99.0, 100.0))
    by_rot = odom_trans_sigma(1.0, 1.0, d, cum_time_s=(99.0, 100.0),
                              cum_rot_time_s=(19.0, 20.0))
    assert by_rot < by_time


def test_heading_bias_needs_both_arms_and_grows_with_displacement():
    """Cross-track from a walking heading bias is sigma_bias(t) * displacement,
    so it needs the time arm for the walk and the displacement arm for the lever."""
    d = dvl(DEGRADED)
    near = odom_trans_sigma(1.0, 1.0, d, cum_time_s=(99.0, 100.0),
                            cum_disp_m=(4.0, 5.0), yaw_bias_drift_rad_s=1e-3)
    far = odom_trans_sigma(1.0, 1.0, d, cum_time_s=(99.0, 100.0),
                           cum_disp_m=(49.0, 50.0), yaw_bias_drift_rad_s=1e-3)
    assert far > near


def test_heading_bias_saturates_at_the_gauss_markov_ceiling():
    """A magnetometer bias is bounded, not an unbounded walk: past the cap the
    term must stop growing with time."""
    d = dvl(DEGRADED)
    # Pin the rotation arm so only the heading-bias term sees the time change;
    # otherwise the DVL bias term moves too and the cap is not what is measured.
    kw = dict(cum_disp_m=(49.0, 50.0), cum_rot_time_s=(19.0, 20.0),
              yaw_bias_drift_rad_s=1e-3, yaw_bias_sigma_max_rad=0.01)
    # Both windows sit well past t = cap/drift^2 = 100 s, so both are saturated
    # and must agree exactly; the uncapped walk keeps growing between them.
    late = odom_trans_sigma(1.0, 1.0, d, cum_time_s=(999.0, 1000.0), **kw)
    later = odom_trans_sigma(1.0, 1.0, d, cum_time_s=(1999.0, 2000.0), **kw)
    uncapped = odom_trans_sigma(
        1.0, 1.0, d, cum_time_s=(1999.0, 2000.0),
        **{**kw, 'yaw_bias_sigma_max_rad': 0.0})
    assert later == pytest.approx(late)
    assert uncapped > 2.0 * later
