"""Synthetic checks for the sonar noise model's geometry and correlation terms."""
from pathlib import Path

import numpy as np
import pytest

from slam_backend.sensor_models.noise_profiles import SonarNoise, load_noise_profile
from slam_backend.sensor_models.sonar_noise import (
    apply_sonar_noise, correlated_field, incidence_cosine)

CONFIG_DIR = Path(__file__).parents[1] / 'config'
SHAPE = (64, 128)


def plane_grid(tilt_deg, shape=SHAPE, distance=5.0):
    """Organized cloud of a plane at `tilt_deg` from facing the sensor head-on."""
    v, u = np.meshgrid(np.linspace(-1.0, 1.0, shape[0]),
                       np.linspace(-1.0, 1.0, shape[1]), indexing='ij')
    t = np.radians(tilt_deg)
    normal = np.array([np.sin(t), 0.0, np.cos(t)])
    dirs = np.stack([u, v, np.full_like(u, 2.0)], axis=-1)
    dirs /= np.linalg.norm(dirs, axis=-1, keepdims=True)
    # Ray-plane intersection with the plane through (0,0,distance).
    scale = (distance * normal[2]) / (dirs @ normal)
    return dirs * scale[..., None]


def noisy(grid, profile, seed=0, sos_scale=1.0, rng=None):
    xyz = grid.reshape(-1, 3)
    r = np.linalg.norm(xyz, axis=1)
    rng = rng or np.random.default_rng(seed)
    cos_inc = incidence_cosine(grid).ravel()
    f_range, _ = correlated_field(rng, grid.shape[:2], profile.corr_length_px)
    f_drop, _ = correlated_field(rng, grid.shape[:2], profile.corr_length_px)
    return apply_sonar_noise(xyz, r, cos_inc, f_range.ravel(), f_drop.ravel(),
                             profile, rng, sos_scale)


# --- correlated field -------------------------------------------------------

@pytest.mark.parametrize('corr_length', [0.0, 3.0, 8.0])
def test_field_has_unit_variance(corr_length):
    """The analytic 2*s*sqrt(pi) rescaling must hold at every correlation length."""
    rng = np.random.default_rng(1)
    fields = [correlated_field(rng, (256, 256), corr_length)[0] for _ in range(32)]
    assert np.std(fields) == pytest.approx(1.0, rel=0.1)
    # Averaged over draws: one field of correlation length s holds only
    # ~(256/(s*sqrt(4pi)))^2 independent cells, so its own mean is noisy.
    assert np.mean(fields) == pytest.approx(0.0, abs=0.05)


def test_field_is_spatially_correlated():
    rng = np.random.default_rng(2)
    iid, _ = correlated_field(rng, SHAPE, 0.0)
    smooth, _ = correlated_field(rng, SHAPE, 6.0)
    lag1 = lambda f: np.corrcoef(f[:, :-1].ravel(), f[:, 1:].ravel())[0, 1]
    assert abs(lag1(iid)) < 0.1
    assert lag1(smooth) > 0.9


def test_field_is_temporally_correlated():
    rng = np.random.default_rng(3)
    f0, w0 = correlated_field(rng, SHAPE, 4.0)
    f1, _ = correlated_field(rng, SHAPE, 4.0, rho_time=0.8, prev_white=w0)
    f_indep, _ = correlated_field(rng, SHAPE, 4.0)
    assert np.corrcoef(f0.ravel(), f1.ravel())[0, 1] == pytest.approx(0.8, abs=0.1)
    assert abs(np.corrcoef(f0.ravel(), f_indep.ravel())[0, 1]) < 0.15


# --- incidence angle --------------------------------------------------------

def test_incidence_cosine_matches_plane_tilt():
    """Central pixels of a tilted plane report cos of that tilt."""
    for tilt in (0.0, 30.0, 60.0):
        cos_inc = incidence_cosine(plane_grid(tilt))
        assert cos_inc[SHAPE[0] // 2, SHAPE[1] // 2] == pytest.approx(
            np.cos(np.radians(tilt)), abs=0.02)


def test_incidence_cosine_degrades_gracefully():
    grid = plane_grid(0.0)
    grid[10, 10] = np.nan
    cos_inc = incidence_cosine(grid)
    assert np.all(np.isfinite(cos_inc))
    assert cos_inc[10, 10] == 1.0


# --- noise terms ------------------------------------------------------------

def test_grazing_incidence_increases_dropout():
    p = SonarNoise(dropout_p0=0.02, dropout_p_grazing=0.5, range_sigma0_m=0.005)
    frac = lambda tilt: np.isnan(noisy(plane_grid(tilt), p, seed=4)[:, 0]).mean()
    assert frac(0.0) < 0.06
    assert frac(75.0) > 0.20
    assert frac(75.0) > frac(45.0) > frac(0.0)


def test_dropouts_are_patchy_when_correlated():
    """Same dropout rate, but correlated dropouts form far fewer, larger blobs."""
    def isolated_frac(corr_length):
        p = SonarNoise(dropout_p0=0.15, corr_length_px=corr_length, range_sigma0_m=0.005)
        drop = np.isnan(noisy(plane_grid(0.0), p, seed=5)[:, 0]).reshape(SHAPE)
        neighbours = (drop[:-2, 1:-1].astype(int) + drop[2:, 1:-1]
                      + drop[1:-1, :-2] + drop[1:-1, 2:])
        core = drop[1:-1, 1:-1]
        return (neighbours[core] == 0).mean()   # dropouts with no dropped neighbour

    assert isolated_frac(0.0) > 0.5
    assert isolated_frac(6.0) < 0.1


def test_range_noise_variance_is_preserved_by_the_correlation_split():
    """range_corr_frac redistributes error between iid and field, never inflates it."""
    grid = plane_grid(0.0, shape=(128, 256))
    for frac in (0.0, 0.5, 1.0):
        p = SonarNoise(range_sigma0_m=0.05, range_corr_frac=frac)
        out = noisy(grid, p, seed=6)
        err = np.linalg.norm(out, axis=1) - np.linalg.norm(grid.reshape(-1, 3), axis=1)
        assert np.nanstd(err) == pytest.approx(0.05, rel=0.15)


def test_range_error_is_correlated_between_pings():
    """End-to-end: consecutive pings on a static scene must not be independent.

    outlier_p is left at 0 deliberately — the multipath term is iid and roughly
    ten times the speckle amplitude, so it masks this correlation entirely in
    raw range statistics.
    """
    p = SonarNoise(range_sigma0_m=0.01, corr_length_px=6.0,
                   corr_rho_time=0.8, range_corr_frac=1.0)
    grid = plane_grid(0.0)
    xyz = grid.reshape(-1, 3)
    truth_r = np.linalg.norm(xyz, axis=1)
    cos_inc = incidence_cosine(grid).ravel()

    rng = np.random.default_rng(9)
    errs, prev = [], None
    for _ in range(2):
        f_range, prev = correlated_field(rng, SHAPE, p.corr_length_px, p.corr_rho_time, prev)
        f_drop, _ = correlated_field(rng, SHAPE, p.corr_length_px)
        out = apply_sonar_noise(xyz, truth_r, cos_inc, f_range.ravel(), f_drop.ravel(),
                                p, rng)
        errs.append(np.linalg.norm(out, axis=1) - truth_r)

    assert np.corrcoef(*errs)[0, 1] == pytest.approx(0.8, abs=0.1)


def test_speed_of_sound_error_is_a_pure_scale():
    grid = plane_grid(0.0)
    p = SonarNoise(dropout_p0=0.0)   # no other term active
    out = noisy(grid, p, seed=7, sos_scale=1.01)
    ratio = np.linalg.norm(out, axis=1) / np.linalg.norm(grid.reshape(-1, 3), axis=1)
    assert np.allclose(ratio, 1.01, rtol=1e-6)


def test_ideal_profile_moves_no_point():
    """All-zero parameters are a no-op. The node short-circuits this case for
    bit-exact republishing; here the model itself must still not move a point."""
    p = load_noise_profile(CONFIG_DIR / 'noise_ideal.yaml').sonar
    grid = plane_grid(40.0)
    assert np.allclose(noisy(grid, p, seed=8), grid.reshape(-1, 3), rtol=1e-12, atol=1e-12)


def test_realistic_profile_enables_every_new_term():
    p = load_noise_profile(CONFIG_DIR / 'noise_realistic.yaml').sonar
    assert p.dropout_p_grazing > 0
    assert p.corr_length_px > 0
    assert 0 < p.corr_rho_time < 1
    assert 0 < p.range_corr_frac <= 1
    assert p.sos_scale_error_pct > 0
