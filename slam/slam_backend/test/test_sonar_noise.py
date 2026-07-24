"""Synthetic checks for the sonar noise model's geometry and correlation terms."""
from pathlib import Path
import warnings

import numpy as np
import pytest

from slam_backend.sensor_models.noise_profiles import SonarNoise, load_noise_profile
from slam_backend.sensor_models.sonar_noise import (
    _fit_pinhole, apply_sonar_noise, correlated_field, incidence_cosine, multipath_range,
    pixel_angular_spacing, reverberation_range, strongest_reflection_range,
    surface_normals, within_sonar_range, DEPTH_MIN_M, MAX_RANGE_M)

CONFIG_DIR = Path(__file__).parents[1] / 'config'
SHAPE = (64, 128)


def test_sonar_range_is_radial_across_its_whole_field_of_view():
    points = np.array([
        [14.9, 0.0, 0.0],
        [14.0, 0.0, 14.0],  # 19.8 m slant range: outside a 15 m acoustic beam
        [0.0, 0.0, 15.0],
        [np.nan, 0.0, 3.0],
    ])
    assert within_sonar_range(points).tolist() == [True, False, False, False]


def pinhole_grid(shape=SHAPE, hfov_deg=90.0, distance=4.0):
    """Organized cloud of a frontal wall sampled by a pinhole camera, as the sim
    depth camera samples it: x = z*tan(theta), uniform in pixel, not in angle."""
    h, w = shape
    fx = (w / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    fy = fx
    j, i = np.meshgrid(np.arange(w), np.arange(h))
    x_over_z = (j - (w - 1) / 2.0) / fx
    y_over_z = (i - (h - 1) / 2.0) / fy
    z = np.full(shape, distance)
    return np.stack([x_over_z * z, y_over_z * z, z], axis=-1)


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
    assert p.argmax_window_px > 1
    assert p.lat_sigma_beam_frac > 0
    assert p.reverb_p > 0
    assert p.multipath_p > 0


# --- item 3: strongest-return ranging ---------------------------------------

def wall_with_bar(bar_range=2.0, wall_range=3.0, bar_cols=slice(64, 65)):
    """A one-pixel-wide near bar standing in front of a flat far wall, as ranges
    on a frontal organized grid (rays straight ahead in z). One column is thin
    relative to any beam window, so its echo is outvoted by the wall's."""
    r = np.full(SHAPE, wall_range)
    r[:, bar_cols] = bar_range
    z = r
    x, y = np.zeros(SHAPE), np.zeros(SHAPE)
    return r, np.stack([x, y, z], axis=-1)


def test_argmax_disabled_is_identity():
    r, _ = wall_with_bar()
    cos = np.ones(SHAPE)
    assert np.array_equal(strongest_reflection_range(r, cos, 0, 1.5), r)
    assert np.array_equal(strongest_reflection_range(r, cos, 1, 1.5), r)


def test_argmax_thin_structure_fades_behind_wall():
    """A one-pixel bar loses to the many wall pixels of the beam behind it."""
    r, _ = wall_with_bar()
    cos = np.ones(SHAPE)
    out = strongest_reflection_range(r, cos, 5, 2.0)
    bar = r == 2.0
    replaced = (out[bar] > 2.5).mean()   # bar pixels now reporting wall range
    assert replaced > 0.8
    # A far wall pixel deep in the wall keeps its range (no nearer strong echo).
    assert out[32, 10] == pytest.approx(3.0)


def test_argmax_never_invents_a_nan_or_out_of_band_range():
    r, _ = wall_with_bar()
    r[:, :4] = np.nan          # a no-return / invalid strip
    cos = np.ones(SHAPE)
    out = strongest_reflection_range(r, cos, 5, 1.5)
    finite = np.isfinite(out)
    assert np.all((out[finite] >= DEPTH_MIN_M) & (out[finite] <= MAX_RANGE_M))
    # A pixel two columns into the invalid strip has no valid candidate: stays NaN.
    assert np.isnan(out[32, 0])


# --- item 4: projection mismatch / beam spacing -----------------------------

def test_angular_spacing_follows_pinhole_law():
    """dtheta/pixel = cos^2(theta)/f for a pinhole, so it shrinks toward the edge."""
    grid = pinhole_grid(hfov_deg=90.0)
    u = grid / np.linalg.norm(grid, axis=2, keepdims=True)
    dh, _ = pixel_angular_spacing(u)
    row = SHAPE[0] // 2
    centre = dh[row, SHAPE[1] // 2]
    edge = dh[row, 5]
    assert centre > edge                      # pinhole packs more angle per pixel at centre
    assert edge / centre == pytest.approx(np.cos(np.radians(45.0)) ** 2, rel=0.1)


def test_pinhole_fit_accepts_empty_border_slices_without_warning():
    grid = pinhole_grid()
    unit = grid / np.linalg.norm(grid, axis=2, keepdims=True)
    valid = np.ones(SHAPE, dtype=bool)
    valid[:, 0] = False
    valid[0, :] = False
    with warnings.catch_warnings():
        warnings.simplefilter('error', RuntimeWarning)
        intrinsics = _fit_pinhole(unit, valid)
    assert intrinsics is not None


def test_beam_frac_jitter_grows_from_centre_to_edge_is_disabled_at_zero():
    grid = pinhole_grid(hfov_deg=90.0)
    xyz = grid.reshape(-1, 3)
    r = np.linalg.norm(xyz, axis=1)
    cos = incidence_cosine(grid).ravel()
    dh, dv = pixel_angular_spacing(grid / np.linalg.norm(grid, axis=2, keepdims=True))

    def jitter_std(cols):
        p = SonarNoise(lat_sigma_beam_frac=0.5)
        rng = np.random.default_rng(3)
        z = np.zeros(len(r))
        out = apply_sonar_noise(xyz, r, cos, z, z, p, rng, beam_h=dh.ravel(), beam_v=dv.ravel())
        lateral = out - xyz            # jitter is perpendicular; x carries most of it here
        sel = np.zeros(SHAPE, bool); sel[:, cols] = True
        return np.std((out[:, 0] - xyz[:, 0])[sel.ravel()])

    centre = jitter_std(slice(SHAPE[1] // 2 - 4, SHAPE[1] // 2 + 4))
    edge = jitter_std(slice(0, 8))
    assert centre > edge               # wider pinhole beam at centre -> more cross-range jitter

    # frac=0 falls back to the per-metre constants; here both are zero -> no jitter.
    p0 = SonarNoise(lat_sigma_beam_frac=0.0)
    rng = np.random.default_rng(3)
    z = np.zeros(len(r))
    out0 = apply_sonar_noise(xyz, r, cos, z, z, p0, rng, beam_h=dh.ravel(), beam_v=dv.ravel())
    assert np.allclose(out0, xyz, atol=1e-9)


# --- item 6: volume reverberation -------------------------------------------

def test_reverberation_injects_correlated_near_returns():
    r = np.full(SHAPE, 6.0)
    cos = np.ones(SHAPE)
    valid = np.ones(SHAPE, bool)
    p = SonarNoise(reverb_p=0.1, reverb_max_m=1.5, corr_length_px=6.0)
    rng = np.random.default_rng(11)
    field, _ = correlated_field(rng, SHAPE, p.corr_length_px)
    out, mask = reverberation_range(r, cos, valid, field, rng, p)

    assert mask.mean() == pytest.approx(0.1, abs=0.04)
    assert np.all((out[mask] >= DEPTH_MIN_M) & (out[mask] <= 1.5))   # near-field only
    assert np.all(out[~mask] == 6.0)                                  # surface untouched
    # Correlated field -> reverb pixels clump rather than scatter per-pixel.
    nb = (mask[:-2, 1:-1].astype(int) + mask[2:, 1:-1] + mask[1:-1, :-2] + mask[1:-1, 2:])
    isolated = (nb[mask[1:-1, 1:-1]] == 0).mean()
    assert isolated < 0.2


def test_reverberation_hits_weak_returns_more():
    """reverb_weak_boost raises the rate on far/grazing surfaces."""
    cos = np.ones(SHAPE)
    valid = np.ones(SHAPE, bool)
    p = SonarNoise(reverb_p=0.05, reverb_max_m=1.5, reverb_weak_boost=5.0)
    rng = np.random.default_rng(12)
    field, _ = correlated_field(rng, SHAPE, 0.0)   # iid, so rate tracks the local probability
    near = reverberation_range(np.full(SHAPE, 2.0), cos, valid, field, rng, p)[1].mean()
    rng = np.random.default_rng(12)
    field, _ = correlated_field(rng, SHAPE, 0.0)
    far = reverberation_range(np.full(SHAPE, 14.0), cos, valid, field, rng, p)[1].mean()
    assert far > near


def test_near_field_gate_drops_close_returns():
    """min_range_m NaNs out returns nearer than the threshold (reverb spray)."""
    grid = plane_grid(0.0, distance=1.0)   # a wall ~1 m away, inside a 1.5 m gate
    p = SonarNoise(range_sigma0_m=0.005, min_range_m=1.5)
    out = noisy(grid, p, seed=13)
    assert np.all(np.isnan(out[:, 0]))     # every return gated away

    far = plane_grid(0.0, distance=5.0)
    out_far = noisy(far, p, seed=13)
    assert np.mean(np.isfinite(out_far[:, 0])) > 0.9   # 5 m wall survives the gate


def test_near_field_gate_off_by_default_keeps_close_returns():
    grid = plane_grid(0.0, distance=1.0)
    p = SonarNoise(range_sigma0_m=0.005)   # min_range_m defaults to 0
    out = noisy(grid, p, seed=13)
    assert np.mean(np.isfinite(out[:, 0])) > 0.9


def test_reverberation_disabled_is_identity():
    r = np.full(SHAPE, 6.0)
    out, mask = reverberation_range(r, np.ones(SHAPE), np.ones(SHAPE, bool),
                                    np.zeros(SHAPE), np.random.default_rng(0),
                                    SonarNoise(reverb_p=0.0))
    assert np.array_equal(out, r) and not mask.any()


# --- item 7: geometric multipath --------------------------------------------

def corner_grid(shape=SHAPE, size=4.0):
    """A right-angle corner: left half is a wall facing +x, right half faces +y,
    meeting along a concave seam. Reflected rays off one face strike the other."""
    h, w = shape
    fx = (w / 2.0) / np.tan(np.radians(45.0))
    fy = fx
    j, i = np.meshgrid(np.arange(w), np.arange(h))
    ax = (j - (w - 1) / 2.0) / fx        # x/z per pixel
    ay = (i - (h - 1) / 2.0) / fy        # y/z per pixel
    # Wall A: x = size (normal -x). Wall B: y = size (normal -y). Take the nearer.
    with np.errstate(divide='ignore', invalid='ignore'):
        zA = np.where(ax > 1e-3, size / ax, np.inf)
        zB = np.where(ay > 1e-3, size / ay, np.inf)
    z = np.minimum(zA, zB)
    z[~np.isfinite(z)] = size
    return np.stack([ax * z, ay * z, z], axis=-1)


def _multipath_fraction(grid, p, seed=5):
    r = np.linalg.norm(grid, axis=2)
    u = grid / r[..., None]
    normal = surface_normals(grid)
    cos = incidence_cosine(grid, normal)
    valid = np.isfinite(r) & (r > DEPTH_MIN_M) & (r < MAX_RANGE_M)
    out = multipath_range(np.where(valid, r, np.nan), grid, u, normal, cos, valid,
                          np.random.default_rng(seed), p)
    changed = valid & np.isfinite(out) & (np.abs(out - r) > 1e-6)
    return changed.sum() / valid.sum(), out, r, valid


def test_multipath_fires_in_a_corner_not_on_a_flat_wall():
    p = SonarNoise(multipath_p=1.0)   # deterministic given a hit, to isolate geometry
    corner_frac, out, r, valid = _multipath_fraction(corner_grid(), p)
    flat_frac, _, _, _ = _multipath_fraction(pinhole_grid(hfov_deg=90.0), p)
    assert corner_frac > 0.05
    assert flat_frac < 0.01
    # every phantom is a late arrival
    changed = valid & (out != r)
    assert np.all(out[changed] > r[changed])


def test_multipath_disabled_is_identity():
    frac, _, _, _ = _multipath_fraction(corner_grid(), SonarNoise(multipath_p=0.0))
    assert frac == 0.0
