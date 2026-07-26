#!/usr/bin/env python3
"""Datasheet-grounded noise model for the depth-camera point cloud standing in
for a WaterLinked Sonar 3D-15 (1.2 MHz mode: 90x40 deg FOV, 0.2-15m range,
0.35/0.60 deg beam separation H/V, 1.5mm range resolution — see
ActiveSlam-Resources/3d-sonar datasheet).

Sits between depth_image_proc and every mapping/SLAM consumer:

    /cloud_in_raw --> sonar_noise --> /cloud_in

Organized-image stage (needs the 2D beam layout; skipped for unorganized clouds):
  A. strongest-return ranging: the device reports the strongest echo in a beam,
     not the nearest surface a z-buffer sees, so the range is taken as an
     intensity-weighted arg-max over a small pixel window (thin structures fade,
     edges bleed toward strong scatterers)
  B. geometric multipath: reflect each beam about the local surface normal and
     march the depth buffer; a second-bounce hit reports a late range, so
     concave geometry (corners, wreck interiors) produces phantoms and flat
     geometry does not
  C. volume reverberation: spurious near-field returns from particles/bubbles/
     tether, spatially correlated and more likely where the surface return is
     weak, since the device reports whichever echo is strongest

Per-point stage, for each finite point with depth_min < range < NO_RETURN_RANGE_M
(points at/beyond the sensor's max range are no-return readings, emitted as NaN):
  1. range noise along the viewing ray:   r += N(0, sigma0 + k*r)
  2. multipath outliers (late arrivals):  r += U(outlier_min, outlier_max)
  3. speed-of-sound scale error:          r *= 1 + eps, one draw per run
  4. range-bin quantization:              r = round(r / q) * q
  5. beam-spreading lateral jitter, perpendicular to the ray, scaled by range and
     by the local beam angular width (the footprint of one beam grows with range)
  6. dropouts: replaced with NaN, at a probability that grows with range and
     with grazing incidence (specular surfaces reflect energy away from the
     transducer, so an oblique wall returns nothing — the dominant real-world
     no-return mechanism). Organized-cloud NaN convention per REP 118, already
     handled by every downstream consumer: tsdf_mapper's isfinite filter,
     pose_graph's skip_nans read, octomap_server's native NaN support.
  7. near-field gate (min_range_m): drop returns nearer than the threshold,
     a tunable knob to clear the reverberation spray around the vehicle.
  8. near-field fade (near_fade_p): the soft form of item 7 — a drop
     probability rising toward the sensor rather than a hard cut, so close
     geometry survives thinned instead of erased.

Range error and dropout are drawn from smooth random fields rather than
independently per point, so both are correlated across neighbouring beams
(corr_length_px) and between consecutive pings (corr_rho_time). Independent
noise is the kind SLAM averages away, which flatters the pipeline; real sonar
speckle is partly frozen and real dropouts arrive in patches.

None of these terms are fitted to real hardware; they are argued from the
datasheet and from acoustics. All-zero parameters (`ideal`) republish unchanged.
"""
import os

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from scipy.ndimage import gaussian_filter
from scipy.special import ndtri
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from slam_backend.sensor_models.noise_profiles import load_noise_profile, resolve_seed, NoiseProfile

DEPTH_MIN_M = 0.2
NO_RETURN_RANGE_M = 14.9   # points at/beyond this receive no noise model
MAX_RANGE_M = 15.0
MULTIPATH_STEPS = 24       # screen-space march resolution
MULTIPATH_TMIN_M = 0.15    # first march step, clear of the origin surface

# Per-sensor offset added to a shared seed so co-launched sims (identical
# profile.seed) don't draw identical RNG streams (imu=+1, dvl=+2, pressure=+3).
SEED_OFFSET = 4


def within_sonar_range(xyz: np.ndarray, max_range_m: float = MAX_RANGE_M) -> np.ndarray:
    """Mask finite point-cloud samples within the sonar's radial beam range."""
    xyz = np.asarray(xyz)
    return np.isfinite(xyz).all(axis=-1) & (np.linalg.norm(xyz, axis=-1) < max_range_m)


def correlated_field(rng, shape, corr_length_px, rho_time=0.0, prev_white=None):
    """Unit-variance Gaussian random field, smooth over `corr_length_px` pixels.

    Returns (field, white) — pass `white` back as `prev_white` on the next ping
    to give the field AR(1) correlation `rho_time` in time. Smoothing white
    noise with a Gaussian of std s scales the variance by 1/(4*pi*s^2), so the
    analytic 2*s*sqrt(pi) restores unit variance exactly.
    """
    white = rng.standard_normal(shape)
    if rho_time > 0.0 and prev_white is not None and prev_white.shape == shape:
        white = rho_time * prev_white + np.sqrt(1.0 - rho_time ** 2) * white
    if corr_length_px <= 0.0:
        return white, white
    field = gaussian_filter(white, corr_length_px, mode='wrap')
    return field * 2.0 * corr_length_px * np.sqrt(np.pi), white


def surface_normals(grid):
    """Unit surface normals from an organized (h, w, 3) cloud, NaN where degenerate."""
    normal = np.cross(np.gradient(grid, axis=1), np.gradient(grid, axis=0))
    norm = np.linalg.norm(normal, axis=2, keepdims=True)
    with np.errstate(invalid='ignore', divide='ignore'):
        return normal / norm


def incidence_cosine(grid, normal=None):
    """|cos| of the angle between each beam and the local surface normal.

    `grid` is an organized (h, w, 3) sensor-frame point cloud. Returns 1.0
    (normal incidence, no grazing penalty) wherever the normal is degenerate —
    borders, NaN neighbours. Depth discontinuities yield edge-on normals and so
    a grazing penalty, which matches real sonar dropping returns at edges.
    """
    if grid.shape[0] < 3 or grid.shape[1] < 3:
        return np.ones(grid.shape[:2])
    if normal is None:
        normal = surface_normals(grid)
    ray = grid / np.linalg.norm(grid, axis=2, keepdims=True)
    with np.errstate(invalid='ignore'):
        cos_inc = np.abs(np.sum(normal * ray, axis=2))
    return np.where(np.isfinite(cos_inc), np.clip(cos_inc, 0.0, 1.0), 1.0)


def pixel_angular_spacing(u_grid):
    """Per-pixel angular step (rad) between adjacent beams, (dtheta_h, dtheta_v).

    Measured from the organized unit-ray grid, so it reflects the sim's actual
    (pinhole) beam layout rather than an assumed-uniform separation. Each pixel
    is one beam in the depth-camera-as-sonar approximation, so this is the beam's
    angular width — what the lateral cross-range jitter scales with.
    """
    h, w = u_grid.shape[:2]
    dth_h = np.full((h, w), np.nan)
    dth_v = np.full((h, w), np.nan)
    if w >= 2:
        gap = np.arccos(np.clip(np.sum(u_grid[:, 1:] * u_grid[:, :-1], axis=2), -1.0, 1.0))
        dth_h[:, 1:-1] = 0.5 * (gap[:, :-1] + gap[:, 1:])
        dth_h[:, 0] = gap[:, 0]
        dth_h[:, -1] = gap[:, -1]
    if h >= 2:
        gap = np.arccos(np.clip(np.sum(u_grid[1:] * u_grid[:-1], axis=2), -1.0, 1.0))
        dth_v[1:-1] = 0.5 * (gap[:-1] + gap[1:])
        dth_v[0] = gap[0]
        dth_v[-1] = gap[-1]
    return dth_h, dth_v


ARGMAX_RANGE_TOL_M = 0.1   # range bin over which window echoes sum into one return


def strongest_reflection_range(r_grid, cos_inc, window_px, beam_sigma_px):
    """Range of the strongest echo in a beam, not the nearest surface.

    A beam is wider than a pixel, so it sums the echoes of every surface it
    covers and reports the range holding the most energy. Each window candidate
    contributes a beam-pattern-weighted cos(incidence)/range^2 (echo strength vs.
    spreading loss); candidates are grouped by range and the range of the
    strongest group wins. A thin structure fills few pixels, so its group loses
    to the many pixels of the wall behind it and it fades; a bright near surface
    pulls neighbouring beams toward its range. NaN candidates (invalid /
    no-return) are excluded, so a pixel never inherits a non-surface neighbour.
    window_px<=1 is the identity (nearest-surface z-buffer).
    """
    k = int(round(window_px))
    if k <= 1:
        return r_grid.copy()
    if k % 2 == 0:
        k += 1
    pad = k // 2
    r_pad = np.pad(r_grid, pad, constant_values=np.nan)
    cos_pad = np.pad(cos_inc, pad, constant_values=np.nan)
    win_r = sliding_window_view(r_pad, (k, k)).reshape(r_grid.shape + (k * k,))
    win_cos = sliding_window_view(cos_pad, (k, k)).reshape(r_grid.shape + (k * k,))

    dy, dx = np.mgrid[-pad:pad + 1, -pad:pad + 1]
    beam_w = np.exp(-0.5 * (dx ** 2 + dy ** 2) / beam_sigma_px ** 2).ravel()
    with np.errstate(invalid='ignore', divide='ignore'):
        intensity = beam_w * win_cos / win_r ** 2
    finite = np.isfinite(win_r)
    intensity = np.where(finite, intensity, 0.0)
    r_safe = np.where(finite, win_r, 0.0)

    # Energy each candidate m collects from window echoes near its own range.
    diff = r_safe[..., :, None] - r_safe[..., None, :]
    same = np.exp(-0.5 * (diff / ARGMAX_RANGE_TOL_M) ** 2) * finite[..., None, :]
    score = np.sum(intensity[..., None, :] * same, axis=-1)
    score = np.where(finite, score, -np.inf)

    best = np.argmax(score, axis=-1)
    out = np.take_along_axis(win_r, best[..., None], axis=-1)[..., 0]
    no_candidate = ~finite.any(axis=-1)
    out[no_candidate] = r_grid[no_candidate]
    return out


def _fit_pinhole(u_grid, valid):
    """Recover (fx, cx, fy, cy) from the organized unit-ray grid's pinhole layout."""
    h, w = u_grid.shape[:2]
    with np.errstate(invalid='ignore', divide='ignore'):
        a = np.where(valid, u_grid[..., 0] / u_grid[..., 2], np.nan)
        b = np.where(valid, u_grid[..., 1] / u_grid[..., 2], np.nan)
    # np.nanmedian warns for every fully invalid border row/column. Those are
    # normal in an organized depth image (no return at the FoV edge), so only
    # reduce slices that contain at least one finite ray.
    a_col = np.full(w, np.nan)
    b_row = np.full(h, np.nan)
    finite_cols = np.isfinite(a).any(axis=0)
    finite_rows = np.isfinite(b).any(axis=1)
    a_col[finite_cols] = np.nanmedian(a[:, finite_cols], axis=0)
    b_row[finite_rows] = np.nanmedian(b[finite_rows, :], axis=1)
    jj, ii = np.arange(w), np.arange(h)
    fj, fi = np.isfinite(a_col), np.isfinite(b_row)
    if fj.sum() < 2 or fi.sum() < 2:
        return None
    sa, ia = np.polyfit(jj[fj], a_col[fj], 1)
    sb, ib = np.polyfit(ii[fi], b_row[fi], 1)
    if sa == 0 or sb == 0:
        return None
    return 1.0 / sa, -ia / sa, 1.0 / sb, -ib / sb


def multipath_range(r_grid, grid, u_grid, normal, cos_inc, valid, rng, p):
    """Screen-space second-bounce multipath, returning a modified range grid.

    Reflect each beam about its surface normal and march the reflected ray
    through the depth buffer; the first re-intersection gives a late range
    r + t_hit. A flat wall reflects away from all geometry and never re-hits, so
    it produces no phantom; a concave corner does. Fires with probability rising
    as the direct return weakens (grazing incidence), where the bounce can be the
    stronger echo.
    """
    if p.multipath_p <= 0.0 or grid.shape[0] < 3 or grid.shape[1] < 3:
        return r_grid.copy()
    intr = _fit_pinhole(u_grid, valid)
    if intr is None:
        return r_grid.copy()
    fx, cx, fy, cy = intr
    h, w = r_grid.shape
    z_buf = grid[..., 2]
    d_ref = u_grid - 2.0 * np.sum(u_grid * normal, axis=2, keepdims=True) * normal

    hit_t = np.full((h, w), np.nan)
    for t in np.linspace(MULTIPATH_TMIN_M, MAX_RANGE_M, MULTIPATH_STEPS):
        q = grid + t * d_ref
        with np.errstate(invalid='ignore', divide='ignore'):
            j = np.round(fx * q[..., 0] / q[..., 2] + cx).astype(np.int64)
            i = np.round(fy * q[..., 1] / q[..., 2] + cy).astype(np.int64)
        on = valid & np.isfinite(j) & np.isfinite(i) & (q[..., 2] > 0.0)
        on &= (i >= 0) & (i < h) & (j >= 0) & (j < w)
        ii = np.where(on, i, 0)
        jj = np.where(on, j, 0)
        z_hit = z_buf[ii, jj]
        crossed = on & valid[ii, jj] & (q[..., 2] >= z_hit) & np.isnan(hit_t)
        hit_t[crossed] = t

    fire = valid & np.isfinite(hit_t) & (
        rng.random((h, w)) < p.multipath_p * (1.0 - cos_inc) ** p.multipath_grazing_exp)
    out = r_grid.copy()
    out[fire] = np.clip(r_grid[fire] + hit_t[fire], DEPTH_MIN_M, MAX_RANGE_M)
    return out


def reverberation_range(r_grid, cos_inc, valid, field, rng, p):
    """Spurious near-field volume returns (particles/bubbles/tether).

    Fires on a spatially correlated field at rate reverb_p, elevated where the
    surface echo is weak — its strength falls as cos(incidence)/range^2, so a far
    or grazing surface is beaten by the near-field volume return more often.
    Replaced range is drawn from a 1/r^2 backscatter density over
    [DEPTH_MIN, reverb_max_m].
    """
    if p.reverb_p <= 0.0:
        return r_grid.copy(), np.zeros(r_grid.shape, bool)
    weak = (r_grid / MAX_RANGE_M) ** 2 / np.clip(cos_inc, 0.05, 1.0)
    p_rev = np.clip(p.reverb_p * (1.0 + p.reverb_weak_boost * weak), 0.0, 1.0)
    mask = valid & (field < ndtri(p_rev))
    n = int(mask.sum())
    out = r_grid.copy()
    if n:
        a, b = 1.0 / DEPTH_MIN_M, 1.0 / p.reverb_max_m
        out[mask] = 1.0 / (a - rng.random(n) * (a - b))   # inverse-CDF of 1/r^2 density
    return out, mask


def near_fade_drop_p(r, p):
    """Drop probability of the near-field fade: near_fade_p at the sensor,
    zero at near_fade_range_m. Thins close returns rather than cutting them,
    so a sparse spray goes and a surface filling every beam stays outlined."""
    r = np.asarray(r, dtype=float)
    if p.near_fade_p <= 0.0 or p.near_fade_range_m <= 0.0:
        return np.zeros(r.shape)
    close = np.clip(1.0 - r / p.near_fade_range_m, 0.0, 1.0)
    return p.near_fade_p * close ** p.near_fade_exp


class SonarNoiseNode(Node):
    def __init__(self, **kwargs):
        super().__init__('sonar_noise', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('noise_seed', -1)
        self.declare_parameter('input_topic', '/cloud_in_raw')
        self.declare_parameter('output_topic', '/cloud_in')
        # Relay unmodified regardless of the profile. The node still runs with
        # the noise model off, because it is also what republishes the cloud on
        # a QoS every consumer can match — see pointcloud_only.launch.py.
        self.declare_parameter('passthrough', False)
        # Near-field gate, overridable from any launch over any profile: drop
        # returns nearer than this to clear the reverberation spray around the
        # vehicle. -1 keeps the profile's own min_range_m (0 = off).
        self.declare_parameter('min_range_m', -1.0)
        # Soft near-field fade, overridden the same way (-1 keeps the profile's).
        self.declare_parameter('near_fade_p', -1.0)
        self.declare_parameter('near_fade_range_m', -1.0)

        yaml_path = self.get_parameter('noise_profile_path').value
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            self.profile = NoiseProfile().sonar
            profile_seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.sonar
            profile_seed = full_profile.seed

        min_range_override = self.get_parameter('min_range_m').value
        if min_range_override >= 0.0:
            self.profile.min_range_m = min_range_override
        fade_p_override = self.get_parameter('near_fade_p').value
        if fade_p_override >= 0.0:
            self.profile.near_fade_p = fade_p_override
        fade_range_override = self.get_parameter('near_fade_range_m').value
        if fade_range_override >= 0.0:
            self.profile.near_fade_range_m = fade_range_override

        seed = resolve_seed(profile_seed, self.get_parameter('noise_seed').value, SEED_OFFSET)
        self._rng = np.random.default_rng(seed if seed != -1 else None)
        self._passthrough = self.get_parameter('passthrough').value or not any([
            self.profile.range_sigma0_m, self.profile.range_sigma_k,
            self.profile.range_quant_m,
            self.profile.lat_sigma_h_per_m, self.profile.lat_sigma_v_per_m,
            self.profile.dropout_p0, self.profile.dropout_p_range,
            self.profile.dropout_p_grazing,
            self.profile.outlier_p, self.profile.sos_scale_error_pct,
            self.profile.argmax_window_px > 1, self.profile.lat_sigma_beam_frac,
            self.profile.reverb_p, self.profile.multipath_p,
            self.profile.min_range_m, self.profile.near_fade_p,
        ])

        # One speed-of-sound error per run: a systematic range scale SLAM cannot
        # average away, unlike every other term here.
        self._sos_scale = 1.0 + self._rng.normal(0.0, self.profile.sos_scale_error_pct)
        self._prev_white = None
        self._prev_white_drop = None

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"SonarNoise started: {input_topic} -> {output_topic} "
            f"(passthrough={self._passthrough})")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if msg.point_step < 12 or msg.width * msg.height == 0:
            self._pub.publish(msg)
            return

        floats_per_point = msg.point_step // 4
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats_per_point).copy()
        xyz = data[:, :3].astype(np.float64)

        # The depth-camera proxy's linear Z clip can yield off-axis points
        # farther than 15 m. A real Sonar 3D-15 has a 15 m *radial* acoustic
        # range, so turn these no-return beams into NaNs before publishing
        # /cloud_in — even when the noise model is in passthrough mode.
        in_range = within_sonar_range(xyz)
        xyz[~in_range] = np.nan
        if self._passthrough:
            data[:, :3] = xyz.astype(np.float32)
            msg.data = data.tobytes()
            self._pub.publish(msg)
            return

        r = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(r) & (r > DEPTH_MIN_M) & (r < NO_RETURN_RANGE_M)
        if np.any(valid):
            shape = (msg.height, msg.width)
            organized = msg.height > 1 and msg.height * msg.width == len(r)
            p = self.profile
            beam_h = beam_v = None

            if organized:
                grid = xyz.reshape(*shape, 3)
                normal = surface_normals(grid)
                cos_grid = incidence_cosine(grid, normal)
                vgrid = valid.reshape(shape)
                r_grid = np.where(vgrid, r.reshape(shape), np.nan)
                u_grid = grid / r.reshape(*shape, 1)

                # Organized-image stage: strongest-return, multipath, reverberation.
                r_grid = strongest_reflection_range(
                    r_grid, cos_grid, p.argmax_window_px, p.argmax_beam_sigma_px)
                r_grid = multipath_range(
                    r_grid, grid, u_grid, normal, cos_grid, vgrid, self._rng, p)
                if p.reverb_p > 0.0:
                    f_reverb, _ = correlated_field(self._rng, shape, p.corr_length_px)
                    r_grid, _ = reverberation_range(
                        r_grid, cos_grid, vgrid, f_reverb, self._rng, p)

                # Rebuild points from the (possibly re-ranged) grid, same beam dirs.
                keep = np.isfinite(r_grid)
                xyz = np.where(keep.reshape(-1, 1), u_grid.reshape(-1, 3) * np.nan_to_num(
                    r_grid).reshape(-1, 1), xyz)
                r = np.where(keep.ravel(), r_grid.ravel(), r)
                valid = valid & keep.ravel()
                cos_inc = cos_grid.ravel()
                if p.lat_sigma_beam_frac > 0.0:
                    dh, dv = pixel_angular_spacing(u_grid)
                    beam_h, beam_v = dh.ravel()[valid], dv.ravel()[valid]
            else:
                cos_inc = np.ones(len(r))

            field_shape = shape if organized else (len(r),)
            f_range, self._prev_white = correlated_field(
                self._rng, field_shape, p.corr_length_px, p.corr_rho_time, self._prev_white)
            f_drop, self._prev_white_drop = correlated_field(
                self._rng, field_shape, p.corr_length_px, p.corr_rho_time, self._prev_white_drop)
            xyz[valid] = apply_sonar_noise(
                xyz[valid], r[valid], cos_inc[valid],
                f_range.ravel()[valid], f_drop.ravel()[valid],
                p, self._rng, self._sos_scale, beam_h, beam_v)

        data[:, :3] = xyz.astype(np.float32)
        msg.data = data.tobytes()
        self._pub.publish(msg)


def apply_sonar_noise(xyz, r, cos_inc, f_range, f_drop, p, rng, sos_scale=1.0,
                      beam_h=None, beam_v=None):
    n = len(r)
    u = xyz / r[:, None]

    # Split the unit-variance error between an independent and a spatially
    # correlated draw; the weights keep the total variance at sigma^2.
    eps = (np.sqrt(1.0 - p.range_corr_frac) * rng.standard_normal(n)
           + np.sqrt(p.range_corr_frac) * f_range)
    r_new = r + (p.range_sigma0_m + p.range_sigma_k * r) * eps

    p_drop = np.clip(
        p.dropout_p0 + p.dropout_p_range * (r / MAX_RANGE_M)
        + p.dropout_p_grazing * (1.0 - cos_inc) ** p.dropout_grazing_exp, 0.0, 1.0)
    # Threshold the correlated field at its own quantile, so dropouts hit at
    # rate p_drop but arrive in patches rather than as per-beam confetti.
    dropout_mask = f_drop < ndtri(p_drop)
    outlier_mask = (~dropout_mask) & (rng.random(n) < p.outlier_p)
    if np.any(outlier_mask):
        r_new[outlier_mask] += rng.uniform(
            p.outlier_range_min_m, p.outlier_range_max_m, size=int(outlier_mask.sum()))

    r_new *= sos_scale

    if p.range_quant_m > 0:
        r_new = np.round(r_new / p.range_quant_m) * p.range_quant_m
    r_new = np.clip(r_new, DEPTH_MIN_M, MAX_RANGE_M)

    # Near-field gate: drop returns nearer than min_range_m. Clears the
    # reverberation spray around the vehicle and any other near-field returns,
    # a tunable knob to pull the noised cloud back toward the clean geometry.
    if p.min_range_m > 0.0:
        dropout_mask = dropout_mask | (r_new < p.min_range_m)

    # Soft counterpart of the gate, thinning the near field instead of clearing it.
    fade_p = near_fade_drop_p(r_new, p)
    if np.any(fade_p > 0.0):
        # Independent draw, not the correlated field: survivors stay scattered.
        dropout_mask = dropout_mask | (rng.random(n) < fade_p)

    x_hat = np.array([1.0, 0.0, 0.0])
    y_hat = np.array([0.0, 1.0, 0.0])
    e_h = x_hat - u[:, 0:1] * u
    e_h /= np.linalg.norm(e_h, axis=1, keepdims=True)
    e_v = y_hat - u[:, 1:2] * u
    e_v /= np.linalg.norm(e_v, axis=1, keepdims=True)

    # Cross-range jitter = beam footprint = angular width * range. With
    # lat_sigma_beam_frac set, the width is the sim's local beam spacing (item 4);
    # otherwise the per-metre constants (uniform-spacing approximation).
    if p.lat_sigma_beam_frac > 0.0 and beam_h is not None:
        sig_h = p.lat_sigma_beam_frac * np.nan_to_num(beam_h) * r
        sig_v = p.lat_sigma_beam_frac * np.nan_to_num(beam_v) * r
    else:
        sig_h = p.lat_sigma_h_per_m * r
        sig_v = p.lat_sigma_v_per_m * r
    delta_h = rng.normal(0.0, 1.0, n) * sig_h
    delta_v = rng.normal(0.0, 1.0, n) * sig_v
    lateral = delta_h[:, None] * e_h + delta_v[:, None] * e_v

    out = u * r_new[:, None] + lateral
    out[dropout_mask] = np.nan
    return out


def main(args=None):
    rclpy.init(args=args)
    node = SonarNoiseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
