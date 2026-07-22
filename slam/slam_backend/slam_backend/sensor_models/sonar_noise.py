#!/usr/bin/env python3
"""Datasheet-grounded noise model for the depth-camera point cloud standing in
for a WaterLinked Sonar 3D-15 (1.2 MHz mode: 90x40 deg FOV, 0.2-15m range,
0.35/0.60 deg beam separation H/V, 1.5mm range resolution — see
ActiveSlam-Resources/3d-sonar datasheet).

Sits between depth_image_proc and every mapping/SLAM consumer:

    /cloud_in_raw --> sonar_noise --> /cloud_in

Per finite point with depth_min < range < NO_RETURN_RANGE_M (points at/beyond
the sensor's max range are "no-return" readings, left untouched so downstream
max-range filters keep discarding them):
  1. range noise along the viewing ray:   r += N(0, sigma0 + k*r)
  2. multipath outliers (late arrivals):  r += U(outlier_min, outlier_max)
  3. speed-of-sound scale error:          r *= 1 + eps, one draw per run
  4. range-bin quantization:              r = round(r / q) * q
  5. beam-spreading lateral jitter, perpendicular to the ray, scaled by range
     (the physical footprint of one beam grows with range)
  6. dropouts: replaced with NaN, at a probability that grows with range and
     with grazing incidence (specular surfaces reflect energy away from the
     transducer, so an oblique wall returns nothing — the dominant real-world
     no-return mechanism). Organized-cloud NaN convention per REP 118, already
     handled by every downstream consumer: tsdf_mapper's isfinite filter,
     pose_graph's skip_nans read, octomap_server's native NaN support.

Range error and dropout are drawn from smooth random fields rather than
independently per point, so both are correlated across neighbouring beams
(corr_length_px) and between consecutive pings (corr_rho_time). Independent
noise is the kind SLAM averages away, which flatters the pipeline; real sonar
speckle is partly frozen and real dropouts arrive in patches.

All-zero parameters (the `ideal` profile) republish the input unchanged.
"""
import os

import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.special import ndtri
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from slam_backend.sensor_models.noise_profiles import load_noise_profile, resolve_seed, NoiseProfile

DEPTH_MIN_M = 0.2
NO_RETURN_RANGE_M = 14.9   # points at/beyond this are left completely untouched
MAX_RANGE_M = 15.0

# Per-sensor offset added to a shared seed so co-launched sims (identical
# profile.seed) don't draw identical RNG streams (imu=+1, dvl=+2, pressure=+3).
SEED_OFFSET = 4


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


def incidence_cosine(grid):
    """|cos| of the angle between each beam and the local surface normal.

    `grid` is an organized (h, w, 3) sensor-frame point cloud. Returns 1.0
    (normal incidence, no grazing penalty) wherever the normal is degenerate —
    borders, NaN neighbours. Depth discontinuities yield edge-on normals and so
    a grazing penalty, which matches real sonar dropping returns at edges.
    """
    if grid.shape[0] < 3 or grid.shape[1] < 3:
        return np.ones(grid.shape[:2])
    normal = np.cross(np.gradient(grid, axis=1), np.gradient(grid, axis=0))
    norm = np.linalg.norm(normal, axis=2)
    ray_norm = np.linalg.norm(grid, axis=2)
    with np.errstate(invalid='ignore', divide='ignore'):
        cos_inc = np.abs(np.sum(normal * grid, axis=2) / (norm * ray_norm))
    return np.where(np.isfinite(cos_inc), np.clip(cos_inc, 0.0, 1.0), 1.0)


class SonarNoiseNode(Node):
    def __init__(self, **kwargs):
        super().__init__('sonar_noise', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('noise_seed', -1)
        self.declare_parameter('input_topic', '/cloud_in_raw')
        self.declare_parameter('output_topic', '/cloud_in')

        yaml_path = self.get_parameter('noise_profile_path').value
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            self.profile = NoiseProfile().sonar
            profile_seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.sonar
            profile_seed = full_profile.seed

        seed = resolve_seed(profile_seed, self.get_parameter('noise_seed').value, SEED_OFFSET)
        self._rng = np.random.default_rng(seed if seed != -1 else None)
        self._passthrough = not any([
            self.profile.range_sigma0_m, self.profile.range_sigma_k,
            self.profile.range_quant_m,
            self.profile.lat_sigma_h_per_m, self.profile.lat_sigma_v_per_m,
            self.profile.dropout_p0, self.profile.dropout_p_range,
            self.profile.dropout_p_grazing,
            self.profile.outlier_p, self.profile.sos_scale_error_pct,
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
        if self._passthrough or msg.point_step < 12 or msg.width * msg.height == 0:
            self._pub.publish(msg)
            return

        floats_per_point = msg.point_step // 4
        data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats_per_point).copy()
        xyz = data[:, :3].astype(np.float64)

        r = np.linalg.norm(xyz, axis=1)
        valid = np.isfinite(r) & (r > DEPTH_MIN_M) & (r < NO_RETURN_RANGE_M)
        if np.any(valid):
            shape = (msg.height, msg.width)
            organized = msg.height > 1 and msg.height * msg.width == len(r)
            cos_inc = incidence_cosine(xyz.reshape(*shape, 3)).ravel() if organized \
                else np.ones(len(r))
            field_shape = shape if organized else (len(r),)
            f_range, self._prev_white = correlated_field(
                self._rng, field_shape, self.profile.corr_length_px,
                self.profile.corr_rho_time, self._prev_white)
            f_drop, self._prev_white_drop = correlated_field(
                self._rng, field_shape, self.profile.corr_length_px,
                self.profile.corr_rho_time, self._prev_white_drop)
            xyz[valid] = apply_sonar_noise(
                xyz[valid], r[valid], cos_inc[valid],
                f_range.ravel()[valid], f_drop.ravel()[valid],
                self.profile, self._rng, self._sos_scale)

        data[:, :3] = xyz.astype(np.float32)
        msg.data = data.tobytes()
        self._pub.publish(msg)


def apply_sonar_noise(xyz, r, cos_inc, f_range, f_drop, p, rng, sos_scale=1.0):
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

    x_hat = np.array([1.0, 0.0, 0.0])
    y_hat = np.array([0.0, 1.0, 0.0])
    e_h = x_hat - u[:, 0:1] * u
    e_h /= np.linalg.norm(e_h, axis=1, keepdims=True)
    e_v = y_hat - u[:, 1:2] * u
    e_v /= np.linalg.norm(e_v, axis=1, keepdims=True)

    delta_h = rng.normal(0.0, p.lat_sigma_h_per_m * r)
    delta_v = rng.normal(0.0, p.lat_sigma_v_per_m * r)
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
