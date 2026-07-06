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
  3. range-bin quantization:              r = round(r / q) * q
  4. beam-spreading lateral jitter, perpendicular to the ray, scaled by range
     (the physical footprint of one beam grows with range)
  5. dropouts: replaced with NaN, at a probability that grows with range
     (organized-cloud NaN convention per REP 118 — already handled by every
     downstream consumer: tsdf_mapper's isfinite filter, pose_graph's
     skip_nans read, and octomap_server's native NaN support)

All-zero parameters (the `ideal` profile) republish the input unchanged.
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from slam_backend.sensor_models.noise_profiles import load_noise_profile, NoiseProfile

DEPTH_MIN_M = 0.2
NO_RETURN_RANGE_M = 14.9   # points at/beyond this are left completely untouched
MAX_RANGE_M = 15.0


class SonarNoiseNode(Node):
    def __init__(self, **kwargs):
        super().__init__('sonar_noise', **kwargs)

        self.declare_parameter('noise_profile_path', '')
        self.declare_parameter('input_topic', '/cloud_in_raw')
        self.declare_parameter('output_topic', '/cloud_in')

        yaml_path = self.get_parameter('noise_profile_path').value
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().warn(f"Invalid noise profile path: '{yaml_path}', using ideal defaults.")
            self.profile = NoiseProfile().sonar
            seed = -1
        else:
            full_profile = load_noise_profile(yaml_path)
            self.profile = full_profile.sonar
            seed = full_profile.seed

        self._rng = np.random.default_rng(seed if seed != -1 else None)
        self._passthrough = not any([
            self.profile.range_sigma0_m, self.profile.range_sigma_k,
            self.profile.range_quant_m,
            self.profile.lat_sigma_h_per_m, self.profile.lat_sigma_v_per_m,
            self.profile.dropout_p0, self.profile.dropout_p_range,
            self.profile.outlier_p,
        ])

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
            xyz[valid] = self._apply_noise(xyz[valid], r[valid])

        data[:, :3] = xyz.astype(np.float32)
        msg.data = data.tobytes()
        self._pub.publish(msg)

    def _apply_noise(self, xyz: np.ndarray, r: np.ndarray) -> np.ndarray:
        n = len(r)
        p = self.profile
        u = xyz / r[:, None]

        r_new = r + self._rng.normal(0.0, p.range_sigma0_m + p.range_sigma_k * r)

        dropout_mask = self._rng.random(n) < (p.dropout_p0 + p.dropout_p_range * (r / MAX_RANGE_M))
        outlier_mask = (~dropout_mask) & (self._rng.random(n) < p.outlier_p)
        if np.any(outlier_mask):
            r_new[outlier_mask] += self._rng.uniform(
                p.outlier_range_min_m, p.outlier_range_max_m, size=int(outlier_mask.sum()))

        if p.range_quant_m > 0:
            r_new = np.round(r_new / p.range_quant_m) * p.range_quant_m
        r_new = np.clip(r_new, DEPTH_MIN_M, MAX_RANGE_M)

        x_hat = np.array([1.0, 0.0, 0.0])
        y_hat = np.array([0.0, 1.0, 0.0])
        e_h = x_hat - u[:, 0:1] * u
        e_h /= np.linalg.norm(e_h, axis=1, keepdims=True)
        e_v = y_hat - u[:, 1:2] * u
        e_v /= np.linalg.norm(e_v, axis=1, keepdims=True)

        delta_h = self._rng.normal(0.0, p.lat_sigma_h_per_m * r)
        delta_v = self._rng.normal(0.0, p.lat_sigma_v_per_m * r)
        lateral = delta_h[:, None] * e_h + delta_v[:, None] * e_v

        out = u * r_new[:, None] + lateral
        out[dropout_mask] = np.nan
        return out


def main(args=None):
    rclpy.init(args=args)
    node = SonarNoiseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
