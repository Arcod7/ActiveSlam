#!/usr/bin/env python3
"""Streaming map-quality metrics: belief map vs. the ground-truth reference
map (gt_map.launch.py, slam:=slam only). Complements benchmark.py, which is
pose-only — nothing currently scores the map itself (see docs/ROADMAP.md).

Octomap backend: /projected_map vs /gt/projected_map (nav_msgs/OccupancyGrid,
2-D). Chosen over /octomap_binary (no python octomap bindings available in
this environment) and over /occupied_cells_vis_array (a viz topic, fragile to
couple metrics to). Grids are aligned by their MapMetaData origin offset
(both instances share the same resolution), then compared over the
overlapping window:
  iou_occ  = |occupied_belief ∩ occupied_gt| / |occupied_belief ∪ occupied_gt|
  coverage = |known_belief ∩ known_gt| / |known_gt|   (known = cell >= 0)

TSDF backend: /tsdf/surface_cloud vs /gt/tsdf/surface_cloud (both world-frame
PointCloud2, ~1 Hz). Subsampled to at most `max_points`, compared via a
nearest-neighbour KD-tree:
  chamfer  = mean(NN dist belief->gt) + mean(NN dist gt->belief)
  coverage = fraction of GT surface points with a belief neighbour within
             tsdf_coverage_radius_m

Appends flushed-per-write rows to <output_dir>/map_metrics.csv (blank fields
for the metric family that doesn't apply to the configured backend).
"""
import os
import time as pytime

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial import cKDTree

from eval_tools.run_paths import new_run_dir


def _align_occupancy_grids(grid_belief: OccupancyGrid, grid_gt: OccupancyGrid):
    """Return (sub_belief, sub_gt) int16 2-D arrays over the overlapping
    window, or None if there's no overlap. Assumes equal resolution."""
    res = grid_gt.info.resolution
    off_x = round((grid_belief.info.origin.position.x - grid_gt.info.origin.position.x) / res)
    off_y = round((grid_belief.info.origin.position.y - grid_gt.info.origin.position.y) / res)

    w_b, h_b = grid_belief.info.width, grid_belief.info.height
    w_g, h_g = grid_gt.info.width, grid_gt.info.height
    if w_b == 0 or h_b == 0 or w_g == 0 or h_g == 0:
        return None

    data_b = np.array(grid_belief.data, dtype=np.int16).reshape(h_b, w_b)
    data_g = np.array(grid_gt.data, dtype=np.int16).reshape(h_g, w_g)

    # belief column i / row j maps to gt column i+off_x / row j+off_y.
    xs_g = np.arange(w_b) + off_x
    ys_g = np.arange(h_b) + off_y
    valid_x = (xs_g >= 0) & (xs_g < w_g)
    valid_y = (ys_g >= 0) & (ys_g < h_g)
    if not np.any(valid_x) or not np.any(valid_y):
        return None

    bx = np.flatnonzero(valid_x)
    by = np.flatnonzero(valid_y)
    bx0, bx1 = bx[0], bx[-1] + 1
    by0, by1 = by[0], by[-1] + 1

    sub_b = data_b[by0:by1, bx0:bx1]
    sub_g = data_g[ys_g[by0]:ys_g[by1 - 1] + 1, xs_g[bx0]:xs_g[bx1 - 1] + 1]
    return sub_b, sub_g


def _occupancy_metrics(grid_belief: OccupancyGrid, grid_gt: OccupancyGrid, occ_threshold: int):
    aligned = _align_occupancy_grids(grid_belief, grid_gt)
    if aligned is None:
        return None
    sub_b, sub_g = aligned

    occ_b = sub_b > occ_threshold
    occ_g = sub_g > occ_threshold
    union = np.count_nonzero(occ_b | occ_g)
    iou_occ = (np.count_nonzero(occ_b & occ_g) / union) if union > 0 else float('nan')

    known_b = sub_b >= 0
    known_g = sub_g >= 0
    n_known_g = np.count_nonzero(known_g)
    coverage = (np.count_nonzero(known_b & known_g) / n_known_g) if n_known_g > 0 else float('nan')

    return coverage, iou_occ, sub_b.size, sub_g.size


def _subsample(pts: np.ndarray, max_points: int, rng: np.random.Generator) -> np.ndarray:
    if len(pts) <= max_points:
        return pts
    idx = rng.choice(len(pts), max_points, replace=False)
    return pts[idx]


def _tsdf_metrics(pts_belief: np.ndarray, pts_gt: np.ndarray, coverage_radius_m: float,
                   max_points: int, rng: np.random.Generator):
    if len(pts_belief) == 0 or len(pts_gt) == 0:
        return None
    pts_belief = _subsample(pts_belief, max_points, rng)
    pts_gt = _subsample(pts_gt, max_points, rng)

    tree_belief = cKDTree(pts_belief)
    tree_gt = cKDTree(pts_gt)
    d_b2g, _ = tree_gt.query(pts_belief)
    d_g2b, _ = tree_belief.query(pts_gt)

    chamfer = float(np.mean(d_b2g) + np.mean(d_g2b))
    rmse_b2g = float(np.sqrt(np.mean(d_b2g ** 2)))
    rmse_g2b = float(np.sqrt(np.mean(d_g2b ** 2)))
    coverage = float(np.mean(d_g2b < coverage_radius_m))

    return coverage, chamfer, rmse_b2g, rmse_g2b, len(pts_belief), len(pts_gt)


class MapMetricsNode(Node):
    def __init__(self, **kwargs):
        super().__init__('map_metrics', **kwargs)

        self.declare_parameter('output_dir', '')
        self.declare_parameter('mapper', 'octomap')
        self.declare_parameter('period_s', 5.0)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('tsdf_coverage_radius_m', 0.4)
        self.declare_parameter('max_points', 20000)

        out_dir = self.get_parameter('output_dir').value
        if not out_dir:
            out_dir = new_run_dir(pytime.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(out_dir, exist_ok=True)

        self._mapper = self.get_parameter('mapper').value
        self._occ_threshold = self.get_parameter('occ_threshold').value
        self._coverage_radius_m = self.get_parameter('tsdf_coverage_radius_m').value
        self._max_points = self.get_parameter('max_points').value
        self._rng = np.random.default_rng()

        self._metrics_file = open(os.path.join(out_dir, 'map_metrics.csv'), 'w')
        self._metrics_file.write(
            't,backend,coverage,iou_occ,chamfer,rmse_belief_to_gt,rmse_gt_to_belief,'
            'n_belief,n_gt\n')

        self._latest_belief_grid = None
        self._latest_gt_grid = None
        self._latest_belief_cloud = None
        self._latest_gt_cloud = None

        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        if self._mapper == 'octomap':
            self.create_subscription(OccupancyGrid, '/projected_map', self._belief_grid_cb, latched_qos)
            self.create_subscription(OccupancyGrid, '/gt/projected_map', self._gt_grid_cb, latched_qos)
        elif self._mapper == 'tsdf':
            self.create_subscription(PointCloud2, '/tsdf/surface_cloud', self._belief_cloud_cb, 1)
            self.create_subscription(PointCloud2, '/gt/tsdf/surface_cloud', self._gt_cloud_cb, 1)

        period_s = self.get_parameter('period_s').value
        self.create_timer(period_s, self._compute_metrics)

        self.get_logger().info(f'MapMetrics started. mapper={self._mapper}, writing to {out_dir}')

    def _belief_grid_cb(self, msg: OccupancyGrid):
        self._latest_belief_grid = msg

    def _gt_grid_cb(self, msg: OccupancyGrid):
        self._latest_gt_grid = msg

    def _belief_cloud_cb(self, msg: PointCloud2):
        self._latest_belief_cloud = msg

    def _gt_cloud_cb(self, msg: PointCloud2):
        self._latest_gt_cloud = msg

    def _compute_metrics(self):
        if self._mapper == 'octomap':
            self._compute_occupancy_metrics()
        elif self._mapper == 'tsdf':
            self._compute_tsdf_metrics()

    def _compute_occupancy_metrics(self):
        if self._latest_belief_grid is None or self._latest_gt_grid is None:
            return
        result = _occupancy_metrics(self._latest_belief_grid, self._latest_gt_grid,
                                     self._occ_threshold)
        if result is None:
            return
        coverage, iou_occ, n_belief, n_gt = result
        t = self.get_clock().now().nanoseconds * 1e-9
        self._metrics_file.write(
            f'{t:.6f},octomap,{coverage:.6f},{iou_occ:.6f},,,,{n_belief},{n_gt}\n')
        self._metrics_file.flush()

    def _compute_tsdf_metrics(self):
        if self._latest_belief_cloud is None or self._latest_gt_cloud is None:
            return
        pts_belief = point_cloud2.read_points_numpy(
            self._latest_belief_cloud, field_names=['x', 'y', 'z'], skip_nans=True).astype(np.float64)
        pts_gt = point_cloud2.read_points_numpy(
            self._latest_gt_cloud, field_names=['x', 'y', 'z'], skip_nans=True).astype(np.float64)
        result = _tsdf_metrics(pts_belief, pts_gt, self._coverage_radius_m,
                                self._max_points, self._rng)
        if result is None:
            return
        coverage, chamfer, rmse_b2g, rmse_g2b, n_belief, n_gt = result
        t = self.get_clock().now().nanoseconds * 1e-9
        self._metrics_file.write(
            f'{t:.6f},tsdf,{coverage:.6f},,{chamfer:.6f},{rmse_b2g:.6f},{rmse_g2b:.6f},'
            f'{n_belief},{n_gt}\n')
        self._metrics_file.flush()

    def destroy_node(self):
        self._metrics_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapMetricsNode()
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
