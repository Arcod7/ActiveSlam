#!/usr/bin/env python3
"""Automatic map snapshots for a SLAM eval run.

Complements benchmark.py (trajectory) and map_metrics.py (streaming scores) by
persisting the maps themselves to <output_dir>/maps/, so a finished run can be
compared against a reference map (e.g. eval/ground_truth/*.bt) offline.

Saved both periodically and once more on shutdown. Periodic overwrite is what
makes a 6 h unattended run safe: whether it ends on a clean SIGINT, a `timeout`,
or a crash, the newest snapshot is at most `period_s` old.

  belief_tsdf.npy / gt_tsdf.npy   Nx3 float32 surface points (tsdf backend)
  belief_octomap.bt / gt_octomap.bt   OctoMap binary tree (via octomap_saver)
  belief_projected_map.npy        2-D occupancy grid + origin/res sidecar

The .bt save shells out to octomap_saver_node rather than reserialising the
octree in-process: this environment has no python octomap bindings, and the
C++ saver is the same tool the manual save used.
"""
import json
import os
import subprocess
import time as pytime

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class MapSaverNode(Node):
    def __init__(self, **kwargs):
        super().__init__('map_saver', **kwargs)

        self.declare_parameter('output_dir', '')
        self.declare_parameter('mapper', 'tsdf')
        self.declare_parameter('period_s', 180.0)
        # octomap_saver spawns a node that calls this service; blank disables
        # the .bt save (e.g. if octomap_server is not running).
        self.declare_parameter('belief_octomap_service', '/octomap_binary')
        self.declare_parameter('gt_octomap_service', '')

        out_dir = self.get_parameter('output_dir').value
        if not out_dir:
            out_dir = os.getcwd()
        self._maps_dir = os.path.join(out_dir, 'maps')
        os.makedirs(self._maps_dir, exist_ok=True)

        self._mapper = self.get_parameter('mapper').value
        self._belief_octomap_srv = self.get_parameter('belief_octomap_service').value
        self._gt_octomap_srv = self.get_parameter('gt_octomap_service').value

        self._latest_belief_cloud = None
        self._latest_gt_cloud = None
        self._latest_belief_grid = None
        self._latest_gt_grid = None

        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        if self._mapper == 'tsdf':
            self.create_subscription(PointCloud2, '/tsdf/surface_cloud',
                                     self._belief_cloud_cb, 1)
            self.create_subscription(PointCloud2, '/gt/tsdf/surface_cloud',
                                     self._gt_cloud_cb, 1)
        else:
            self.create_subscription(OccupancyGrid, '/projected_map',
                                     self._belief_grid_cb, latched_qos)
            self.create_subscription(OccupancyGrid, '/gt/projected_map',
                                     self._gt_grid_cb, latched_qos)

        period_s = self.get_parameter('period_s').value
        self.create_timer(period_s, self.save_all)
        self.get_logger().info(
            f'MapSaver started. mapper={self._mapper}, saving to {self._maps_dir} '
            f'every {period_s:.0f}s and on shutdown')

    def _belief_cloud_cb(self, msg):
        self._latest_belief_cloud = msg

    def _gt_cloud_cb(self, msg):
        self._latest_gt_cloud = msg

    def _belief_grid_cb(self, msg):
        self._latest_belief_grid = msg

    def _gt_grid_cb(self, msg):
        self._latest_gt_grid = msg

    def _save_cloud(self, msg, name):
        if msg is None:
            return
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=['x', 'y', 'z'], skip_nans=True).astype(np.float32)
            tmp = os.path.join(self._maps_dir, name + '.tmp.npy')
            np.save(tmp, pts)
            os.replace(tmp, os.path.join(self._maps_dir, name + '.npy'))
        except Exception as e:                      # never let a save kill the node
            self.get_logger().warn(f'{name}: cloud save failed: {e}')

    def _save_grid(self, msg, name):
        if msg is None:
            return
        try:
            grid = np.array(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width)
            tmp = os.path.join(self._maps_dir, name + '.tmp.npy')
            np.save(tmp, grid)
            os.replace(tmp, os.path.join(self._maps_dir, name + '.npy'))
            meta = {'resolution': msg.info.resolution,
                    'origin_x': msg.info.origin.position.x,
                    'origin_y': msg.info.origin.position.y,
                    'width': msg.info.width, 'height': msg.info.height}
            with open(os.path.join(self._maps_dir, name + '.json'), 'w') as f:
                json.dump(meta, f)
        except Exception as e:
            self.get_logger().warn(f'{name}: grid save failed: {e}')

    def _save_octomap(self, service, name):
        if not service:
            return
        path = os.path.join(self._maps_dir, name + '.bt')
        try:
            args = ['ros2', 'run', 'octomap_server', 'octomap_saver_node',
                    '--ros-args', '-p', f'octomap_path:={path}']
            if service != '/octomap_binary':
                args[4:4] = ['-r', f'octomap_binary:={service}']
            subprocess.run(args, timeout=30, capture_output=True, text=True)
        except Exception as e:
            self.get_logger().warn(f'{name}: octomap save failed: {e}')

    def save_all(self):
        if self._mapper == 'tsdf':
            self._save_cloud(self._latest_belief_cloud, 'belief_tsdf')
            self._save_cloud(self._latest_gt_cloud, 'gt_tsdf')
        else:
            self._save_grid(self._latest_belief_grid, 'belief_projected_map')
            self._save_grid(self._latest_gt_grid, 'gt_projected_map')
        # The belief octomap runs in both backends (frontier planning map under
        # tsdf); the gt octomap only when the gt map mirrors octomap.
        self._save_octomap(self._belief_octomap_srv, 'belief_octomap')
        self._save_octomap(self._gt_octomap_srv, 'gt_octomap')
        self.get_logger().info('map snapshot written', throttle_duration_sec=60.0)

    def destroy_node(self):
        try:
            self.save_all()                          # one last snapshot on the way out
        except Exception as e:
            self.get_logger().warn(f'final save failed: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
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
