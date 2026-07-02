"""Frontier extractor node.

Subscribes to /projected_map (the 2-D OctoMap projection) and /StoneFish/Odometry.
On each tick it:
  1. Finds occupied-cell ↔ unknown-cell boundaries (frontiers) and clusters them.
  2. Lets a GoalManager pick the next goal, with hysteresis, stuck detection,
     and a timed blacklist for unreachable goals.
  3. Publishes the goal on /frontier_slam/goal and the A*-planned path on
     /frontier_slam/path.  Visualisation is delegated to FrontierVisualizer.

The goal's Z is set to the robot's current Z — but only for marker placement.
The waypoint controller maintains its own fixed depth setpoint and ignores
goal.point.z.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from frontier_slam.control_utils import yaw_from_quat
from frontier_slam.frontier_detection import find_frontier_clusters
from frontier_slam.goal_manager import GoalManager
from frontier_slam.path_planner import CostGrid, build_cost_grid, find_path
from frontier_slam.session_log import open_session_log
from frontier_slam.visualizer import FrontierVisualizer


_LOG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'logs',
)

CSV_COLUMNS = [
    't_ros', 'rx', 'ry', 'rz', 'gx', 'gy',
    'dist_m', 'clusters', 'stuck_pct', 'blacklist_n',
    'free_cells', 'occ_cells', 'mapped_cells', 'event',
]


class FrontierExtractor(Node):
    MIN_CLUSTER_CELLS = 1
    UPDATE_HZ         = 0.5
    REPLAN_HZ         = 3.0
    REPLAN_FAIL_MAX   = 6    # consecutive A* failures before blacklisting goal as unreachable

    def __init__(self):
        super().__init__('frontier_extractor')

        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        odom_topic = str(self.get_parameter('odom_topic').value)

        self._map: OccupancyGrid | None = None
        self._robot_pos: np.ndarray | None = None
        self._current_goal_xy: np.ndarray | None = None
        self._current_path: list = []
        self._cg: CostGrid | None = None
        self._robot_yaw: float = 0.0
        self._robot_speed: float = 0.0
        self._last_stuck_pct: int = 0
        self._astar_fail_count: int = 0
        self._astar_fail_goal: np.ndarray | None = None

        self._goals = GoalManager(
            min_explore_dist=3.0,
            goal_vanish_dist=3.0,
            goal_radius=2.0,
            stuck_timeout=30.0,
            stuck_min_progress=0.5,
            blacklist_duration=30.0,
            arrival_blacklist_duration=20.0,
        )

        self._log = open_session_log('extractor', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(OccupancyGrid, '/projected_map', self._map_cb,  1)
        self.create_subscription(Odometry,      odom_topic,      self._odom_cb, 10)
        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal', 1)
        self._path_pub = self.create_publisher(Path,         '/frontier_slam/path', 1)
        self._viz      = FrontierVisualizer(self)

        self.create_timer(1.0 / self.UPDATE_HZ,  self._update)
        self.create_timer(1.0 / self.REPLAN_HZ,  self._replan)
        self.get_logger().info(f'frontier_extractor ready — logging to {self._log.path}')

    # ------------------------------------------------------------------
    # ROS callbacks
    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._robot_pos   = np.array([p.x, p.y, p.z])
        self._robot_yaw   = yaw_from_quat(msg.pose.pose.orientation)
        v = msg.twist.twist.linear
        self._robot_speed = math.hypot(v.x, v.y)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Map statistics
    def _map_stats(self) -> tuple[int, int, int]:
        """Return (free_cells, occupied_cells, mapped_cells) from current map."""
        data = np.asarray(self._map.data, dtype=np.int8)
        free = int(np.sum(data == 0))
        occ  = int(np.sum(data == 100))
        return free, occ, free + occ

    # ------------------------------------------------------------------
    # Main loop
    def _update(self) -> None:
        if self._map is None or self._robot_pos is None:
            return

        free_cells, occ_cells, mapped_cells = self._map_stats()

        clusters = find_frontier_clusters(self._map, self.MIN_CLUSTER_CELLS)
        if not clusters:
            self.get_logger().info('No frontiers found', throttle_duration_sec=5.0)
            return

        # Tag each cluster with its distance from the robot.
        for c in clusters:
            c.distance = float(np.hypot(c.wx - self._robot_pos[0],
                                        c.wy - self._robot_pos[1]))

        selection = self._goals.select(clusters, self._robot_pos[:2], self._now())

        if selection is None:
            self.get_logger().info(
                'All frontiers within reach — scanning for new areas',
                throttle_duration_sec=5.0,
            )
            return

        # Surface the stuck event for the human-readable ROS log.
        if selection.event == 'STUCK_BLACKLIST' and selection.stuck_goal is not None:
            sx, sy, sec, prog = selection.stuck_goal
            self.get_logger().warn(
                f'STUCK: goal ({sx:.1f},{sy:.1f}) unreachable after {sec:.0f}s '
                f'(progress={prog:.2f}m) — blacklisting'
            )

        if selection.event == 'ALL_BLACKLISTED' or math.isnan(selection.gx):
            self.get_logger().info(
                'All candidates blacklisted — waiting', throttle_duration_sec=5.0,
            )
            self._current_goal_xy = None
            self._current_path    = []
            self._write_csv(float('nan'), float('nan'), float('nan'),
                            len(clusters), 0,
                            free_cells, occ_cells, mapped_cells, selection.event or 'ALL_BLACKLISTED')
            return

        self._last_stuck_pct = selection.stuck_pct
        self._publish_goal(selection.gx, selection.gy, clusters)
        dist = float(np.hypot(selection.gx - self._robot_pos[0],
                              selection.gy - self._robot_pos[1]))
        self.get_logger().info(
            f'robot=({self._robot_pos[0]:.1f},{self._robot_pos[1]:.1f},{self._robot_pos[2]:.1f})  '
            f'goal=({selection.gx:.1f},{selection.gy:.1f})  dist={dist:.1f}m  '
            f'clusters={len(clusters)}  stuck={selection.stuck_pct}%  '
            f'blacklist={self._goals.blacklist_size}  '
            f'mapped={mapped_cells}(free={free_cells},occ={occ_cells})'
        )
        self._write_csv(selection.gx, selection.gy, dist,
                        len(clusters), selection.stuck_pct,
                        free_cells, occ_cells, mapped_cells, selection.event)

    # ------------------------------------------------------------------
    # Publishing
    def _publish_goal(self, gx: float, gy: float, clusters: list) -> None:
        self._current_goal_xy = np.array([gx, gy])
        goal = PointStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x, goal.point.y, goal.point.z = gx, gy, float(self._robot_pos[2])
        self._goal_pub.publish(goal)
        self._viz.publish_markers(clusters, gx, gy, self._robot_pos)

    def _replan(self) -> None:
        if self._map is None or self._robot_pos is None:
            return

        self._cg = build_cost_grid(self._map)

        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = 'world_ned'
        if self._current_goal_xy is not None and not np.any(np.isnan(self._current_goal_xy)):
            # Reset failure counter when the goal changes.
            if (self._astar_fail_goal is None or
                    np.hypot(self._current_goal_xy[0] - self._astar_fail_goal[0],
                             self._current_goal_xy[1] - self._astar_fail_goal[1]) > 1.0):
                self._astar_fail_count = 0
                self._astar_fail_goal  = self._current_goal_xy.copy()

            waypoints = find_path(self._cg, self._robot_pos[:2], self._current_goal_xy)
            self._current_path = waypoints

            if waypoints:
                self._astar_fail_count = 0
                gz = float(self._robot_pos[2])
                for wx, wy in waypoints:
                    ps = PoseStamped()
                    ps.header = path.header
                    ps.pose.position.x = wx
                    ps.pose.position.y = wy
                    ps.pose.position.z = gz
                    ps.pose.orientation.w = 1.0
                    path.poses.append(ps)
            else:
                self._astar_fail_count += 1
                gxy = self._current_goal_xy
                if self._astar_fail_count >= self.REPLAN_FAIL_MAX:
                    self.get_logger().warn(
                        f'A* failed {self.REPLAN_FAIL_MAX}× for '
                        f'({gxy[0]:.1f},{gxy[1]:.1f}) — blacklisting as unreachable'
                    )
                    self._goals.mark_unreachable(gxy, self._now())
                    self._current_goal_xy  = None
                    self._current_path     = []
                    self._astar_fail_count = 0
                    self._astar_fail_goal  = None
                else:
                    self.get_logger().warn(
                        f'A* found no path to ({gxy[0]:.1f},{gxy[1]:.1f}) '
                        f'({self._astar_fail_count}/{self.REPLAN_FAIL_MAX})'
                    )
        self._path_pub.publish(path)
        self._viz.publish_inflated_map(self._cg, self._map)
        self._viz.publish_debug_image(
            self._cg, self._map,
            self._robot_pos, self._robot_yaw, self._robot_speed,
            self._current_path, self._current_goal_xy, self._last_stuck_pct,
        )

    # ------------------------------------------------------------------
    # Logging
    def _write_csv(self, gx, gy, dist, n_clusters, stuck_pct,
                   free_cells, occ_cells, mapped_cells, event) -> None:
        rp = self._robot_pos
        self._log.write([
            self._now(),
            float(rp[0]), float(rp[1]), float(rp[2]),
            float(gx), float(gy), float(dist),
            n_clusters, stuck_pct, self._goals.blacklist_size,
            free_cells, occ_cells, mapped_cells, event,
        ])


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExtractor()
    try:
        rclpy.spin(node)
    finally:
        node._log.close()
        node.destroy_node()
        rclpy.shutdown()
