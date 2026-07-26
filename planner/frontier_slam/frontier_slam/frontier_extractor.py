"""Frontier extractor node.

Subscribes to /projected_map (the 2-D OctoMap projection) and /StoneFish/Odometry.
On each tick it:
  1. Finds occupied-cell ↔ unknown-cell boundaries (frontiers) and clusters them.
  2. Lets a GoalManager pick the next goal, with hysteresis, stuck detection,
     and a timed blacklist for unreachable goals.
  3. Publishes the goal on /frontier_slam/goal and the A*-planned path on
     /frontier_slam/path.  Visualisation is delegated to FrontierVisualizer.

The goal's Z is the cruise depth (`depth_setpoint` launch arg, locked from
the first odom reading if unset) — never the robot's own live Z (Change 12:
that was a feedback loop that let the robot sink undetected). waypoint_
controller drives depth off whichever goal is active, this one included, so
that value has to be a real, externally-anchored target, not a moving copy
of the robot's own position.

/frontier_slam/suspend (std_msgs/Bool) lets an external planner (see
revisit_planner.py) take over goal publication temporarily: while suspended,
_update stops publishing its own goal and mutating GoalManager state (so it
can't fight the external planner), but _replan keeps running at REPLAN_HZ so
the externally-adopted goal still gets an obstacle-aware A* path. The
external goal is adopted from the same /frontier_slam/goal topic this node
itself publishes to — own-publication callbacks are ignored while not
suspended, since rclpy delivers a node's own publications to its own
subscriptions.
"""
import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String

from frontier_slam.control_utils import yaw_from_quat
from frontier_slam.frontier_detection import (
    find_frontier_clusters,
    frontier_cell_points,
    standoff_point_from_tsdf_surface,
)
from frontier_slam.goal_manager import GoalManager
from frontier_slam.path_planner import (
    CostGrid, HARD_INFLATION_M, INFLATION_M, PLAN_INFLATION_M,
    build_cost_grid, find_path,
)
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
    TSDF_SOLID_STALE_S = 5.0
    TSDF_SURFACE_STALE_S = 5.0

    def __init__(self):
        super().__init__('frontier_extractor')

        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        self.declare_parameter('depth_setpoint', -1.0)
        self.declare_parameter('motion_status_topic', '/motion/status')
        self.declare_parameter('tsdf_solid_points_topic', '/tsdf/occupied_voxels')
        self.declare_parameter('tsdf_solid_containment_radius_m', 0.20)
        self.declare_parameter('tsdf_surface_normals_topic', '/tsdf/surface_normals_cloud')
        self.declare_parameter('tsdf_frontier_standoff_m', 1.0)
        self.declare_parameter('tsdf_surface_normal_max_distance_m', 1.0)
        # Display only: the Z range one /projected_map cell collapses, drawn as
        # the band_columns marker. Must match the mapper that publishes the map
        # (tsdf_mapper's projected_map_band_m, or octomap's occupancy_min/max_z).
        self.declare_parameter('projected_map_band_m', 3.0)
        self.declare_parameter('hard_inflation_m', HARD_INFLATION_M)
        self.declare_parameter('inflation_m', INFLATION_M)
        self.declare_parameter('plan_inflation_m', PLAN_INFLATION_M)
        # Survey working area. Frontier exploration in open water has no
        # natural bound -- it follows free space outward indefinitely, so a
        # run spends its budget leaving the structure instead of surveying it.
        # <= 0 disables the bound, which is the default.
        self.declare_parameter('survey_radius_m', 0.0)
        # NaN centre = the deployment point, taken from the first odometry
        # fix. A survey area is defined relative to where the vehicle was put
        # in the water unless the operator names a different centre.
        self.declare_parameter('survey_center_x', float('nan'))
        self.declare_parameter('survey_center_y', float('nan'))
        odom_topic = str(self.get_parameter('odom_topic').value)
        depth_arg = float(self.get_parameter('depth_setpoint').value)
        # Same value, same launch arg, as waypoint_controller's own
        # depth_setpoint (frontier_slam.launch.py passes both from a single
        # `depth` argument) — the goal Z this node publishes for its own
        # picks, so waypoint_controller's depth control (goal.point.z) has a
        # real, non-self-referential target to drive to instead of the
        # robot's own live Z (Change 12: that was a feedback loop that let
        # the robot sink undetected). -1 (unset) locks to the first odom
        # reading here, independently of waypoint_controller's own lock —
        # both start from the same odom stream, so they converge regardless.
        self._cruise_z: float | None = None if depth_arg < 0 else depth_arg
        motion_status_topic = str(self.get_parameter('motion_status_topic').value)
        tsdf_solid_points_topic = str(
            self.get_parameter('tsdf_solid_points_topic').value)
        self._tsdf_solid_radius = max(0.0, float(
            self.get_parameter('tsdf_solid_containment_radius_m').value))
        tsdf_surface_normals_topic = str(
            self.get_parameter('tsdf_surface_normals_topic').value)
        self._tsdf_frontier_standoff = max(0.0, float(
            self.get_parameter('tsdf_frontier_standoff_m').value))
        self._tsdf_surface_normal_max_distance = max(0.0, float(
            self.get_parameter('tsdf_surface_normal_max_distance_m').value))
        self._projected_map_band = abs(float(
            self.get_parameter('projected_map_band_m').value))
        self._hard_inflation_m = float(self.get_parameter('hard_inflation_m').value)
        self._inflation_m = float(self.get_parameter('inflation_m').value)
        self._plan_inflation_m = float(self.get_parameter('plan_inflation_m').value)

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
        self._suspended: bool = False
        self._tsdf_solid_points = np.empty((0, 3), dtype=np.float64)
        self._tsdf_solid_tree: cKDTree | None = None
        self._tsdf_solid_received_at: float | None = None
        self._tsdf_surface_points = np.empty((0, 3), dtype=np.float64)
        self._tsdf_surface_normals = np.empty((0, 3), dtype=np.float64)
        self._tsdf_surface_tree: cKDTree | None = None
        self._tsdf_surface_received_at: float | None = None

        survey_radius = float(self.get_parameter('survey_radius_m').value)
        cx = float(self.get_parameter('survey_center_x').value)
        cy = float(self.get_parameter('survey_center_y').value)
        self._survey_center_fixed = not (math.isnan(cx) or math.isnan(cy))
        self._goals = GoalManager(
            min_explore_dist=3.0,
            goal_vanish_dist=3.0,
            goal_radius=2.0,
            stuck_timeout=30.0,
            stuck_min_progress=0.5,
            blacklist_duration=30.0,
            arrival_blacklist_duration=20.0,
            survey_radius=max(0.0, survey_radius),
            survey_center=((cx, cy) if self._survey_center_fixed else None),
        )
        if survey_radius > 0.0:
            where = (f'({cx:.1f}, {cy:.1f})' if self._survey_center_fixed
                     else 'the deployment point')
            self.get_logger().info(
                f'Survey area: {survey_radius:.1f} m around {where}; frontier '
                'goals outside it are not candidates')

        self._log = open_session_log('extractor', CSV_COLUMNS, _LOG_DIR)

        self.create_subscription(OccupancyGrid, '/projected_map', self._map_cb,  1)
        self.create_subscription(Odometry,      odom_topic,      self._odom_cb, 10)
        self.create_subscription(Bool, '/frontier_slam/suspend', self._suspend_cb, 1)
        self.create_subscription(PointStamped, '/frontier_slam/goal', self._external_goal_cb, 1)
        self.create_subscription(String, motion_status_topic, self._motion_status_cb, 1)
        self.create_subscription(
            PointCloud2, tsdf_solid_points_topic, self._tsdf_solid_cb, 1)
        self.create_subscription(
            PointCloud2, tsdf_surface_normals_topic, self._tsdf_surface_normals_cb, 1)
        self._goal_pub = self.create_publisher(PointStamped, '/frontier_slam/goal', 1)
        self._path_pub = self.create_publisher(Path,         '/frontier_slam/path', 1)
        self._viz      = FrontierVisualizer(self)

        self.create_timer(1.0 / self.UPDATE_HZ,  self._update)
        self.create_timer(1.0 / self.REPLAN_HZ,  self._replan)
        self.get_logger().info(
            f'frontier_extractor ready — TSDF solid rejection={self._tsdf_solid_radius:.2f}m '
            f'standoff={self._tsdf_frontier_standoff:.2f}m '
            f'solid_topic={tsdf_solid_points_topic} '
            f'normals_topic={tsdf_surface_normals_topic} — logging to {self._log.path}')

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
        if self._cruise_z is None:   # first odom, no depth_setpoint launch arg
            self._cruise_z = float(p.z)
        # Anchor the survey area on the deployment point, once, unless the
        # operator named a centre explicitly.
        if (self._goals.survey_radius > 0.0
                and self._goals.survey_center is None
                and not self._survey_center_fixed):
            self._goals.survey_center = (float(p.x), float(p.y))
            self.get_logger().info(
                f'Survey area anchored at deployment point '
                f'({p.x:.1f}, {p.y:.1f})')

    def _tsdf_solid_cb(self, msg: PointCloud2) -> None:
        points = _parse_xyz_cloud(msg)
        if points is None:
            self.get_logger().warn(
                'TSDF solid cloud has no readable float32 x/y/z fields — ignoring',
                throttle_duration_sec=10.0)
            return
        self._tsdf_solid_points = points
        self._tsdf_solid_tree = cKDTree(points) if len(points) else None
        self._tsdf_solid_received_at = self._now()

    def _tsdf_surface_normals_cb(self, msg: PointCloud2) -> None:
        parsed = _parse_xyz_normals_cloud(msg)
        if parsed is None:
            self.get_logger().warn(
                'TSDF normal cloud has no readable float32 x/y/z/normal fields — ignoring',
                throttle_duration_sec=10.0)
            return
        points, normals = parsed
        self._tsdf_surface_points = points
        self._tsdf_surface_normals = normals
        self._tsdf_surface_tree = cKDTree(points) if len(points) else None
        self._tsdf_surface_received_at = self._now()

    def _suspend_cb(self, msg: Bool) -> None:
        if msg.data != self._suspended:
            self.get_logger().info(
                'Goal publication suspended — external planner active' if msg.data
                else 'Goal publication resumed')
        self._suspended = msg.data

    def _external_goal_cb(self, msg: PointStamped) -> None:
        if not self._suspended:
            return   # not suspended: this is our own publication looping back, ignore
        self._current_goal_xy = np.array([msg.point.x, msg.point.y])

    def _motion_status_cb(self, msg: String) -> None:
        """Consume a planner-agnostic motion failure report.

        Motion nodes never select replacement goals.  This planner treats a
        BLOCKED report as equivalent to repeated A* failure: blacklist the
        committed goal and let its normal frontier selection pick the next one.
        Other planners can publish or interpret the same topic independently.
        """
        if not msg.data.startswith('BLOCKED'):
            return
        if self._current_goal_xy is None or self._suspended:
            return
        gxy = self._current_goal_xy.copy()
        self._goals.mark_unreachable(gxy, self._now())
        self._current_goal_xy = None
        self._current_path = []
        self._astar_fail_count = 0
        self._astar_fail_goal = None
        self.get_logger().warn(
            f'Motion reported {msg.data} for ({gxy[0]:.1f},{gxy[1]:.1f}) '
            '— blacklisting and selecting a new frontier goal')

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

    def _tsdf_solid_contains(self, xy: np.ndarray) -> bool:
        """Check strict 3-D solid-voxel containment at the vehicle depth."""
        if (self._tsdf_solid_tree is None or self._tsdf_solid_received_at is None
                or self._now() - self._tsdf_solid_received_at > self.TSDF_SOLID_STALE_S):
            return False
        query = np.array([xy[0], xy[1], self._robot_pos[2]], dtype=np.float64)
        distance, _ = self._tsdf_solid_tree.query(
            query, distance_upper_bound=self._tsdf_solid_radius)
        return math.isfinite(distance)

    def _tsdf_surface_standoff(self, frontier_xy: np.ndarray) -> np.ndarray:
        """Move a frontier goal outward from its nearest TSDF wall surface."""
        if (self._tsdf_frontier_standoff <= 0.0 or self._tsdf_surface_tree is None
                or self._tsdf_surface_received_at is None
                or self._now() - self._tsdf_surface_received_at > self.TSDF_SURFACE_STALE_S):
            return frontier_xy
        query = np.array([frontier_xy[0], frontier_xy[1], self._robot_pos[2]])
        distance, index = self._tsdf_surface_tree.query(
            query, distance_upper_bound=self._tsdf_surface_normal_max_distance)
        if not math.isfinite(distance):
            return frontier_xy
        standoff = standoff_point_from_tsdf_surface(
            self._tsdf_surface_points[index], self._tsdf_surface_normals[index],
            self._tsdf_frontier_standoff)
        return frontier_xy if standoff is None else standoff

    # ------------------------------------------------------------------
    # Main loop
    def _update(self) -> None:
        if self._map is None or self._robot_pos is None:
            return
        if self._suspended:
            return   # external planner owns goal publication and GoalManager state

        free_cells, occ_cells, mapped_cells = self._map_stats()

        clusters = find_frontier_clusters(self._map, self.MIN_CLUSTER_CELLS)
        if not clusters:
            self.get_logger().info('No frontiers found', throttle_duration_sec=5.0)
            return

        # Put a TSDF frontier goal in free space, offset along the outward
        # surface normal.  The nearest normal is queried at the vehicle depth;
        # vertical surfaces give no horizontal offset and keep the old target.
        raw_centroids = [(c.wx, c.wy) for c in clusters]
        for c in clusters:
            c.wx, c.wy = self._tsdf_surface_standoff(np.array([c.wx, c.wy]))

        self._publish_frontier_debug(clusters, raw_centroids)

        # Tag each cluster with its distance from the robot.
        for c in clusters:
            c.distance = float(np.hypot(c.wx - self._robot_pos[0],
                                        c.wy - self._robot_pos[1]))

        # The OctoMap projection has no depth information: a 2-D frontier may
        # land inside a vertical wall at the vehicle's current depth.  TSDF's
        # confidently-solid voxels give the missing 3-D veto.  Do not apply a
        # clearance buffer here; boundary frontiers are valid, only solid
        # containment must be rejected.
        original_count = len(clusters)
        if self._tsdf_solid_tree is not None:
            clusters = [
                c for c in clusters
                if not self._tsdf_solid_contains(np.array([c.wx, c.wy]))
            ]
        rejected_solid = original_count - len(clusters)
        if rejected_solid:
            self.get_logger().info(
                f'Rejected {rejected_solid} frontier(s) inside solid TSDF voxels',
                throttle_duration_sec=5.0)

        if (self._current_goal_xy is not None
                and self._tsdf_solid_contains(self._current_goal_xy)):
            self.get_logger().warn(
                f'Current goal ({self._current_goal_xy[0]:.1f},'
                f'{self._current_goal_xy[1]:.1f}) is inside a solid TSDF voxel '
                '— blacklisting it')
            self._goals.mark_unreachable(self._current_goal_xy, self._now())
            self._current_goal_xy = None
            self._current_path = []

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
                'Survey area fully explored — every remaining frontier is '
                'outside it' if selection.event == 'OUTSIDE_SURVEY_AREA'
                else 'All candidates blacklisted — waiting',
                throttle_duration_sec=5.0,
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
    def _publish_frontier_debug(self, clusters: list, raw_centroids: list) -> None:
        """Show what a frontier was derived from, before selection filters it."""
        frontier_xy, border_xy = frontier_cell_points(self._map)
        pairs = [(raw, (c.wx, c.wy))
                 for raw, c in zip(raw_centroids, clusters)
                 if math.hypot(c.wx - raw[0], c.wy - raw[1]) > 1e-3]
        self._viz.publish_frontier_debug(
            frontier_xy, border_xy, pairs, self._robot_pos,
            float(self._map.info.resolution), self._projected_map_band)

    def _publish_goal(self, gx: float, gy: float, clusters: list) -> None:
        self._current_goal_xy = np.array([gx, gy])
        # _cruise_z, not self._robot_pos[2]: waypoint_controller drives depth
        # off this goal's Z now, and the robot's own live Z is exactly the
        # self-referential value Change 12 stopped using for that.
        gz = self._cruise_z if self._cruise_z is not None else float(self._robot_pos[2])
        goal = PointStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = 'world_ned'
        goal.point.x, goal.point.y, goal.point.z = gx, gy, gz
        self._goal_pub.publish(goal)
        self._viz.publish_markers(clusters, gx, gy, self._robot_pos)

    def _replan(self) -> None:
        if self._map is None or self._robot_pos is None:
            return

        self._cg = build_cost_grid(
            self._map, hard_m=self._hard_inflation_m,
            soft_m=self._inflation_m, plan_m=self._plan_inflation_m)

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
        self._viz.publish_planning_dashboard(
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


def _parse_xyz_cloud(msg: PointCloud2) -> 'np.ndarray | None':
    """Parse finite XYZ points from a float32 PointCloud2 with any layout."""
    offsets = {
        field.name: field.offset for field in msg.fields
        if field.datatype == PointField.FLOAT32
    }
    if (any(name not in offsets for name in ('x', 'y', 'z'))
            or msg.point_step <= 0 or msg.width == 0 or msg.height == 0):
        return np.empty((0, 3), dtype=np.float64) if msg.width == 0 else None
    byte_order = '>' if msg.is_bigendian else '<'
    cloud_dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [byte_order + 'f4'] * 3,
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': msg.point_step,
    })
    try:
        data = np.ndarray(
            shape=(msg.height, msg.width), dtype=cloud_dtype, buffer=msg.data,
            strides=(msg.row_step, msg.point_step))
    except (TypeError, ValueError):
        return None
    xyz = np.column_stack((data['x'].ravel(), data['y'].ravel(), data['z'].ravel()))
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float64, copy=False)


def _parse_xyz_normals_cloud(msg: PointCloud2) -> 'tuple[np.ndarray, np.ndarray] | None':
    """Parse finite XYZ points and normals from the TSDF normal cloud."""
    names = ('x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z')
    offsets = {
        field.name: field.offset for field in msg.fields
        if field.datatype == PointField.FLOAT32
    }
    if any(name not in offsets for name in names) or msg.point_step <= 0:
        return None
    if msg.width == 0 or msg.height == 0:
        empty = np.empty((0, 3), dtype=np.float64)
        return empty, empty
    byte_order = '>' if msg.is_bigendian else '<'
    cloud_dtype = np.dtype({
        'names': names,
        'formats': [byte_order + 'f4'] * len(names),
        'offsets': [offsets[name] for name in names],
        'itemsize': msg.point_step,
    })
    try:
        data = np.ndarray(
            shape=(msg.height, msg.width), dtype=cloud_dtype, buffer=msg.data,
            strides=(msg.row_step, msg.point_step))
    except (TypeError, ValueError):
        return None
    points = np.column_stack((data['x'].ravel(), data['y'].ravel(), data['z'].ravel()))
    normals = np.column_stack((
        data['normal_x'].ravel(), data['normal_y'].ravel(), data['normal_z'].ravel()))
    valid = np.isfinite(points).all(axis=1) & np.isfinite(normals).all(axis=1)
    return points[valid].astype(np.float64, copy=False), normals[valid].astype(np.float64, copy=False)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExtractor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._log.close()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
