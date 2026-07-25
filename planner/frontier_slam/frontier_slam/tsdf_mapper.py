"""TSDF mapper node — VDBFusion backend.

VDBFusion sign convention (raw **metric** values, in metres):
  d > 0  →  free space   (between camera and surface)
  d = 0  →  surface      (zero crossing)
  d < 0  →  occupied     (behind surface / solid)

IMPORTANT: VDBFusion.integrate(points, origin) requires points and origin to
BOTH be in the same world frame.  The 4×4 extrinsic overload only extracts the
camera origin from T[:3,3] — it does NOT rotate the points.  We therefore
rotate the points ourselves before calling integrate().

Subscribed topics:
  /cloud_in        (sensor_msgs/PointCloud2)  depth camera point cloud

Published topics:
  /tsdf/surface_cloud          (sensor_msgs/PointCloud2)   marching-cubes surface
  /tsdf/surface_normals        (visualization_msgs/MarkerArray)  sampled normals (RViz)
  /tsdf/surface_normals_cloud  (sensor_msgs/PointCloud2)
                                 fields x y z normal_x normal_y normal_z —
                                 the same sampled points + normals in machine-
                                 readable form, consumed by wall_looking
  /tsdf/voxels           (visualization_msgs/MarkerArray)
                           single CUBE_LIST at the true voxel resolution (fixed
                           size — a size varying with weight is what made
                           neighbouring voxels overlap and look cluttered).
                           A voxel is only included if BOTH:
                             weight >= voxel_min_weight        (observed often enough)
                             solid-confidence >= voxel_min_solid_confidence
                               where solid-confidence = (trunc - d) / (2*trunc)
                               (0.5 at the surface d=0, 1.0 at full saturation d=-trunc)
                           colour, two channels:
                             hue        = observation weight (log-scale): orange
                                          = just past the floor, green = heavily
                                          observed
                             saturation = confidence it is a wall (TSDF depth):
                                          pale = least-confident shown voxel,
                                          vivid = deep solid
  /tsdf/occupied_voxels  (sensor_msgs/PointCloud2) confidently solid TSDF
                           voxel centres for collision-aware goal validation
  /projected_map         (nav_msgs/OccupancyGrid) 2-D planning map for the
                           frontier planner + A*, published only when
                           publish_projected_map:=true (mode:=frontier). A thin
                           Z-band around target_depth_m is projected straight
                           from this TSDF grid so the planning map and the
                           belief map can never disagree — replacing the
                           separate octomap_server that used to build it.
  /tsdf/free_voxels      (sensor_msgs/PointCloud2) observed-empty voxel centres
                           (d > 0), published only when publish_free_voxels:=true.
                           The free/unknown half that /tsdf/occupied_voxels
                           cannot express; tsdf_to_octomap consumes both to
                           build an octomap::OcTree from this grid.
"""

from collections import deque, OrderedDict
import time

import numpy as np
import rclpy
import small_gicp
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, Int32
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from scipy.spatial.transform import Rotation, Slerp
from vdbfusion import VDBVolume


class TSDFMapper(Node):
    PUBLISH_HZ   = 1.0   # surface cloud + normals
    VOXEL_VIZ_HZ = 0.5   # voxel CUBE_LIST (expensive iteration)

    def __init__(self) -> None:
        super().__init__('tsdf_mapper')

        # ── parameters ──────────────────────────────────────────────────
        self.declare_parameter('world_frame',      'world_ned')
        self.declare_parameter('cloud_frame',      'bluerov2/Dcam')
        self.declare_parameter('voxel_size',       0.2)
        self.declare_parameter('trunc_distance',   0.6)    # metres, ≥ 3× voxel_size
        self.declare_parameter('space_carving',    True)
        self.declare_parameter('min_weight',       2.0)
        self.declare_parameter('voxel_min_weight', 10.0)   # hide voxels observed fewer times
        self.declare_parameter('voxel_min_solid_confidence', 0.80)  # see module docstring
        self.declare_parameter('normal_every',     10)
        self.declare_parameter('max_voxels_viz',   40_000)
        self.declare_parameter('show_free_voxels', False)
        # Discard points beyond the simulated Sonar 3D-15 beam range before
        # integration. The depth-camera proxy may produce farther off-axis
        # points, but the physical Water Linked sensor has a 15 m radial range.
        self.declare_parameter('max_range_m', 15.0)
        # Free-space carving for no-return pixels: synthesize a pseudo-point
        # past the sensor max so space_carving frees the traversed voxels.
        # vdbfusion has no carve-only ray API, so the endpoint itself writes a
        # surface — carve_range_m keeps it outside the mapped envelope.
        self.declare_parameter('carve_no_return', False)
        self.declare_parameter('carve_range_m', 16.0)
        # 2-D planning map derived from this TSDF grid, published on
        # /projected_map for the frontier planner + A* (see module docstring).
        # Off by default so only the belief instance under mode:=frontier
        # opts in; never enabled on tsdf_mapper_gt.
        self.declare_parameter('publish_projected_map', False)
        # Free (d>0) voxel centres on /tsdf/free_voxels, for tsdf_to_octomap.
        self.declare_parameter('publish_free_voxels', False)
        # Cruise depth (world_ned Z, +down) the projection band centres on.
        # -1.0 = auto: lock to the first base_link->world TF Z, then hold it.
        self.declare_parameter('target_depth_m', -1.0)
        self.declare_parameter('projected_map_band_m', 1.0)
        self.declare_parameter('projected_map_margin_cells', 10)
        self.declare_parameter('base_link_frame', 'bluerov2/base_link')
        # Map rebuild consumer (pose_graph.py publisher side): off by default,
        # and never set true on the ground-truth instance (tsdf_mapper_gt) --
        # only the belief map should ever be reset+re-integrated.
        self.declare_parameter('enable_rebuild', False)
        self.declare_parameter('cache_voxel_size', 0.1)
        self.declare_parameter('cache_max_scans', 6000)
        self.declare_parameter('rebuild_chunk_scans', 10)
        self.declare_parameter('rebuild_tick_s', 0.02)
        # Clouds and their exact-time TF are published by different ROS nodes.
        # The cloud often reaches this single-threaded executor first.  A
        # blocking lookup in the cloud callback cannot receive the queued TF,
        # so keep a short, bounded FIFO of ROS async-TF futures and drain it
        # after returning to spin().
        self.declare_parameter('tf_wait_timeout_s', 0.5)
        self.declare_parameter('tf_queue_size', 10)
        self.declare_parameter('tf_queue_tick_s', 0.02)

        self._world_frame  = self.get_parameter('world_frame').value
        self._cloud_frame  = self.get_parameter('cloud_frame').value
        self._min_weight   = float(self.get_parameter('min_weight').value)
        self._voxel_min_weight = float(self.get_parameter('voxel_min_weight').value)
        self._voxel_min_solid_confidence = float(
            self.get_parameter('voxel_min_solid_confidence').value)
        self._normal_every = int(self.get_parameter('normal_every').value)
        self._max_viz      = int(self.get_parameter('max_voxels_viz').value)
        self._show_free    = bool(self.get_parameter('show_free_voxels').value)
        self._max_range    = float(self.get_parameter('max_range_m').value)
        self._carve_no_return = bool(self.get_parameter('carve_no_return').value)
        self._carve_range  = float(self.get_parameter('carve_range_m').value)
        self._projected_map_enabled = bool(
            self.get_parameter('publish_projected_map').value)
        self._free_voxels_enabled = bool(
            self.get_parameter('publish_free_voxels').value)
        self._target_depth = float(self.get_parameter('target_depth_m').value)
        self._projected_map_band = abs(float(
            self.get_parameter('projected_map_band_m').value))
        self._projected_map_margin = max(0, int(
            self.get_parameter('projected_map_margin_cells').value))
        self._base_link_frame = str(self.get_parameter('base_link_frame').value)
        # Locked band centre: seeded from target_depth_m, or filled on the
        # first TF lookup when target_depth_m < 0. None = not yet resolved.
        self._projected_map_z = self._target_depth if self._target_depth >= 0.0 else None
        voxel_size         = float(self.get_parameter('voxel_size').value)
        trunc              = float(self.get_parameter('trunc_distance').value)
        space_carving      = bool(self.get_parameter('space_carving').value)

        self._voxel_size = voxel_size
        self._trunc      = trunc
        self._space_carving = space_carving
        # solid-confidence = (trunc - d) / (2*trunc) >= voxel_min_solid_confidence
        #   <=>  d <= trunc * (1 - 2*voxel_min_solid_confidence)
        self._voxel_max_d = trunc * (1.0 - 2.0 * self._voxel_min_solid_confidence)
        self._volume     = VDBVolume(voxel_size, trunc, space_carving=space_carving)

        if not self._volume.pyopenvdb_support_enabled:
            self.get_logger().warn(
                'VDBFusion built without pyopenvdb — voxel visualisation disabled')

        self.get_logger().info(
            f'TSDF  voxel={voxel_size}m  trunc={trunc}m  space_carving={space_carving}')

        # ── TF ──────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._tf_wait_timeout_s = max(
            0.0, float(self.get_parameter('tf_wait_timeout_s').value))
        self._tf_queue_size = max(1, int(self.get_parameter('tf_queue_size').value))
        tf_queue_tick_s = max(
            0.001, float(self.get_parameter('tf_queue_tick_s').value))
        self._tf_queue = deque()
        self._monotonic = time.monotonic
        self._cloud_received = 0
        self._cloud_integrated = 0
        self._tf_deferred = 0
        self._tf_recovered = 0
        self._tf_expired = 0
        self._tf_failed = 0
        self._tf_overflow = 0

        # ── pub/sub ──────────────────────────────────────────────────────
        self.create_subscription(PointCloud2, '/cloud_in', self._cloud_cb, 5)
        self.create_timer(tf_queue_tick_s, self._tf_queue_tick)
        self.create_timer(10.0, self._log_cloud_tf_stats)

        self._enable_rebuild = bool(self.get_parameter('enable_rebuild').value)
        self._cache_voxel_size = float(self.get_parameter('cache_voxel_size').value)
        self._cache_max_scans = int(self.get_parameter('cache_max_scans').value)
        self._rebuild_chunk_scans = int(self.get_parameter('rebuild_chunk_scans').value)
        # stamp key (sec, nsec) -> [pts_cam_f32 (M,3) downsampled, T_world_cam (4,4)].
        # Points are cached in CAMERA frame, not world -- a rebuild correction
        # only ever needs to update T (see _write_back_cache), the cached
        # points themselves are frame-invariant until re-projected at replay.
        self._scan_cache: OrderedDict = OrderedDict()
        self._pending_path_old = None
        self._pending_path_new = None
        self._replay_queue: list = []
        if self._enable_rebuild:
            self.create_subscription(Int32, '/slam/rebuild/begin', self._rebuild_begin_cb, 10)
            self.create_subscription(Path, '/slam/rebuild/path_old', self._rebuild_path_old_cb, 10)
            self.create_subscription(Path, '/slam/rebuild/path_new', self._rebuild_path_new_cb, 10)
            rebuild_tick_s = float(self.get_parameter('rebuild_tick_s').value)
            self.create_timer(rebuild_tick_s, self._replay_tick)
            self.get_logger().info(
                'Map rebuild consumer enabled: will reset+re-integrate cached scans '
                'on a validated /slam/rebuild/path_old + path_new pair')

        self._cloud_pub   = self.create_publisher(PointCloud2, '/tsdf/surface_cloud',   1)
        self._normals_pub = self.create_publisher(MarkerArray, '/tsdf/surface_normals',  1)
        self._normals_cloud_pub = self.create_publisher(
            PointCloud2, '/tsdf/surface_normals_cloud', 1)
        self._voxels_pub  = self.create_publisher(MarkerArray, '/tsdf/voxels',           1)
        self._solid_cloud_pub = self.create_publisher(
            PointCloud2, '/tsdf/occupied_voxels', 1)

        # 2-D planning map on /projected_map. Needs the pyopenvdb grid path for
        # free-space (d>0) voxels — the surface-vertex fallback has none, so a
        # surface-derived projection would be all-unknown-free and useless.
        self._projected_map_pub = None
        if self._projected_map_enabled:
            if self._volume.pyopenvdb_support_enabled:
                latched_qos = QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST)
                self._projected_map_pub = self.create_publisher(
                    OccupancyGrid, '/projected_map', latched_qos)
                self.get_logger().info(
                    'Publishing TSDF-derived /projected_map '
                    f'(band {self._projected_map_band:.1f}m around '
                    f'target_depth={self._target_depth:.1f}m)')
            else:
                self.get_logger().error(
                    'publish_projected_map requested but VDBFusion lacks pyopenvdb '
                    '— /projected_map disabled; use mapper:=octomap for the planning map')

        # Free (d>0) voxel centres for tsdf_to_octomap. Same grid-path
        # requirement as /projected_map: the surface-vertex fallback knows
        # nothing about empty space.
        self._free_cloud_pub = None
        if self._free_voxels_enabled:
            if self._volume.pyopenvdb_support_enabled:
                self._free_cloud_pub = self.create_publisher(
                    PointCloud2, '/tsdf/free_voxels', 1)
            else:
                self.get_logger().error(
                    'publish_free_voxels requested but VDBFusion lacks pyopenvdb '
                    '— /tsdf/free_voxels disabled')

        # Latest marching-cubes vertices, so the voxel view can be derived from
        # them when pyopenvdb (the grid path) is unavailable.
        self._last_surface_verts = None

        self.create_timer(1.0 / self.PUBLISH_HZ,   self._publish_surface)
        self.create_timer(1.0 / self.VOXEL_VIZ_HZ, self._publish_voxels)

        self.get_logger().info('tsdf_mapper ready')

    # ────────────────────────────────────────────────────────────────────
    # Cloud callback — integrate now when TF is ready, otherwise defer
    # ────────────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._cloud_received += 1
        # Once one cloud is waiting, enqueue newer clouds behind it even if
        # their TF is already ready. This keeps integration and rebuild-cache
        # timestamps in capture order.
        if not self._tf_queue:
            tf_msg = self._lookup_cloud_transform(msg)
            if tf_msg is not None:
                if self._integrate_cloud(msg, tf_msg):
                    self._cloud_integrated += 1
                return

        self._tf_deferred += 1
        if len(self._tf_queue) >= self._tf_queue_size:
            _msg, _queued_at, future = self._tf_queue.popleft()
            future.cancel()
            self._tf_overflow += 1
            self.get_logger().warn(
                'Cloud/TF queue full; dropping oldest scan',
                throttle_duration_sec=5.0)
        future = self._wait_for_cloud_transform(msg)
        self._tf_queue.append((msg, self._monotonic(), future))

    def _lookup_cloud_transform(self, msg: PointCloud2):
        source_frame = msg.header.frame_id or self._cloud_frame
        try:
            # Deliberately non-blocking. A timeout here runs inside the same
            # single-threaded executor that must receive the missing TF.
            return self._tf_buffer.lookup_transform(
                self._world_frame, source_frame, msg.header.stamp)
        except tf2_ros.TransformException:
            return None

    def _wait_for_cloud_transform(self, msg: PointCloud2):
        source_frame = msg.header.frame_id or self._cloud_frame
        return self._tf_buffer.wait_for_transform_async(
            self._world_frame, source_frame, msg.header.stamp)

    def _tf_queue_tick(self) -> None:
        """Drain ready async TF futures in FIFO order, at most one scan per tick."""
        while self._tf_queue:
            msg, queued_at, future = self._tf_queue[0]
            if future.done():
                self._tf_queue.popleft()
                if future.cancelled():
                    continue
                # wait_for_transform_async only signals availability: its future
                # resolves to True on Humble and to the transform on newer
                # distros, so look the transform up ourselves either way.
                tf_msg = self._lookup_cloud_transform(msg)
                if tf_msg is None:
                    self._tf_failed += 1
                    self.get_logger().warn(
                        'TF signalled ready but lookup failed; dropping scan',
                        throttle_duration_sec=5.0)
                    continue
                self._tf_recovered += 1
                if self._integrate_cloud(msg, tf_msg):
                    self._cloud_integrated += 1
                return

            if self._monotonic() - queued_at < self._tf_wait_timeout_s:
                return

            self._tf_queue.popleft()
            future.cancel()
            self._tf_expired += 1
            self.get_logger().warn(
                'Exact-time TF did not arrive before deadline; dropping scan',
                throttle_duration_sec=5.0)

    def _log_cloud_tf_stats(self) -> None:
        self.get_logger().info(
            f'Cloud/TF stats: received={self._cloud_received} '
            f'integrated={self._cloud_integrated} deferred={self._tf_deferred} '
            f'recovered={self._tf_recovered} expired={self._tf_expired} '
            f'failed={self._tf_failed} overflow={self._tf_overflow} '
            f'queued={len(self._tf_queue)}')

    def _integrate_cloud(self, msg: PointCloud2, tf_msg) -> bool:
        """Filter and integrate one cloud using its exact capture-time TF."""
        pts_cam = _parse_pointcloud2(msg)   # (N,3) float64, sensor frame
        if pts_cam is None or len(pts_cam) == 0:
            return False

        # Filter by radial beam range. This keeps TSDF aligned with /cloud_in
        # and the real Sonar 3D-15's 15 m acoustic range.
        pts_cam = pts_cam[np.linalg.norm(pts_cam, axis=1) < self._max_range]

        # Appended after the range filter — these sit past max_range by design.
        if self._carve_no_return:
            raw = _parse_pointcloud2_raw(msg)
            if raw is not None:
                pseudo = synth_no_return_points(raw, msg.width, self._carve_range)
                if len(pseudo):
                    pts_cam = np.vstack([pts_cam, pseudo])

        if len(pts_cam) == 0:
            return False

        T = _tf_to_matrix(tf_msg.transform)   # T_world_cam (4×4, float64)
        R, t = T[:3, :3], T[:3, 3]

        # ── KEY FIX ──────────────────────────────────────────────────────
        # VDBFusion.integrate(points, origin) expects points in WORLD frame.
        # The 4×4 extrinsic overload only extracts T[:3,3] (origin) and does
        # NOT rotate the points.  We must rotate them ourselves.
        pts_world = pts_cam @ R.T + t      # (N,3) world frame, float64
        origin    = t                       # camera origin in world frame, float64

        self._volume.integrate(pts_world, origin)

        if self._enable_rebuild:
            self._cache_scan(msg.header.stamp, pts_cam, T)

        self.get_logger().info(
            f'Integrated {len(pts_world)} pts  cam=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})',
            throttle_duration_sec=2.0)
        return True

    def _cache_scan(self, stamp, pts_cam: np.ndarray, T_world_cam: np.ndarray) -> None:
        """Downsample and store this scan (camera frame) + its capture pose,
        FIFO-evicting the oldest entry once over cache_max_scans. Downsampled
        here (not raw) to bound memory: ~6-30 KB/scan at cache_voxel_size=0.1
        depending on scene density, so cache_max_scans=6000 caps this well
        under cache_max_scans * 30 KB ~= 180 MB worst case."""
        cloud, _tree = small_gicp.preprocess_points(
            pts_cam, downsampling_resolution=self._cache_voxel_size)
        pts_down = cloud.points()[:, :3].astype(np.float32)
        key = (stamp.sec, stamp.nanosec)
        self._scan_cache[key] = (pts_down, T_world_cam)
        _evict_fifo(self._scan_cache, self._cache_max_scans)

    # ────────────────────────────────────────────────────────────────────
    # Map rebuild consumer (pose_graph.py is the publisher side)
    #
    # pose_graph publishes only the per-keyframe pose correction (old world
    # pose -> new corrected world pose), not scans -- this node keeps its own
    # cache of every integrated scan (_cache_scan, camera frame) and replays
    # ALL of them with an interpolated correction applied, not just the ~1/m
    # keyframe scans. /slam/rebuild/begin is informational only (logs the
    # expected keyframe count); the actual trigger is a validated
    # path_old/path_new pair of equal, nonzero length.
    # ────────────────────────────────────────────────────────────────────

    def _rebuild_begin_cb(self, msg: Int32) -> None:
        self.get_logger().info(f'Map rebuild starting: {int(msg.data)} keyframes moved')

    def _rebuild_path_old_cb(self, msg: Path) -> None:
        self._pending_path_old = msg
        self._maybe_process_rebuild()

    def _rebuild_path_new_cb(self, msg: Path) -> None:
        self._pending_path_new = msg
        self._maybe_process_rebuild()

    def _maybe_process_rebuild(self) -> None:
        path_old, path_new = self._pending_path_old, self._pending_path_new
        if path_old is None or path_new is None:
            return
        self._pending_path_old = None
        self._pending_path_new = None
        if len(path_old.poses) != len(path_new.poses) or len(path_old.poses) == 0:
            self.get_logger().warn(
                f'Rebuild path pair mismatched (old={len(path_old.poses)}, '
                f'new={len(path_new.poses)}), ignoring')
            return

        keyframe_ts = np.array(
            [_stamp_to_float(p.header.stamp) for p in path_old.poses])
        order = np.argsort(keyframe_ts)
        keyframe_ts = keyframe_ts[order]
        T_olds = [_posestamped_to_matrix(path_old.poses[i]) for i in order]
        T_news = [_posestamped_to_matrix(path_new.poses[i]) for i in order]
        corrections = _compute_corrections(T_olds, T_news)

        self._write_back_cache(keyframe_ts, corrections)

        self._volume = VDBVolume(self._voxel_size, self._trunc, space_carving=self._space_carving)
        self._replay_queue = list(self._scan_cache.keys())
        self.get_logger().info(
            f'Map rebuild: {len(keyframe_ts)} keyframe corrections, replaying '
            f'{len(self._replay_queue)} cached scans')

    def _write_back_cache(self, keyframe_ts: np.ndarray, corrections: np.ndarray) -> None:
        for key, (pts, T) in self._scan_cache.items():
            t = key[0] + key[1] * 1e-9
            A = _interpolate_correction(t, keyframe_ts, corrections)
            self._scan_cache[key] = (pts, A @ T)

    def _replay_tick(self) -> None:
        if not self._replay_queue:
            return
        chunk, self._replay_queue = (
            self._replay_queue[:self._rebuild_chunk_scans],
            self._replay_queue[self._rebuild_chunk_scans:])
        for key in chunk:
            entry = self._scan_cache.get(key)
            if entry is None:
                continue   # evicted between snapshot and replay
            pts_cam, T = entry
            R, t = T[:3, :3], T[:3, 3]
            pts_world = pts_cam.astype(np.float64) @ R.T + t
            self._volume.integrate(pts_world, t)
        if not self._replay_queue:
            self.get_logger().info('Map rebuild replay complete')

    # ────────────────────────────────────────────────────────────────────
    # Surface cloud + normals (marching cubes via VDBFusion)
    # ────────────────────────────────────────────────────────────────────

    def _publish_surface(self) -> None:
        try:
            verts, _tris = self._volume.extract_triangle_mesh(
                fill_holes=False, min_weight=float(self._min_weight))
        except Exception as exc:
            self.get_logger().warn(
                f'extract_triangle_mesh failed: {exc}', throttle_duration_sec=5.0)
            return

        if len(verts) == 0:
            return

        verts = np.asarray(verts, dtype=np.float32)
        self._last_surface_verts = verts

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = self._world_frame

        self._cloud_pub.publish(_make_pointcloud2(header, verts))

        # Normals need the TSDF grid (pyopenvdb-only); the surface cloud does not.
        if self._volume.pyopenvdb_support_enabled:
            sampled = verts[::self._normal_every]
            normals  = _compute_normals_vdb(self._volume.tsdf, sampled, self._voxel_size)
            self._normals_pub.publish(_normals_markers(sampled, normals, header))

            valid = ~np.isnan(normals).any(axis=1)
            self._normals_cloud_pub.publish(
                _make_normals_cloud(header, sampled[valid], normals[valid]))

        self.get_logger().info(f'Surface: {len(verts)} pts', throttle_duration_sec=5.0)

    # ────────────────────────────────────────────────────────────────────
    # Voxel CUBE_LIST visualisation
    # ────────────────────────────────────────────────────────────────────

    def _publish_voxels(self) -> None:
        if not self._volume.pyopenvdb_support_enabled:
            self._publish_voxels_from_surface()
            return

        if self._projected_map_pub is not None or self._free_cloud_pub is not None:
            # One walk feeds the solid viz, the 2-D planning map and the free
            # cloud: keep every voxel at/above the low map weight (min_weight)
            # so free (d>0) cells survive, then re-filter to solids here for
            # the CUBE_LIST + /tsdf/occupied_voxels.
            coords, all_d, all_w = _extract_voxel_arrays(
                self._volume.tsdf, self._volume.weights, self._min_weight)
            self._publish_projected_map(coords, all_d, all_w)
            self._publish_free_voxels(coords, all_d)
            if coords is None:
                pts = d_vals = w_vals = None
            else:
                solid = (all_w >= self._voxel_min_weight) & (all_d <= self._voxel_max_d)
                if np.any(solid):
                    pts = (coords[solid].astype(np.float32) * self._voxel_size
                           + self._voxel_size / 2.0)
                    d_vals = all_d[solid]
                    w_vals = all_w[solid]
                else:
                    pts = d_vals = w_vals = None
        else:
            # Only iterate voxels we'll actually show (early filtering inside):
            # confidently solid (TSDF-derived) AND observed often enough (weight).
            max_d = self._voxel_max_d if not self._show_free else None
            pts, d_vals, w_vals = _extract_voxels(
                self._volume.tsdf, self._volume.weights,
                self._voxel_size, min_weight=self._voxel_min_weight, max_d=max_d)

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self._world_frame)

        if pts is None:
            self._solid_cloud_pub.publish(
                _make_pointcloud2(header, np.empty((0, 3), dtype=np.float32)))
            del_m = Marker()
            del_m.header.stamp    = now
            del_m.header.frame_id = self._world_frame
            del_m.ns     = 'tsdf_voxels'
            del_m.id     = 0
            del_m.action = Marker.DELETE
            self._voxels_pub.publish(MarkerArray(markers=[del_m]))
            return

        # This remains a solid-only cloud even if the optional voxel
        # visualisation includes free space.  It is consumed by the frontier
        # planner to veto only goals physically inside a TSDF solid voxel.
        solid_pts = pts if not self._show_free else pts[d_vals <= self._voxel_max_d]
        self._solid_cloud_pub.publish(_make_pointcloud2(header, solid_pts))

        n = len(pts)
        if n > self._max_viz:
            sel    = np.random.choice(n, self._max_viz, replace=False)
            pts    = pts[sel]
            d_vals = d_vals[sel]
            w_vals = w_vals[sel]

        # Hue = observation weight (log scale): orange = just past the
        # observation floor, green = heavily observed — a quick read of how much
        # a voxel has been seen. Saturation = confidence it is a wall (TSDF
        # depth, stretched across the shown solid band): pale = least-confident
        # shown voxel, vivid = deep solid. Cube size stays fixed at the grid
        # resolution: varying it by weight made neighbouring cubes overlap.
        w_max  = max(float(w_vals.max()), self._voxel_min_weight + 1.0)
        w_norm = np.clip(
            np.log1p(np.clip(w_vals - self._voxel_min_weight, 0.0, None))
            / np.log1p(w_max - self._voxel_min_weight), 0.0, 1.0)
        band      = self._voxel_max_d + self._trunc
        wall_conf = np.clip((self._voxel_max_d - d_vals) / max(band, 1e-6), 0.6, 1.0)
        colors = _confidence_colormap(w_norm, saturation=wall_conf)

        m = Marker()
        m.header.stamp    = now
        m.header.frame_id = self._world_frame
        m.ns       = 'tsdf_voxels'
        m.id       = 0
        m.type     = Marker.CUBE_LIST
        m.action   = Marker.ADD
        m.lifetime = Duration(sec=4)
        m.scale.x  = self._voxel_size
        m.scale.y  = self._voxel_size
        m.scale.z  = self._voxel_size
        m.points   = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        m.colors   = [ColorRGBA(r=float(c[0]), g=float(c[1]),
                                b=float(c[2]), a=float(c[3])) for c in colors]

        self._voxels_pub.publish(MarkerArray(markers=[m]))
        self.get_logger().info(f'Voxels: {len(pts)} published', throttle_duration_sec=5.0)

    # ────────────────────────────────────────────────────────────────────
    # 2-D planning map (/projected_map) derived from the TSDF grid
    # ────────────────────────────────────────────────────────────────────

    def _band_center_z(self) -> 'float | None':
        """Target depth (world_ned Z) the projection band centres on, or None if
        auto-lock is requested but no base_link TF is available yet."""
        if self._projected_map_z is not None:
            return self._projected_map_z
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._base_link_frame, Time())
        except tf2_ros.TransformException:
            return None
        self._projected_map_z = float(tf.transform.translation.z)
        self.get_logger().info(
            f'projected_map band centre locked to z={self._projected_map_z:.2f}m')
        return self._projected_map_z

    def _publish_projected_map(self, coords, d_vals, w_vals) -> None:
        if self._projected_map_pub is None or coords is None:
            return
        z = self._band_center_z()
        if z is None:
            return
        grid = _build_projected_map(
            coords, d_vals, w_vals, self._voxel_size,
            z_lo=z - self._projected_map_band, z_hi=z + self._projected_map_band,
            occ_max_d=self._voxel_max_d, occ_min_weight=self._voxel_min_weight,
            margin=self._projected_map_margin,
            frame=self._world_frame, stamp=self.get_clock().now().to_msg())
        if grid is not None:
            self._projected_map_pub.publish(grid)

    def _publish_free_voxels(self, coords, d_vals) -> None:
        """Observed-empty voxel centres, the free half tsdf_to_octomap needs."""
        if self._free_cloud_pub is None:
            return
        header = Header(stamp=self.get_clock().now().to_msg(),
                        frame_id=self._world_frame)
        if coords is None:
            self._free_cloud_pub.publish(
                _make_pointcloud2(header, np.empty((0, 3), dtype=np.float32)))
            return
        free = d_vals > 0.0
        pts = (coords[free].astype(np.float32) * self._voxel_size
               + self._voxel_size / 2.0)
        self._free_cloud_pub.publish(_make_pointcloud2(header, pts))

    def _publish_voxels_from_surface(self) -> None:
        """Occupancy-voxel view built from the marching-cubes surface, for
        pyopenvdb-less builds (the prebuilt wheel). Snaps surface vertices to
        the TSDF grid and shows one cube per occupied voxel — the blocky
        'robot's-mind' counterpart to the smooth ground-truth surface. Also
        feeds /tsdf/occupied_voxels so frontier solid-rejection works here too."""
        verts = self._last_surface_verts
        now    = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self._world_frame)
        if verts is None or len(verts) == 0:
            return

        # Unique occupied cells, then their centres in world coordinates.
        cells   = np.unique(np.floor(verts / self._voxel_size).astype(np.int64), axis=0)
        centers = (cells.astype(np.float32) + 0.5) * self._voxel_size

        self._solid_cloud_pub.publish(_make_pointcloud2(header, centers))

        if len(centers) > self._max_viz:
            centers = centers[np.random.choice(len(centers), self._max_viz, replace=False)]

        # No per-voxel weight without the grid, so shade by height for depth —
        # the colormap is repurposed here as a plain low-to-high gradient.
        z = centers[:, 2]
        t = (z - z.min()) / max(float(np.ptp(z)), 1e-6)
        colors = _confidence_colormap(t)

        m = Marker()
        m.header   = header
        m.ns       = 'tsdf_voxels'
        m.id       = 0
        m.type     = Marker.CUBE_LIST
        m.action   = Marker.ADD
        m.lifetime = Duration(sec=4)
        m.scale.x  = self._voxel_size
        m.scale.y  = self._voxel_size
        m.scale.z  = self._voxel_size
        m.points   = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in centers]
        m.colors   = [ColorRGBA(r=float(c[0]), g=float(c[1]),
                                b=float(c[2]), a=float(c[3])) for c in colors]

        self._voxels_pub.publish(MarkerArray(markers=[m]))
        self.get_logger().info(
            f'Voxels (surface-derived): {len(centers)} published',
            throttle_duration_sec=5.0)


# ══════════════════════════════════════════════════════════════════════════════
# Module-level helpers
# ══════════════════════════════════════════════════════════════════════════════

def _parse_pointcloud2(msg: PointCloud2) -> 'np.ndarray | None':
    """Return (N,3) float64 array of finite XYZ points, or None."""
    if msg.point_step < 12 or msg.width == 0:
        return None
    floats = msg.point_step // 4
    data   = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats)
    xyz    = data[:, :3]
    valid  = np.isfinite(xyz).all(axis=1)
    if not np.any(valid):
        return None
    return xyz[valid].astype(np.float64)


def _parse_pointcloud2_raw(msg: PointCloud2) -> 'np.ndarray | None':
    """Return the organized (H*W, 3) float64 XYZ grid, NaNs kept in place."""
    if msg.point_step < 12 or msg.width == 0:
        return None
    floats = msg.point_step // 4
    data = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, floats)
    return data[:, :3].astype(np.float64)


def fit_pinhole_intrinsics(xyz: np.ndarray, width: int) -> 'tuple | None':
    """Recover (fx, cx, fy, cy) from an organized cloud's valid pixels.

    For a pinhole camera x/z = (u - cx)/fx, so x/z is linear in the column
    index u (and y/z in the row index v). Least-squares fitting the two lines
    avoids duplicating the sensor's FoV/resolution as node parameters.
    Returns None when there are too few valid pixels to fit."""
    n = len(xyz)
    if width <= 0 or n < width:
        return None
    valid = np.isfinite(xyz).all(axis=1) & (np.abs(xyz[:, 2]) > 1e-9)
    if valid.sum() < 8:
        return None
    idx = np.nonzero(valid)[0]
    u = (idx % width).astype(np.float64)
    v = (idx // width).astype(np.float64)
    xz = xyz[idx, 0] / xyz[idx, 2]
    yz = xyz[idx, 1] / xyz[idx, 2]

    def _line(t, s):
        # s = t/f - c/f  =>  slope 1/f, intercept -c/f
        if np.ptp(t) < 1e-9:
            return None
        slope, intercept = np.polyfit(t, s, 1)
        if abs(slope) < 1e-12:
            return None
        return 1.0 / slope, -intercept / slope

    x_fit, y_fit = _line(u, xz), _line(v, yz)
    if x_fit is None or y_fit is None:
        return None
    return x_fit[0], x_fit[1], y_fit[0], y_fit[1]


def synth_no_return_points(xyz: np.ndarray, width: int,
                           carve_range_m: float) -> np.ndarray:
    """Pseudo-points at carve_range_m along each no-return pixel's ray.

    Feeding these to VDBVolume.integrate(space_carving=True) frees the voxels
    the ray traverses. Returns an empty (0,3) array when the geometry can't be
    recovered (no valid pixels to fit against) or nothing is missing."""
    empty = np.empty((0, 3), dtype=np.float64)
    if xyz is None or len(xyz) == 0:
        return empty
    invalid = ~np.isfinite(xyz).all(axis=1)
    if not np.any(invalid):
        return empty
    intr = fit_pinhole_intrinsics(xyz, width)
    if intr is None:
        return empty
    fx, cx, fy, cy = intr

    idx = np.nonzero(invalid)[0]
    u = (idx % width).astype(np.float64)
    v = (idx // width).astype(np.float64)
    dirs = np.column_stack([(u - cx) / fx, (v - cy) / fy, np.ones(len(idx))])
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    return dirs * carve_range_m


def _tf_to_matrix(tf_transform) -> np.ndarray:
    """Convert a ROS Transform message to a 4×4 float64 numpy matrix (T_world_cam)."""
    t = tf_transform.translation
    q = tf_transform.rotation
    R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def _evict_fifo(cache: OrderedDict, max_size: int) -> None:
    """Pop oldest entries (insertion order) until cache fits max_size.
    Pure function, no rclpy -- unit-testable in isolation."""
    while len(cache) > max_size:
        cache.popitem(last=False)


def _posestamped_to_matrix(msg: PoseStamped) -> np.ndarray:
    p, q = msg.pose.position, msg.pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _stamp_to_float(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _compute_corrections(T_olds: list, T_news: list) -> np.ndarray:
    """Per-keyframe correction A_i = T_new_i @ inv(T_old_i). Returns (K,4,4).
    Pure function, no rclpy -- unit-testable in isolation."""
    return np.array([T_new @ np.linalg.inv(T_old)
                      for T_old, T_new in zip(T_olds, T_news)])


def _interpolate_correction(t: float, keyframe_ts: np.ndarray,
                            corrections: np.ndarray) -> np.ndarray:
    """Correction A(t) for an arbitrary scan stamp, by lerping translation and
    Slerp-ing rotation between the two keyframes bracketing t (nearest-
    keyframe assignment would step by >voxel size mid-segment and re-create
    the double-surface artifact this whole cache exists to avoid). Clamped to
    the nearest end correction for t outside [keyframe_ts[0], keyframe_ts[-1]].
    keyframe_ts must be sorted ascending. Pure function, no rclpy."""
    k = len(keyframe_ts)
    if k == 1 or t <= keyframe_ts[0]:
        return corrections[0]
    if t >= keyframe_ts[-1]:
        return corrections[-1]

    i = int(np.searchsorted(keyframe_ts, t, side='right') - 1)
    i = min(max(i, 0), k - 2)
    t0, t1 = keyframe_ts[i], keyframe_ts[i + 1]
    u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)

    A0, A1 = corrections[i], corrections[i + 1]
    translation = (1.0 - u) * A0[:3, 3] + u * A1[:3, 3]
    slerp = Slerp([0.0, 1.0], Rotation.from_matrix([A0[:3, :3], A1[:3, :3]]))
    rotation = slerp(u).as_matrix()

    A = np.eye(4, dtype=np.float64)
    A[:3, :3] = rotation
    A[:3, 3] = translation
    return A


def _extract_voxels(tsdf_grid, weights_grid, voxel_size: float,
                    min_weight: float = 1.0,
                    max_d: 'float | None' = None
                    ) -> 'tuple[np.ndarray|None, np.ndarray|None, np.ndarray|None]':
    """Iterate the VDB TSDF grid and collect active leaf voxels.

    Parameters
    ----------
    tsdf_grid, weights_grid : pyopenvdb FloatGrid
    voxel_size : metres — used to convert corner → centre
    min_weight : skip voxels with weight < this
    max_d      : if given, skip voxels with d > max_d  (early free-space filter)

    Returns
    -------
    (world_centers, d_vals, w_vals) as float32 arrays, or (None, None, None).

    VDB coordinate convention
    -------------------------
    ``indexToWorld(coord)`` returns the **corner** of the voxel.
    VDBFusion computes SDF at the **centre** = corner + voxel_size/2 in each axis.
    We add the offset so the published cubes sit at the correct world position.
    """
    w_acc = weights_grid.getAccessor()
    half  = voxel_size / 2.0

    pts_list: list = []
    d_list:   list = []
    w_list:   list = []

    for item in tsdf_grid.iterOnValues():
        if item.count != 1:          # skip interior tiles (count > 1)
            continue
        d = item.value
        if max_d is not None and d > max_d:
            continue                  # early filter: skip free space
        coord = item.min
        w = w_acc.getValue(coord)
        if w < min_weight:
            continue
        corner = tsdf_grid.transform.indexToWorld(coord)   # pyopenvdb Vec3d
        pts_list.append((corner[0] + half, corner[1] + half, corner[2] + half))
        d_list.append(d)
        w_list.append(w)

    if not pts_list:
        return None, None, None

    return (np.array(pts_list, dtype=np.float32),
            np.array(d_list,   dtype=np.float32),
            np.array(w_list,   dtype=np.float32))


def _extract_voxel_arrays(tsdf_grid, weights_grid, min_weight: float
                          ) -> 'tuple[np.ndarray|None, np.ndarray|None, np.ndarray|None]':
    """One iterOnValues() pass over active leaf voxels with weight >= min_weight.

    Returns (coords, d_vals, w_vals) where coords is (N,3) int64 **signed VDB
    index** coordinates (item.min), not world metres — the caller derives both
    world centres (coord*voxel_size + half, VDBFusion uses a translation-free
    linear transform) and integer grid cells from them without float rounding.
    Returns (None, None, None) when no voxel qualifies.
    """
    w_acc = weights_grid.getAccessor()
    coords_list: list = []
    d_list:      list = []
    w_list:      list = []

    for item in tsdf_grid.iterOnValues():
        if item.count != 1:          # skip interior tiles (count > 1)
            continue
        coord = item.min
        w = w_acc.getValue(coord)
        if w < min_weight:
            continue
        coords_list.append((coord[0], coord[1], coord[2]))
        d_list.append(item.value)
        w_list.append(w)

    if not coords_list:
        return None, None, None

    return (np.array(coords_list, dtype=np.int64),
            np.array(d_list,      dtype=np.float32),
            np.array(w_list,      dtype=np.float32))


def _build_projected_map(coords: np.ndarray, d_vals: np.ndarray, w_vals: np.ndarray,
                         voxel_size: float, z_lo: float, z_hi: float,
                         occ_max_d: float, occ_min_weight: float,
                         margin: int, frame: str, stamp
                         ) -> 'OccupancyGrid | None':
    """Project banded TSDF voxels to a 2-D nav_msgs/OccupancyGrid.

    coords are signed VDB indices (see _extract_voxel_arrays). Only voxels whose
    world Z (coord_z*voxel_size + half) falls in [z_lo, z_hi] project. Column
    classification (occupied wins):
      occupied  weight >= occ_min_weight AND d <= occ_max_d  (== /tsdf/occupied_voxels)
      free      d > 0                                        (weight already >= map min)
    The grid matches the OctoMap projection convention find_frontier_clusters /
    build_cost_grid expect: int8 data, row-major row=y/col=x, -1 unknown / 0 free
    / 100 occupied, origin at the corner of cell (0,0), identity orientation.
    Returns None when nothing lies in the band.
    """
    half = voxel_size / 2.0
    world_z = coords[:, 2].astype(np.float64) * voxel_size + half
    in_band = (world_z >= z_lo) & (world_z <= z_hi)
    if not np.any(in_band):
        return None

    cxy = coords[in_band, :2]
    d = d_vals[in_band]
    w = w_vals[in_band]
    occ = (w >= occ_min_weight) & (d <= occ_max_d)
    free = d > 0.0

    occ_cells = np.unique(cxy[occ], axis=0) if np.any(occ) else np.empty((0, 2), np.int64)
    free_cells = np.unique(cxy[free], axis=0) if np.any(free) else np.empty((0, 2), np.int64)
    if len(occ_cells) == 0 and len(free_cells) == 0:
        return None

    known = np.vstack([occ_cells, free_cells])
    ix_min, iy_min = int(known[:, 0].min()), int(known[:, 1].min())
    ix_max, iy_max = int(known[:, 0].max()), int(known[:, 1].max())
    width = (ix_max - ix_min + 1) + 2 * margin
    height = (iy_max - iy_min + 1) + 2 * margin

    grid = np.full((height, width), -1, dtype=np.int8)
    # Free first so occupied wins any column holding both.
    if len(free_cells):
        grid[free_cells[:, 1] - iy_min + margin, free_cells[:, 0] - ix_min + margin] = 0
    if len(occ_cells):
        grid[occ_cells[:, 1] - iy_min + margin, occ_cells[:, 0] - ix_min + margin] = 100

    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.info.resolution = float(voxel_size)
    msg.info.width = int(width)
    msg.info.height = int(height)
    msg.info.origin.position.x = float((ix_min - margin) * voxel_size)
    msg.info.origin.position.y = float((iy_min - margin) * voxel_size)
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = grid.reshape(-1).tolist()
    return msg


def _compute_normals_vdb(tsdf_grid, world_points: np.ndarray,
                         voxel_size: float) -> np.ndarray:
    """Finite-difference gradient of the VDB TSDF at world positions.

    Returns (N,3) float32; rows with no valid gradient are NaN.
    """
    acc     = tsdf_grid.getAccessor()
    normals = np.full((len(world_points), 3), np.nan, dtype=np.float32)
    two_h   = 2.0 * voxel_size

    for row, wp in enumerate(world_points):
        ijk  = tsdf_grid.transform.worldToIndex((float(wp[0]), float(wp[1]), float(wp[2])))
        i    = int(round(ijk[0]))
        j    = int(round(ijk[1]))
        k    = int(round(ijk[2]))
        dx   = (acc.getValue((i + 1, j, k)) - acc.getValue((i - 1, j, k))) / two_h
        dy   = (acc.getValue((i, j + 1, k)) - acc.getValue((i, j - 1, k))) / two_h
        dz   = (acc.getValue((i, j, k + 1)) - acc.getValue((i, j, k - 1))) / two_h
        n    = np.array([dx, dy, dz], dtype=np.float32)
        nm   = float(np.linalg.norm(n))
        if nm > 1e-6:
            normals[row] = n / nm

    return normals


def _confidence_colormap(conf_norm: np.ndarray, saturation=None) -> np.ndarray:
    """RGBA colormap for observation-count confidence, normalised to [0, 1].

    0 = just cleared the min-observation-count floor (least-trusted voxel
    still shown this scan), 1 = the most-observed voxel in this scan
    (log-scaled). Low confidence -> orange, high confidence -> green.

    saturation: optional [0, 1] array. When given, colour saturation is scaled
    by it (pale toward grey when low, full when high) without changing the hue;
    saturation=1 reproduces the original fully-saturated colour exactly. Omitted
    keeps the original behaviour.
    """
    t = np.clip(conf_norm, 0.0, 1.0)
    c = np.zeros((len(t), 4), dtype=np.float32)
    c[:, 0] = 1.0 - 0.9 * t     # red:   1.0 -> 0.1
    c[:, 1] = 0.55 + 0.35 * t   # green: 0.55 -> 0.9
    c[:, 2] = 0.2 * t           # blue:  0.0 -> 0.2
    if saturation is not None:
        s = (0.2 + 0.8 * np.clip(saturation, 0.0, 1.0)).reshape(-1, 1)
        c[:, :3] = np.float32(0.7) * (1.0 - s) + c[:, :3] * s
    # Opaque, not translucent: any alpha < 1 pushes the whole CUBE_LIST into
    # OGRE's transparent render queue, which draws in insertion order rather
    # than depth order — with thousands of cubes in one marker, overlapping
    # ones then render in the wrong front/back order (the "everything
    # overlays wrong" look). octomap_rviz_plugins sidesteps this the same
    # way: its "Occupied Voxels" mode uses Voxel Alpha = 1 for exactly this
    # reason (its near-invisible "Free Voxels" haze mode uses ~0.01, where
    # the same sorting glitch is imperceptible). We already gate this view
    # to confidently-solid, well-observed voxels, so there's no meaningful
    # transparency information left to encode anyway.
    c[:, 3] = 1.0
    return c


def _make_pointcloud2(header: Header, points: np.ndarray) -> PointCloud2:
    msg             = PointCloud2()
    msg.header      = header
    msg.height      = 1
    msg.width       = len(points)
    msg.is_dense    = True
    msg.is_bigendian = False
    msg.point_step  = 12
    msg.row_step    = 12 * len(points)
    msg.fields      = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = points.astype(np.float32).tobytes() if len(points) > 0 else b''
    return msg


def _make_normals_cloud(header: Header, points: np.ndarray,
                        normals: np.ndarray) -> PointCloud2:
    """PointCloud2 with x,y,z + normal_x,normal_y,normal_z (PCL field naming)."""
    msg              = PointCloud2()
    msg.header       = header
    msg.height       = 1
    msg.width        = len(points)
    msg.is_dense     = True
    msg.is_bigendian = False
    msg.point_step   = 24
    msg.row_step     = 24 * len(points)
    msg.fields       = [
        PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='normal_x', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='normal_y', offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name='normal_z', offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    if len(points) > 0:
        msg.data = np.hstack([points, normals]).astype(np.float32).tobytes()
    else:
        msg.data = b''
    return msg


def _normals_markers(points: np.ndarray, normals: np.ndarray,
                     header: Header) -> MarkerArray:
    markers  = MarkerArray()
    length   = 0.3
    lifetime = Duration(sec=2)
    for i, (p, n) in enumerate(zip(points, normals)):
        if np.any(np.isnan(n)):
            continue
        m           = Marker()
        m.header    = header
        m.ns        = 'tsdf_normals'
        m.id        = i
        m.type      = Marker.ARROW
        m.action    = Marker.ADD
        m.lifetime  = lifetime
        m.scale.x   = 0.03    # shaft diameter
        m.scale.y   = 0.07    # head diameter
        m.scale.z   = 0.07    # head length
        m.color     = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.8)
        m.points    = [
            Point(x=float(p[0]),               y=float(p[1]),               z=float(p[2])),
            Point(x=float(p[0] + n[0]*length), y=float(p[1] + n[1]*length),
                  z=float(p[2] + n[2]*length)),
        ]
        markers.markers.append(m)
    return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TSDFMapper()
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
