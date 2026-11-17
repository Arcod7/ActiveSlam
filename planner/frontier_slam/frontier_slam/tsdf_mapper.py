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
                           voxels inside the planning band — the Z slice
                           /projected_map is built from — are tinted blue over
                           that colour, so the height the planner actually
                           reasons over is visible against the map it ignores.
                           Only available with publish_projected_map:=true.
  /tsdf/viz_cap          (std_msgs/String) one-line warning for the RViz HUD
                           panel when max_voxels_viz thins the CUBE_LIST;
                           empty string clears it
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
import math
import threading
import time

import numpy as np
import rclpy
import small_gicp
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from scipy.spatial.transform import Rotation, Slerp
from vdbfusion import VDBVolume


# Blue-ish tint blended over the voxel colour inside the planning band —
# keep in sync with the "Map voxels (TSDF)" group in legend_rviz.
BAND_TINT     = np.array([0.05, 0.20, 1.00], dtype=np.float32)
BAND_TINT_MIX = 0.5   # 1.0 would erase the weight/confidence shading


class TSDFMapper(Node):
    """Threading: ingest (cloud + TF drain + replay), stats, and each of the
    three outputs -- surface, voxel CUBE_LIST, planning map -- own a
    MutuallyExclusiveCallbackGroup and run under a MultiThreadedExecutor, so a
    slow grid walk cannot starve /cloud_in or the TF callbacks that resolve
    deferred scans. _volume_lock serialises every VDB access across those
    threads; each output is duty-capped by its own gate and cached by revision
    so it re-extracts only when the map actually changed.

    The three outputs are deliberately not one job. The CUBE_LIST walk is
    Python per active voxel and grows with the carved volume (8 s on a 110k
    map, measured 2026-07-31), and it is decorative. /projected_map is what the
    planner routes on and /tsdf/surface_cloud is what the wall controller
    steers on; sharing one gate and one group put both behind the CUBE_LIST."""

    PUBLISH_HZ   = 1.0   # surface cloud + normals (upper bound; see viz_max_duty)
    VOXEL_VIZ_HZ = 0.5   # voxel CUBE_LIST (expensive iteration)
    PLANNING_HZ  = 2.0   # /projected_map (thin Z band, vectorised)

    def __init__(self) -> None:
        super().__init__('tsdf_mapper')

        # ── parameters ──────────────────────────────────────────────────
        self.declare_parameter('world_frame',      'world_ned')
        self.declare_parameter('cloud_frame',      'bluerov2/Dcam')
        self.declare_parameter('voxel_size',       0.2)
        # <= 0 resolves to 3x voxel_size, the minimum VDBFusion needs for a
        # usable gradient. Set it explicitly to map structure thinner than
        # 2x trunc, which a single signed field cannot hold (see
        # directional_tsdf).
        self.declare_parameter('trunc_distance',   0.0)
        self.declare_parameter('space_carving',    True)
        # WIP: one volume per view-direction bin instead of one shared field,
        # so a surface seen from both faces stops averaging itself away.
        self.declare_parameter('directional_tsdf', False)
        self.declare_parameter('min_weight',       2.0)
        self.declare_parameter('voxel_min_weight', 10.0)   # hide voxels observed fewer times
        self.declare_parameter('voxel_min_solid_confidence', 0.80)  # see module docstring
        self.declare_parameter('normal_every',     10)
        self.declare_parameter('max_voxels_viz',   100_000)
        self.declare_parameter('show_free_voxels', False)
        # Discard points beyond the simulated Sonar 3D-15 beam range before
        # integration. The depth-camera proxy may produce farther off-axis
        # points, but the physical Water Linked sensor has a 15 m radial range.
        self.declare_parameter('max_range_m', 15.0)
        # Free-space carving for no-return pixels: mark every voxel the ray
        # crossed free, with no surface at the far end (_carve_no_return_rays).
        self.declare_parameter('carve_no_return', False)
        # <= 0 resolves to max_range_m: a no-return ray carries no information
        # past the range the sensor could have detected a return at.
        self.declare_parameter('carve_range_m', 0.0)
        # Pixel decimation for carve rays. At 15 m two adjacent Sonar 3D-15
        # pixels are ~0.09 m apart, so stride 2 still samples below voxel_size.
        self.declare_parameter('carve_pixel_stride', 2)
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
        self.declare_parameter('projected_map_band_m', 3.0)
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
        # Sized to ride out one full viz cycle rather than one cloud period: a
        # marching-cubes pass on a large map takes seconds, and a 0.5s deadline
        # expired every scan once the map grew, freezing it permanently.
        self.declare_parameter('tf_wait_timeout_s', 5.0)
        self.declare_parameter('tf_queue_size', 30)
        self.declare_parameter('tf_queue_tick_s', 0.02)
        # Ceiling on the wall-clock fraction the surface/voxel extractions may
        # consume; the rest is left to ingest.
        self.declare_parameter('viz_max_duty', 0.25)
        # Cap on sampled normals per surface publish (0 = normal_every only).
        self.declare_parameter('max_normals', 4000)

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
        if self._carve_range <= 0.0:
            self._carve_range = self._max_range
        self._carve_stride = max(1, int(self.get_parameter('carve_pixel_stride').value))
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
        directional        = bool(self.get_parameter('directional_tsdf').value)

        if trunc <= 0.0:
            trunc = 3.0 * voxel_size

        self._voxel_size = voxel_size
        self._trunc      = trunc
        self._space_carving = space_carving
        self._directional = directional
        # solid-confidence = (trunc - d) / (2*trunc) >= voxel_min_solid_confidence
        #   <=>  d <= trunc * (1 - 2*voxel_min_solid_confidence)
        self._voxel_max_d = trunc * (1.0 - 2.0 * self._voxel_min_solid_confidence)
        self._volumes    = self._new_volumes()
        # Every VDB access is serialised on this; the volumes themselves are not
        # thread-safe and a rebuild swaps them outright.
        self._volume_lock = threading.Lock()
        self._pyopenvdb = bool(self._volumes[0].pyopenvdb_support_enabled)
        # Bumped on every integrate/replay/reset so viz can skip unchanged maps.
        self._map_revision = 0

        if not self._pyopenvdb:
            self.get_logger().warn(
                'VDBFusion built without pyopenvdb — voxel visualisation disabled')

        self.get_logger().info(
            f'TSDF  voxel={voxel_size}m  trunc={trunc}m  space_carving={space_carving}'
            + (f'  directional={len(self._volumes)} bins (WIP)' if directional else ''))

        # Thresholds that filter what the map publishes rather than what it
        # stores, so they can move without touching the grid. Settable at
        # runtime for exactly that reason: retuning a wall threshold should not
        # cost the map built so far.
        self.add_on_set_parameters_callback(self._on_set_parameters)

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

        # ── callback groups (see class docstring) ────────────────────────
        # TransformListener already runs /tf in its own ReentrantCallbackGroup,
        # so it stays live whatever these two are doing.
        self._ingest_group = MutuallyExclusiveCallbackGroup()
        self._viz_group    = MutuallyExclusiveCallbackGroup()
        self._stats_group  = MutuallyExclusiveCallbackGroup()
        # One group each, so the CUBE_LIST walk cannot stall the two outputs a
        # moving vehicle steers on. Sharing _viz_group starved /tsdf/surface_cloud
        # for the whole of a grid walk -- past the controller's 5 s staleness
        # limit once the walk reached 8 s, which dropped its wall-side lock.
        self._surface_group  = MutuallyExclusiveCallbackGroup()
        self._planning_group = MutuallyExclusiveCallbackGroup()

        viz_max_duty = float(self.get_parameter('viz_max_duty').value)
        self._max_normals = max(0, int(self.get_parameter('max_normals').value))
        self._surface_gate  = _DutyGate(viz_max_duty)
        self._voxels_gate   = _DutyGate(viz_max_duty)
        self._planning_gate = _DutyGate(viz_max_duty)
        self._surface_revision  = -1
        self._voxels_revision   = -1
        self._planning_revision = -1
        self._surface_cache  = None
        self._voxels_cache   = None
        self._planning_cache = None

        # ── pub/sub ──────────────────────────────────────────────────────
        # Depth matches the TF queue: a viz cycle can block ingest for its whole
        # duration, and scans dropped by DDS never reach the deferral queue.
        self.create_subscription(PointCloud2, '/cloud_in', self._cloud_cb,
                                 self._tf_queue_size,
                                 callback_group=self._ingest_group)
        self.create_timer(tf_queue_tick_s, self._tf_queue_tick,
                          callback_group=self._ingest_group)
        self.create_timer(10.0, self._log_cloud_tf_stats,
                          callback_group=self._stats_group)

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
            self.create_subscription(Int32, '/slam/rebuild/begin', self._rebuild_begin_cb, 10,
                                     callback_group=self._ingest_group)
            self.create_subscription(Path, '/slam/rebuild/path_old', self._rebuild_path_old_cb, 10,
                                     callback_group=self._ingest_group)
            self.create_subscription(Path, '/slam/rebuild/path_new', self._rebuild_path_new_cb, 10,
                                     callback_group=self._ingest_group)
            rebuild_tick_s = float(self.get_parameter('rebuild_tick_s').value)
            self.create_timer(rebuild_tick_s, self._replay_tick,
                              callback_group=self._ingest_group)
            self.get_logger().info(
                'Map rebuild consumer enabled: will reset+re-integrate cached scans '
                'on a validated /slam/rebuild/path_old + path_new pair')

        self._cache_scans_pub = self.create_publisher(Int32, '/tsdf/cache_scans', 10)
        self._cloud_pub   = self.create_publisher(PointCloud2, '/tsdf/surface_cloud',   1)
        self._normals_pub = self.create_publisher(MarkerArray, '/tsdf/surface_normals',  1)
        self._normals_cloud_pub = self.create_publisher(
            PointCloud2, '/tsdf/surface_normals_cloud', 1)
        self._voxels_pub  = self.create_publisher(MarkerArray, '/tsdf/voxels',           1)
        # Transient-local: the HUD row survives an RViz restart between two
        # voxel ticks instead of coming up blank on a capped map.
        self._viz_cap_pub = self.create_publisher(
            String, '/tsdf/viz_cap',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
        self._viz_cap      = ''
        self._viz_cap_last = None
        self._solid_cloud_pub = self.create_publisher(
            PointCloud2, '/tsdf/occupied_voxels', 1)

        # 2-D planning map on /projected_map. Needs the pyopenvdb grid path for
        # free-space (d>0) voxels — the surface-vertex fallback has none, so a
        # surface-derived projection would be all-unknown-free and useless.
        self._projected_map_pub = None
        if self._projected_map_enabled:
            if self._pyopenvdb:
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
            if self._pyopenvdb:
                self._free_cloud_pub = self.create_publisher(
                    PointCloud2, '/tsdf/free_voxels', 1)
            else:
                self.get_logger().error(
                    'publish_free_voxels requested but VDBFusion lacks pyopenvdb '
                    '— /tsdf/free_voxels disabled')

        # Latest marching-cubes vertices, so the voxel view can be derived from
        # them when pyopenvdb (the grid path) is unavailable.
        self._last_surface_verts = None

        self.create_timer(1.0 / self.PUBLISH_HZ,   self._publish_surface,
                          callback_group=self._surface_group)
        self.create_timer(1.0 / self.VOXEL_VIZ_HZ, self._publish_voxels,
                          callback_group=self._viz_group)
        if self._projected_map_pub is not None:
            self.create_timer(1.0 / self.PLANNING_HZ, self._publish_projected_map,
                              callback_group=self._planning_group)

        self.get_logger().info('tsdf_mapper ready')

    # ────────────────────────────────────────────────────────────────────
    # Live parameters
    # ────────────────────────────────────────────────────────────────────

    LIVE_PARAMS = ('voxel_min_weight', 'voxel_min_solid_confidence',
                   'target_depth_m', 'projected_map_band_m')

    def _on_set_parameters(self, params):
        """Apply the publish-time thresholds; refuse what the grid was built on.

        Rejecting the rest is the point: voxel_size and trunc_distance are
        baked into the VDB volume, and silently accepting them would report a
        resolution the map does not have.
        """
        unsupported = [p.name for p in params if p.name not in self.LIVE_PARAMS]
        if unsupported:
            return SetParametersResult(
                successful=False,
                reason=f"{', '.join(unsupported)}: launch-time only")
        for p in params:
            value = float(p.value)
            if p.name == 'voxel_min_weight':
                self._voxel_min_weight = value
            elif p.name == 'voxel_min_solid_confidence':
                self._voxel_min_solid_confidence = value
                self._voxel_max_d = self._trunc * (1.0 - 2.0 * value)
            elif p.name == 'projected_map_band_m':
                # Like the centre, live: the projection is recomputed from the
                # volume each cycle, so widening the band costs no map.
                self._projected_map_band = abs(value)
            else:
                self._target_depth = value
                # Negative hands the band centre back to the next TF lookup.
                self._projected_map_z = value if value >= 0.0 else None
            self.get_logger().info(f'{p.name} = {value:g} (live)')
        # The viz and projection caches redraw on a revision change only.
        self._map_revision += 1
        return SetParametersResult(successful=True)

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
            f'queued={len(self._tf_queue)} rev={self._map_revision} '
            f'viz: surface={self._surface_gate.last_s:.2f}s '
            f'voxels={self._voxels_gate.last_s:.2f}s '
            f'planning={self._planning_gate.last_s:.2f}s')

    # ────────────────────────────────────────────────────────────────────
    # Directional TSDF (WIP)
    #
    # A TSDF's sign is defined by the side the sensor observed a surface from,
    # so a structure thinner than 2x trunc that is seen from both faces writes
    # opposing fields into the same voxels. VDBFusion weight-averages them, the
    # zero crossing flattens to ~0 and the surface dissolves into speckle.
    #
    # Binning each ray by its dominant view axis keeps the two faces in separate
    # volumes, which never average; reads merge by picking the most solid bin
    # per voxel rather than averaging. Splietker & Behnke (IROS 2019) bin by
    # surface normal — view direction is the cheap proxy, since a face is only
    # ever observed from the side it points at.
    #
    # Not yet validated against the ground-truth map: costs 6 grid walks per viz
    # cycle, and the merge biases free/solid disputes toward solid.
    # ────────────────────────────────────────────────────────────────────

    DIRECTION_BINS = 6      # +X -X +Y -Y +Z -Z

    def _new_volumes(self) -> list:
        n = self.DIRECTION_BINS if self._directional else 1
        return [VDBVolume(self._voxel_size, self._trunc,
                          space_carving=self._space_carving) for _ in range(n)]

    def _integrate_into(self, pts_world: np.ndarray, origin: np.ndarray) -> None:
        """Integrate one scan, split across direction bins when directional."""
        if not self._directional:
            self._volumes[0].integrate(pts_world, origin)
            return
        bins = _direction_bins(pts_world - origin)
        for b, volume in enumerate(self._volumes):
            sel = pts_world[bins == b]
            if len(sel):
                volume.integrate(sel, origin)

    def _carve_no_return_rays(self, dirs_world: np.ndarray,
                              origin: np.ndarray) -> int:
        """Write d=+trunc (fully free) into every voxel a no-return ray crossed.

        integrate() cannot express this: its rays end at a point, and that
        endpoint always writes a zero crossing — a surface. A no-return ray has
        no endpoint, so the voxels are written directly instead, saturated free
        out to carve_range_m and nothing beyond it.
        """
        bins = (_direction_bins(dirs_world) if self._directional
                else np.zeros(len(dirs_world), dtype=int))
        # Ray marching is the expensive half and touches no grid, so it stays
        # outside the lock; only the writes are serialised against viz.
        parts = [(volume, free_ray_voxels(dirs_world[bins == b], origin,
                                          self._voxel_size, self._carve_range))
                 for b, volume in enumerate(self._volumes)]
        total = 0
        with self._volume_lock:
            for volume, voxels in parts:
                for ijk in voxels.tolist():
                    volume.update_tsdf(self._trunc, ijk)
                total += len(voxels)
        return total

    def _integrate_cloud(self, msg: PointCloud2, tf_msg) -> bool:
        """Filter and integrate one cloud using its exact capture-time TF."""
        pts_cam = _parse_pointcloud2(msg)   # (N,3) float64, sensor frame
        if pts_cam is None or len(pts_cam) == 0:
            return False

        # Filter by radial beam range. This keeps TSDF aligned with /cloud_in
        # and the real Sonar 3D-15's 15 m acoustic range.
        pts_cam = pts_cam[np.linalg.norm(pts_cam, axis=1) < self._max_range]

        carve_dirs = None
        if self._carve_no_return:
            raw = _parse_pointcloud2_raw(msg)
            if raw is not None:
                carve_dirs = no_return_ray_dirs(raw, msg.width, self._carve_stride)

        if len(pts_cam) == 0 and (carve_dirs is None or not len(carve_dirs)):
            return False

        T = _tf_to_matrix(tf_msg.transform)   # T_world_cam (4×4, float64)
        R, t = T[:3, :3], T[:3, 3]

        # ── KEY FIX ──────────────────────────────────────────────────────
        # VDBFusion.integrate(points, origin) expects points in WORLD frame.
        # The 4×4 extrinsic overload only extracts T[:3,3] (origin) and does
        # NOT rotate the points.  We must rotate them ourselves.
        pts_world = pts_cam @ R.T + t      # (N,3) world frame, float64
        origin    = t                       # camera origin in world frame, float64

        carved = 0
        if len(pts_world):
            with self._volume_lock:
                self._integrate_into(pts_world, origin)
        if carve_dirs is not None and len(carve_dirs):
            carved = self._carve_no_return_rays(carve_dirs @ R.T, origin)
        self._map_revision += 1

        if self._enable_rebuild:
            self._cache_scan(msg.header.stamp, pts_cam, T)

        self.get_logger().info(
            f'Integrated {len(pts_world)} pts  carved {carved} voxels  '
            f'cam=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})',
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
        # Once this saturates at cache_max_scans, a rebuild can no longer
        # re-integrate the start of the run and permanently drops that surface.
        self._cache_scans_pub.publish(Int32(data=len(self._scan_cache)))

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

        with self._volume_lock:
            self._volumes = self._new_volumes()
        self._map_revision += 1
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
        with self._volume_lock:
            for key in chunk:
                entry = self._scan_cache.get(key)
                if entry is None:
                    continue   # evicted between snapshot and replay
                pts_cam, T = entry
                R, t = T[:3, :3], T[:3, 3]
                pts_world = pts_cam.astype(np.float64) @ R.T + t
                self._integrate_into(pts_world, t)
        self._map_revision += 1
        if not self._replay_queue:
            self.get_logger().info('Map rebuild replay complete')

    # ────────────────────────────────────────────────────────────────────
    # Surface cloud + normals (marching cubes via VDBFusion)
    # ────────────────────────────────────────────────────────────────────

    def _publish_surface(self) -> None:
        started = self._monotonic()
        if (self._map_revision != self._surface_revision
                and self._surface_gate.ready(started)):
            self._rebuild_surface_cache(started)

        if self._surface_cache is None:
            return
        # Re-sent unchanged so RViz keeps the arrows alive past their lifetime.
        cloud, markers, normals_cloud = self._surface_cache
        stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(_restamp(cloud, stamp))
        if markers is not None:
            self._normals_pub.publish(_restamp(markers, stamp))
            self._normals_cloud_pub.publish(_restamp(normals_cloud, stamp))

    def _normal_stride(self, n_verts: int) -> int:
        """normal_every, widened so at most max_normals samples are built."""
        stride = max(1, self._normal_every)
        if self._max_normals > 0:
            stride = max(stride, -(-n_verts // self._max_normals))
        return stride

    def _rebuild_surface_cache(self, started: float) -> None:
        """Re-run marching cubes and rebuild the cached surface messages."""
        revision = self._map_revision
        sampled = normals = None
        with self._volume_lock:
            try:
                # Each direction bin meshes on its own — a shared marching-cubes
                # pass is exactly the averaging this mode avoids. reshape keeps
                # an empty bin at (0,3) so the concatenate below still matches.
                per_bin = [np.asarray(v.extract_triangle_mesh(
                    fill_holes=False, min_weight=float(self._min_weight))[0],
                    dtype=np.float32).reshape(-1, 3) for v in self._volumes]
            except Exception as exc:
                self.get_logger().warn(
                    f'extract_triangle_mesh failed: {exc}', throttle_duration_sec=5.0)
                per_bin = None
            verts = None if per_bin is None else np.concatenate(per_bin)
            # Normals need the TSDF grid (pyopenvdb-only); the cloud does not.
            if verts is not None and len(verts) and self._pyopenvdb:
                stride = self._normal_stride(len(verts))
                # Sampled per bin so each vertex's normal comes from the grid
                # that actually meshed it.
                pairs = [(p[::stride], v) for p, v in zip(per_bin, self._volumes)
                         if len(p[::stride])]
                if pairs:
                    sampled = np.concatenate([p for p, _ in pairs])
                    normals = np.concatenate([
                        _compute_normals_vdb(v.tsdf, p, self._voxel_size)
                        for p, v in pairs])
        finished = self._monotonic()
        self._surface_gate.record(finished, finished - started)
        if verts is None:
            return

        self._surface_revision = revision
        if len(verts) == 0:
            return
        self._last_surface_verts = verts

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = self._world_frame

        markers = normals_cloud = None
        if normals is not None:
            markers = _normals_markers(sampled, normals, header)
            valid = ~np.isnan(normals).any(axis=1)
            normals_cloud = _make_normals_cloud(header, sampled[valid], normals[valid])
        self._surface_cache = (_make_pointcloud2(header, verts), markers, normals_cloud)

        self.get_logger().info(
            f'Surface: {len(verts)} pts in {self._surface_gate.last_s:.2f}s',
            throttle_duration_sec=5.0)

    # ────────────────────────────────────────────────────────────────────
    # Voxel CUBE_LIST visualisation
    # ────────────────────────────────────────────────────────────────────

    def _publish_voxels(self) -> None:
        if not self._pyopenvdb:
            self._publish_voxels_from_surface()
            return

        started = self._monotonic()
        if (self._map_revision != self._voxels_revision
                and self._voxels_gate.ready(started)):
            self._rebuild_voxel_cache(started)

        if self._voxels_cache is None:
            return
        solid_cloud, markers, free_cloud = self._voxels_cache
        stamp = self.get_clock().now().to_msg()
        self._solid_cloud_pub.publish(_restamp(solid_cloud, stamp))
        self._voxels_pub.publish(_restamp(markers, stamp))
        self._publish_viz_cap(self._viz_cap)
        if free_cloud is not None:
            self._free_cloud_pub.publish(_restamp(free_cloud, stamp))

    def _publish_viz_cap(self, text: str) -> None:
        """Latched, and only on change -- a HUD row is state, not a stream."""
        if text != self._viz_cap_last:
            self._viz_cap_last = text
            self._viz_cap_pub.publish(String(data=text))

    def _rebuild_voxel_cache(self, started: float) -> None:
        """Walk the VDB grid once and rebuild every cached voxel-view message.

        Only the walk needs the volume lock; everything downstream is numpy and
        message building over plain arrays."""
        revision = self._map_revision
        banded = self._free_cloud_pub is not None
        coords = all_d = all_w = None
        pts = d_vals = w_vals = None
        with self._volume_lock:
            if banded:
                # One walk feeds the solid viz and the free cloud: keep every
                # voxel at/above the low map weight (min_weight) so free (d>0)
                # cells survive, then re-filter to solids below for the
                # CUBE_LIST + /tsdf/occupied_voxels.
                coords, all_d, all_w = _merge_voxel_parts([
                    _extract_voxel_arrays(v.tsdf, v.weights, self._min_weight)
                    for v in self._volumes])
            else:
                # Only iterate voxels we'll actually show (early filtering inside):
                # confidently solid (TSDF-derived) AND observed often enough (weight).
                max_d = self._voxel_max_d if not self._show_free else None
                pts, d_vals, w_vals = _merge_voxel_parts([
                    _extract_voxels(v.tsdf, v.weights, self._voxel_size,
                                    min_weight=self._voxel_min_weight, max_d=max_d)
                    for v in self._volumes])
        finished = self._monotonic()
        self._voxels_gate.record(finished, finished - started)
        self._voxels_revision = revision

        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self._world_frame)

        free_cloud = None
        if banded:
            free_cloud = self._build_free_voxels_msg(coords, all_d, header)
            if coords is not None:
                solid = (all_w >= self._voxel_min_weight) & (all_d <= self._voxel_max_d)
                if np.any(solid):
                    pts = (coords[solid].astype(np.float32) * self._voxel_size
                           + self._voxel_size / 2.0)
                    d_vals = all_d[solid]
                    w_vals = all_w[solid]

        if pts is None:
            del_m = Marker()
            del_m.header.stamp    = now
            del_m.header.frame_id = self._world_frame
            del_m.ns     = 'tsdf_voxels'
            del_m.id     = 0
            del_m.action = Marker.DELETE
            self._viz_cap = ''
            self._voxels_cache = (
                _make_pointcloud2(header, np.empty((0, 3), dtype=np.float32)),
                MarkerArray(markers=[del_m]),
                free_cloud)
            return

        # This remains a solid-only cloud even if the optional voxel
        # visualisation includes free space.  It is consumed by the frontier
        # planner to veto only goals physically inside a TSDF solid voxel.
        solid_pts = pts if not self._show_free else pts[d_vals <= self._voxel_max_d]
        solid_cloud = _make_pointcloud2(header, solid_pts)

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

        band_mask = self._planning_band_voxel_mask(pts)
        if band_mask is not None:
            colors[band_mask, :3] = (
                BAND_TINT_MIX * BAND_TINT
                + (1.0 - BAND_TINT_MIX) * colors[band_mask, :3])

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

        self._viz_cap = _cap_status(len(pts), n)
        self._voxels_cache = (solid_cloud, MarkerArray(markers=[m]), free_cloud)
        self.get_logger().info(
            f'Voxels: {len(pts)} of {n} published in {self._voxels_gate.last_s:.2f}s',
            throttle_duration_sec=5.0)

    # ────────────────────────────────────────────────────────────────────
    # 2-D planning map (/projected_map) derived from the TSDF grid
    # ────────────────────────────────────────────────────────────────────

    def _publish_projected_map(self) -> None:
        """Own timer, own gate, own callback group.

        This is the map the planner inflates and routes on, so it must not
        share a duty gate with the CUBE_LIST: that tied a safety input to a
        decorative one and pushed both to a ~30 s period on a large map."""
        started = self._monotonic()
        if (self._map_revision != self._planning_revision
                and self._planning_gate.ready(started)):
            self._rebuild_planning_cache(started)

        if self._planning_cache is None:
            return
        self._projected_map_pub.publish(
            _restamp(self._planning_cache, self.get_clock().now().to_msg()))

    def _rebuild_planning_cache(self, started: float) -> None:
        """Read only the Z band the projection uses, densely.

        The band is a handful of voxel layers, so a dense copyToArray over its
        bounding box is bounded by the map's XY footprint alone and stays
        vectorised throughout — where the full iterOnValues() walk is Python
        per voxel and grows with every carved free cell in the volume.

        Only the work is charged to the duty gate, not the wait for
        _volume_lock. The CUBE_LIST walk holds that lock for seconds, and
        charging its contention here muted the planning map for 12 s after a
        0.1 s read — measured 2026-07-31, a 4.20 s "read" against a 4.07 s
        concurrent walk. The gate exists to bound CPU spent, and blocking on a
        mutex spends none."""
        revision = self._map_revision
        z = self._band_center_z()
        if z is None:
            return
        kz_lo, kz_hi = _band_index_range(
            z - self._projected_map_band, z + self._projected_map_band,
            self._voxel_size)
        with self._volume_lock:
            work_started = self._monotonic()
            coords, d_vals, w_vals = _merge_voxel_parts([
                _extract_band_arrays(v.tsdf, v.weights, self._min_weight,
                                     kz_lo, kz_hi)
                for v in self._volumes])
            elapsed = self._monotonic() - work_started
        self._planning_gate.record(self._monotonic(), elapsed)
        self._planning_revision = revision

        grid = self._build_projected_map_msg(
            coords, d_vals, w_vals, self.get_clock().now().to_msg())
        if grid is not None:            # else keep the last good band
            self._planning_cache = grid
        self.get_logger().info(
            f'Planning map: {0 if coords is None else len(coords)} banded voxels '
            f'in {self._planning_gate.last_s:.2f}s',
            throttle_duration_sec=5.0)

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

    def _build_projected_map_msg(self, coords, d_vals, w_vals, stamp):
        if self._projected_map_pub is None or coords is None:
            return None
        z = self._band_center_z()
        if z is None:
            return None
        return _build_projected_map(
            coords, d_vals, w_vals, self._voxel_size,
            z_lo=z - self._projected_map_band, z_hi=z + self._projected_map_band,
            occ_max_d=self._voxel_max_d, occ_min_weight=self._voxel_min_weight,
            margin=self._projected_map_margin,
            frame=self._world_frame, stamp=stamp)

    def _planning_band_voxel_mask(self, pts) -> 'np.ndarray | None':
        """Which voxels lie in the Z band /projected_map is built from.

        Everything outside it is invisible to the planner, so this is the slice
        its decisions actually rest on — the same [z-band, z+band] test
        _build_projected_map applies. Returns None when no projected map is
        published (there is no planning band to show) or its centre has not
        resolved yet.
        """
        if pts is None or len(pts) == 0 or self._projected_map_pub is None:
            return None
        z = self._band_center_z()
        if z is None:
            return None
        mask = np.abs(pts[:, 2] - z) <= self._projected_map_band
        return mask if np.any(mask) else None

    def _build_free_voxels_msg(self, coords, d_vals, header):
        """Observed-empty voxel centres, the free half tsdf_to_octomap needs."""
        if self._free_cloud_pub is None:
            return None
        if coords is None:
            return _make_pointcloud2(header, np.empty((0, 3), dtype=np.float32))
        free = d_vals > 0.0
        pts = (coords[free].astype(np.float32) * self._voxel_size
               + self._voxel_size / 2.0)
        return _make_pointcloud2(header, pts)

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

        n_total = len(centers)
        if n_total > self._max_viz:
            centers = centers[np.random.choice(n_total, self._max_viz, replace=False)]

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
        self._publish_viz_cap(_cap_status(len(centers), n_total))
        self.get_logger().info(
            f'Voxels (surface-derived): {len(centers)} of {n_total} published',
            throttle_duration_sec=5.0)


# ══════════════════════════════════════════════════════════════════════════════
# Module-level helpers
# ══════════════════════════════════════════════════════════════════════════════

class _DutyGate:
    """Bounds a slow periodic task to a fraction of wall-clock time.

    After a run lasting `elapsed`, blocks the next one until
    elapsed*(1/max_duty - 1) has passed, so the task can never occupy more than
    max_duty of its thread however large the map grows. Timings come from the
    caller, so this is pure and unit-testable in isolation."""

    def __init__(self, max_duty: float) -> None:
        self._max_duty = min(max(float(max_duty), 1e-3), 1.0)
        self._next_ok = 0.0
        self.last_s = 0.0

    def ready(self, now: float) -> bool:
        return now >= self._next_ok

    def record(self, now: float, elapsed: float) -> None:
        self.last_s = max(0.0, elapsed)
        self._next_ok = now + self.last_s * (1.0 / self._max_duty - 1.0)


def _restamp(msg, stamp):
    """Refresh a cached message's stamp in place before republishing."""
    if isinstance(msg, MarkerArray):
        for m in msg.markers:
            m.header.stamp = stamp
    else:
        msg.header.stamp = stamp
    return msg


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


def no_return_ray_dirs(xyz: np.ndarray, width: int,
                       stride: int = 1) -> np.ndarray:
    """Unit rays (sensor frame) for the pixels that returned nothing.

    Directions come from intrinsics fitted to the cloud itself rather than a
    restatement of the sensor's FoV. Returns an empty (0,3) array when the
    geometry can't be recovered or nothing is missing."""
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
    if stride > 1:
        keep = (u % stride == 0) & (v % stride == 0)
        u, v = u[keep], v[keep]
    dirs = np.column_stack([(u - cx) / fx, (v - cy) / fy, np.ones(len(u))])
    return dirs / np.linalg.norm(dirs, axis=1, keepdims=True)


def free_ray_voxels(dirs_world: np.ndarray, origin: np.ndarray,
                    voxel_size: float, max_range: float) -> np.ndarray:
    """Unique VDB index coords the rays cross, out to max_range.

    Sampled at half a voxel so no cell along a ray is stepped over; the origin
    cell is included, nothing past max_range is."""
    if not len(dirs_world) or max_range <= 0.0:
        return np.empty((0, 3), dtype=np.int64)
    step = voxel_size * 0.5
    t = np.arange(0.0, max_range, step, dtype=np.float64)
    pts = origin + dirs_world[:, None, :] * t[None, :, None]
    ijk = np.floor(pts.reshape(-1, 3) / voxel_size).astype(np.int64)

    # Row-unique via a single packed key — np.unique(axis=0) lexsorts and is
    # several times slower on the ~10^6 samples a full no-return frame makes.
    lo, hi = ijk.min(axis=0), ijk.max(axis=0)
    span = hi - lo + 1
    key = (((ijk[:, 0] - lo[0]) * span[1]) + (ijk[:, 1] - lo[1])) * span[2] \
        + (ijk[:, 2] - lo[2])
    return ijk[np.unique(key, return_index=True)[1]]


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


def _direction_bins(dirs: np.ndarray) -> np.ndarray:
    """Bin index per ray: dominant axis * 2, +1 when that component is negative.

    Opposite views of one surface land in different bins, which is the whole
    point — see TSDFMapper's directional-TSDF note.
    """
    axis = np.argmax(np.abs(dirs), axis=1)
    negative = dirs[np.arange(len(dirs)), axis] < 0.0
    return axis * 2 + negative


def _merge_voxel_parts(parts: list) -> tuple:
    """Collapse per-bin (keys, d, w) triples to one, keeping the lowest d per key.

    Picking the most solid bin rather than averaging is what stops a two-sided
    surface cancelling itself; keys are VDB index coords or world centres, both
    bit-identical across bins since every bin shares one grid transform.
    """
    parts = [p for p in parts if p[0] is not None]
    if not parts:
        return None, None, None
    if len(parts) == 1:
        return parts[0]

    keys = np.concatenate([p[0] for p in parts])
    d_vals = np.concatenate([p[1] for p in parts])
    w_vals = np.concatenate([p[2] for p in parts])
    # Primary key is the last lexsort argument: sorts by x, y, z, then d.
    order = np.lexsort((d_vals, keys[:, 2], keys[:, 1], keys[:, 0]))
    keys, d_vals, w_vals = keys[order], d_vals[order], w_vals[order]
    first = np.ones(len(keys), dtype=bool)
    first[1:] = np.any(keys[1:] != keys[:-1], axis=1)
    return keys[first], d_vals[first], w_vals[first]


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


def _band_index_range(z_lo: float, z_hi: float,
                      voxel_size: float) -> 'tuple[int, int]':
    """Inclusive VDB Z indices whose voxel centres fall in [z_lo, z_hi].

    Centre of index k is k*voxel_size + voxel_size/2, the convention
    _extract_voxel_arrays and _build_projected_map both use."""
    half = voxel_size / 2.0
    return (int(math.ceil((z_lo - half) / voxel_size)),
            int(math.floor((z_hi - half) / voxel_size)))


def _extract_band_arrays(tsdf_grid, weights_grid, min_weight: float,
                         kz_lo: int, kz_hi: int
                         ) -> 'tuple[np.ndarray|None, np.ndarray|None, np.ndarray|None]':
    """Vectorised counterpart of _extract_voxel_arrays over a Z slab only.

    Reads the slab densely with copyToArray instead of stepping active voxels
    in Python. Inactive cells come back at the grids' background values, and
    the weights background is 0.0, so the ``w >= min_weight`` filter selects
    exactly the active set for any min_weight > 0 (checked against
    activeVoxelCount). Returns the same (coords, d_vals, w_vals) triple, with
    coords as signed VDB index coordinates.
    """
    if kz_hi < kz_lo:
        return None, None, None
    try:
        (i0, j0, k0), (i1, j1, k1) = tsdf_grid.evalActiveVoxelBoundingBox()
    except Exception:
        return None, None, None
    k0, k1 = max(k0, kz_lo), min(k1, kz_hi)
    if i1 < i0 or j1 < j0 or k1 < k0:
        return None, None, None

    shape = (i1 - i0 + 1, j1 - j0 + 1, k1 - k0 + 1)
    d = np.zeros(shape, dtype=np.float32)
    w = np.zeros(shape, dtype=np.float32)
    tsdf_grid.copyToArray(d, ijk=(i0, j0, k0))
    weights_grid.copyToArray(w, ijk=(i0, j0, k0))

    keep = w >= min_weight
    if not np.any(keep):
        return None, None, None
    ii, jj, kk = np.nonzero(keep)
    coords = np.column_stack([ii + i0, jj + j0, kk + k0]).astype(np.int64)
    return coords, d[keep], w[keep]


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


def _cap_status(shown: int, total: int) -> str:
    """One HUD line for /tsdf/viz_cap when max_voxels_viz thins the CUBE_LIST.

    The cap is a render budget on the marker only -- /tsdf/occupied_voxels
    still carries every solid voxel -- so the wording has to say the map is
    complete, or a thinned view reads as a mapping failure. Empty string when
    nothing was dropped, which clears the HUD row rather than leaving a stale
    warning once the map shrinks back under the cap.
    """
    if total <= 0 or shown >= total:
        return ''
    pct = 100.0 * shown / total
    return (f'VOXEL VIEW CAPPED - showing {shown} of {total} ({pct:.0f}%) - '
            f'display limit only, map is complete')


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
    # One thread each for ingest, viz and stats, plus the listener's own /tf
    # group — a slow extraction must never hold up cloud or TF callbacks.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
