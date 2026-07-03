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
                                 readable form, consumed by wall_follower
  /tsdf/voxels           (visualization_msgs/MarkerArray)
                           CUBE_LIST per weight bucket:
                             size  ∝ weight  (log-scale, 10 buckets, 0.1×–1.0× voxel)
                             color ∝ d value (red=occupied · green=surface · blue=free)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from scipy.spatial.transform import Rotation
from vdbfusion import VDBVolume

_N_BUCKETS = 10   # weight buckets for size encoding


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
        self.declare_parameter('surface_thresh_m', 0.15)   # |d| < this → show as surface
        self.declare_parameter('normal_every',     10)
        self.declare_parameter('max_voxels_viz',   40_000)
        self.declare_parameter('show_free_voxels', False)
        # Discard points beyond this range before integration.
        # Depth sensors return valid readings at their physical maximum range
        # when looking into open water ("no return").  Without this filter,
        # every direction the camera sweeps produces occupied voxels at max
        # range, creating ghost geometry during rotation.
        self.declare_parameter('max_range_m', 15.0)

        self._world_frame  = self.get_parameter('world_frame').value
        self._cloud_frame  = self.get_parameter('cloud_frame').value
        self._min_weight   = float(self.get_parameter('min_weight').value)
        self._surf_thresh  = float(self.get_parameter('surface_thresh_m').value)
        self._normal_every = int(self.get_parameter('normal_every').value)
        self._max_viz      = int(self.get_parameter('max_voxels_viz').value)
        self._show_free    = bool(self.get_parameter('show_free_voxels').value)
        self._max_range    = float(self.get_parameter('max_range_m').value)
        voxel_size         = float(self.get_parameter('voxel_size').value)
        trunc              = float(self.get_parameter('trunc_distance').value)
        space_carving      = bool(self.get_parameter('space_carving').value)

        self._voxel_size = voxel_size
        self._trunc      = trunc
        self._volume     = VDBVolume(voxel_size, trunc, space_carving=space_carving)

        if not self._volume.pyopenvdb_support_enabled:
            self.get_logger().warn(
                'VDBFusion built without pyopenvdb — voxel visualisation disabled')

        self.get_logger().info(
            f'TSDF  voxel={voxel_size}m  trunc={trunc}m  space_carving={space_carving}')

        self._latest_frame = self._cloud_frame

        # ── TF ──────────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── pub/sub ──────────────────────────────────────────────────────
        self.create_subscription(PointCloud2, '/cloud_in', self._cloud_cb, 5)

        self._cloud_pub   = self.create_publisher(PointCloud2, '/tsdf/surface_cloud',   1)
        self._normals_pub = self.create_publisher(MarkerArray, '/tsdf/surface_normals',  1)
        self._normals_cloud_pub = self.create_publisher(
            PointCloud2, '/tsdf/surface_normals_cloud', 1)
        self._voxels_pub  = self.create_publisher(MarkerArray, '/tsdf/voxels',           1)

        self.create_timer(1.0 / self.PUBLISH_HZ,   self._publish_surface)
        self.create_timer(1.0 / self.VOXEL_VIZ_HZ, self._publish_voxels)

        self.get_logger().info('tsdf_mapper ready')

    # ────────────────────────────────────────────────────────────────────
    # Cloud callback — integrate into TSDF immediately
    # ────────────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if msg.header.frame_id:
            self._latest_frame = msg.header.frame_id

        pts_cam = _parse_pointcloud2(msg)   # (N,3) float64, sensor frame
        if pts_cam is None or len(pts_cam) == 0:
            return

        # Range filter — drop points at/beyond the sensor's physical max range.
        # Those are "no-return" readings (open water), not real surfaces.
        pts_cam = pts_cam[np.linalg.norm(pts_cam, axis=1) < self._max_range]
        if len(pts_cam) == 0:
            return

        # TF at the cloud's capture time.
        # Do NOT fall back to the latest TF on ExtrapolationException: during
        # rotation the latest TF differs from the capture-time TF, placing
        # points in wrong world positions and creating permanent ghost voxels.
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._world_frame, self._latest_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (tf2_ros.ExtrapolationException,
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f'TF unavailable ({type(exc).__name__}), skipping scan',
                throttle_duration_sec=5.0)
            return

        T = _tf_to_matrix(tf_msg.transform)   # T_world_cam (4×4, float64)
        R, t = T[:3, :3], T[:3, 3]

        # ── KEY FIX ──────────────────────────────────────────────────────
        # VDBFusion.integrate(points, origin) expects points in WORLD frame.
        # The 4×4 extrinsic overload only extracts T[:3,3] (origin) and does
        # NOT rotate the points.  We must rotate them ourselves.
        pts_world = pts_cam @ R.T + t      # (N,3) world frame, float64
        origin    = t                       # camera origin in world frame, float64

        self._volume.integrate(pts_world, origin)

        self.get_logger().info(
            f'Integrated {len(pts_world)} pts  cam=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})',
            throttle_duration_sec=2.0)

    # ────────────────────────────────────────────────────────────────────
    # Surface cloud + normals (marching cubes via VDBFusion)
    # ────────────────────────────────────────────────────────────────────

    def _publish_surface(self) -> None:
        if not self._volume.pyopenvdb_support_enabled:
            return

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

        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = self._world_frame

        self._cloud_pub.publish(_make_pointcloud2(header, verts))

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
            return

        # Only iterate voxels we'll actually show (early filtering inside)
        max_d = self._surf_thresh if not self._show_free else None
        pts, d_vals, w_vals = _extract_voxels(
            self._volume.tsdf, self._volume.weights,
            self._voxel_size, min_weight=1.0, max_d=max_d)

        if pts is None:
            return

        n = len(pts)
        if n > self._max_viz:
            sel    = np.random.choice(n, self._max_viz, replace=False)
            pts    = pts[sel]
            d_vals = d_vals[sel]
            w_vals = w_vals[sel]

        # SDF → colour  (values are raw metres; normalise to [-1, 1])
        d_norm = np.clip(d_vals / self._trunc, -1.0, 1.0)
        colors  = _tsdf_colormap(d_norm)

        # Weight → size bucket (log scale, 10 levels)
        w_max   = max(float(w_vals.max()), 2.0)
        w_norm  = np.log1p(np.clip(w_vals - 1.0, 0.0, None)) / np.log1p(w_max - 1.0)
        w_norm  = np.clip(w_norm, 0.0, 1.0)
        buckets = np.clip((w_norm * _N_BUCKETS).astype(int), 0, _N_BUCKETS - 1)

        size_min = 0.1 * self._voxel_size
        size_max = 1.0 * self._voxel_size
        now      = self.get_clock().now().to_msg()
        lifetime = Duration(sec=4)
        markers  = MarkerArray()

        for b in range(_N_BUCKETS):
            mask = buckets == b
            if not np.any(mask):
                del_m = Marker()
                del_m.header.stamp    = now
                del_m.header.frame_id = self._world_frame
                del_m.ns     = 'tsdf_voxels'
                del_m.id     = b
                del_m.action = Marker.DELETE
                markers.markers.append(del_m)
                continue

            scale  = float(size_min + (b + 0.5) / _N_BUCKETS * (size_max - size_min))
            b_pts  = pts[mask]
            b_clrs = colors[mask]

            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = self._world_frame
            m.ns       = 'tsdf_voxels'
            m.id       = b
            m.type     = Marker.CUBE_LIST
            m.action   = Marker.ADD
            m.lifetime = lifetime
            m.scale.x  = scale
            m.scale.y  = scale
            m.scale.z  = scale
            m.points   = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in b_pts]
            m.colors   = [ColorRGBA(r=float(c[0]), g=float(c[1]),
                                    b=float(c[2]), a=float(c[3])) for c in b_clrs]
            markers.markers.append(m)

        self._voxels_pub.publish(markers)
        self.get_logger().info(f'Voxels: {len(pts)} published', throttle_duration_sec=5.0)


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


def _tf_to_matrix(tf_transform) -> np.ndarray:
    """Convert a ROS Transform message to a 4×4 float64 numpy matrix (T_world_cam)."""
    t = tf_transform.translation
    q = tf_transform.rotation
    R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


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


def _tsdf_colormap(d_norm: np.ndarray) -> np.ndarray:
    """RGBA colormap for normalised VDBFusion TSDF values in [-1, 1].

    VDBFusion convention:
      d = -1  →  occupied  →  RED
      d =  0  →  surface   →  GREEN
      d = +1  →  free      →  BLUE
    """
    t  = np.clip((d_norm + 1.0) / 2.0, 0.0, 1.0)  # 0=occupied, 0.5=surface, 1=free
    c  = np.zeros((len(d_norm), 4), dtype=np.float32)
    lo = t < 0.5                        # occupied → surface  (red → green)
    c[lo, 0] = 1.0 - 2.0 * t[lo]       # red:   1 → 0
    c[lo, 1] = 2.0 * t[lo]             # green: 0 → 1
    hi = ~lo                            # surface → free  (green → blue)
    c[hi, 1] = 2.0 - 2.0 * t[hi]       # green: 1 → 0
    c[hi, 2] = 2.0 * t[hi] - 1.0       # blue:  0 → 1
    c[:, 3]  = 0.75
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
