"""RViz and planning-dashboard-image publishers for the frontier exploration system.

All rendering state is derived from parameters — the class only owns
the three ROS publishers it creates on construction.
"""
import math

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from frontier_slam.path_planner import CostGrid, PAD_CELLS

# --- Poster colour palette (RGB 0-1) -- keep in sync with the poster legend ---
#   Occupied 0.063,0.255,0.620 (blue, set on the OctoMap display in frontier.rviz)
#   Path     set on the Path display in frontier.rviz   |  Background dark (kept)
C_FRONTIER = (0.000, 0.659, 0.757)   # teal-cyan
C_GOAL     = (0.961, 0.761, 0.157)   # amber
C_OCCUPIED = (16,  65,  158)         # blue   (planning dashboard, 0-255)
C_FREE     = (245, 245, 245)         # near-white
C_UNKNOWN  = (208, 217, 238)         # light blue
C_PATH     = (38,  174, 96)          # green
C_GOAL255  = (245, 194, 40)          # amber  (planning dashboard, 0-255)
C_PATH01   = (0.149, 0.682, 0.376)   # C_PATH as RGB 0-1

# Beads marching along the planned path (metres).
FLOW_SPACING = 0.45   # gap between beads
FLOW_STEP    = 0.09   # bead travel per publish
FLOW_BEAD    = 0.22   # bead diameter


class FrontierVisualizer:
    def __init__(self, node):
        self._node = node
        self._viz_pub          = node.create_publisher(MarkerArray,   '/frontier_slam/frontiers',    1)
        self._inflated_map_pub = node.create_publisher(OccupancyGrid, '/frontier_slam/inflated_map', 1)
        self._dashboard_img_pub = node.create_publisher(
            Image, '/frontier_slam/planning_dashboard', 1)
        self._debug_pub = node.create_publisher(
            MarkerArray, '/frontier_slam/frontier_debug', 1)
        self._flow_pub = node.create_publisher(
            MarkerArray, '/frontier_slam/path_flow', 1)
        self._flow_phase = 0.0

    # ------------------------------------------------------------------
    def publish_path_flow(self, path: list, z: float) -> None:
        """Beads travelling along the planned path, brightening toward the goal.

        Driven by its own timer rather than by replan: the publish rate is the
        animation rate, and 3 Hz replan reads as a stutter, not as flow.
        """
        m = Marker()
        m.header.stamp    = self._node.get_clock().now().to_msg()
        m.header.frame_id = 'world_ned'
        m.ns     = 'path_flow'
        m.id     = 0
        m.type   = Marker.SPHERE_LIST
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = FLOW_BEAD
        m.lifetime = Duration(sec=1)

        beads, total = ([], 0.0) if len(path) < 2 else _flow_beads(
            path, FLOW_SPACING, self._flow_phase)
        m.action = Marker.ADD if beads else Marker.DELETE
        self._flow_phase = (self._flow_phase + FLOW_STEP) % FLOW_SPACING
        for x, y, s in beads:
            m.points.append(Point(x=x, y=y, z=z))
            m.colors.append(ColorRGBA(r=C_PATH01[0], g=C_PATH01[1], b=C_PATH01[2],
                                      a=0.35 + 0.6 * (s / total if total else 0.0)))
        markers = MarkerArray()
        markers.markers.append(m)
        self._flow_pub.publish(markers)

    # ------------------------------------------------------------------
    def publish_markers(self, clusters, gx: float, gy: float,
                        robot_pos: np.ndarray) -> None:
        """Publish frontier arrows (pointing into the unknown) and the goal as a MarkerArray."""
        now      = self._node.get_clock().now().to_msg()
        lifetime = Duration(sec=3)
        gz       = float(robot_pos[2])
        markers  = MarkerArray()

        for i, c in enumerate(clusters):
            markers.markers.append(_arrow(
                ns='frontiers', mid=i, x=c.wx, y=c.wy, z=gz,
                dx=c.dx, dy=c.dy, length=0.6, rgba=(*C_FRONTIER, 0.9),
                stamp=now, lifetime=lifetime,
            ))
        markers.markers.append(_sphere(
            ns='goal', mid=0, x=gx, y=gy, z=gz,
            scale=0.8, rgba=(*C_GOAL, 0.95),
            stamp=now, lifetime=lifetime,
        ))

        self._viz_pub.publish(markers)

    # ------------------------------------------------------------------
    def publish_frontier_debug(self, frontier_xy: np.ndarray, border_xy: np.ndarray,
                               standoff_pairs: list, robot_pos: np.ndarray,
                               resolution: float, band_m: float) -> None:
        """Publish the evidence behind each frontier, one RViz namespace per question.

        `frontier_cells`  the un-inflated occupied cells that border unknown —
                          the boundary the inflation overlay covers up.
        `unknown_border`  the unknown cells that made them frontiers.
        `band_columns`    the Z range a single 2-D cell collapses. The planning
                          map is a column projection, so the voxel answering for
                          a cell may sit anywhere in this box, not at cruise
                          depth — the usual reason a frontier floats in water
                          that looks empty. band_m must match the mapper's own
                          projection band or the box lies about the extent.
        `standoff`        cluster centroid → the goal after the TSDF push-off,
                          which is why a goal sits off the wall by design.
        """
        now      = self._node.get_clock().now().to_msg()
        lifetime = Duration(sec=4)
        z        = float(robot_pos[2])
        markers  = MarkerArray()

        markers.markers.append(_cube_list(
            ns='frontier_cells', points=frontier_xy, z=z,
            scale=(resolution, resolution, resolution * 0.5),
            rgba=(*C_FRONTIER, 0.9), stamp=now, lifetime=lifetime))
        markers.markers.append(_cube_list(
            ns='unknown_border', points=border_xy, z=z,
            scale=(resolution, resolution, resolution * 0.5),
            rgba=(0.60, 0.60, 0.68, 0.45), stamp=now, lifetime=lifetime))
        markers.markers.append(_cube_list(
            ns='band_columns', points=frontier_xy, z=z,
            scale=(resolution, resolution, max(2.0 * band_m, resolution)),
            rgba=(*C_FRONTIER, 0.10), stamp=now, lifetime=lifetime))

        line = Marker()
        line.header.stamp    = now
        line.header.frame_id = 'world_ned'
        line.ns     = 'standoff'
        line.id     = 0
        line.type   = Marker.LINE_LIST
        line.action = Marker.ADD if standoff_pairs else Marker.DELETE
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color   = ColorRGBA(r=C_FRONTIER[0], g=C_FRONTIER[1], b=C_FRONTIER[2], a=0.9)
        line.lifetime = lifetime
        for (raw_x, raw_y), (out_x, out_y) in standoff_pairs:
            line.points.append(Point(x=float(raw_x), y=float(raw_y), z=z))
            line.points.append(Point(x=float(out_x), y=float(out_y), z=z))
        markers.markers.append(line)

        self._debug_pub.publish(markers)

    # ------------------------------------------------------------------
    def publish_inflated_map(self, cg: CostGrid | None, grid_msg) -> None:
        """Publish the three-zone inflation overlay as an OccupancyGrid."""
        if cg is None or grid_msg is None:
            return
        info = grid_msg.info
        if cg.raw.shape != (info.height, info.width):
            return

        raw = cg.raw
        out = raw.copy()
        p   = PAD_CELLS
        out[cg.plan_zone[p:-p, p:-p]    & (raw != 100)] = 35
        out[cg.soft_zone[p:-p, p:-p]    & (raw != 100)] = 50
        out[cg.hard_blocked[p:-p, p:-p] & (raw != 100)] = 75

        msg = OccupancyGrid()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'world_ned'
        msg.info            = info
        msg.data            = out.flatten().tolist()
        self._inflated_map_pub.publish(msg)

    # ------------------------------------------------------------------
    def publish_planning_dashboard(self, cg: CostGrid | None, grid_msg,
                            robot_pos: np.ndarray, robot_yaw: float,
                            robot_speed: float, path: list,
                            goal_xy: np.ndarray | None, stuck_pct: int) -> None:
        """Publish a colour-coded overhead map as an RGB8 image."""
        if grid_msg is None or robot_pos is None:
            return
        info = grid_msg.info
        h, w = info.height, info.width
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        raw = (cg.raw if (cg is not None and cg.raw.shape == (h, w))
               else np.asarray(grid_msg.data, dtype=np.int8).reshape(h, w))
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[raw == -1]  = C_UNKNOWN        # unknown  (light blue)
        img[raw == 0]   = C_FREE           # free     (near white)
        img[raw == 100] = C_OCCUPIED       # occupied (blue)

        if cg is not None and cg.raw.shape == (h, w):
            p = PAD_CELLS
            img[cg.plan_zone[p:-p, p:-p]    & (raw != 100)] = (180, 160, 60)
            img[cg.soft_zone[p:-p, p:-p]    & (raw != 100)] = (200, 100, 50)
            img[cg.hard_blocked[p:-p, p:-p] & (raw != 100)] = (150, 30,  30)

        if path:
            pts = [(int((wy - oy) / res), int((wx - ox) / res)) for wx, wy in path]
            for i in range(len(pts) - 1):
                _draw_line(img, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], C_PATH)

        if goal_xy is not None:
            gc = int((goal_xy[0] - ox) / res)
            gr = int((goal_xy[1] - oy) / res)
            if 0 <= gr < h and 0 <= gc < w:
                for d in range(-4, 5):
                    if 0 <= gr + d < h: img[gr + d, gc] = C_GOAL255
                    if 0 <= gc + d < w: img[gr, gc + d] = C_GOAL255

        rc  = int((robot_pos[0] - ox) / res)
        rr  = int((robot_pos[1] - oy) / res)
        arm = max(4, int(4 + robot_speed * 20))
        dc  = int(round(math.cos(robot_yaw) * arm))
        dr  = int(round(math.sin(robot_yaw) * arm))
        color = _arrow_color(goal_xy is not None, stuck_pct)
        if 0 <= rr < h and 0 <= rc < w:
            _draw_line(img, rr, rc, rr + dr, rc + dc, color)
            img[max(0, rr - 1):min(h, rr + 2), max(0, rc - 1):min(w, rc + 2)] = color
            tr, tc = rr + dr, rc + dc
            if 0 <= tr < h and 0 <= tc < w:
                img[max(0, tr - 1):min(h, tr + 2), max(0, tc - 1):min(w, tc + 2)] = color

        # Flip vertical axis (OccupancyGrid row-0 = south, image row-0 = top)
        out = np.flipud(img)

        # 4-pixel status bar at top — colour encodes robot state
        bar = ((0, 150, 150) if goal_xy is None  else
               (200, 80,  0) if stuck_pct >= 100 else
               (200, 200, 0) if stuck_pct >= 50  else
               (0,  120,  0))
        out[0:4, :] = bar

        msg = Image()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'world_ned'
        msg.height          = h
        msg.width           = w
        msg.encoding        = 'rgb8'
        msg.is_bigendian    = False
        msg.step            = w * 3
        msg.data            = out.tobytes()
        self._dashboard_img_pub.publish(msg)


# ------------------------------------------------------------------
# Module-level helpers — no node state

def _flow_beads(path: list, spacing: float, phase: float) -> tuple:
    """Points every `spacing` m along the polyline, the first at arc length
    `phase`. Returns ([(x, y, arc_length)], total_length)."""
    beads  = []
    target = phase
    acc    = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 1e-9:
            continue
        while target <= acc + seg:
            t = (target - acc) / seg
            beads.append((float(x0 + (x1 - x0) * t),
                          float(y0 + (y1 - y0) * t), target))
            target += spacing
        acc += seg
    return beads, acc


def _arrow(*, ns, mid, x, y, z, dx, dy, length, rgba, stamp, lifetime) -> Marker:
    """ARROW marker from (x,y) pointing along (dx,dy) for `length` metres."""
    m = Marker()
    m.header.stamp    = stamp
    m.header.frame_id = 'world_ned'
    m.ns     = ns
    m.id     = mid
    m.type   = Marker.ARROW
    m.action = Marker.ADD
    m.points = [
        Point(x=x,            y=y,            z=z),
        Point(x=x + dx * length, y=y + dy * length, z=z),
    ]
    m.scale.x = 0.12   # shaft diameter
    m.scale.y = 0.28   # head diameter
    m.color   = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    m.lifetime = lifetime
    return m


MAX_DEBUG_CELLS = 20_000   # one cube each; past this RViz stalls on the array


def _cube_list(*, ns, points, z, scale, rgba, stamp, lifetime) -> Marker:
    """CUBE_LIST over an (N,2) world-XY array, all cubes at height `z`."""
    m = Marker()
    m.header.stamp    = stamp
    m.header.frame_id = 'world_ned'
    m.ns     = ns
    m.id     = 0
    m.type   = Marker.CUBE_LIST
    m.action = Marker.ADD if len(points) else Marker.DELETE
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color   = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    m.lifetime = lifetime
    shown = points[:MAX_DEBUG_CELLS]
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=z) for p in shown]
    return m


def _sphere(*, ns, mid, x, y, z, scale, rgba, stamp, lifetime) -> Marker:
    m = Marker()
    m.header.stamp    = stamp
    m.header.frame_id = 'world_ned'
    m.ns     = ns
    m.id     = mid
    m.type   = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color   = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    m.lifetime = lifetime
    return m


def _draw_line(img: np.ndarray, r0: int, c0: int, r1: int, c1: int,
               color: tuple) -> None:
    n  = max(abs(r1 - r0), abs(c1 - c0), 1)
    rs = np.round(np.linspace(r0, r1, n)).astype(int)
    cs = np.round(np.linspace(c0, c1, n)).astype(int)
    hh, ww = img.shape[:2]
    ok = (rs >= 0) & (rs < hh) & (cs >= 0) & (cs < ww)
    img[rs[ok], cs[ok]] = color


def _arrow_color(has_goal: bool, stuck_pct: int) -> tuple:
    if not has_goal:
        return (0, 200, 200)    # cyan  = scanning / no valid goal
    if stuck_pct >= 100:
        return (220, 100, 0)    # orange = maxed stuck
    if stuck_pct >= 50:
        return (220, 220, 0)    # yellow = approaching stuck timeout
    return (50, 150, 255)       # blue  = navigating normally
