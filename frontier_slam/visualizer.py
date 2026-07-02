"""RViz and debug-image publishers for the frontier exploration system.

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
C_OCCUPIED = (16,  65,  158)         # blue   (debug-image, 0-255)
C_FREE     = (245, 245, 245)         # near-white
C_UNKNOWN  = (208, 217, 238)         # light blue
C_PATH     = (38,  174, 96)          # green
C_GOAL255  = (245, 194, 40)          # amber  (debug-image, 0-255)


class FrontierVisualizer:
    def __init__(self, node):
        self._node = node
        self._viz_pub          = node.create_publisher(MarkerArray,   '/frontier_slam/frontiers',    1)
        self._inflated_map_pub = node.create_publisher(OccupancyGrid, '/frontier_slam/inflated_map', 1)
        self._debug_img_pub    = node.create_publisher(Image,         '/frontier_slam/debug_image',  1)

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
    def publish_debug_image(self, cg: CostGrid | None, grid_msg,
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
        self._debug_img_pub.publish(msg)


# ------------------------------------------------------------------
# Module-level helpers — no node state

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
