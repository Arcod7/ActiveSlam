"""Pure functions for detecting frontier clusters in an OccupancyGrid.

A frontier cell is OCCUPIED (=100) and directly adjacent to UNKNOWN (<0). These
wall-surface cells are stable targets: they don't recede as the robot approaches,
and navigating near them lets the sonar illuminate the unknown space beyond the wall
from a new angle. The 2D projected map from OctoMap satisfies this convention.
"""

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import binary_dilation, label


@dataclass
class Cluster:
    wx: float  # world-frame x of centroid (m)
    wy: float  # world-frame y of centroid (m)
    size: int  # number of cells in the cluster
    distance: float = 0.0  # to a reference point — filled in by the caller
    dx: float = 1.0  # unit direction toward the unknown (world x)
    dy: float = 0.0  # unit direction toward the unknown (world y)


def point_inside_tsdf_solid(point_xyz: np.ndarray, solid_xyz: np.ndarray,
                            containment_radius_m: float) -> bool:
    """Return whether a 3-D point lies inside a published solid TSDF voxel.

    ``solid_xyz`` contains centres of confidently occupied TSDF voxels, not
    surface samples.  The radius should be about half the voxel diagonal; it
    tests containment only, rather than creating a wall-clearance buffer that
    would discard the occupied-boundary frontiers this planner intentionally
    uses as exploration targets.
    """
    if solid_xyz is None or len(solid_xyz) == 0 or containment_radius_m <= 0.0:
        return False
    delta = np.asarray(solid_xyz, dtype=np.float64) - np.asarray(point_xyz, dtype=np.float64)
    return bool(np.min(np.einsum('ij,ij->i', delta, delta))
                <= containment_radius_m * containment_radius_m)


def standoff_point_from_tsdf_surface(surface_xyz: np.ndarray,
                                     normal_xyz: np.ndarray,
                                     standoff_m: float) -> np.ndarray | None:
    """Return a horizontal free-space standoff point from a TSDF surface.

    VDBFusion's signed distance is positive in free space and negative in a
    solid.  Its TSDF gradient therefore points outward into free space.  Only
    the horizontal normal component is used because frontier navigation holds
    depth.  ``None`` means the normal belongs to a floor/ceiling or is invalid.
    """
    surface = np.asarray(surface_xyz, dtype=np.float64)
    normal = np.asarray(normal_xyz, dtype=np.float64)
    if surface.shape != (3,) or normal.shape != (3,) or not np.isfinite(normal).all():
        return None
    horizontal = normal[:2]
    length = float(np.linalg.norm(horizontal))
    if length < 1e-6:
        return None
    return surface[:2] + max(0.0, standoff_m) * horizontal / length


def find_frontier_clusters(grid_msg, min_cluster_cells: int = 5) -> list:
    """Return clusters of frontier cells in the OccupancyGrid.

    The returned distance field is left at 0.0; the caller is expected to fill
    it relative to whatever reference (usually the robot pose) it cares about.
    """
    res = grid_msg.info.resolution
    ox = grid_msg.info.origin.position.x
    oy = grid_msg.info.origin.position.y
    w, h = grid_msg.info.width, grid_msg.info.height

    grid = np.array(grid_msg.data, dtype=np.int8).reshape(h, w)
    occupied = grid == 100
    unknown = grid < 0
    frontier = occupied & binary_dilation(unknown)

    labeled, n = label(frontier)
    if n == 0:
        return []

    clusters = []
    for i in range(1, n + 1):
        mask = labeled == i
        rows, cols = np.where(mask)
        if len(rows) < min_cluster_cells:
            continue
        wx = ox + cols.mean() * res
        wy = oy + rows.mean() * res
        # Direction toward the unknown: from the cluster centroid to the
        # centroid of the unknown cells it borders (unit vector, world frame;
        # col -> world x, row -> world y, matching wx/wy above).
        rmin, rmax = max(0, rows.min() - 1), min(h - 1, rows.max() + 1)
        cmin, cmax = max(0, cols.min() - 1), min(w - 1, cols.max() + 1)
        adj_unknown_crop = (
            binary_dilation(mask[rmin : rmax + 1, cmin : cmax + 1])
            & unknown[rmin : rmax + 1, cmin : cmax + 1]
        )
        ur, uc = np.where(adj_unknown_crop)
        ur = ur + rmin
        uc = uc + cmin
        dx, dy = 1.0, 0.0
        if len(ur):
            vx, vy = uc.mean() - cols.mean(), ur.mean() - rows.mean()
            norm = float(np.hypot(vx, vy))
            if norm > 1e-6:
                dx, dy = vx / norm, vy / norm
        clusters.append(
            Cluster(
                wx=float(wx), wy=float(wy), size=len(rows), dx=float(dx), dy=float(dy)
            )
        )
    return clusters
