"""Tests for the frontier-provenance cell extraction."""

import numpy as np

from frontier_slam.frontier_detection import (
    find_frontier_clusters,
    frontier_cell_points,
)


class _Info:
    def __init__(self, resolution, width, height, ox, oy):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = type('O', (), {'position': type('P', (), {'x': ox, 'y': oy})()})()


class _Grid:
    def __init__(self, data, resolution=1.0, ox=0.0, oy=0.0):
        array = np.asarray(data, dtype=np.int8)
        self.info = _Info(resolution, array.shape[1], array.shape[0], ox, oy)
        self.data = array.reshape(-1).tolist()


# Column 1 is a wall; column 0 is unknown behind it, column 2 free in front.
_WALL = [
    [-1, 100, 0],
    [-1, 100, 0],
    [-1, 100, 0],
]


def test_reports_the_occupied_cells_that_border_unknown():
    frontier_xy, _ = frontier_cell_points(_Grid(_WALL))

    assert sorted(map(tuple, frontier_xy)) == [(1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]


def test_reports_the_unknown_cells_that_made_them_frontiers():
    _, border_xy = frontier_cell_points(_Grid(_WALL))

    assert sorted(map(tuple, border_xy)) == [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0)]


def test_free_space_is_neither_frontier_nor_border():
    frontier_xy, border_xy = frontier_cell_points(_Grid(_WALL))
    both = np.vstack([frontier_xy, border_xy])

    assert not np.any(both[:, 0] == 2.0)


def test_cells_use_the_same_world_convention_as_the_cluster_centroid():
    grid = _Grid(_WALL, resolution=0.2, ox=-4.0, oy=7.0)
    frontier_xy, _ = frontier_cell_points(grid)
    cluster = find_frontier_clusters(grid, min_cluster_cells=1)[0]

    assert np.allclose(frontier_xy.mean(axis=0), [cluster.wx, cluster.wy])


def test_a_fully_known_map_reports_no_cells():
    frontier_xy, border_xy = frontier_cell_points(_Grid([[0, 100], [0, 100]]))

    assert len(frontier_xy) == 0
    assert len(border_xy) == 0
