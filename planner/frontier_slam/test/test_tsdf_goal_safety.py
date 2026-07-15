"""Tests for TSDF solid-voxel frontier-goal validation."""

from frontier_slam.frontier_detection import (
    point_inside_tsdf_solid,
    standoff_point_from_tsdf_surface,
)

import numpy as np


def test_rejects_goal_at_the_centre_of_a_solid_voxel():
    solids = np.array([[2.0, -1.0, 8.0]])

    assert point_inside_tsdf_solid([2.0, -1.0, 8.0], solids, 0.20)


def test_keeps_frontier_on_the_surface_side_of_a_solid_voxel():
    # A frontier is normally on/near the observed wall surface.  The solid
    # TSDF voxel is behind that surface, so strict containment must not turn
    # this into an over-conservative wall-clearance filter.
    solids = np.array([[2.4, -1.0, 8.0]])

    assert not point_inside_tsdf_solid([2.0, -1.0, 8.0], solids, 0.20)


def test_uses_current_depth_when_testing_a_2d_frontier_goal():
    solids = np.array([[2.0, -1.0, 9.0]])

    assert not point_inside_tsdf_solid([2.0, -1.0, 8.0], solids, 0.20)


def test_standoff_moves_outward_along_the_horizontal_tsdf_normal():
    goal = standoff_point_from_tsdf_surface(
        [2.0, -1.0, 8.0], [3.0, 4.0, 0.0], 1.5)

    assert np.allclose(goal, [2.9, 0.2])


def test_standoff_skips_floor_or_ceiling_normals():
    goal = standoff_point_from_tsdf_surface(
        [2.0, -1.0, 8.0], [0.0, 0.0, 1.0], 1.0)

    assert goal is None
