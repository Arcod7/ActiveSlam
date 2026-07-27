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


def test_standoff_flips_a_far_face_normal_away_from_the_unknown():
    # Thin walls carry solid voxels on both faces: the nearest sample can be
    # the far one, whose outward normal points into the unknown beyond.
    goal = standoff_point_from_tsdf_surface(
        [2.0, 0.0, 8.0], [1.0, 0.0, 0.0], 1.0, unknown_dir_xy=[1.0, 0.0])

    assert np.allclose(goal, [1.0, 0.0])


def test_standoff_keeps_a_near_face_normal_pointing_out_of_the_unknown():
    goal = standoff_point_from_tsdf_surface(
        [2.0, 0.0, 8.0], [-1.0, 0.0, 0.0], 1.0, unknown_dir_xy=[1.0, 0.0])

    assert np.allclose(goal, [1.0, 0.0])


def test_unknown_direction_wins_over_a_vehicle_that_drove_past_the_wall():
    # Vehicle at x=4 is beyond the wall at x=2; map evidence still says the
    # unknown is at +x, so the goal must not be placed inside it.
    goal = standoff_point_from_tsdf_surface(
        [2.0, 0.0, 8.0], [1.0, 0.0, 0.0], 1.0,
        unknown_dir_xy=[1.0, 0.0], reference_xy=[4.0, 0.0])

    assert np.allclose(goal, [1.0, 0.0])


def test_degenerate_unknown_direction_falls_back_to_the_vehicle_side():
    goal = standoff_point_from_tsdf_surface(
        [2.0, 0.0, 8.0], [1.0, 0.0, 0.0], 1.0,
        unknown_dir_xy=[0.0, 0.0], reference_xy=[0.0, 0.0])

    assert np.allclose(goal, [1.0, 0.0])


def test_standoff_without_any_orientation_hint_keeps_the_raw_normal():
    goal = standoff_point_from_tsdf_surface(
        [2.0, 0.0, 8.0], [1.0, 0.0, 0.0], 1.0)

    assert np.allclose(goal, [3.0, 0.0])
