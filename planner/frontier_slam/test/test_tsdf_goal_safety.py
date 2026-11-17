"""Tests for TSDF solid-voxel frontier-goal validation."""

from frontier_slam.frontier_detection import (
    averaged_surface_normal,
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


def _flat_wall(n: int = 5, spacing: float = 0.2) -> tuple:
    """A wall in the x=2 plane: samples along y, all facing +x."""
    offsets = (np.arange(n) - (n - 1) / 2.0) * spacing
    points = np.column_stack((np.full(n, 2.0), offsets, np.full(n, 8.0)))
    return points, np.tile([1.0, 0.0, 0.0], (n, 1))


def test_average_pulls_a_noisy_sample_back_onto_a_flat_wall():
    points, normals = _flat_wall()
    noisy = np.array([0.7, 0.7, 0.0]) / np.sqrt(2.0)

    blended = averaged_surface_normal(noisy, points, normals, [2.0, 0.0, 8.0], 0.8)

    assert np.allclose(np.linalg.norm(blended), 1.0)
    assert blended[0] > noisy[0]   # swung back toward the wall's own +x normal
    assert blended[1] < noisy[1]


def test_average_of_a_corner_bisects_the_two_walls():
    # +x-facing wall meeting a +y-facing one, equal counts and distances.
    points = np.array([[2.0, 0.0, 8.0], [2.2, 0.2, 8.0]])
    normals = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    blended = averaged_surface_normal(
        [1.0, 0.0, 0.0], points, normals, [2.1, 0.1, 8.0], 1.0)

    assert np.allclose(blended, [np.sqrt(0.5), np.sqrt(0.5), 0.0])


def test_average_ignores_the_far_face_of_a_thin_wall():
    # Opposing normals would cancel to nothing if they were averaged in.
    near, near_normals = _flat_wall()
    points = np.vstack((near, near + [0.3, 0.0, 0.0]))
    normals = np.vstack((near_normals, -near_normals))

    blended = averaged_surface_normal(
        [1.0, 0.0, 0.0], points, normals, [2.0, 0.0, 8.0], 0.8)

    assert np.allclose(blended, [1.0, 0.0, 0.0])


def test_average_falls_back_to_the_anchor_when_disabled_or_alone():
    points, normals = _flat_wall()
    anchor = np.array([0.6, 0.8, 0.0])

    assert np.allclose(
        averaged_surface_normal(anchor, points, normals, [2.0, 0.0, 8.0], 0.0), anchor)
    assert np.allclose(
        averaged_surface_normal(anchor, np.empty((0, 3)), np.empty((0, 3)),
                                [2.0, 0.0, 8.0], 0.8), anchor)


def test_average_skips_samples_beyond_the_radius():
    # A tilted sample outside the radius gets zero weight, not a small one.
    points = np.array([[2.0, 0.0, 8.0], [2.0, 5.0, 8.0]])
    normals = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    blended = averaged_surface_normal(
        [1.0, 0.0, 0.0], points, normals, [2.0, 0.0, 8.0], 0.8)

    assert np.allclose(blended, [1.0, 0.0, 0.0])


def test_averaged_normal_feeds_the_standoff_unchanged_in_shape():
    points, normals = _flat_wall()
    blended = averaged_surface_normal(
        [0.9, 0.4, 0.0], points, normals, [2.0, 0.0, 8.0], 0.8)
    goal = standoff_point_from_tsdf_surface([2.0, 0.0, 8.0], blended, 1.5)

    assert np.allclose(goal, [3.5, 0.0])
