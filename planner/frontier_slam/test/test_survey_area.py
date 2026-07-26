"""Tests for the survey working area that bounds frontier goal selection."""

from dataclasses import dataclass

from frontier_slam.goal_manager import GoalManager

import numpy as np


@dataclass
class _Cluster:
    wx: float
    wy: float
    size: int = 10
    distance: float = 10.0


def _manager(**kwargs):
    return GoalManager(min_explore_dist=3.0, goal_vanish_dist=3.0,
                       goal_radius=2.0, stuck_timeout=30.0,
                       stuck_min_progress=0.5, blacklist_duration=30.0,
                       arrival_blacklist_duration=20.0, **kwargs)


def test_unbounded_by_default_so_existing_runs_are_unchanged():
    gm = _manager()

    assert gm.inside_survey_area(1000.0, -1000.0)


def test_radius_without_a_centre_is_still_unbounded():
    # The centre is only known once odometry arrives; until then nothing may
    # be rejected, or the first goals would be dropped at the origin.
    gm = _manager(survey_radius=20.0)

    assert gm.inside_survey_area(500.0, 500.0)


def test_goal_inside_the_radius_is_a_candidate():
    gm = _manager(survey_center=(10.0, 10.0), survey_radius=20.0)

    assert gm.inside_survey_area(20.0, 15.0)


def test_goal_beyond_the_radius_is_rejected():
    gm = _manager(survey_center=(10.0, 10.0), survey_radius=20.0)

    assert not gm.inside_survey_area(45.0, 10.0)


def test_boundary_is_inclusive():
    gm = _manager(survey_center=(0.0, 0.0), survey_radius=20.0)

    assert gm.inside_survey_area(20.0, 0.0)


def test_selection_prefers_an_in_area_cluster_over_a_better_scored_outside_one():
    # The outside cluster scores better (closer, same size) and would win
    # without the bound -- that is exactly the open-water excursion the area
    # exists to prevent.
    gm = _manager(survey_center=(0.0, 0.0), survey_radius=25.0)
    near_outside = _Cluster(wx=60.0, wy=0.0, size=40, distance=5.0)
    far_inside = _Cluster(wx=20.0, wy=0.0, size=10, distance=20.0)

    sel = gm.select([near_outside, far_inside], np.array([0.0, 0.0]), 100.0)

    assert (sel.gx, sel.gy) == (20.0, 0.0)


def test_all_candidates_outside_reports_a_finished_survey_not_a_fault():
    gm = _manager(survey_center=(0.0, 0.0), survey_radius=25.0)
    outside = [_Cluster(wx=60.0, wy=0.0), _Cluster(wx=-70.0, wy=10.0)]

    sel = gm.select(outside, np.array([0.0, 0.0]), 100.0)

    assert sel.event == 'OUTSIDE_SURVEY_AREA'
    assert np.isnan(sel.gx)


def test_out_of_area_candidates_do_not_consume_the_blacklist():
    # An unreachable-by-policy goal is not a failed goal; blacklisting it
    # would expire real entries early and mask genuine stuck detection.
    gm = _manager(survey_center=(0.0, 0.0), survey_radius=25.0)

    gm.select([_Cluster(wx=60.0, wy=0.0)], np.array([0.0, 0.0]), 100.0)

    assert gm.blacklist_size == 0


def test_commitment_is_dropped_when_the_area_runs_out():
    # Otherwise the vehicle keeps driving to a goal the policy just rejected.
    gm = _manager(survey_center=(0.0, 0.0), survey_radius=25.0)
    gm.select([_Cluster(wx=20.0, wy=0.0)], np.array([0.0, 0.0]), 100.0)

    gm.select([_Cluster(wx=60.0, wy=0.0)], np.array([0.0, 0.0]), 101.0)

    assert gm._committed is None
