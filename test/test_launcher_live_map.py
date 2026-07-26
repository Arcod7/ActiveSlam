"""Retuning what the map publishes must not cost the map it has built.

The wall threshold, the observation count and (under TSDF) the projection band
depth are applied by tsdf_mapper when it publishes, not baked into the grid, so
they are pushed to the running node instead of restarting the mapper group.
Voxel size is the counter-case: the VDB volume is built from it.

Also covers the purple-row rule — which options the running stack has not taken
yet — since a value that is neither live nor a restart is what makes it needed.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher as ui                 # noqa: E402
import launcher_core as core          # noqa: E402
import launcher_model as model        # noqa: E402

RUNNING = ["core", "tf", "cloud", "mapper", "gt_map", "slam", "eval", "planner"]


def _groups(tmp_path):
    share = tmp_path / "share"
    (share / "rviz").mkdir(parents=True, exist_ok=True)
    (share / "rviz" / "demo.rviz").write_text("Panels: []\n")
    return core.build_groups(bringup_share=str(share))


@pytest.fixture
def tsdf_values():
    v = dict(model.DEFAULTS)
    v.update({"mode": "frontier", "mapper": "tsdf", "slam": "slam", "rviz": False})
    return v


@pytest.fixture
def sup(tmp_path, tsdf_values):
    s = core.Supervisor(str(tmp_path), str(tmp_path / "logs"), _groups(tmp_path))
    s.running_ids = lambda: list(RUNNING)
    for gid in RUNNING:
        s.applied[gid] = dict(tsdf_values)
    s.applied_values = dict(tsdf_values)
    return s


@pytest.mark.parametrize("pid,value", [("voxel_min_weight", 42.0),
                                       ("voxel_min_solid_confidence", 0.6)])
def test_wall_thresholds_restart_nothing(sup, tsdf_values, pid, value):
    assert sup.plan(dict(tsdf_values, **{pid: value})) == ([], [], [])


@pytest.mark.parametrize("pid", ["voxel_min_weight", "voxel_min_solid_confidence",
                                 "robot_depth_target"])
def test_thresholds_push_to_both_mappers(tsdf_values, pid):
    """The ground-truth map is scored against the belief map, so both move."""
    assert (model.mapper_live_targets(pid, tsdf_values)
            == [("tsdf_mapper", model.LIVE_MAPPER[pid]),
                ("tsdf_mapper_gt", model.LIVE_MAPPER[pid])])


def test_no_ground_truth_mapper_without_slam(tsdf_values):
    targets = model.mapper_live_targets("voxel_min_weight",
                                        dict(tsdf_values, slam="none"))
    assert targets == [("tsdf_mapper", "voxel_min_weight")]


def test_depth_is_live_under_tsdf_and_launch_time_under_octomap(sup, tsdf_values):
    octomap = dict(tsdf_values, mapper="octomap")
    assert model.mapper_live_targets("robot_depth_target", tsdf_values)
    assert not model.mapper_live_targets("robot_depth_target", octomap)
    # The band is an octomap_server launch parameter, so that backend must bounce.
    for gid in RUNNING:
        sup.applied[gid] = dict(octomap)
    assert "mapper" in sup.plan(dict(octomap, robot_depth_target=4.0))[2]


def test_depth_keeps_the_tsdf_map(sup, tsdf_values):
    assert sup.plan(dict(tsdf_values, robot_depth_target=4.0))[2] == ["planner"]


def test_voxel_size_still_rebuilds_the_grid(sup, tsdf_values):
    assert sup.plan(dict(tsdf_values, voxel_size=0.4))[2] == ["mapper", "gt_map"]


def test_unapplied_marks_a_value_the_stack_has_not_taken(tsdf_values):
    assert ui.unapplied_ids(tsdf_values, dict(tsdf_values, mode="goto")) == {"mode"}


def test_live_values_are_never_unapplied(tsdf_values):
    """They reach their node on the keypress, so there is nothing to apply."""
    edited = dict(tsdf_values, voxel_min_weight=42.0, wall_standoff=2.5)
    assert ui.unapplied_ids(tsdf_values, edited) == set()


def test_a_stopped_stack_has_nothing_unapplied(tsdf_values):
    assert ui.unapplied_ids({}, dict(tsdf_values, mode="goto")) == set()


def test_hidden_options_are_not_marked(tsdf_values):
    """A row the current mode does not show cannot explain its own colour."""
    teleop = dict(tsdf_values, mode="teleop", scan_style="spin")
    assert "scan_style" not in ui.unapplied_ids(tsdf_values, teleop)
