"""Restarting the SLAM group mid-run must not re-anchor the world frame.

The SLAM stack dead-reckons X/Y from wherever it is told to start, so a restart
seeded at the spawn pose reports an instantaneous position error equal to the
distance driven. These tests pin the seed, the cascade that keeps the map in the
same frame as the pose, and the masking that stops a moving vehicle from reading
as a permanently pending restart.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher_core as core          # noqa: E402
import launcher_model as model        # noqa: E402

RUNNING = ["core", "tf", "cloud", "mapper", "gt_map", "slam", "eval", "planner"]
STATEFUL = ["mapper", "gt_map", "slam", "eval", "planner"]


def _groups(tmp_path):
    """The group table, with a share the rviz group can copy its config from."""
    share = tmp_path / "share"
    (share / "rviz").mkdir(parents=True, exist_ok=True)
    (share / "rviz" / "demo.rviz").write_text("Panels: []\n")
    return core.build_groups(bringup_share=str(share))


@pytest.fixture
def restart_groups(tmp_path):
    return {g.id: g for g in _groups(tmp_path)}


@pytest.fixture
def spawn_values():
    """A running SLAM + frontier configuration spawned at the origin."""
    v = dict(model.DEFAULTS)
    v.update({"slam": "slam", "mode": "frontier", "mapper": "tsdf", "rviz": False,
              "robot_x": 0.0, "robot_y": 0.0})
    return v


@pytest.fixture
def driven_values(spawn_values):
    """`spawn_values` after the vehicle has driven ~9 m from spawn.

    launch_values() supplies the seed keys from the live odometry readback while
    leaving robot_x/robot_y at the configured launch pose.
    """
    v = dict(spawn_values)
    v.update({"slam_seed_x": 8.4, "slam_seed_y": -3.1})
    return v


@pytest.fixture
def restart_sup(tmp_path, spawn_values):
    sup = core.Supervisor(str(tmp_path), str(tmp_path / "logs"), _groups(tmp_path))
    sup.running_ids = lambda: list(RUNNING)
    for gid in RUNNING:
        sup.applied[gid] = dict(spawn_values)
    return sup


def _args(argv, *prefixes):
    return [a for a in argv if a.startswith(prefixes)]


def test_live_pose_alone_is_not_a_pending_restart(restart_sup, driven_values):
    """launch_values() masks the odometry readback, so driving stays quiet."""
    assert restart_sup.plan(driven_values)[2] == []


def test_slam_restart_drags_the_stateful_set(restart_sup, driven_values):
    driven_values["loop_closure"] = not driven_values["loop_closure"]
    assert restart_sup.plan(driven_values)[2] == STATEFUL


def test_row_hint_matches_what_apply_does(restart_sup, driven_values):
    driven_values["loop_closure"] = not driven_values["loop_closure"]
    assert (restart_sup.pending_for("loop_closure", driven_values)
            == [(g, "restart") for g in STATEFUL])


def test_non_slam_restart_does_not_cascade(restart_sup, driven_values):
    driven_values["tsdf_octomap"] = not driven_values["tsdf_octomap"]
    assert restart_sup.plan(driven_values)[2] == ["mapper"]


def test_slam_is_seeded_at_the_live_pose(restart_groups, driven_values):
    argv = restart_groups["slam"].commands(driven_values)[0]
    assert _args(argv, "initial_x:", "initial_y:") == ["initial_x:=8.4", "initial_y:=-3.1"]


def test_simulator_spawn_keeps_the_configured_pose(restart_groups, driven_values):
    """Only SLAM follows the vehicle; respawning still uses the launch pose."""
    argv = restart_groups["core"].commands(driven_values)[0]
    assert _args(argv, "robot_x:", "robot_y:") == ["robot_x:=0.0", "robot_y:=0.0"]


def test_seed_falls_back_to_the_launch_pose(restart_groups, spawn_values):
    """Cold start and the reset path pass no seed."""
    argv = restart_groups["slam"].commands(spawn_values)[0]
    assert _args(argv, "initial_x:", "initial_y:") == ["initial_x:=0.0", "initial_y:=0.0"]
