"""The odometry readback must not outlive the vehicle it describes.

RosLink caches /StoneFish/Odometry so the TUI can show ground truth and seed a
mid-run SLAM restart at the live pose. Stopping the stack leaves that cache
holding the pose the run ended on, while the next launch respawns the vehicle
at the configured spawn pose — dead reckoning then starts metres away from the
robot and reports that gap as instantaneous position error. These pin the seed
falling back to the launch pose whenever there is nothing live to read.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher as ui                 # noqa: E402
import launcher_core as core          # noqa: E402
import launcher_model as model        # noqa: E402

SPAWN = {"robot_x": 0.0, "robot_y": 0.0, "robot_z": -2.0,
         "robot_roll": 0.0, "robot_pitch": 0.0, "robot_yaw": 0.0}
DRIVEN = {"robot_x": 8.4, "robot_y": -3.1, "robot_z": -2.0,
          "robot_roll": 0.0, "robot_pitch": 0.0, "robot_yaw": 90.0}


@pytest.fixture
def link():
    link = ui.RosLink()
    link.robot_pose = dict(DRIVEN)
    link.slam_pose = dict(DRIVEN)
    return link


def test_forget_pose_drops_both_readbacks(link):
    link.forget_pose()
    assert link.robot_pose is None
    assert link.slam_pose is None


def test_seed_follows_a_live_vehicle(link):
    assert ui.slam_seed(link.robot_pose, SPAWN) == {"slam_seed_x": 8.4,
                                                    "slam_seed_y": -3.1}


def test_seed_returns_to_spawn_once_the_vehicle_is_gone(link):
    link.forget_pose()
    assert ui.slam_seed(link.robot_pose, SPAWN) == {"slam_seed_x": 0.0,
                                                    "slam_seed_y": 0.0}


def test_stopped_stack_relaunches_slam_at_the_spawn_pose(tmp_path, link):
    """The end-to-end shape of the bug: stop after driving, launch again."""
    share = tmp_path / "share"
    (share / "rviz").mkdir(parents=True, exist_ok=True)
    (share / "rviz" / "demo.rviz").write_text("Panels: []\n")
    groups = {g.id: g for g in core.build_groups(bringup_share=str(share))}

    values = dict(model.DEFAULTS)
    values.update({"slam": "slam", "mode": "frontier", "mapper": "tsdf",
                   "rviz": False})
    # The robot_* options are the spawn pose and never follow the vehicle; it is
    # RosLink's cached odometry that tracks it, and only slam_seed reads that.
    values.update(DRIVEN)
    link.forget_pose()                      # K: the stack, and the vehicle, stop
    values.update(SPAWN)
    values.update(ui.slam_seed(link.robot_pose, SPAWN))

    argv = groups["slam"].commands(values)[0]
    seed = [a for a in argv if a.startswith(("initial_x:", "initial_y:"))]
    assert seed == ["initial_x:=0.0", "initial_y:=0.0"]
    spawn = [a for a in groups["core"].commands(values)[0]
             if a.startswith(("robot_x:", "robot_y:"))]
    assert spawn == ["robot_x:=0.0", "robot_y:=0.0"]
