"""goto is frontier with the operator picking the goal, so switching between the
two must not disturb the running stack.

Both modes launch the same planner layer and the same mapper command; they differ
only in who publishes /frontier_slam/goal, which is a runtime topic. These tests
pin that: no restart of any group (the map survives) and the planning dashboard
stays up. Teleop is held to the same rule for everything but the planner layer
itself — the map and the pose graph are what a run accumulates, and changing who
picks the goal is no reason to discard either.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher_core as core          # noqa: E402
import launcher_model as model        # noqa: E402

PLANNER_RUNNING = ["core", "tf", "cloud", "mapper", "planner", "rqt"]


def _groups(tmp_path):
    share = tmp_path / "share"
    (share / "rviz").mkdir(parents=True, exist_ok=True)
    (share / "rviz" / "demo.rviz").write_text("Panels: []\n")
    return core.build_groups(bringup_share=str(share))


@pytest.fixture
def groups(tmp_path):
    return {g.id: g for g in _groups(tmp_path)}


@pytest.fixture
def frontier_values():
    v = dict(model.DEFAULTS)
    v.update({"mode": "frontier", "mapper": "tsdf", "rviz": False, "rqt": True})
    return v


@pytest.fixture
def slam_values(frontier_values):
    return dict(frontier_values, slam="slam")


@pytest.fixture
def sup(tmp_path, frontier_values):
    s = core.Supervisor(str(tmp_path), str(tmp_path / "logs"), _groups(tmp_path))
    s.running_ids = lambda: list(PLANNER_RUNNING)
    for gid in PLANNER_RUNNING:
        s.applied[gid] = dict(frontier_values)
    s.applied_values = dict(frontier_values)
    return s


def _goto(values):
    return dict(values, mode="goto")


def test_goto_switch_restarts_nothing(sup, frontier_values):
    assert sup.plan(_goto(frontier_values)) == ([], [], [])


def test_goto_switch_is_flagged_as_no_work_on_the_mode_row(sup, frontier_values):
    assert sup.pending_for("mode", _goto(frontier_values)) == []


def test_dashboard_stays_up_under_goto(groups, frontier_values):
    assert groups["rqt"].visible(_goto(frontier_values))


def test_goto_builds_the_frontier_commands(groups, frontier_values):
    for gid in ("mapper", "planner"):
        assert (groups[gid].commands(_goto(frontier_values))
                == groups[gid].commands(frontier_values))


def test_no_mode_switch_touches_the_mapper(sup, groups, frontier_values):
    """The map survives every mode change — it is the expensive state here.

    The mapper used to read mode as planner-vs-teleop, for a /projected_map it
    only published under a planner, and paid for that with a restart that threw
    the map away on the way into teleop. It publishes in every mode now.
    """
    for mode in ("teleop", "goto"):
        switched = dict(frontier_values, mode=mode)
        assert sup.plan(switched)[2] == [], f"{mode} restarted a group"
        # The planner layer starting and stopping with the mode is the point of
        # the mode; nothing may be restarted in place, and least of all the map.
        pending = sup.pending_for("mode", switched)
        assert [gid for gid, act in pending if act == "restart"] == [], \
            f"the mode row promises a restart on {mode}"
        assert "mapper" not in [gid for gid, _ in pending], \
            f"{mode} disturbs the mapper"
        assert (groups["mapper"].commands(switched)
                == groups["mapper"].commands(frontier_values)), \
            f"{mode} changed the mapper's command line"


def test_the_pose_graph_survives_a_mode_switch(sup, groups, slam_values):
    """Keyframes are kept through teleop and goto, not just through frontier."""
    for mode in ("teleop", "goto"):
        switched = dict(slam_values, mode=mode)
        assert groups["slam"].visible(switched), f"slam stopped under {mode}"
        assert not groups["slam"].changed(slam_values, switched), \
            f"{mode} restarted the pose graph"


def test_teleop_switch_stops_the_planner_layer(sup, frontier_values):
    teleop = dict(frontier_values, mode="teleop")
    assert sup.plan(teleop)[0] == ["planner", "rqt"]


def test_applied_values_follows_the_last_apply(sup, frontier_values, monkeypatch):
    """goto is read from here, so it must track apply() even with nothing to do."""
    monkeypatch.setattr(sup, "start", lambda *a, **k: None)
    monkeypatch.setattr(sup, "stop_many", lambda *a, **k: None)
    sup.apply(_goto(frontier_values))
    assert sup.applied_values["mode"] == "goto"
