"""goto is frontier with the operator picking the goal, so switching between the
two must not disturb the running stack.

Both modes launch the same planner layer and the same mapper command; they differ
only in who publishes /frontier_slam/goal, which is a runtime topic. These tests
pin that: no restart of any group (the map survives), the planning dashboard
stays up, and the mode a teleop switch does change still bounces what it must.
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


def test_teleop_switch_still_bounces_the_mapper(sup, frontier_values):
    """The mapper reads mode as planner-vs-teleop: that distinction still moves."""
    teleop = dict(frontier_values, mode="teleop")
    assert sup.plan(teleop)[2] == ["mapper"]
    assert ("mapper", "restart") in sup.pending_for("mode", teleop)


def test_teleop_switch_stops_the_planner_layer(sup, frontier_values):
    teleop = dict(frontier_values, mode="teleop")
    assert sup.plan(teleop)[0] == ["planner", "rqt"]


def test_applied_values_follows_the_last_apply(sup, frontier_values, monkeypatch):
    """goto is read from here, so it must track apply() even with nothing to do."""
    monkeypatch.setattr(sup, "start", lambda *a, **k: None)
    monkeypatch.setattr(sup, "stop_many", lambda *a, **k: None)
    sup.apply(_goto(frontier_values))
    assert sup.applied_values["mode"] == "goto"
