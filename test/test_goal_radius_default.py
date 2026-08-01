"""The TUI's arrival-radius default must match the package constant.

`launcher_model` is pure standard library by design, so it cannot import
`frontier_slam.mission_params` and its default is necessarily a copy. This is
what stops the copy drifting: a TUI default below the executor's own radius
recreates the deadlock the single value was introduced to remove.
"""
from frontier_slam.mission_params import GOAL_RADIUS_M
import launcher_model
import pytest


def test_the_launcher_default_matches_the_package_constant():
    param = next(p for p in launcher_model.PARAMS if p.id == "goal_radius_m")
    assert param.default == pytest.approx(GOAL_RADIUS_M)
