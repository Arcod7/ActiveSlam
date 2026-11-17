"""Every control the TUI offers must reach a node.

A parameter that can be set but changes no launch command reads as configured
while the node quietly keeps its own default. That is how `sigma_allow_xy_m`
ran at the launch default of 0.045 against a configured 0.3 -- a 6.7x tighter
revisit threshold -- and how `wall_z_band_m` pinned the vehicle to a 1.4 m
depth slice. Both were in `launcher_model.LIVE`, which only pushes a value when
it CHANGES, so nothing applied them at startup.

If one of these fails, either pass the parameter from its group in
`build_groups()` or add it to `launcher_core.WIRING_EXEMPT` with the reason it
cannot reach a node. Do not delete the case.
"""
import pytest

import launcher_core
import launcher_model


def _values(**overrides):
    values = {p.id: p.default for p in launcher_model.PARAMS}
    values.update(overrides)
    return values


CONFIGURATIONS = {
    "sim slam + frontier + wall-oriented": dict(
        slam="slam", mode="frontier", revisit=True, loop_closure=True,
        mapper="tsdf_directional", motion="walloriented", scene="target"),
    "sim slam + octomap + wall-looking": dict(
        slam="slam", mode="frontier", revisit=True, loop_closure=True,
        mapper="octomap", motion="walllooking", scene="target"),
    "ground truth, no slam": dict(
        slam="none", mode="teleop", revisit=False, loop_closure=False,
        mapper="tsdf_directional", scene="target"),
    "revisit disabled": dict(
        slam="slam", mode="frontier", revisit=False, loop_closure=True,
        mapper="tsdf_directional", motion="walloriented"),
    "real vehicle": dict(
        slam="slam", mode="frontier", revisit=True, real_sonar=True,
        mapper="tsdf_directional", motion="walloriented"),
}


@pytest.mark.parametrize("name", sorted(CONFIGURATIONS))
def test_every_visible_control_reaches_a_node(name):
    report = launcher_core.wiring_report(
        _values(**CONFIGURATIONS[name]), launcher_model.PARAMS)
    assert report is None, f"\n[{name}]\n{report}"


def test_the_guard_actually_catches_an_unwired_parameter():
    """A guard that cannot fail is not a guard. Hide a parameter's argument
    from the planner group and confirm the check notices."""
    values = _values(**CONFIGURATIONS["sim slam + frontier + wall-oriented"])
    real_build = launcher_core.build_groups

    def crippled(bringup_share=""):
        groups = real_build(bringup_share)
        for group in groups:
            if group.id != "planner":
                continue
            inner = group.build
            group.build = lambda v, _i=inner: [
                [tok for tok in argv if not tok.startswith("sigma_allow_xy_m:=")]
                for argv in _i(v)]
        return groups

    launcher_core.build_groups = crippled
    try:
        report = launcher_core.wiring_report(values, launcher_model.PARAMS)
    finally:
        launcher_core.build_groups = real_build
    assert report is not None and "sigma_allow_xy_m" in report


def test_exempt_entries_all_carry_a_reason():
    for pid, reason in launcher_core.WIRING_EXEMPT.items():
        assert isinstance(reason, str) and len(reason) > 15, pid


def test_exempt_entries_still_exist_as_parameters():
    """Stops the exemption list rotting into a list of deleted names."""
    known = {p.id for p in launcher_model.PARAMS}
    unknown = sorted(set(launcher_core.WIRING_EXEMPT) - known)
    assert not unknown, f"exempt but no longer a parameter: {unknown}"
