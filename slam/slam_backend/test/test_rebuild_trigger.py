"""Pure-logic tests for the map-rebuild trigger's drift measurement.

The rebuild publishes corrections against the pose each keyframe held at the
last re-integration, so the threshold that decides whether to publish must be
measured against that same baseline. Measuring one iSAM update instead makes
the trigger unreachable whenever loop closure runs continuously: the map
drifts metres behind the graph in centimetre steps.

Run: python3 -m pytest slam/slam_backend/test/ -q
"""
import numpy as np
from scipy.spatial.transform import Rotation

from slam_backend.pose_graph import pose_delta, worst_pose_drift


def _T(xyz=(0.0, 0.0, 0.0), yaw_deg=0.0) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('z', yaw_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


def test_delta_is_zero_for_an_unmoved_pose():
    T = _T((1.0, 2.0, 3.0), yaw_deg=30.0)

    dist, angle = pose_delta(T, T)

    assert dist < 1e-9
    assert angle < 1e-9


def test_delta_measures_translation():
    dist, angle = pose_delta(_T((0.0, 0.0, 0.0)), _T((0.3, 0.4, 0.0)))

    assert dist == 0.5
    assert angle < 1e-9


def test_delta_measures_rotation():
    dist, angle = pose_delta(_T(yaw_deg=0.0), _T(yaw_deg=20.0))

    assert dist == 0.0
    assert np.isclose(np.degrees(angle), 20.0)


def test_worst_drift_is_zero_when_nothing_has_been_corrected():
    # A run before its first closure has no baseline entries at all; that must
    # read as no drift rather than raise.
    assert worst_pose_drift([]) == (0.0, 0.0, 0)


def test_worst_drift_takes_the_largest_over_all_keyframes():
    pairs = [(_T((0.0, 0.0, 0.0)), _T((0.05, 0.0, 0.0))),
             (_T((0.0, 0.0, 0.0)), _T((0.90, 0.0, 0.0))),
             (_T((0.0, 0.0, 0.0)), _T((0.20, 0.0, 0.0)))]

    dist, angle, n = worst_pose_drift(pairs)

    assert np.isclose(dist, 0.90)
    assert n == 3


def test_many_small_updates_accumulate_past_a_threshold_one_update_never_reaches():
    # The regression this exists for. Forty 0.025 m corrections is a metre of
    # map staleness; measured per update none of them approaches 0.3 m, which
    # is why rebuild_count stayed 0 across every run of the ablation.
    step = 0.025
    baseline = _T((0.0, 0.0, 0.0))
    per_update = [pose_delta(_T((i * step, 0.0, 0.0)),
                             _T(((i + 1) * step, 0.0, 0.0)))[0]
                  for i in range(40)]
    cumulative, _, _ = worst_pose_drift([(baseline, _T((40 * step, 0.0, 0.0)))])

    assert max(per_update) < 0.3        # no single update ever triggers
    assert cumulative > 0.3             # the accumulated staleness does
    assert np.isclose(cumulative, 1.0)


def test_rotation_alone_can_trigger_when_translation_does_not():
    pairs = [(_T((0.0, 0.0, 0.0), yaw_deg=0.0), _T((0.01, 0.0, 0.0), yaw_deg=12.0))]

    dist, angle, _ = worst_pose_drift(pairs)

    assert dist < 0.3
    assert angle > 0.15                 # rebuild_min_move_rad default
