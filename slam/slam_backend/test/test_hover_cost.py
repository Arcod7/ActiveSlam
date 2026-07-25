"""End-to-end cost check: a hover must not make per-keyframe work grow.

Drives the real PoseGraphNode (real iSAM2, real registration) rather than a
stub, because the growth being fixed came from the interaction of candidate
selection, re-detection and the graph itself. The vehicle is held at one spot,
which is the reported failure case: every keyframe is inside
loop_closure_radius_m of every other, so closure candidacy degenerates to
all-pairs unless it is explicitly bounded.
"""
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter

from slam_backend.pose_graph import PoseGraphNode

HOVER_KEYFRAMES = 40


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    params = [Parameter(k, value=v) for k, v in overrides.items()]
    return PoseGraphNode(parameter_overrides=params)


def scene_cloud(n=600, seed=0):
    """A fixed structured scene, so registrations have something to lock onto."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-3.0, 3.0, size=(n, 3))
    pts[:, 2] = np.sin(pts[:, 0]) + 0.5 * np.cos(pts[:, 1] * 1.7)
    return pts


def hover_pose(step):
    """Station-keeping with the small drift that keeps tripping the gates."""
    T = np.eye(4)
    T[:3, 3] = [0.02 * np.sin(step), 0.02 * np.cos(step), 0.0]
    yaw = 0.01 * step
    c, s = np.cos(yaw), np.sin(yaw)
    T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return T


def run_hover(node, n=HOVER_KEYFRAMES):
    """Feed n keyframes at one spot; returns the per-keyframe ICP call counts."""
    counts = []
    stamp = node.get_clock().now().to_msg()
    for step in range(n):
        node._add_keyframe(hover_pose(step), scene_cloud(seed=step % 3), stamp)
        counts.append(node._icp_calls)
    return counts


def test_hover_icp_cost_stays_flat():
    """The defect: registrations per keyframe grew with the graph. The last
    keyframe of a hover must cost no more than the first few."""
    node = make_node(loop_closure_min_gap=5, loop_closure_max_candidates=4)
    try:
        counts = run_hover(node)
    finally:
        node.destroy_node()

    early = max(counts[5:10])
    late = max(counts[-10:])
    assert late <= early + 1, f"per-keyframe ICP grew: early={early} late={late}"


def test_hover_icp_cost_is_bounded_by_the_candidate_cap():
    """One sequential match plus at most loop_closure_max_candidates closures."""
    node = make_node(loop_closure_min_gap=5, loop_closure_max_candidates=4)
    try:
        counts = run_hover(node)
    finally:
        node.destroy_node()

    assert max(counts) <= 5, f"cap exceeded: {max(counts)}"


def test_raising_the_cap_raises_the_cost():
    """Guards against the bound coming from something other than the cap —
    if candidates were being starved upstream, this would not move."""
    tight = make_node(loop_closure_min_gap=5, loop_closure_max_candidates=2)
    loose = make_node(loop_closure_min_gap=5, loop_closure_max_candidates=12,
                      loop_closure_cluster_radius_m=0.0)
    try:
        tight_counts = run_hover(tight, n=25)
        loose_counts = run_hover(loose, n=25)
    finally:
        tight.destroy_node()
        loose.destroy_node()

    assert max(tight_counts) < max(loose_counts)


def test_hover_keyframes_saturate():
    """The density cap must stop a hover growing the graph at all, once the
    vehicle's own gate is consulted the way _cloud_in_cb consults it."""
    node = make_node(keyframe_max_per_cell=3, keyframe_cell_radius_m=0.5,
                     keyframe_cell_angle_rad=0.5, keyframe_dist_m=0.001,
                     keyframe_angle_rad=0.001)
    try:
        stamp = node.get_clock().now().to_msg()
        node._add_keyframe(hover_pose(0), scene_cloud(), stamp)
        admitted = sum(1 for step in range(1, 30)
                       if node._should_create_keyframe(hover_pose(step))
                       and not node._add_keyframe(hover_pose(step), scene_cloud(), stamp))
    finally:
        node.destroy_node()

    assert admitted <= 3, f"hover admitted {admitted} extra keyframes"
