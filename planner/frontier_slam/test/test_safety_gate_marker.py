"""Label, colour and anchoring rules for the vehicle marker the motion safety
gate publishes — the arrow in RViz and the state row of the eval HUD panel."""

from types import SimpleNamespace

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from frontier_slam.safety_gate import (
    DISABLED_COLOR, ENABLED_COLOR, INIT_SCAN_COLOR, MotionSafetyGate,
    REVISIT_COLOR,
)


def _gate(enabled=True, revisit_state=None, revisit_age_s=0.0,
          revisit_cause=None, cause_age_s=0.0,
          activity=None, activity_age_s=0.0):
    """A stand-in with just the fields _marker_state reads — no ROS node."""
    gate = SimpleNamespace(
        _state=SimpleNamespace(enabled=enabled),
        _revisit_state=revisit_state,
        _revisit_state_time=100.0 - revisit_age_s,
        _revisit_cause=revisit_cause,
        _revisit_cause_time=100.0 - cause_age_s,
        _activity=activity,
        _activity_time=100.0 - activity_age_s,
        _marker_state_timeout_s=3.0,
        _now=lambda: 100.0,
    )
    gate._fresh = lambda *a: MotionSafetyGate._fresh(gate, *a)
    return gate


def _state(**kwargs):
    return MotionSafetyGate._marker_state(_gate(**kwargs))


def test_enabled_without_planner_state_is_green():
    assert _state() == ('MOTION ENABLED', ENABLED_COLOR)


def test_revisiting_is_cyan():
    assert _state(revisit_state='revisiting') == ('REVISITING', REVISIT_COLOR)


def test_revisiting_names_the_axis_that_drove_the_trigger():
    assert _state(revisit_state='revisiting', revisit_cause='position') == (
        'REVISITING — POSITION DRIFT', REVISIT_COLOR)
    assert _state(revisit_state='revisiting', revisit_cause='heading') == (
        'REVISITING — HEADING DRIFT', REVISIT_COLOR)


def test_revisiting_without_a_usable_cause_stays_unqualified():
    # Empty string is what the planner publishes outside a revisit, and an
    # unknown value must not reach the HUD raw.
    assert _state(revisit_state='revisiting', revisit_cause='') == (
        'REVISITING', REVISIT_COLOR)
    assert _state(revisit_state='revisiting', revisit_cause='something_new') == (
        'REVISITING', REVISIT_COLOR)
    assert _state(revisit_state='revisiting', revisit_cause='position',
                  cause_age_s=4.0) == ('REVISITING', REVISIT_COLOR)


def test_initial_scan_is_white():
    assert _state(activity='INIT_SCAN') == ('INITIAL SCAN', INIT_SCAN_COLOR)


def test_other_activities_stay_green_with_their_own_label():
    assert _state(activity='FOLLOW_PATH') == ('DRIVING TO WAYPOINT', ENABLED_COLOR)
    assert _state(revisit_state='cooldown', activity='SCAN') == (
        'SCANNING FOR FRONTIERS', ENABLED_COLOR)


def test_unmapped_activity_falls_back_to_its_own_name():
    assert _state(activity='SOME_NEW_MODE') == ('SOME NEW MODE', ENABLED_COLOR)


def test_stale_states_fall_back_to_plain_enabled():
    assert _state(revisit_state='revisiting', revisit_age_s=4.0) == (
        'MOTION ENABLED', ENABLED_COLOR)
    assert _state(activity='INIT_SCAN', activity_age_s=4.0) == (
        'MOTION ENABLED', ENABLED_COLOR)


def test_revisit_outranks_initial_scan():
    assert _state(revisit_state='revisiting', activity='INIT_SCAN') == (
        'REVISITING', REVISIT_COLOR)


def test_disabled_gate_stays_purple_whatever_the_planner_says():
    assert _state(enabled=False, revisit_state='revisiting') == (
        'MOTION DISABLED', DISABLED_COLOR)
    assert _state(enabled=False, activity='INIT_SCAN') == (
        'MOTION DISABLED', DISABLED_COLOR)


# ---------------------------------------------------------------- anchoring

def _odom(x=0.0, y=0.0, z=0.0, frame='world_ned', child='odom'):
    msg = Odometry()
    msg.header.frame_id = frame
    msg.child_frame_id = child
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    return msg


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def _marker_gate(marker_frame='world_ned'):
    """A stand-in with the fields the marker publishers read."""
    gate = _gate()
    gate._marker_frame = marker_frame
    gate._body_fixed_marker = False
    gate._last_pose = None
    gate._last_truth_pose = None
    gate._marker_pub = _Pub()
    gate._truth_marker_pub = _Pub()
    gate._state.update_odometry = lambda _t: None
    gate.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: None))
    for name in ('_odom_cb', '_truth_odom_cb', '_publish_robot_marker',
                 '_arrow_marker', '_marker_state'):
        setattr(gate, name, getattr(MotionSafetyGate, name).__get__(gate))
    return gate


def test_matching_child_frame_makes_the_marker_body_fixed():
    gate = _marker_gate('bluerov2/base_link')
    gate._odom_cb(_odom(child='bluerov2/base_link'))
    assert gate._body_fixed_marker
    # TF treats a leading slash as the same frame, so the comparison must too.
    gate._odom_cb(_odom(child='/bluerov2/base_link'))
    assert gate._body_fixed_marker


def test_world_anchored_marker_is_not_body_fixed():
    gate = _marker_gate('world_ned')
    gate._odom_cb(_odom(child='bluerov2/base_link'))
    assert not gate._body_fixed_marker


def test_body_fixed_arrow_sits_at_the_origin_and_lets_tf_place_it():
    gate = _marker_gate('bluerov2/base_link')
    gate._odom_cb(_odom(x=7.0, y=-3.0, z=2.0, child='bluerov2/base_link'))
    gate._publish_robot_marker()
    arrow, text = gate._marker_pub.sent
    assert arrow.header.frame_id == 'bluerov2/base_link'
    assert (arrow.pose.position.x, arrow.pose.position.y,
            arrow.pose.position.z) == (0.0, 0.0, 0.0)
    # The label still floats above the vehicle, not above the world origin.
    assert (text.pose.position.x, text.pose.position.y,
            text.pose.position.z) == (0.0, 0.0, -1.2)


def test_world_anchored_arrow_still_carries_the_watched_pose():
    gate = _marker_gate('world_ned')
    gate._odom_cb(_odom(x=7.0, y=-3.0, z=2.0))
    gate._publish_robot_marker()
    arrow, text = gate._marker_pub.sent
    assert arrow.header.frame_id == 'world_ned'
    assert (arrow.pose.position.x, arrow.pose.position.y,
            arrow.pose.position.z) == (7.0, -3.0, 2.0)
    assert text.pose.position.z == 2.0 - 1.2


def test_truth_arrow_is_drawn_in_the_frame_its_pose_is_expressed_in():
    # A body-fixed belief arrow must not drag truth into the body frame, or the
    # world pose would be transformed a second time and the drift gap would lie.
    gate = _marker_gate('bluerov2/base_link')
    gate._truth_odom_cb(_odom(x=7.0, frame='world_ned'))
    truth, = gate._truth_marker_pub.sent
    assert truth.header.frame_id == 'world_ned'
    assert truth.pose.position.x == 7.0


def test_truth_arrow_falls_back_to_the_marker_frame_when_unstamped():
    gate = _marker_gate('world_ned')
    gate._truth_odom_cb(_odom(frame=''))
    truth, = gate._truth_marker_pub.sent
    assert truth.header.frame_id == 'world_ned'
    assert truth.type == Marker.ARROW


def test_pose_default_is_the_identity_the_body_fixed_arrow_relies_on():
    assert Pose().orientation.w == 1.0
