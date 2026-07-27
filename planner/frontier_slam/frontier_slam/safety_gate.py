"""Fail-closed ROS 2 gate for normalized vehicle body commands.

Also publishes the vehicle on /motion/robot_marker as a state-coloured arrow
(green = motion enabled, cyan = enabled and revisiting, white = enabled and
running the initial scan, purple = disabled) plus a matching text label the
RViz eval HUD renders — the gate is the only node that knows both the pose it
watches and whether motion is armed. A revisit is labelled with the axis that
drove it, so the HUD answers "why" and not only "what".

Two arrows on two topics, so RViz can show either alone:
  /motion/robot_marker     the pose the gate watches — the SLAM estimate under
                           pose source slam — faded, at publish_hz, with the
                           text label.
  /motion/robot_marker_gt  ground truth, opaque, republished on every truth
                           odometry message rather than on the safety tick.
The gap between the two is the live drift the vehicle is being steered on. Under
pose source none the watched pose already is truth, so only the first is
published, opaque.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker

from frontier_slam.safety_logic import MotionSafetyState

# Green when the gate passes motion, purple when it is disabled — the same
# reading as the launcher's target-point sphere.
ENABLED_COLOR = ColorRGBA(r=0.16, g=0.86, b=0.16, a=0.95)
DISABLED_COLOR = ColorRGBA(r=0.70, g=0.20, b=0.90, a=0.95)
# Cyan while the revisit planner drives the vehicle back to a known keyframe,
# white while a motion executor is running its initial scan.
REVISIT_COLOR = ColorRGBA(r=0.00, g=0.80, b=0.90, a=0.95)
INIT_SCAN_COLOR = ColorRGBA(r=1.00, g=1.00, b=1.00, a=0.95)
REVISITING_STATE = 'revisiting'
INIT_SCAN_ACTIVITY = 'INIT_SCAN'
# Which axis of the pose marginal drove the D-optimality that fired the
# revisit — revisit_planner.py's CAUSE_* strings. The trigger is the combined
# scalar, so this is attribution, not a second threshold. Keep in step with the
# launcher's REVISIT_CAUSE_TEXT, which renders the same strings as a sentence.
REVISIT_CAUSE_LABELS = {
    'position': 'POSITION DRIFT',
    'heading': 'HEADING DRIFT',
}
# The belief arrow is the same vehicle in the same state, drawn where the gate
# thinks it is — faint so it reads as a second opinion, not a second robot.
BELIEF_ALPHA = 0.50

# HUD wording for the labels the motion executors publish on
# /frontier_slam/activity — keep in step with the launcher's ACTIVITY_TEXT.
ACTIVITY_LABELS = {
    'INIT_SCAN': 'INITIAL SCAN',
    'SCAN': 'SCANNING FOR FRONTIERS',
    'GOAL_REACHED': 'AT GOAL, SCANNING',
    'FOLLOW_PATH': 'DRIVING TO WAYPOINT',
    'TRACK': 'FOLLOWING WALL',
    'WALL_SWITCH_SCAN': 'REACQUIRING WALL',
    'NO_WALL_SCAN': 'SCANNING, NO WALL IN VIEW',
    'NO_PATH_PROGRESS': 'STALLED, WAITING FOR REPLAN',
    'EMERG_STOP': 'OBSTACLE AHEAD, BACKING OFF',
    'CTRL_STUCK': 'STUCK, SPINNING TO ESCAPE',
    'CTRL_STUCK_ESCAPE': 'STUCK, SPINNING TO ESCAPE',
    'ODOM_STALE': 'ODOMETRY STALE, HOLDING',
}


class MotionSafetyGate(Node):
    """Pass body demand only when explicitly enabled with fresh inputs."""

    def __init__(self) -> None:
        super().__init__('motion_safety_gate')
        self.declare_parameter('command_topic', '/motion/body_command')
        self.declare_parameter('output_topic', '/motion/body_command_safe')
        self.declare_parameter('enable_topic', '/motion/enable')
        self.declare_parameter('status_topic', '/motion/safety_status')
        self.declare_parameter('odom_topic', '/StoneFish/Odometry')
        # Visualisation only — never feeds the gate's freshness check, or a live
        # ground truth would hold the gate open through a dead estimator. Empty
        # on a real vehicle, where there is no truth to draw.
        self.declare_parameter('truth_odom_topic', '/StoneFish/Odometry')
        # Its own topic, so RViz can toggle the two arrows separately and so the
        # ground-truth one is not pinned to this node's 20 Hz safety tick.
        self.declare_parameter('truth_marker_topic', '/motion/robot_marker_gt')
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('odom_timeout_s', 0.5)
        self.declare_parameter('max_abs_command', 1.0)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('marker_topic', '/motion/robot_marker')
        self.declare_parameter('marker_frame', 'world_ned')
        self.declare_parameter('revisit_state_topic',
                               '/frontier_slam/revisit_state')
        self.declare_parameter('revisit_cause_topic',
                               '/frontier_slam/revisit_cause')
        self.declare_parameter('activity_topic', '/frontier_slam/activity')
        # Both sources publish at 1 Hz; fall back to the plain enabled colour
        # when either stops, rather than latching its state forever.
        self.declare_parameter('marker_state_timeout_s', 3.0)

        command_topic = str(self.get_parameter('command_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        enable_topic = str(self.get_parameter('enable_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be positive')

        self._state = MotionSafetyState(
            command_timeout_s=float(
                self.get_parameter('command_timeout_s').value),
            odom_timeout_s=float(self.get_parameter('odom_timeout_s').value),
            max_abs_command=float(
                self.get_parameter('max_abs_command').value),
            require_odom=bool(self.get_parameter('require_odom').value),
        )
        self._state.set_enabled(
            bool(self.get_parameter('start_enabled').value))
        self._command_topic = command_topic
        self._last_status: str | None = None
        self._marker_frame = str(self.get_parameter('marker_frame').value)
        self._marker_state_timeout_s = float(
            self.get_parameter('marker_state_timeout_s').value)
        self._last_pose = None
        self._last_truth_pose = None
        self._revisit_state: str | None = None
        self._revisit_state_time = 0.0
        self._revisit_cause: str | None = None
        self._revisit_cause_time = 0.0
        self._activity: str | None = None
        self._activity_time = 0.0

        self._output_pub = self.create_publisher(Twist, output_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        # Depth 10, not 1: each tick writes three markers back to back, and a
        # one-deep queue drops some of them — the arrow then steps behind truth.
        self._marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('marker_topic').value), 10)
        self.create_subscription(
            Twist, command_topic, self._command_cb, 10)
        self.create_subscription(Bool, enable_topic, self._enable_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        truth_topic = str(self.get_parameter('truth_odom_topic').value)
        # Same topic under pose source none: the watched pose already is the
        # truth, so there is nothing to draw twice.
        self._truth_is_distinct = bool(truth_topic) and truth_topic != odom_topic
        self._truth_marker_pub = None
        if self._truth_is_distinct:
            self._truth_marker_pub = self.create_publisher(
                Marker, str(self.get_parameter('truth_marker_topic').value), 10)
            self.create_subscription(
                Odometry, truth_topic, self._truth_odom_cb, 10)
        self.create_subscription(
            String, str(self.get_parameter('revisit_state_topic').value),
            self._revisit_state_cb, 10)
        self.create_subscription(
            String, str(self.get_parameter('revisit_cause_topic').value),
            self._revisit_cause_cb, 10)
        self.create_subscription(
            String, str(self.get_parameter('activity_topic').value),
            self._activity_cb, 10)
        self.create_timer(1.0 / publish_hz, self._tick)
        self.publish_zero()

        self.get_logger().info(
            f'motion safety gate ready — command={command_topic} '
            f'output={output_topic} enable={enable_topic} odom={odom_topic} '
            f'start_enabled={self._state.enabled}')

    def _now(self) -> float:
        # Safety timeouts must still expire if ROS simulation time pauses.
        return time.monotonic()

    @staticmethod
    def _values(msg: Twist) -> tuple[float, ...]:
        return (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )

    @staticmethod
    def _message(values) -> Twist:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = values[:3]
        msg.angular.x, msg.angular.y, msg.angular.z = values[3:]
        return msg

    def _command_cb(self, msg: Twist) -> None:
        if not self._state.update_command(self._values(msg), self._now()):
            self.get_logger().error(
                'Rejected body command: expected finite values within '
                f'[-{self._state.max_abs_command:.1f}, '
                f'{self._state.max_abs_command:.1f}]')

    def _enable_cb(self, msg: Bool) -> None:
        was_enabled = self._state.enabled
        self._state.set_enabled(msg.data)
        if self._state.enabled != was_enabled:
            action = 'ENABLED' if self._state.enabled else 'DISABLED'
            self.get_logger().warn(f'motion gate {action}')
        if not self._state.enabled:
            self.publish_zero()

    def _odom_cb(self, msg: Odometry) -> None:
        self._state.update_odometry(self._now())
        self._last_pose = msg.pose.pose

    def _truth_odom_cb(self, msg: Odometry) -> None:
        """Draw ground truth as it arrives, not on the safety tick.

        Ground truth comes in at the simulator's rate — several times the gate's
        publish_hz — and sampling it at 20 Hz is visible as the arrow stepping
        while the vehicle slides on smoothly.
        """
        self._last_truth_pose = msg.pose.pose
        if self._truth_marker_pub is None:
            return
        _label, color = self._marker_state()
        self._truth_marker_pub.publish(self._arrow_marker(
            'motion_state_gt', msg.pose.pose, color,
            self.get_clock().now().to_msg()))

    def _revisit_state_cb(self, msg: String) -> None:
        self._revisit_state = msg.data
        self._revisit_state_time = self._now()

    def _revisit_cause_cb(self, msg: String) -> None:
        self._revisit_cause = msg.data
        self._revisit_cause_time = self._now()

    def _activity_cb(self, msg: String) -> None:
        self._activity = msg.data
        self._activity_time = self._now()

    def _fresh(self, value: str | None, stamp: float) -> str | None:
        """The value while it is younger than the timeout, else None."""
        if value is None or self._now() - stamp > self._marker_state_timeout_s:
            return None
        return value

    def _marker_state(self) -> tuple[str, ColorRGBA]:
        """HUD label and arrow colour, resolved together so they cannot
        disagree. Gate state first — a disabled gate is the reading that
        matters most; revisit outranks the initial scan, which in practice
        never overlap."""
        if not self._state.enabled:
            return 'MOTION DISABLED', DISABLED_COLOR
        if self._fresh(self._revisit_state,
                       self._revisit_state_time) == REVISITING_STATE:
            cause = REVISIT_CAUSE_LABELS.get(
                self._fresh(self._revisit_cause, self._revisit_cause_time))
            return (f'REVISITING — {cause}' if cause else 'REVISITING',
                    REVISIT_COLOR)
        activity = self._fresh(self._activity, self._activity_time)
        if activity == INIT_SCAN_ACTIVITY:
            return ACTIVITY_LABELS[activity], INIT_SCAN_COLOR
        if activity:
            return (ACTIVITY_LABELS.get(activity, activity.replace('_', ' ')),
                    ENABLED_COLOR)
        return 'MOTION ENABLED', ENABLED_COLOR

    def _publish_robot_marker(self) -> None:
        """The vehicle as an arrow coloured by whether motion is enabled, plus
        the revisit and initial-scan phases, and the same state as a text
        label floating above it.

        An RViz Odometry display carries a fixed colour, so the state has to
        come from the marker: this replaces the ground-truth arrow in
        demo.rviz. This arrow is the pose the gate watches and the controller
        acts on — the SLAM estimate under pose source slam, ground truth under
        none. Ground truth gets its own topic and its own arrow (see
        _truth_odom_cb); this one fades to BELIEF_ALPHA whenever that arrow is
        up, so the opaque one is always the vehicle's real position. The eval
        HUD panel reads the text marker off this topic, so the panel and the
        arrow always agree.
        """
        if self._last_pose is None:
            return
        label, color = self._marker_state()
        stamp = self.get_clock().now().to_msg()

        # Faded only once truth is actually being drawn: on a real vehicle no
        # ground-truth arrow ever appears, and a permanently faint robot with
        # nothing opaque beside it just looks like a rendering fault.
        arrow_color = color
        if self._last_truth_pose is not None:
            arrow_color = ColorRGBA(r=color.r, g=color.g, b=color.b,
                                    a=BELIEF_ALPHA)
        self._marker_pub.publish(self._arrow_marker(
            'motion_state', self._last_pose, arrow_color, stamp))

        text = Marker()
        text.header.frame_id = self._marker_frame
        text.header.stamp = stamp
        text.ns = 'motion_state_text'
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self._last_pose.position.x
        text.pose.position.y = self._last_pose.position.y
        text.pose.position.z = self._last_pose.position.z - 1.2  # NED: -z is up
        text.pose.orientation.w = 1.0
        text.scale.z = 0.4      # character height
        text.color = color      # full opacity: the label is not the faded arrow
        text.text = label
        self._marker_pub.publish(text)

    def _arrow_marker(self, ns: str, pose, color: ColorRGBA, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._marker_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 1.0    # shaft length, along the vehicle's +X
        marker.scale.y = 0.16   # shaft diameter
        marker.scale.z = 0.16   # head diameter
        marker.color = color
        return marker

    def _has_multiple_command_sources(self) -> bool:
        return len(self.get_publishers_info_by_topic(self._command_topic)) > 1

    def _tick(self) -> None:
        decision = self._state.evaluate(
            self._now(), self._has_multiple_command_sources())
        self._output_pub.publish(self._message(decision.output))
        status = String()
        status.data = decision.state
        self._status_pub.publish(status)
        self._publish_robot_marker()
        if decision.state != self._last_status:
            if decision.state == self._state.ACTIVE:
                self.get_logger().info(
                    f'motion safety state: {decision.state}')
            else:
                self.get_logger().warn(
                    f'motion safety state: {decision.state}')
            self._last_status = decision.state

    def publish_zero(self) -> None:
        self._output_pub.publish(self._message(self._state.zero))


def main(args=None) -> None:
    # Keep the context alive when Ctrl-C arrives so the final zero command can
    # be published before shutdown. The default Python SIGINT handler raises
    # KeyboardInterrupt, which is handled below.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionSafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_zero()
            rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
