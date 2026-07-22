"""Fail-closed ROS 2 adapter from gated body demand to ArduSub MAVLink."""

import time

from geometry_msgs.msg import Twist
from pymavlink import mavutil
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from frontier_slam.ardusub_control import (
    ArduSubCommandSender,
    BACKEND_LOCAL_NED_VELOCITY,
    BACKEND_MANUAL_CONTROL,
    BodyDemand,
    ReauthorizationLatch,
    SUPPORTED_BACKENDS,
    adapter_readiness,
    validate_body_command,
)


class ArduSubAdapter(Node):
    """Send MAVLink with fresh gate, command and vehicle heartbeat state."""

    def __init__(self) -> None:
        """Configure the fail-closed adapter and open its MAVLink endpoint."""
        super().__init__('ardusub_adapter')
        self.declare_parameter('backend', BACKEND_MANUAL_CONTROL)
        self.declare_parameter('connection_url', 'udpin:0.0.0.0:14560')
        self.declare_parameter('source_system', 191)
        self.declare_parameter('source_component', 191)
        self.declare_parameter('target_system', 0)
        self.declare_parameter('target_component', 0)
        self.declare_parameter('command_topic', '/motion/body_command_safe')
        self.declare_parameter('gate_status_topic', '/motion/safety_status')
        self.declare_parameter(
            'adapter_status_topic', '/motion/ardusub_status')
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('poll_rate_hz', 100.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('gate_status_timeout_s', 0.5)
        self.declare_parameter('heartbeat_timeout_s', 2.0)
        self.declare_parameter('neutral_burst_count', 5)
        self.declare_parameter('require_armed', True)
        self.declare_parameter('required_mode', '')
        self.declare_parameter('enable_vertical', False)
        self.declare_parameter('manual_authority', 0.15)
        self.declare_parameter('max_surge_mps', 0.15)
        self.declare_parameter('max_sway_mps', 0.15)
        self.declare_parameter('max_vertical_mps', 0.10)
        self.declare_parameter('max_yaw_rate_rps', 0.15)

        self._backend = str(self.get_parameter('backend').value)
        if self._backend not in SUPPORTED_BACKENDS:
            raise ValueError(
                f'backend must be one of {", ".join(SUPPORTED_BACKENDS)}')
        self._connection_url = str(
            self.get_parameter('connection_url').value)
        source_system = int(self.get_parameter('source_system').value)
        source_component = int(self.get_parameter('source_component').value)
        if not 1 <= source_system <= 255 or not 1 <= source_component <= 255:
            raise ValueError('MAVLink source IDs must be within [1, 255]')
        self._configured_target_system = int(
            self.get_parameter('target_system').value)
        self._configured_target_component = int(
            self.get_parameter('target_component').value)
        if (not 0 <= self._configured_target_system <= 255 or
                not 0 <= self._configured_target_component <= 255):
            raise ValueError('MAVLink target IDs must be within [0, 255]')

        send_rate_hz = self._positive_parameter('send_rate_hz')
        poll_rate_hz = self._positive_parameter('poll_rate_hz')
        self._command_timeout_s = self._positive_parameter(
            'command_timeout_s')
        self._gate_status_timeout_s = self._positive_parameter(
            'gate_status_timeout_s')
        self._heartbeat_timeout_s = self._positive_parameter(
            'heartbeat_timeout_s')
        self._neutral_burst_count = int(
            self.get_parameter('neutral_burst_count').value)
        if self._neutral_burst_count <= 0:
            raise ValueError('neutral_burst_count must be positive')
        self._require_armed = bool(
            self.get_parameter('require_armed').value)
        requested_mode = str(
            self.get_parameter('required_mode').value).upper()
        if not requested_mode:
            requested_mode = (
                'GUIDED' if self._backend == BACKEND_LOCAL_NED_VELOCITY
                else 'ALT_HOLD')
        self._required_mode = requested_mode

        command_topic = str(self.get_parameter('command_topic').value)
        gate_status_topic = str(
            self.get_parameter('gate_status_topic').value)
        adapter_status_topic = str(
            self.get_parameter('adapter_status_topic').value)

        self._link = mavutil.mavlink_connection(
            self._connection_url,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )
        self._sender = ArduSubCommandSender(
            self._link,
            backend=self._backend,
            manual_authority=float(
                self.get_parameter('manual_authority').value),
            max_surge_mps=float(self.get_parameter('max_surge_mps').value),
            max_sway_mps=float(self.get_parameter('max_sway_mps').value),
            max_vertical_mps=float(
                self.get_parameter('max_vertical_mps').value),
            max_yaw_rate_rps=float(
                self.get_parameter('max_yaw_rate_rps').value),
            enable_vertical=bool(
                self.get_parameter('enable_vertical').value),
        )

        self._status_pub = self.create_publisher(
            String, adapter_status_topic, 10)
        self.create_subscription(
            Twist, command_topic, self._command_cb, 10)
        self.create_subscription(
            String, gate_status_topic, self._gate_status_cb, 10)

        self._start_time = self._now()
        self._command: BodyDemand | None = None
        self._command_time: float | None = None
        self._invalid_command = False
        self._gate_status: str | None = None
        self._gate_status_time: float | None = None
        self._heartbeat_time: float | None = None
        self._target_system: int | None = None
        self._target_component: int | None = None
        self._armed = False
        self._vehicle_mode = 'UNKNOWN'
        self._had_authority = False
        self._neutral_remaining = 0
        self._reauthorization = ReauthorizationLatch()
        self._last_status: str | None = None

        self.create_timer(1.0 / poll_rate_hz, self._poll_mavlink)
        self.create_timer(1.0 / send_rate_hz, self._send_tick)
        self.get_logger().warn(
            f'ArduSub adapter ready but NOT authorized — backend={self._backend} '
            f'mode={self._required_mode} connection={self._connection_url} '
            f'vertical={self._sender.enable_vertical}; adapter never arms or '
            'changes vehicle mode')

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    @staticmethod
    def _twist_values(msg: Twist) -> tuple[float, ...]:
        return (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )

    def _command_cb(self, msg: Twist) -> None:
        demand = validate_body_command(self._twist_values(msg))
        self._command = demand
        self._command_time = self._now() if demand is not None else None
        self._invalid_command = demand is None

    def _gate_status_cb(self, msg: String) -> None:
        self._gate_status = msg.data
        self._gate_status_time = self._now()

    def _poll_mavlink(self) -> None:
        try:
            while True:
                msg = self._link.recv_match(blocking=False)
                if msg is None:
                    return
                if msg.get_type() != 'HEARTBEAT':
                    continue
                if (getattr(msg, 'autopilot', None) ==
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                    continue
                source_system = int(msg.get_srcSystem())
                source_component = int(msg.get_srcComponent())
                if (self._configured_target_system and source_system !=
                        self._configured_target_system):
                    continue
                if (self._configured_target_component and source_component !=
                        self._configured_target_component):
                    continue
                if (self._target_system is not None and source_system !=
                        self._target_system):
                    continue
                if (self._target_component is not None and source_component !=
                        self._target_component):
                    continue
                self._heartbeat_time = self._now()
                self._target_system = source_system
                self._target_component = source_component
                self._armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._vehicle_mode = mavutil.mode_string_v10(msg).upper()
        except Exception as exc:  # transport errors are fail-closed
            self._heartbeat_time = None
            self._publish_status(f'LINK_RECEIVE_ERROR:{type(exc).__name__}')

    def _readiness(self, now: float) -> str:
        return adapter_readiness(
            now=now,
            heartbeat_time=self._heartbeat_time,
            heartbeat_timeout_s=self._heartbeat_timeout_s,
            gate_status=self._gate_status,
            gate_status_time=self._gate_status_time,
            gate_status_timeout_s=self._gate_status_timeout_s,
            command=self._command,
            command_time=self._command_time,
            command_timeout_s=self._command_timeout_s,
            invalid_command=self._invalid_command,
            armed=self._armed,
            require_armed=self._require_armed,
            vehicle_mode=self._vehicle_mode,
            required_mode=self._required_mode,
        )

    def _send_tick(self) -> None:
        now = self._now()
        readiness = self._reauthorization.evaluate(
            self._readiness(now), self._had_authority)
        if readiness == 'ACTIVE':
            try:
                self._send(self._command, now)
                self._had_authority = True
                self._neutral_remaining = self._neutral_burst_count
            except Exception as exc:  # encoding/socket errors are fail-closed
                readiness = self._reauthorization.evaluate(
                    f'LINK_SEND_ERROR:{type(exc).__name__}',
                    self._had_authority,
                )
        elif self._had_authority and self._neutral_remaining > 0:
            try:
                self._send(BodyDemand(), now)
                self._neutral_remaining -= 1
                if self._neutral_remaining == 0:
                    self._had_authority = False
            except Exception as exc:
                readiness = f'NEUTRAL_SEND_ERROR:{type(exc).__name__}'

        self._publish_status(readiness)

    def _send(self, demand: BodyDemand | None, now: float) -> None:
        if (demand is None or self._target_system is None or
                self._target_component is None):
            raise RuntimeError('MAVLink target or command unavailable')
        elapsed_ms = int((now - self._start_time) * 1000.0) & 0xFFFFFFFF
        self._sender.send(
            demand, self._target_system, self._target_component, elapsed_ms)

    def _publish_status(self, status_text: str) -> None:
        status = String()
        status.data = status_text
        self._status_pub.publish(status)
        if status_text == self._last_status:
            return
        if status_text == 'ACTIVE':
            self.get_logger().info(
                f'ArduSub command output ACTIVE ({self._backend})')
        else:
            self.get_logger().warn(
                f'ArduSub command output inhibited: {status_text}')
        self._last_status = status_text

    def stop(self) -> None:
        """Send a best-effort neutral burst if this process held authority."""
        if self._had_authority:
            for _ in range(self._neutral_burst_count):
                try:
                    self._send(BodyDemand(), self._now())
                except Exception:
                    break
        self._had_authority = False
        try:
            self._link.close()
        except Exception:
            pass


def main(args=None) -> None:
    """Run the ROS-to-ArduSub adapter until shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArduSubAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
