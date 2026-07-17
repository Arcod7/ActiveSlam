"""Pure command validation and MAVLink encoding for the ArduSub adapter."""

import math
from dataclasses import dataclass
from typing import Iterable

try:
    from pymavlink import mavutil
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        'The ArduSub adapter requires pymavlink. Install the repository '
        'requirements.txt before selecting an ArduSub actuator backend.'
    ) from exc


BACKEND_MANUAL_CONTROL = 'manual_control'
BACKEND_LOCAL_NED_VELOCITY = 'local_ned_velocity'
SUPPORTED_BACKENDS = (BACKEND_MANUAL_CONTROL, BACKEND_LOCAL_NED_VELOCITY)

# Use vx/vy/vz and yaw_rate; ignore position, acceleration and yaw.
VELOCITY_YAW_RATE_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)


@dataclass(frozen=True)
class BodyDemand:
    """Normalized body demand in the project's NED convention."""

    surge: float = 0.0       # +X, forward
    sway: float = 0.0        # +Y, starboard
    heave: float = 0.0       # +Z, down
    yaw_rate: float = 0.0    # rotation about +Z


@dataclass(frozen=True)
class LocalNedVelocity:
    """Metric body-frame velocity setpoint for MAVLink message 84."""

    vx: float
    vy: float
    vz: float
    yaw_rate: float


class ReauthorizationLatch:
    """Require an explicit gate-disable cycle after active control faults."""

    def __init__(self) -> None:
        """Start clear; the adapter still needs all normal prerequisites."""
        self.fault: str | None = None

    def evaluate(self, readiness: str, had_authority: bool) -> str:
        """Latch losses during authority and clear only on explicit disable."""
        if readiness == 'GATE_DISABLED':
            self.fault = None
            return readiness
        if self.fault is not None:
            return f'REAUTHORIZE_REQUIRED:{self.fault}'
        if had_authority and readiness != 'ACTIVE':
            self.fault = readiness
            return f'REAUTHORIZE_REQUIRED:{self.fault}'
        return readiness


def adapter_readiness(
        *, now: float, heartbeat_time: float | None,
        heartbeat_timeout_s: float, gate_status: str | None,
        gate_status_time: float | None, gate_status_timeout_s: float,
        command: BodyDemand | None, command_time: float | None,
        command_timeout_s: float, invalid_command: bool, armed: bool,
        require_armed: bool, vehicle_mode: str,
        required_mode: str) -> str:
    """Evaluate every prerequisite before the adapter may send motion."""
    if heartbeat_time is None:
        return 'NO_HEARTBEAT'
    if now - heartbeat_time > heartbeat_timeout_s:
        return 'STALE_HEARTBEAT'
    if gate_status_time is None or gate_status is None:
        return 'NO_GATE_STATUS'
    if now - gate_status_time > gate_status_timeout_s:
        return 'STALE_GATE_STATUS'
    if gate_status != 'ACTIVE':
        return f'GATE_{gate_status}'
    if invalid_command:
        return 'INVALID_COMMAND'
    if command is None or command_time is None:
        return 'NO_COMMAND'
    if now - command_time > command_timeout_s:
        return 'STALE_COMMAND'
    if require_armed and not armed:
        return 'VEHICLE_DISARMED'
    if vehicle_mode != required_mode:
        return f'MODE_MISMATCH:{vehicle_mode}'
    return 'ACTIVE'


def validate_body_command(values: Iterable[float]) -> BodyDemand | None:
    """Validate a six-axis Twist tuple and reject unsupported roll/pitch."""
    command = tuple(float(value) for value in values)
    if len(command) != 6:
        return None
    if not all(
            math.isfinite(value) and abs(value) <= 1.0
            for value in command):
        return None
    if command[3] != 0.0 or command[4] != 0.0:
        return None
    return BodyDemand(
        surge=command[0], sway=command[1], heave=command[2],
        yaw_rate=command[5])


def manual_control_axes(
        demand: BodyDemand, authority: float,
        enable_vertical: bool = False) -> tuple[int, int, int, int]:
    """Convert normalized demand to ArduSub's legacy MANUAL_CONTROL axes."""
    if not 0.0 <= authority <= 1.0:
        raise ValueError('manual authority must be within [0, 1]')
    x = round(1000.0 * authority * demand.surge)
    y = round(1000.0 * authority * demand.sway)
    r = round(1000.0 * authority * demand.yaw_rate)
    heave = demand.heave if enable_vertical else 0.0
    # ArduSub retains its legacy Z range: 0=up, 500=neutral, 1000=down.
    z = round(500.0 + 500.0 * authority * heave)
    return x, y, z, r


def local_ned_velocity(
        demand: BodyDemand, max_surge_mps: float, max_sway_mps: float,
        max_vertical_mps: float, max_yaw_rate_rps: float,
        enable_vertical: bool = False) -> LocalNedVelocity:
    """Scale normalized body demand into conservative metric setpoints."""
    limits = (
        max_surge_mps, max_sway_mps, max_vertical_mps, max_yaw_rate_rps)
    if not all(math.isfinite(limit) and limit >= 0.0 for limit in limits):
        raise ValueError(
            'local-NED velocity limits must be finite and non-negative')
    return LocalNedVelocity(
        vx=demand.surge * max_surge_mps,
        vy=demand.sway * max_sway_mps,
        vz=demand.heave * max_vertical_mps if enable_vertical else 0.0,
        yaw_rate=demand.yaw_rate * max_yaw_rate_rps,
    )


class ArduSubCommandSender:
    """Encode one of the supported body-demand MAVLink interfaces."""

    def __init__(
            self, connection, backend: str, manual_authority: float = 0.15,
            max_surge_mps: float = 0.15, max_sway_mps: float = 0.15,
            max_vertical_mps: float = 0.10,
            max_yaw_rate_rps: float = 0.15,
            enable_vertical: bool = False) -> None:
        """Configure encoding limits without opening or owning the link."""
        if backend not in SUPPORTED_BACKENDS:
            raise ValueError(
                f'backend must be one of {", ".join(SUPPORTED_BACKENDS)}')
        if not 0.0 <= manual_authority <= 1.0:
            raise ValueError('manual_authority must be within [0, 1]')
        # Validate limits once, including when the manual backend is selected.
        local_ned_velocity(
            BodyDemand(), max_surge_mps, max_sway_mps, max_vertical_mps,
            max_yaw_rate_rps, enable_vertical)
        self.connection = connection
        self.backend = backend
        self.manual_authority = manual_authority
        self.max_surge_mps = max_surge_mps
        self.max_sway_mps = max_sway_mps
        self.max_vertical_mps = max_vertical_mps
        self.max_yaw_rate_rps = max_yaw_rate_rps
        self.enable_vertical = enable_vertical

    def send(
            self, demand: BodyDemand, target_system: int,
            target_component: int, time_boot_ms: int) -> None:
        """Encode and send one demand without changing mode or arm state."""
        if self.backend == BACKEND_MANUAL_CONTROL:
            x, y, z, r = manual_control_axes(
                demand, self.manual_authority, self.enable_vertical)
            self.connection.mav.manual_control_send(
                target_system, x, y, z, r, 0)
            return

        setpoint = local_ned_velocity(
            demand,
            self.max_surge_mps,
            self.max_sway_mps,
            self.max_vertical_mps,
            self.max_yaw_rate_rps,
            self.enable_vertical,
        )
        self.connection.mav.set_position_target_local_ned_send(
            time_boot_ms,
            target_system,
            target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            VELOCITY_YAW_RATE_TYPE_MASK,
            0.0, 0.0, 0.0,
            setpoint.vx, setpoint.vy, setpoint.vz,
            0.0, 0.0, 0.0,
            0.0, setpoint.yaw_rate,
        )
