"""Pure fail-closed state machine used by the body-command safety gate."""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


@dataclass(frozen=True)
class GateDecision:
    """One safety-gate evaluation result."""

    output: Tuple[float, ...]
    state: str


class MotionSafetyState:
    """Validate command/odometry freshness before allowing body motion."""

    ACTIVE = 'ACTIVE'
    DISABLED = 'DISABLED'
    INVALID_COMMAND = 'INVALID_COMMAND'
    MULTIPLE_COMMAND_SOURCES = 'MULTIPLE_COMMAND_SOURCES'
    NO_COMMAND = 'NO_COMMAND'
    NO_ODOMETRY = 'NO_ODOMETRY'
    STALE_COMMAND = 'STALE_COMMAND'
    STALE_ODOMETRY = 'STALE_ODOMETRY'

    def __init__(self, command_size: int = 6,
                 command_timeout_s: float = 0.5,
                 odom_timeout_s: float = 0.5,
                 max_abs_command: float = 1.0,
                 require_odom: bool = True) -> None:
        if command_size <= 0:
            raise ValueError('command_size must be positive')
        if command_timeout_s <= 0.0 or odom_timeout_s <= 0.0:
            raise ValueError('timeouts must be positive')
        if max_abs_command <= 0.0:
            raise ValueError('max_abs_command must be positive')

        self.command_size = int(command_size)
        self.command_timeout_s = float(command_timeout_s)
        self.odom_timeout_s = float(odom_timeout_s)
        self.max_abs_command = float(max_abs_command)
        self.require_odom = bool(require_odom)
        self.enabled = False
        self._command: Tuple[float, ...] | None = None
        self._command_time: float | None = None
        self._odom_time: float | None = None
        self._invalid_command = False

    @property
    def zero(self) -> Tuple[float, ...]:
        return (0.0,) * self.command_size

    def set_enabled(self, enabled: bool) -> None:
        """Change gate state; every enable transition requires a new command."""
        enabled = bool(enabled)
        if enabled and not self.enabled:
            self._command = None
            self._command_time = None
            self._invalid_command = False
        elif not enabled:
            self._command = None
            self._command_time = None
            self._invalid_command = False
        self.enabled = enabled

    def update_command(self, values: Iterable[float], now: float) -> bool:
        """Store a valid command, or invalidate output until a valid one arrives."""
        command = tuple(float(value) for value in values)
        valid = (
            len(command) == self.command_size
            and all(math.isfinite(value) for value in command)
            and all(abs(value) <= self.max_abs_command for value in command)
        )
        if not valid:
            self._command = None
            self._command_time = None
            self._invalid_command = True
            return False

        self._command = command
        self._command_time = float(now)
        self._invalid_command = False
        return True

    def update_odometry(self, now: float) -> None:
        self._odom_time = float(now)

    def evaluate(self, now: float,
                 multiple_command_sources: bool = False) -> GateDecision:
        """Return either the latest safe command or an all-zero command."""
        now = float(now)
        if not self.enabled:
            return GateDecision(self.zero, self.DISABLED)
        if multiple_command_sources:
            return GateDecision(self.zero, self.MULTIPLE_COMMAND_SOURCES)
        if self._invalid_command:
            return GateDecision(self.zero, self.INVALID_COMMAND)
        if self._command is None or self._command_time is None:
            return GateDecision(self.zero, self.NO_COMMAND)
        if now - self._command_time > self.command_timeout_s:
            return GateDecision(self.zero, self.STALE_COMMAND)
        if self.require_odom:
            if self._odom_time is None:
                return GateDecision(self.zero, self.NO_ODOMETRY)
            if now - self._odom_time > self.odom_timeout_s:
                return GateDecision(self.zero, self.STALE_ODOMETRY)
        return GateDecision(self._command, self.ACTIVE)
