"""Pure-logic cable-safe sweep scan: right half, left full, return to start."""
from dataclasses import dataclass
import math

from frontier_slam.control_utils import wrap_angle


RIGHT = 'RIGHT'
LEFT = 'LEFT'
RETURN = 'RETURN'
DONE = 'DONE'


@dataclass(frozen=True)
class SweepCommand:
    """One tick's worth of sweep guidance; caller scales `yaw_sign` by its own yaw speed."""

    yaw_sign: float   # -1.0, 0.0, or +1.0
    phase: str
    done: bool


class SweepScan:
    """Odometry-confirmed right-then-left sweep, bounded at 2x timeout_s worst case."""

    TOLERANCE_RAD = math.radians(2.0)

    def __init__(self, sweep_deg: float, yaw_rate_rad_s: float,
                 timeout_multiplier: float = 3.0) -> None:
        if sweep_deg <= 0.0:
            raise ValueError('sweep_deg must be positive')
        if sweep_deg > 360.0:
            raise ValueError('sweep_deg must be <= 360 (not representable as a signed heading offset)')
        if yaw_rate_rad_s <= 0.0:
            raise ValueError('yaw_rate_rad_s must be positive')
        if timeout_multiplier <= 0.0:
            raise ValueError('timeout_multiplier must be positive')

        self._half = math.radians(sweep_deg) / 2.0
        total_travel = 2.0 * math.radians(sweep_deg)   # half + full + half
        self.timeout_s = timeout_multiplier * total_travel / yaw_rate_rad_s

        self._start_yaw: float | None = None
        self._start_time: float | None = None
        self._return_deadline: float | None = None
        self._net_yaw = 0.0
        self._phase = DONE

    def start(self, yaw: float, now: float) -> None:
        """Begin a new cycle at the given heading/time."""
        self._start_yaw = float(yaw)
        self._start_time = float(now)
        self._return_deadline = None
        self._net_yaw = 0.0
        self._phase = RIGHT

    def reset(self) -> None:
        """Discard any in-progress cycle; the next tick needs a fresh `start`."""
        self._start_yaw = None
        self._start_time = None
        self._return_deadline = None
        self._net_yaw = 0.0
        self._phase = DONE

    def _enter_return(self, now: float) -> None:
        self._phase = RETURN
        self._return_deadline = now + self.timeout_s

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def active(self) -> bool:
        return self._phase != DONE

    def update(self, yaw: float, now: float) -> SweepCommand:
        """Advance one tick from a new (yaw, now) sample; return the command to drive."""
        if self._phase == DONE:
            return SweepCommand(0.0, DONE, True)

        now = float(now)
        self._net_yaw = wrap_angle(float(yaw) - self._start_yaw)

        if self._phase != RETURN and (now - self._start_time) >= self.timeout_s:
            self._enter_return(now)

        if self._phase == RIGHT and self._net_yaw <= -self._half:
            self._phase = LEFT
        elif self._phase == LEFT and self._net_yaw >= self._half:
            self._enter_return(now)
        elif self._phase == RETURN and (abs(self._net_yaw) <= self.TOLERANCE_RAD
                                        or now >= self._return_deadline):
            self._phase = DONE
            return SweepCommand(0.0, DONE, True)

        if self._phase == RIGHT:
            sign = -1.0
        elif self._phase == LEFT:
            sign = 1.0
        else:   # RETURN — turn back toward zero net yaw, whichever side it's on
            sign = -1.0 if self._net_yaw > 0.0 else (1.0 if self._net_yaw < 0.0 else 0.0)

        return SweepCommand(sign, self._phase, False)


def scan_yaw_command(sweep: SweepScan, style: str, yaw: float, now: float,
                     scan_yaw_speed: float, repeat: bool) -> float:
    """Yaw command for one tick; 'spin' bypasses the sweep engine entirely."""
    if style == 'spin':
        return scan_yaw_speed
    if not sweep.active:
        if not repeat:
            return 0.0
        sweep.start(yaw, now)
    return sweep.update(yaw, now).yaw_sign * scan_yaw_speed
