"""Small wrapped-angle Kalman filter used by dead-reckoning attitude fusion."""
import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class YawKalmanFilter:
    """Propagate yaw with IMU increments and correct it with compass heading."""

    def __init__(self, gyro_process_sigma_rad_s: float) -> None:
        self._process_variance = max(0.0, gyro_process_sigma_rad_s) ** 2
        self.angle: float | None = None
        self.variance = float('inf')
        self._last_imu_heading: float | None = None
        self._last_imu_time: float | None = None

    def predict_imu(self, heading: float, stamp_s: float,
                    sigma_rad: float) -> float:
        """Advance the estimate using the wrapped increment in IMU heading."""
        heading = wrap_angle(heading)
        if self._last_imu_heading is None or self.angle is None:
            self.angle = heading
            self.variance = max(0.0, sigma_rad) ** 2
        else:
            delta = wrap_angle(heading - self._last_imu_heading)
            dt = max(0.0, stamp_s - self._last_imu_time)
            self.angle = wrap_angle(self.angle + delta)
            # Two independent absolute-heading samples form one increment.
            self.variance += 2.0 * max(0.0, sigma_rad) ** 2
            self.variance += self._process_variance * dt
        self._last_imu_heading = heading
        self._last_imu_time = stamp_s
        return self.angle

    def correct_compass(self, heading: float, sigma_rad: float) -> float:
        """Apply a compass measurement correction, including exact-heading mode."""
        heading = wrap_angle(heading)
        measurement_variance = max(0.0, sigma_rad) ** 2
        if self.angle is None or measurement_variance == 0.0:
            self.angle = heading
            self.variance = measurement_variance
            return self.angle
        gain = self.variance / (self.variance + measurement_variance)
        self.angle = wrap_angle(self.angle + gain * wrap_angle(heading - self.angle))
        self.variance *= 1.0 - gain
        return self.angle
