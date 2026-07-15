"""Pure tests for wrapped IMU/compass yaw fusion."""
import math

import pytest

from slam_backend.attitude_filter import wrap_angle, YawKalmanFilter


def test_imu_increments_propagate_yaw_estimate():
    """A clean IMU increment advances the estimate by the same angle."""
    yaw = YawKalmanFilter(gyro_process_sigma_rad_s=0.0)
    yaw.predict_imu(0.1, 1.0, sigma_rad=0.0)
    assert yaw.predict_imu(0.3, 2.0, sigma_rad=0.0) == pytest.approx(0.3)


def test_compass_correction_pulls_drifting_yaw_toward_heading():
    """An absolute compass observation corrects accumulated IMU drift."""
    yaw = YawKalmanFilter(gyro_process_sigma_rad_s=0.1)
    yaw.predict_imu(0.0, 0.0, sigma_rad=0.1)
    yaw.predict_imu(0.6, 1.0, sigma_rad=0.1)
    corrected = yaw.correct_compass(0.0, sigma_rad=0.05)
    assert abs(corrected) < 0.6


def test_exact_compass_heading_overrides_the_estimate():
    """The exact-orientation profile retains the Stonefish compass heading."""
    yaw = YawKalmanFilter(gyro_process_sigma_rad_s=0.1)
    yaw.predict_imu(0.8, 0.0, sigma_rad=0.1)
    assert yaw.correct_compass(-0.4, sigma_rad=0.0) == pytest.approx(-0.4)


def test_filter_handles_heading_wraparound():
    """A small crossing of +/-pi remains a small correction."""
    yaw = YawKalmanFilter(gyro_process_sigma_rad_s=0.0)
    yaw.predict_imu(math.radians(179.0), 0.0, sigma_rad=0.1)
    yaw.predict_imu(math.radians(-179.0), 1.0, sigma_rad=0.1)
    assert wrap_angle(yaw.angle - math.radians(-179.0)) == pytest.approx(0.0)
