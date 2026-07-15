"""Tests for the sensor-noise profile definitions."""
from pathlib import Path

from slam_backend.sensor_models.noise_profiles import load_noise_profile


CONFIG_DIR = Path(__file__).parents[1] / 'config'


def test_odom_pos_only_is_realistic_position_with_exact_attitude_and_sonar():
    """Keep DVL/pressure noise while passing Stonefish attitude and sonar exactly."""
    profile = load_noise_profile(CONFIG_DIR / 'noise_odom_pos_only.yaml')
    realistic = load_noise_profile(CONFIG_DIR / 'noise_realistic.yaml')
    ideal = load_noise_profile(CONFIG_DIR / 'noise_ideal.yaml')

    assert profile.name == 'odom_pos_only'
    assert profile.pressure == realistic.pressure
    assert profile.dvl == realistic.dvl
    assert profile.imu.sigma_roll_rad == 0.0
    assert profile.imu.sigma_pitch_rad == 0.0
    assert profile.imu.sigma_yaw_rad == 0.0
    assert profile.imu.gyro_bias_drift_rad_s == 0.0
    assert profile.compass.sigma_yaw_rad == 0.0
    assert profile.compass.bias_drift_rad_s == 0.0
    assert profile.sonar == ideal.sonar
