"""Tests for the sensor-noise profile definitions."""
from pathlib import Path

from slam_backend.sensor_models.noise_profiles import (
    load_composite_profile, load_noise_profile)


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


def test_composite_profile_overrides_only_named_sections():
    """A dvl:=degraded override run must give consumers degraded DVL terms
    while every other section stays on the default profile."""
    default = str(CONFIG_DIR / 'noise_realistic.yaml')
    degraded = str(CONFIG_DIR / 'noise_degraded.yaml')
    profile = load_composite_profile(default, {'dvl': degraded})
    realistic = load_noise_profile(default)

    assert profile.dvl == load_noise_profile(degraded).dvl
    assert profile.dvl != realistic.dvl
    assert profile.imu == realistic.imu
    assert profile.compass == realistic.compass
    assert profile.pressure == realistic.pressure
    assert 'dvl' in profile.name


def test_composite_profile_same_or_empty_paths_are_noops():
    default = str(CONFIG_DIR / 'noise_realistic.yaml')
    profile = load_composite_profile(default, {'dvl': default, 'imu': ''})
    realistic = load_noise_profile(default)
    assert profile.dvl == realistic.dvl
    assert profile.imu == realistic.imu
    assert profile.name == realistic.name


def test_composite_profile_missing_default_uses_ideal_defaults():
    degraded = str(CONFIG_DIR / 'noise_degraded.yaml')
    profile = load_composite_profile('', {'dvl': degraded})
    assert profile.dvl == load_noise_profile(degraded).dvl
    assert profile.imu.sigma_yaw_rad == 0.05   # NoiseProfile() default
