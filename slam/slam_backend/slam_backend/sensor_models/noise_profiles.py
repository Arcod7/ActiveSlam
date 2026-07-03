from dataclasses import dataclass, field
import yaml

@dataclass
class PressureNoise:
    sigma_depth_m: float = 0.01
    bias_m: float = 0.0
    publish_rate_hz: float = 10.0

@dataclass
class IMUNoise:
    sigma_roll_rad: float = 0.01
    sigma_pitch_rad: float = 0.01
    sigma_yaw_rad: float = 0.05
    gyro_bias_drift_rad_s: float = 0.0
    publish_rate_hz: float = 50.0

@dataclass
class DVLNoise:
    sigma_pct: float = 0.01          # velocity noise as fraction of speed
    sigma_floor_m_s: float = 0.001   # minimum noise floor (m/s)
    bias_m_s: float = 0.001          # per-axis constant velocity bias (m/s)
    scale_error_pct: float = 0.0     # per-run constant scale factor error
    publish_rate_hz: float = 10.0

@dataclass
class NoiseProfile:
    name: str = "realistic"
    seed: int = -1                    # -1 = random; fixed seed for reproducible runs
    pressure: PressureNoise = field(default_factory=PressureNoise)
    imu: IMUNoise = field(default_factory=IMUNoise)
    dvl: DVLNoise = field(default_factory=DVLNoise)

def load_noise_profile(yaml_path: str) -> NoiseProfile:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    profile = NoiseProfile(name=data.get('name', 'unknown'))
    profile.seed = data.get('seed', -1)
    if 'pressure' in data:
        profile.pressure = PressureNoise(**data['pressure'])
    if 'imu' in data:
        profile.imu = IMUNoise(**data['imu'])
    if 'dvl' in data:
        profile.dvl = DVLNoise(**data['dvl'])
    return profile
