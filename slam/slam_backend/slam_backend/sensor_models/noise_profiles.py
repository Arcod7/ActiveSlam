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
class SonarNoise:
    """Datasheet-grounded noise model for the depth-camera cloud standing in
    for a WaterLinked Sonar 3D-15 (1.2 MHz mode). All defaults are zero so a
    profile that omits the `sonar:` section is an exact passthrough."""
    range_quant_m: float = 0.0          # range-bin quantization (datasheet: 1.5mm)
    range_sigma0_m: float = 0.0         # range noise floor (constant term)
    range_sigma_k: float = 0.0          # range noise growth per metre (speckle/SNR)
    lat_sigma_h_per_m: float = 0.0      # horizontal beam-spread jitter per metre range
    lat_sigma_v_per_m: float = 0.0      # vertical beam-spread jitter per metre range
    dropout_p0: float = 0.0            # base dropout probability
    dropout_p_range: float = 0.0       # extra dropout probability at max range
    outlier_p: float = 0.0             # multipath outlier probability
    outlier_range_min_m: float = 0.3   # late-arrival excess range, lower bound
    outlier_range_max_m: float = 3.0   # late-arrival excess range, upper bound

@dataclass
class NoiseProfile:
    name: str = "realistic"
    seed: int = -1                    # -1 = random; fixed seed for reproducible runs
    pressure: PressureNoise = field(default_factory=PressureNoise)
    imu: IMUNoise = field(default_factory=IMUNoise)
    dvl: DVLNoise = field(default_factory=DVLNoise)
    sonar: SonarNoise = field(default_factory=SonarNoise)

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
    if 'sonar' in data:
        profile.sonar = SonarNoise(**data['sonar'])
    return profile

def resolve_seed(profile_seed: int, override: int, offset: int) -> int:
    """Combine the profile's baked-in seed with a per-run override, then add
    a per-sensor offset so multiple sims sharing one seed don't draw
    identical RNG streams. -1 (random) passes through unchanged."""
    base = override if override != -1 else profile_seed
    return -1 if base == -1 else base + offset
