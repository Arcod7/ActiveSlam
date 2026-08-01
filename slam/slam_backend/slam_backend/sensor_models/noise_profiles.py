from dataclasses import dataclass, field
import os

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
    # White noise on the rate channel. Separate from gyro_bias_drift_rad_s,
    # which is a random walk on the integrated angle: a gyro measures rate
    # directly, so its rate output is not the derivative of its noisy angle.
    sigma_gyro_rad_s: float = 0.0
    publish_rate_hz: float = 50.0


@dataclass
class CompassNoise:
    """Absolute magnetic-heading measurement noise and slowly varying bias."""

    sigma_yaw_rad: float = 0.05
    bias_drift_rad_s: float = 0.0
    publish_rate_hz: float = 10.0

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
    dropout_p_grazing: float = 0.0     # extra dropout probability at grazing incidence
    dropout_grazing_exp: float = 2.0   # shape of the (1 - cos incidence) dropout term
    outlier_p: float = 0.0             # multipath outlier probability
    outlier_range_min_m: float = 0.3   # late-arrival excess range, lower bound
    outlier_range_max_m: float = 3.0   # late-arrival excess range, upper bound
    corr_length_px: float = 0.0        # spatial correlation length of the noise fields (px)
    corr_rho_time: float = 0.0         # AR(1) correlation between consecutive pings
    range_corr_frac: float = 0.0       # fraction of range-noise variance that is correlated
    sos_scale_error_pct: float = 0.0   # per-run speed-of-sound range scale error
    argmax_window_px: float = 0.0      # strongest-return search window (px, 0/1 = nearest-surface)
    argmax_beam_sigma_px: float = 1.5  # beam-pattern width for the strongest-return weighting
    lat_sigma_beam_frac: float = 0.0   # lateral jitter as a fraction of local beam spacing (0 = use per-m consts)
    reverb_p: float = 0.0              # volume-reverberation return probability
    reverb_max_m: float = 1.5          # near-field extent of reverberation returns
    reverb_weak_boost: float = 0.0     # extra reverb where the surface return is weak (far/grazing)
    multipath_p: float = 0.0           # geometric (screen-space) multipath probability
    multipath_grazing_exp: float = 1.0 # shape of the (1 - cos incidence) multipath weighting
    min_range_m: float = 0.0           # drop returns nearer than this (near-field gate, 0 = off)
    near_fade_p: float = 0.0           # near-field fade: drop probability at the sensor (0 = off)
    near_fade_range_m: float = 0.0     # range at which the fade reaches zero
    near_fade_exp: float = 1.0         # fade shape; >1 confines the thinning closer to the sensor

@dataclass
class NoiseProfile:
    name: str = "realistic"
    seed: int = -1                    # -1 = random; fixed seed for reproducible runs
    pressure: PressureNoise = field(default_factory=PressureNoise)
    imu: IMUNoise = field(default_factory=IMUNoise)
    compass: CompassNoise = field(default_factory=CompassNoise)
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
    if 'compass' in data:
        profile.compass = CompassNoise(**data['compass'])
    if 'dvl' in data:
        profile.dvl = DVLNoise(**data['dvl'])
    if 'sonar' in data:
        profile.sonar = SonarNoise(**data['sonar'])
    return profile

def load_composite_profile(default_path: str,
                           per_sensor_paths: "dict[str, str]") -> NoiseProfile:
    """Profile whose sections may come from different files.

    The sensor sims each accept a per-sensor profile override; consumers of the
    same numbers (dead_reckoning, pose_graph) must see the identical mix, or
    their noise models describe a different simulation than the one running.
    `per_sensor_paths` maps a section name ('dvl', ...) to a YAML path; empty
    paths and paths equal to the default are no-ops.
    """
    if default_path and os.path.exists(default_path):
        profile = load_noise_profile(default_path)
    else:
        profile = NoiseProfile()
    for section, path in per_sensor_paths.items():
        if not path or path == default_path or not os.path.exists(path):
            continue
        override = load_noise_profile(path)
        setattr(profile, section, getattr(override, section))
        profile.name = f'{profile.name}+{section}:{override.name}'
    return profile


def resolve_seed(profile_seed: int, override: int, offset: int) -> int:
    """Combine the profile's baked-in seed with a per-run override, then add
    a per-sensor offset so multiple sims sharing one seed don't draw
    identical RNG streams. -1 (random) passes through unchanged."""
    base = override if override != -1 else profile_seed
    return -1 if base == -1 else base + offset
