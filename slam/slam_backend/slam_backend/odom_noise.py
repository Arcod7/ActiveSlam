"""Per-edge dead-reckoning noise, derived from the DVL profile.

A single hand-set `odom_sigma_trans` cannot describe DVL dead reckoning: the
error on an edge depends on how far and how long the vehicle travelled, and the
profile's four DVL terms enter with different exponents.

  random velocity noise   sqrt(dt) in time      -- a covariance can represent it
  scale error             linear in distance    -- systematic
  velocity bias           linear in time        -- systematic

Folding the two systematic terms into a Gaussian sigma still understates them,
because they are perfectly correlated across consecutive edges while a
BetweenFactor treats each edge as independent evidence. It is nonetheless much
closer than ignoring them, and unlike the constant it responds to the profile:
realistic -> degraded moves a typical edge's sigma by ~3.8x, so a degraded-noise
experiment actually reaches the D-optimality the revisit trigger reads.
"""
import math

# Floor for a near-stationary edge, where the proportional terms vanish but
# quantisation and attitude error do not.
DEFAULT_FLOOR_M = 0.002


def odom_trans_sigma(dist_m: float, dt_s: float, dvl,
                     floor_m: float = DEFAULT_FLOOR_M) -> float:
    """Translation sigma for one dead-reckoned keyframe edge."""
    if not (dt_s > 0.0) or not math.isfinite(dist_m) or dist_m < 0.0:
        return floor_m
    sigma_v = max(dvl.sigma_floor_m_s, dvl.sigma_pct * (dist_m / dt_s))
    rate = dvl.publish_rate_hz if dvl.publish_rate_hz > 0.0 else 1.0
    random_walk = sigma_v * math.sqrt(dt_s / rate)
    scale = dvl.scale_error_pct * dist_m
    bias = dvl.bias_m_s * dt_s
    return max(floor_m, math.hypot(math.hypot(random_walk, scale), bias))
