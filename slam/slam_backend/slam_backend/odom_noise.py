"""Per-edge dead-reckoning noise, derived from the sensor profile.

A single hand-set `odom_sigma_trans` cannot describe dead reckoning: the error
on an edge depends on how far and how long the vehicle travelled, and the terms
enter with different exponents.

  random velocity noise   sqrt(dt) in time            -- a covariance suits it
  DVL scale error         linear in DISPLACEMENT      -- systematic, run-long
  DVL velocity bias       linear in ||integral R dt|| -- systematic, run-long
  heading noise           sqrt(distance)              -- cross-track walk
  heading bias            linear in DISPLACEMENT      -- systematic, run-long

The last two dominate. dead_reckoning.py rotates DVL velocity into the world
frame with the ESTIMATED attitude, so every radian of heading error becomes
cross-track position error; a Monte Carlo of the chain (eval_tools/scripts/
odom_error_mc.py, 300 trials on a recorded 155 m path, degraded profile) puts
attitude at 0.790 m RMS of a 0.798 m total, against 0.324/0.276/0.131 m for the
three DVL terms. A model carrying only the DVL terms budgets for the small ones.

Coherence is what separates the two systematic groups from the random ones. A
term drawn once per run is perfectly correlated across every edge, while a
BetweenFactor treats each edge as independent evidence; summing per-edge
variances then understates the run total by roughly the edge count. Passing the
cumulative arms makes each edge carry its slice of the single run-long variable,
term^2 * (after^2 - before^2), which telescopes to the correct total.

Which cumulative quantity matters is not the same for every term, because each
systematic error is fixed in the BODY frame and turning decorrelates it. Scaling
body velocity by (1+eps) and integrating gives (1+eps) * integral(R v dt), so
the position error is eps * NET DISPLACEMENT, not eps * path length; the same
holds for a heading bias, whose cross-track error is psi * displacement. A
constant velocity bias b contributes b * ||integral R dt||, not b * elapsed
time. On the shipwreck survey both corrections are large: 155 m of path inside a
26 m displacement (5.9x), and 56.5 s of rotation-integrated arm inside 295 s
elapsed (5.2x). Accumulating against arc length or wall-clock ignores the
vehicle's tortuosity and overstates the systematic budget by roughly it.
"""
import math

# Floor for a near-stationary edge, where the proportional terms vanish but
# quantisation does not.
DEFAULT_FLOOR_M = 0.002


def fused_yaw_stats(imu, compass) -> "tuple[float, float]":
    """Steady-state (sigma, correlation time) of the fused heading error.

    Solves the YawKalmanFilter recursion at its fixed point rather than fitting:
    between compass corrections the filter adds 2*sigma_imu^2 per IMU tick plus
    the gyro random walk, and each correction multiplies the variance by
    (1 - gain). Both outputs are therefore properties of the profile alone.

    The correlation time is what decides whether heading noise accumulates as a
    random walk in cross-track (short tau, the white-compass case) or coherently
    with displacement (long tau, a Gauss-Markov compass bias).
    """
    m_var = max(compass.sigma_yaw_rad, 1e-9) ** 2
    imu_rate = imu.publish_rate_hz if imu.publish_rate_hz > 0.0 else 1.0
    comp_rate = compass.publish_rate_hz if compass.publish_rate_hz > 0.0 else 1.0
    ticks = max(1.0, imu_rate / comp_rate)
    dt_imu = 1.0 / imu_rate
    dt_comp = 1.0 / comp_rate
    # Variance injected between two corrections.
    q = ticks * 2.0 * imu.sigma_yaw_rad ** 2 + imu.gyro_bias_drift_rad_s ** 2 * dt_comp
    # Fixed point of  x - q = x*m/(x+m)  ->  x^2 - q*x - q*m = 0.
    x = 0.5 * (q + math.sqrt(q * q + 4.0 * q * m_var))
    gain = x / (x + m_var) if (x + m_var) > 0.0 else 1.0
    posterior_var = x * (1.0 - gain)
    # AR(1) decay per correction interval, converted to an e-folding time.
    keep = max(1e-6, min(1.0 - 1e-6, 1.0 - gain))
    tau = dt_comp / -math.log(keep)
    return math.sqrt(max(posterior_var, 0.0)), max(tau, dt_imu)


def odom_trans_sigma(dist_m: float, dt_s: float, dvl,
                     floor_m: float = DEFAULT_FLOOR_M,
                     cum_dist_m: "tuple[float, float] | None" = None,
                     cum_time_s: "tuple[float, float] | None" = None,
                     cum_disp_m: "tuple[float, float] | None" = None,
                     cum_rot_time_s: "tuple[float, float] | None" = None,
                     yaw_sigma_rad: float = 0.0,
                     yaw_corr_time_s: float = 0.0,
                     yaw_bias_drift_rad_s: float = 0.0,
                     yaw_bias_sigma_max_rad: float = 0.0) -> float:
    """Translation sigma for one dead-reckoned keyframe edge.

    `cum_*` are (before, after) cumulative totals since the graph's first
    keyframe; passing them switches the run-long systematic terms from the
    independent-per-edge treatment to the coherent one. Each systematic term
    takes the arm it actually accumulates against:

      cum_disp_m      displacement, for DVL scale and heading bias
      cum_rot_time_s  ||integral R dt||, for the body-frame DVL velocity bias
      cum_dist_m      arc length, accepted only as a fallback for cum_disp_m
      cum_time_s      elapsed time, fallback arm and the heading-bias growth

    Every arm must be MONOTONE, so callers pass a running maximum. Displacement
    and the rotation-integrated arm both shrink when the vehicle turns back
    toward its anchor, and a chain of independent factors can only ever grow a
    marginal: the negative slice is unrepresentable, so clamping it to zero
    would ratchet the total up past the truth (3.2x and 4.2x in variance on the
    shipwreck survey). The running maximum is the tightest monotone upper bound
    available here, leaving 1.17x and 1.57x in sigma against the true end
    value. Removing that residual needs the scale and bias carried as estimated
    STATES rather than as noise, which is the honest fix and is not this one.
    """
    if not (dt_s > 0.0) or not math.isfinite(dist_m) or dist_m < 0.0:
        return floor_m
    sigma_v = max(dvl.sigma_floor_m_s, dvl.sigma_pct * (dist_m / dt_s))
    rate = dvl.publish_rate_hz if dvl.publish_rate_hz > 0.0 else 1.0
    random_walk_var = (sigma_v * math.sqrt(dt_s / rate)) ** 2

    scale_arm = cum_disp_m if cum_disp_m is not None else cum_dist_m
    if scale_arm is not None:
        d0, d1 = scale_arm
        scale_var = (dvl.scale_error_pct ** 2) * max(0.0, d1 * d1 - d0 * d0)
    else:
        scale_var = (dvl.scale_error_pct * dist_m) ** 2

    # The DVL bias is constant in the BODY frame, so what it contributes in the
    # world is b * ||integral R dt||, not b * elapsed time: turning rotates the
    # offset through different world directions and it partly cancels. On the
    # shipwreck survey (1631 deg of yaw over 295 s) the rotation-integrated arm
    # is 56.5 s against 294.7 s elapsed, a 5.2x difference.
    bias_arm = cum_rot_time_s if cum_rot_time_s is not None else cum_time_s
    if bias_arm is not None:
        a0, a1 = bias_arm
        bias_var = (dvl.bias_m_s ** 2) * max(0.0, a1 * a1 - a0 * a0)
    else:
        bias_var = (dvl.bias_m_s * dt_s) ** 2

    # Cross-track from heading error. The short-correlated part is a random walk
    # in distance -- variance sigma^2 * 2*tau * d * v, with v = d/dt -- so it is
    # legitimately additive per edge and needs no cumulative arm.
    speed = dist_m / dt_s
    yaw_walk_var = (yaw_sigma_rad ** 2) * 2.0 * yaw_corr_time_s * dist_m * speed
    # A heading bias holds over the whole run, so its cross-track error is
    # sigma_bias(t) * displacement(t) -- coherent in displacement like DVL
    # scale, but with a sigma that itself grows as the bias walks. The edge
    # takes the difference of that product, which telescopes to the run total.
    # Capping at yaw_bias_sigma_max_rad turns the unbounded walk into the
    # bounded Gauss-Markov bias a magnetometer actually has.
    yaw_bias_var = 0.0
    if yaw_bias_drift_rad_s > 0.0 and cum_time_s is not None and cum_disp_m is not None:
        cap = (yaw_bias_sigma_max_rad ** 2 if yaw_bias_sigma_max_rad > 0.0
               else float('inf'))
        (t0, t1), (r0, r1) = cum_time_s, cum_disp_m
        drift_sq = yaw_bias_drift_rad_s ** 2
        v0 = min(drift_sq * t0, cap) * r0 * r0
        v1 = min(drift_sq * t1, cap) * r1 * r1
        yaw_bias_var = max(0.0, v1 - v0)

    total = (random_walk_var + scale_var + bias_var
             + yaw_walk_var + yaw_bias_var)
    return max(floor_m, math.sqrt(total))
