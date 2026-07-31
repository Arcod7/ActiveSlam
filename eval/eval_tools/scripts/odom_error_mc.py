#!/usr/bin/env python3
"""Monte Carlo of the dead-reckoning chain: what does the odometry error
actually do, and does odom_trans_sigma() describe it?

Three seeds of a full simulation cannot separate four noise terms. This replays
a recorded ground-truth trajectory through the same arithmetic the sensor sims
and dead_reckoning.py perform -- dvl_sim's scale/bias/white velocity noise,
imu_sim's gyro bias walk, compass_sim's bias walk and white heading noise, the
YawKalmanFilter fusion, and the rotate-then-integrate step -- for N independent
trials, and reports the empirical error distribution against distance.

Each term can be enabled alone, which is the only way to see which one the
budget is actually spent on. The model's own prediction is overlaid, so the
comparison is like for like.

Faithfulness notes, i.e. what this reproduces exactly and what it abstracts:
  exact      DVL noise composition, integration in the estimated attitude,
             yaw filter arithmetic, IMU/compass rates and bias walks
  abstracted the vehicle follows the recorded GT path regardless of its own
             estimate -- true in the sim only when the controller runs off
             ground truth, so treat closed-loop coupling as out of scope
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', '..', '..', 'slam', 'slam_backend'))
from slam_backend.sensor_models.noise_profiles import load_noise_profile  # noqa: E402
from slam_backend.odom_noise import odom_trans_sigma, fused_yaw_stats  # noqa: E402

IMU_HZ = 50.0


def load_gt(path, hz):
    """GT trajectory resampled to a uniform rate: t, xy, yaw, roll, pitch."""
    a = np.loadtxt(path)
    t, p, q = a[:, 0] - a[0, 0], a[:, 1:4], a[:, 4:8]
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    yaw = np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    tu = np.arange(0.0, t[-1], 1.0 / hz)
    out = {'t': tu, 'dt': 1.0 / hz}
    for name, col in (('x', p[:, 0]), ('y', p[:, 1]), ('z', p[:, 2]),
                      ('yaw', yaw), ('roll', roll), ('pitch', pitch)):
        out[name] = np.interp(tu, t, col)
    pos = np.column_stack([out['x'], out['y'], out['z']])
    out['pos'] = pos
    out['arc'] = np.cumsum(np.r_[0.0, np.linalg.norm(np.diff(pos, axis=0), axis=1)])
    out['disp'] = np.linalg.norm(pos - pos[0], axis=1)
    # ||integral R dt|| over the horizontal rows: the arm a body-frame velocity
    # bias accumulates against once the vehicle turns.
    M = np.zeros((3, 3))
    arm = np.zeros(len(tu))
    for i in range(len(tu)):
        M = M + _rot(out['yaw'][i], out['roll'][i], out['pitch'][i]) * out['dt']
        arm[i] = np.linalg.norm(M[:2, :])
    # Monotone running maxima: odom_trans_sigma needs arms that never shrink.
    out['rot_arm'] = np.maximum.accumulate(arm)
    out['disp_max'] = np.maximum.accumulate(out['disp'])
    return out


def simulate(gt, profile, trials, terms, rng):
    """Vectorised over trials. Returns per-step horizontal error, (steps, trials)."""
    dvl, imu, comp = profile.dvl, profile.imu, profile.compass
    dt = gt['dt']
    n = len(gt['t'])
    dvl_every = max(1, int(round(IMU_HZ / dvl.publish_rate_hz)))
    comp_every = max(1, int(round(IMU_HZ / comp.publish_rate_hz)))

    on = lambda k: k in terms  # noqa: E731

    # One draw per trial for the whole run, exactly as dvl_sim does at startup.
    scale = 1.0 + (rng.normal(0, dvl.scale_error_pct, trials) if on('scale')
                   else np.zeros(trials))
    # dvl_sim adds bias_m_s as a fixed constant on every body axis, identical in
    # every run: a systematic offset, not a random variable. Modelled here as
    # what it is; --bias-random draws it per trial instead, which is what
    # odom_trans_sigma assumes and what real per-unit calibration would give.
    if on('bias'):
        bias = (rng.normal(0, dvl.bias_m_s, (trials, 3)) if on('bias_random')
                else np.full((trials, 3), dvl.bias_m_s))
    else:
        bias = np.zeros((trials, 3))

    imu_bias = np.zeros(trials)
    comp_bias = np.zeros(trials)
    kf_angle = gt['yaw'][0] + np.zeros(trials)
    kf_var = np.full(trials, imu.sigma_yaw_rad ** 2)
    last_imu_heading = None
    pos = np.zeros((trials, 2)) + gt['pos'][0, :2]
    err = np.zeros((n, trials))
    last_dvl_i = 0

    for i in range(n):
        if on('imu_bias'):
            imu_bias += rng.normal(0, imu.gyro_bias_drift_rad_s * np.sqrt(dt), trials)
        imu_heading = gt['yaw'][i] + imu_bias
        if on('imu_white'):
            imu_heading = imu_heading + rng.normal(0, imu.sigma_yaw_rad, trials)
        if last_imu_heading is None:
            kf_angle = imu_heading.copy()
            kf_var = np.full(trials, imu.sigma_yaw_rad ** 2)
        else:
            kf_angle = kf_angle + (imu_heading - last_imu_heading)
            kf_var = kf_var + 2.0 * imu.sigma_yaw_rad ** 2
            kf_var = kf_var + imu.gyro_bias_drift_rad_s ** 2 * dt
        last_imu_heading = imu_heading

        if i % comp_every == 0:
            if on('compass_bias'):
                comp_bias += rng.normal(
                    0, comp.bias_drift_rad_s * np.sqrt(dt * comp_every), trials)
            meas = gt['yaw'][i] + comp_bias
            if on('compass_white'):
                meas = meas + rng.normal(0, comp.sigma_yaw_rad, trials)
            m_var = comp.sigma_yaw_rad ** 2
            gain = kf_var / (kf_var + m_var)
            kf_angle = kf_angle + gain * (meas - kf_angle)
            kf_var = kf_var * (1.0 - gain)

        if i % dvl_every == 0 and i > 0:
            span = dt * (i - last_dvl_i)
            d_world = gt['pos'][i] - gt['pos'][last_dvl_i]
            psi_t, r_t, p_t = gt['yaw'][last_dvl_i], gt['roll'][last_dvl_i], gt['pitch'][last_dvl_i]
            v_body = _to_body(d_world, psi_t, r_t, p_t) / span
            speed = np.linalg.norm(v_body)
            sig = max(speed * dvl.sigma_pct, dvl.sigma_floor_m_s)
            v = v_body[None, :] * scale[:, None] + bias
            if on('white'):
                v = v + rng.normal(0, sig, (trials, 3))
            psi_e = kf_angle if on('yaw_error') else np.full(trials, psi_t)
            r_e = (r_t + rng.normal(0, imu.sigma_roll_rad, trials) if on('rollpitch')
                   else np.full(trials, r_t))
            p_e = (p_t + rng.normal(0, imu.sigma_pitch_rad, trials) if on('rollpitch')
                   else np.full(trials, p_t))
            pos += _to_world_xy(v * span, psi_e, r_e, p_e)
            last_dvl_i = i

        err[i] = np.linalg.norm(pos - gt['pos'][i, :2], axis=1)
    return err


def _to_body(d_world, yaw, roll, pitch):
    return _rot(yaw, roll, pitch).T @ d_world


def _rot(yaw, roll, pitch):
    cy, sy = np.cos(yaw), np.sin(yaw)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _to_world_xy(v_body, yaw, roll, pitch):
    """Rotate per-trial body vectors by per-trial attitudes, keep x/y."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    vx, vy, vz = v_body[:, 0], v_body[:, 1], v_body[:, 2]
    # Rz(yaw) Ry(pitch) Rx(roll) applied row-wise.
    bx = cp * vx + sp * sr * vy + sp * cr * vz
    by = cr * vy - sr * vz
    return np.column_stack([cy * bx - sy * by, sy * bx + cy * by])


def model_sigma(gt, profile, coherent, use_displacement=False, attitude=False):
    """What odom_trans_sigma() accumulates over the same path, as a chain of
    keyframe edges every keyframe_dist_m of travel."""
    dvl = profile.dvl
    arc, disp, t = gt['arc'], gt['disp'], gt['t']
    marks = list(range(0, len(t), int(len(t) / 200)))
    yaw_sigma, yaw_tau = fused_yaw_stats(profile.imu, profile.compass) if attitude else (0.0, 0.0)
    drift = profile.compass.bias_drift_rad_s if attitude else 0.0
    var = 0.0
    out_s, out_sig = [], []
    for a, b in zip(marks[:-1], marks[1:]):
        d = arc[b] - arc[a]
        dt_edge = t[b] - t[a]
        cum_arc = (arc[a], arc[b]) if coherent else None
        cum_disp = ((gt['disp_max'][a], gt['disp_max'][b])
                    if (coherent and use_displacement) else None)
        cum_t = (t[a], t[b]) if coherent else None
        cum_rot = ((gt['rot_arm'][a], gt['rot_arm'][b])
                   if (coherent and use_displacement) else None)
        s = odom_trans_sigma(d, dt_edge, dvl, cum_dist_m=cum_arc,
                             cum_time_s=cum_t, cum_disp_m=cum_disp,
                             cum_rot_time_s=cum_rot,
                             yaw_sigma_rad=yaw_sigma, yaw_corr_time_s=yaw_tau,
                             yaw_bias_drift_rad_s=drift)
        var += s ** 2
        out_s.append(arc[b])
        out_sig.append(np.sqrt(var))
    return np.asarray(out_s), np.asarray(out_sig)


ALL_TERMS = ('white', 'scale', 'bias', 'imu_bias', 'imu_white',
             'compass_bias', 'compass_white', 'yaw_error', 'rollpitch')
ATTITUDE = ('imu_bias', 'imu_white', 'compass_bias', 'compass_white',
            'yaw_error', 'rollpitch')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gt_tum')
    ap.add_argument('--profile', required=True)
    ap.add_argument('--trials', type=int, default=400)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--bias-random', action='store_true',
                    help='draw the DVL bias per trial instead of the constant '
                         'offset dvl_sim actually injects')
    ap.add_argument('--png', default='')
    args = ap.parse_args()

    profile = load_noise_profile(args.profile)
    gt = load_gt(args.gt_tum, IMU_HZ)
    rng = np.random.default_rng(args.seed)
    print(f'{os.path.basename(args.gt_tum)}: {gt["t"][-1]:.0f} s, '
          f'arc {gt["arc"][-1]:.1f} m, displacement {gt["disp"][-1]:.1f} m, '
          f'profile {profile.name}, {args.trials} trials\n')

    extra = ('bias_random',) if args.bias_random else ()
    cases = [('ALL', ALL_TERMS + extra),
             ('dvl white', ('white',)),
             ('dvl scale', ('scale',)),
             ('dvl bias', ('bias',) + extra),
             ('attitude (all)', ATTITUDE),
             ('  yaw filter only', ('imu_white', 'compass_white', 'yaw_error')),
             ('  compass bias only', ('compass_bias', 'yaw_error')),
             ('  roll/pitch only', ('rollpitch',))]

    marks = [0.25, 0.5, 0.75, 1.0]
    idx = [int(f * (len(gt['t']) - 1)) for f in marks]
    print(f'{"term":22s}' + ''.join(f'{gt["arc"][i]:>10.0f} m' for i in idx))
    print(f'{"":22s}' + ''.join(f'{"RMS err":>12s}' for _ in idx))
    results = {}
    for name, terms in cases:
        err = simulate(gt, profile, args.trials, set(terms), rng)
        results[name] = err
        rms = [np.sqrt(np.mean(err[i] ** 2)) for i in idx]
        print(f'{name:22s}' + ''.join(f'{v:12.3f}' for v in rms))

    print()
    for label, coherent, use_disp, att in (
            ('model, independent', False, False, False),
            ('model, coherent/arc', True, False, False),
            ('model, coherent/disp', True, True, False),
            ('model, + attitude', True, True, True)):
        s, sig = model_sigma(gt, profile, coherent, use_disp, att)
        vals = [sig[np.argmin(np.abs(s - gt['arc'][i]))] for i in idx]
        print(f'{label:22s}' + ''.join(f'{v:12.3f}' for v in vals))

    all_err = results['ALL']
    print(f'\nTruth at end: RMS {np.sqrt(np.mean(all_err[-1] ** 2)):.3f} m, '
          f'median {np.median(all_err[-1]):.3f} m, '
          f'p95 {np.percentile(all_err[-1], 95):.3f} m')
    arc_end = gt['arc'][-1]
    p = np.polyfit(np.log(gt['arc'][10:]),
                   np.log(np.sqrt(np.mean(all_err[10:] ** 2, axis=1))), 1)
    print(f'Empirical growth law: err_rms ~ {np.exp(p[1]):.4f} * arc^{p[0]:.2f} '
          f'(at {arc_end:.0f} m)')

    if args.png:
        plot(gt, results, profile, args.png)
        print(f'wrote {args.png}')


def plot(gt, results, profile, png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for name, err in results.items():
        ax.plot(gt['arc'], np.sqrt(np.mean(err ** 2, axis=1)),
                lw=2.0 if name == 'ALL' else 1.1,
                ls='-' if not name.startswith(' ') else ':', label=name)
    for label, coh, disp in (('model coherent/arc', True, False),
                             ('model coherent/disp', True, True),
                             ('model independent', False, False)):
        s, sig = model_sigma(gt, profile, coh, disp)
        ax.plot(s, sig, 'k', lw=1.4,
                ls={'model coherent/arc': '--', 'model coherent/disp': '-.',
                    'model independent': ':'}[label], label=label)
    ax.set_xlabel('ground-truth arc length (m)')
    ax.set_ylabel('horizontal error, RMS over trials (m)')
    ax.set_title('Dead-reckoning error by term, against what the graph budgets')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png, dpi=130)


if __name__ == '__main__':
    main()
