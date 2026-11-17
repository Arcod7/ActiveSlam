#!/usr/bin/env python3
"""Is the non-monotone marginal iSAM2's incremental recovery, or the model?

Builds the same chain pose_graph builds -- a dead-reckoning BetweenFactor plus
a ZPR/attitude prior per node, no loop closures -- over a recorded trajectory,
and recovers each node's marginal two ways:

  incremental  isam.marginalCovariance(newest) right after each update, which
               is exactly what pose_graph.py caches on the keyframe
  batch        one Marginals() over the final full graph

If batch is monotone and incremental is not, the oscillation is iSAM2
relinearisation and says nothing about the noise model.
"""
import sys
import numpy as np
import gtsam

import os
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(_REPO, 'slam', 'slam_backend'))
sys.path.insert(0, _HERE)
from odom_error_mc import load_gt  # noqa: E402
from slam_backend.odom_noise import odom_trans_sigma, fused_yaw_stats  # noqa: E402
from slam_backend.sensor_models.noise_profiles import load_noise_profile  # noqa: E402

XYH = [3, 4, 2]
# --scan adds the sequential scan-match BetweenFactor at pose_graph's constant
# sigmas, to measure how much it deflates the marginal on an identical chain.
SCAN = '--scan' in sys.argv
SCAN_TRANS, SCAN_ROT = 0.12, 0.08
CFG = os.path.join(_REPO, 'slam', 'slam_backend', 'config')
gt = load_gt(sys.argv[1], 50.0)
prof = load_noise_profile(f'{CFG}/noise_realistic.yaml')
prof.dvl = load_noise_profile(f'{CFG}/noise_degraded.yaml').dvl
yaw_sigma, yaw_tau = fused_yaw_stats(prof.imu, prof.compass)

# Keyframe every 0.5 m of arc, as keyframe_dist_m does.
marks = [0]
for i in range(len(gt['arc'])):
    if gt['arc'][i] - gt['arc'][marks[-1]] >= 0.5:
        marks.append(i)
print(f'{len(marks)} keyframes over {gt["arc"][-1]:.0f} m')

def pose_at(i):
    r = gtsam.Rot3.Ypr(gt['yaw'][i], gt['pitch'][i], gt['roll'][i])
    return gtsam.Pose3(r, gtsam.Point3(*gt['pos'][i]))

prior_sig = np.array([prof.imu.sigma_roll_rad, prof.imu.sigma_pitch_rad,
                      prof.imu.sigma_yaw_rad, 1e3, 1e3,
                      prof.pressure.sigma_depth_m])
zpr = gtsam.noiseModel.Diagonal.Sigmas(prior_sig)
anchor = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.02] * 3 + [0.002] * 3))

params = gtsam.ISAM2Params()
isam = gtsam.ISAM2(params)
full = gtsam.NonlinearFactorGraph()
vals_all = gtsam.Values()
cum_d = cum_t = cum_disp = cum_rot = 0.0
rot_int = np.zeros((3, 3))
inc = []

for n, i in enumerate(marks):
    g = gtsam.NonlinearFactorGraph()
    v = gtsam.Values()
    sym = gtsam.symbol('x', n)
    P = pose_at(i)
    if n == 0:
        g.addPriorPose3(sym, P, anchor)
        full.addPriorPose3(sym, P, anchor)
    else:
        prev_i = marks[n - 1]
        T = pose_at(prev_i).between(P)
        d = float(np.linalg.norm(T.translation()))
        dt = gt['t'][i] - gt['t'][prev_i]
        d0, t0, r0, a0 = cum_d, cum_t, cum_disp, cum_rot
        cum_d += d
        cum_t += dt
        cum_disp = max(cum_disp, float(np.linalg.norm(gt['pos'][i] - gt['pos'][0])))
        rot_int += pose_at(i).rotation().matrix() * dt
        cum_rot = max(cum_rot, float(np.linalg.norm(rot_int[:2, :])))
        s = odom_trans_sigma(d, dt, prof.dvl, cum_dist_m=(d0, cum_d),
                             cum_time_s=(t0, cum_t), cum_disp_m=(r0, cum_disp),
                             cum_rot_time_s=(a0, cum_rot),
                             yaw_sigma_rad=yaw_sigma, yaw_corr_time_s=yaw_tau,
                             yaw_bias_drift_rad_s=prof.compass.bias_drift_rad_s)
        nm = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.02] * 3 + [s] * 3))
        g.add(gtsam.BetweenFactorPose3(gtsam.symbol('x', n - 1), sym, T, nm))
        full.add(gtsam.BetweenFactorPose3(gtsam.symbol('x', n - 1), sym, T, nm))
        if SCAN:
            # The sequential scan-match factor pose_graph adds on every
            # consecutive pair, at its hard-coded constant sigma. Same
            # measurement as the odometry edge here, so this isolates what the
            # extra factor does to the MARGINAL, with keyframing held fixed.
            sm = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([SCAN_ROT] * 3 + [SCAN_TRANS] * 3))
            g.add(gtsam.BetweenFactorPose3(gtsam.symbol('x', n - 1), sym, T, sm))
            full.add(gtsam.BetweenFactorPose3(gtsam.symbol('x', n - 1), sym, T, sm))
    g.addPriorPose3(sym, P, zpr)
    full.addPriorPose3(sym, P, zpr)
    v.insert(sym, P)
    vals_all.insert(sym, P)
    isam.update(g, v)
    inc.append(isam.marginalCovariance(sym)[np.ix_(XYH, XYH)])

marg = gtsam.Marginals(full, vals_all)
bat = [marg.marginalCovariance(gtsam.symbol('x', n))[np.ix_(XYH, XYH)]
       for n in range(len(marks))]

def report(name, mats):
    sxy = np.array([np.linalg.det(M[:2, :2]) ** 0.25 for M in mats])
    dec = 100 * np.mean(np.diff(sxy[1:]) < -1e-12)
    print(f'{name:12s} sigma_xy {sxy[1]:.4f} -> {sxy[-1]:.4f} | '
          f'decreasing steps {dec:.0f}% | max drop {np.diff(sxy).min():+.4f}')
    return sxy

a = report('incremental', inc)
b = report('batch', bat)
print(f'\nbatch/incremental at the end: {b[-1] / a[-1]:.2f}x')
