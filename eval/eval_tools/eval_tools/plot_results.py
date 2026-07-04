#!/usr/bin/env python3
"""Offline plotting for a benchmark run directory produced by benchmark.py.

Usage:
  ros2 run eval_tools plot_results <run_dir>

Reads gt_traj.tum / slam_traj.tum / odom_traj.tum and metrics.csv, and saves
<run_dir>/results.png with: XY trajectory, absolute position error, ATE,
RPE translation, and D-optimality (uncertainty) over time.
"""
import sys
import os
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _load_tum(filepath: str):
    if not os.path.exists(filepath):
        return np.empty((0,)), np.empty((0, 3))
    times, positions = [], []
    with open(filepath) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.split()
            times.append(float(parts[0]))
            positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(times), np.array(positions)


def _load_metrics(filepath: str):
    rows = {'t': [], 'abs_error': [], 'ate': [], 'rpe_trans': [], 'rpe_rot_deg': [], 'dopt': []}
    if not os.path.exists(filepath):
        return {k: np.array(v) for k, v in rows.items()}
    with open(filepath) as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in rows:
                val = row.get(key, '')
                rows[key].append(float(val) if val not in ('', None) else np.nan)
    return {k: np.array(v) for k, v in rows.items()}


def plot_run(run_dir: str):
    t_gt, pos_gt = _load_tum(os.path.join(run_dir, 'gt_traj.tum'))
    t_slam, pos_slam = _load_tum(os.path.join(run_dir, 'slam_traj.tum'))
    t_odom, pos_odom = _load_tum(os.path.join(run_dir, 'odom_traj.tum'))
    metrics = _load_metrics(os.path.join(run_dir, 'metrics.csv'))

    t0 = t_gt[0] if len(t_gt) else (metrics['t'][0] if len(metrics['t']) else 0.0)

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))

    ax = axes[0, 0]
    if len(pos_gt):
        ax.plot(pos_gt[:, 0], pos_gt[:, 1], label='Ground truth', color='tab:green')
    if len(pos_odom):
        ax.plot(pos_odom[:, 0], pos_odom[:, 1], label='Dead reckoning', color='tab:red', alpha=0.7)
    if len(pos_slam):
        ax.plot(pos_slam[:, 0], pos_slam[:, 1], label='SLAM estimate', color='tab:blue')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('XY trajectory')
    ax.axis('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    if len(metrics['t']):
        ax.plot(metrics['t'] - t0, metrics['abs_error'], color='tab:blue')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('error (m)')
    ax.set_title('Absolute position error (unaligned)')
    ax.grid(True, alpha=0.3)

    ax = axes[0, 2]
    if len(metrics['t']):
        ax.plot(metrics['t'] - t0, metrics['ate'], color='tab:purple')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('ATE RMSE (m)')
    ax.set_title('Cumulative ATE')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    if len(metrics['t']):
        ax.plot(metrics['t'] - t0, metrics['rpe_trans'], color='tab:orange')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('RPE trans (m)')
    ax.set_title('Relative pose error (translation)')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    if len(metrics['t']):
        ax.plot(metrics['t'] - t0, metrics['rpe_rot_deg'], color='tab:brown')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('RPE rot (deg)')
    ax.set_title('Relative pose error (rotation)')
    ax.grid(True, alpha=0.3)

    ax = axes[1, 2]
    if len(metrics['t']):
        ax.plot(metrics['t'] - t0, metrics['dopt'], color='tab:red')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('D-opt = det(cov_pos)^(1/3)')
    ax.set_title('Pose uncertainty (D-optimality)')
    ax.grid(True, alpha=0.3)

    fig.suptitle(f'Benchmark run: {os.path.basename(run_dir.rstrip("/"))}')
    fig.tight_layout()
    out_path = os.path.join(run_dir, 'results.png')
    fig.savefig(out_path, dpi=150)
    print(f'Saved {out_path}')


def main():
    if len(sys.argv) < 2:
        print('Usage: ros2 run eval_tools plot_results <run_dir>')
        sys.exit(1)
    plot_run(sys.argv[1])


if __name__ == '__main__':
    main()
