#!/usr/bin/env python3
"""Bundle and render the final map of a finished run.

Every eval run already persists its maps (map_saver.py) and its three
trajectories, but as separate files that lose their relationship once the run
directory is out of context. This packs each run into one self-describing
.npz -- belief map, ground-truth map, all three trajectories, the true and
believed final robot pose, and the run's metrics -- and renders a picture of
it.

  python3 eval/eval_tools/scripts/map_snapshot.py eval/runs/<batch>/*_s101
  python3 eval/eval_tools/scripts/map_snapshot.py eval/runs/<batch>/* --gallery
  python3 eval/eval_tools/scripts/map_snapshot.py <run_dir> --interactive

Writes into each run's own maps/ directory:
  snapshot.npz   everything below, loadable anywhere numpy is available
  snapshot.png   top-down belief vs ground truth, trajectory and final pose

Reading one back:

  import numpy as np, json
  d = np.load('.../maps/snapshot.npz', allow_pickle=False)
  meta = json.loads(str(d['meta']))
  belief, gt = d['belief_points'], d['gt_points']     # Nx3 metres, world_ned
  gt_traj = d['gt_traj']                              # Kx8 TUM rows
  d['final_pose_gt'], d['final_pose_slam']            # [x y z qx qy qz qw]

`explored_voxels` in the metadata is the count of distinct voxel_size cells in
the ground-truth cloud: what the sonar actually observed, independent of how
well the belief map placed it. Map coverage is a *fraction of* that, so the two
must be read together -- an arm can raise coverage by observing less.
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRID, AXIS, SURFACE = '#e1e0d9', '#c3c2b7', '#fcfcfb'
C_SLAM, C_ODOM = '#2a78d6', '#eb6834'
VOXEL_SIZE = 0.2


def _load_tum(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        data = np.loadtxt(path, ndmin=2)
    except Exception:
        return None
    return data if len(data) >= 1 and data.shape[1] >= 8 else None


def _last_row(csv_path, col):
    if not os.path.exists(csv_path):
        return None
    val = None
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            v = row.get(col, '')
            if v not in ('', None):
                val = float(v)
    return val


def _voxel_count(points, voxel_size=VOXEL_SIZE):
    if points is None or len(points) == 0:
        return 0
    return int(len(np.unique(np.floor(points / voxel_size).astype(np.int32),
                             axis=0)))


def collect(run_dir):
    """Everything about one finished run, or None if it has no maps."""
    maps_dir = os.path.join(run_dir, 'maps')
    belief_path = os.path.join(maps_dir, 'belief_tsdf.npy')
    gt_path = os.path.join(maps_dir, 'gt_tsdf.npy')
    if not (os.path.exists(belief_path) and os.path.exists(gt_path)):
        return None

    belief = np.load(belief_path).astype(np.float32)
    gt = np.load(gt_path).astype(np.float32)
    gt_traj = _load_tum(os.path.join(run_dir, 'gt_traj.tum'))
    slam_traj = _load_tum(os.path.join(run_dir, 'slam_traj.tum'))
    odom_traj = _load_tum(os.path.join(run_dir, 'odom_traj.tum'))

    base = os.path.basename(run_dir.rstrip('/'))
    arm, _, seed = base.rpartition('_s')
    manifest = {}
    manifest_path = os.path.join(run_dir, 'manifest.json')
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            manifest = json.load(f)

    metrics_csv = os.path.join(run_dir, 'metrics.csv')
    map_csv = os.path.join(run_dir, 'map_metrics.csv')
    path_m = (float(np.linalg.norm(np.diff(gt_traj[:, 1:3], axis=0), axis=1).sum())
              if gt_traj is not None and len(gt_traj) > 1 else float('nan'))

    explored = _voxel_count(gt)
    coverage = _last_row(map_csv, 'coverage')
    meta = {
        'run': base, 'arm': arm or base, 'seed': int(seed) if seed.isdigit() else -1,
        'status': manifest.get('status', 'unknown'),
        'voxel_size_m': VOXEL_SIZE,
        'path_m': round(path_m, 2),
        'ate_m': _last_row(metrics_csv, 'ate'),
        'coverage': coverage,
        'chamfer_m': _last_row(map_csv, 'chamfer'),
        'lc_count': _last_row(metrics_csv, 'lc_count'),
        'revisit_count': _last_row(metrics_csv, 'revisit_count'),
        'rebuild_count': _last_row(metrics_csv, 'rebuild_count'),
        'belief_points': len(belief), 'gt_points': len(gt),
        'belief_voxels': _voxel_count(belief),
        'explored_voxels': explored,
        # The absolute quantity the ratio hides: observed surface that the
        # belief map also placed within the coverage radius.
        'correctly_placed_voxels': (round(explored * coverage) if coverage
                                    else None),
        'args': manifest.get('args', {}),
    }
    return {'meta': meta, 'belief': belief, 'gt': gt, 'gt_traj': gt_traj,
            'slam_traj': slam_traj, 'odom_traj': odom_traj, 'dir': run_dir}


def write_npz(run):
    out = os.path.join(run['dir'], 'maps', 'snapshot.npz')
    empty = np.zeros((0, 8), dtype=np.float32)
    payload = {
        'belief_points': run['belief'], 'gt_points': run['gt'],
        'gt_traj': run['gt_traj'] if run['gt_traj'] is not None else empty,
        'slam_traj': run['slam_traj'] if run['slam_traj'] is not None else empty,
        'odom_traj': run['odom_traj'] if run['odom_traj'] is not None else empty,
        'final_pose_gt': (run['gt_traj'][-1, 1:8] if run['gt_traj'] is not None
                          else np.zeros(7)),
        'final_pose_slam': (run['slam_traj'][-1, 1:8] if run['slam_traj'] is not None
                            else np.zeros(7)),
        'meta': np.array(json.dumps(run['meta'])),
    }
    np.savez_compressed(out, **payload)
    return out


def _height_image(points, extent, voxel_size=VOXEL_SIZE):
    """Top-down max-height raster: what the map looks like from above."""
    xlo, xhi, ylo, yhi = extent
    nx = max(int((xhi - xlo) / voxel_size), 1)
    ny = max(int((yhi - ylo) / voxel_size), 1)
    img = np.full((ny, nx), np.nan, dtype=np.float32)
    if len(points) == 0:
        return img
    ix = ((points[:, 1] - xlo) / voxel_size).astype(np.int32)   # east
    iy = ((points[:, 0] - ylo) / voxel_size).astype(np.int32)   # north
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy = ix[ok], iy[ok]
    # NED z is positive down, so the shallowest (most negative) reading is the
    # top of the structure -- take the minimum to render a height map.
    z = points[ok, 2]
    flat = iy.astype(np.int64) * nx + ix
    order = np.argsort(-z)              # deepest first, so shallowest wins
    np.put(img, flat[order], z[order])
    return img


def _draw_map(ax, points, extent, traj, final_gt, final_slam, title, cmap):
    img = _height_image(points, extent)
    xlo, xhi, ylo, yhi = extent
    if np.isfinite(img).any():
        ax.imshow(img, origin='lower', extent=(xlo, xhi, ylo, yhi),
                  cmap=cmap, interpolation='nearest', zorder=1,
                  vmin=np.nanpercentile(img, 2), vmax=np.nanpercentile(img, 98))
    if traj is not None:
        ax.plot(traj[:, 2], traj[:, 1], '-', color=INK, lw=0.9, alpha=0.75,
                zorder=3)
    if final_gt is not None:
        ax.plot(final_gt[1], final_gt[0], 'o', color=INK, markersize=7,
                markeredgecolor=SURFACE, markeredgewidth=1.4, zorder=5)
        ax.annotate('robot (true)', (final_gt[1], final_gt[0]),
                    textcoords='offset points', xytext=(9, 4), fontsize=7.5,
                    color=INK)
    if final_slam is not None:
        ax.plot(final_slam[1], final_slam[0], 'X', color=C_SLAM, markersize=8,
                markeredgecolor=SURFACE, markeredgewidth=1.2, zorder=5)
        ax.annotate('robot (believed)', (final_slam[1], final_slam[0]),
                    textcoords='offset points', xytext=(9, -11), fontsize=7.5,
                    color=C_SLAM)
    ax.set_title(title, fontsize=9.5, color=INK)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_xlabel('East (m)')
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.grid(True, color=GRID, lw=0.8, alpha=0.9)
    ax.set_axisbelow(True)


def _extent(run, margin=3.0):
    pts = [p for p in (run['belief'], run['gt']) if len(p)]
    if not pts:
        return (-10, 10, -10, 10)
    allp = np.vstack(pts)
    return (float(allp[:, 1].min() - margin), float(allp[:, 1].max() + margin),
            float(allp[:, 0].min() - margin), float(allp[:, 0].max() + margin))


def render(run):
    m = run['meta']
    extent = _extent(run)
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 5.6), sharey=True)
    fig.patch.set_facecolor(SURFACE)
    for ax in axes:
        ax.set_facecolor(SURFACE)

    final_gt = run['gt_traj'][-1, 1:8] if run['gt_traj'] is not None else None
    final_slam = (run['slam_traj'][-1, 1:8] if run['slam_traj'] is not None
                  else None)

    _draw_map(axes[0], run['belief'], extent, run['slam_traj'], final_gt,
              final_slam,
              f"Belief map — {m['belief_voxels']:,} voxels", 'Blues_r')
    _draw_map(axes[1], run['gt'], extent, run['gt_traj'], final_gt, None,
              f"Ground truth (same sonar, true pose) — "
              f"{m['explored_voxels']:,} voxels observed", 'Greys_r')
    axes[0].set_ylabel('North (m)')

    cov = m['coverage'] if m['coverage'] is not None else float('nan')
    correct = m['correctly_placed_voxels']
    fig.suptitle(f"{m['run']}  ·  ATE {m['ate_m']:.2f} m  ·  {m['path_m']:.0f} m "
                 f"travelled  ·  {m['lc_count']:.0f} closures, "
                 f"{m['revisit_count']:.0f} revisits",
                 fontsize=11, color=INK, y=0.99)
    fig.text(0.5, 0.935,
             f'observed {m["explored_voxels"]:,} voxels · {cov:.1%} of them '
             f'placed within 0.4 m = {correct:,} correctly mapped',
             ha='center', fontsize=8.5, color=MUTED)
    handles = [Line2D([], [], color=INK, lw=1.2, label='trajectory'),
               Line2D([], [], color=INK, marker='o', ls='none',
                      label='final pose (true)'),
               Line2D([], [], color=C_SLAM, marker='X', ls='none',
                      label='final pose (believed)')]
    fig.legend(handles=handles, loc='lower center', ncol=3,
               bbox_to_anchor=(0.5, -0.01), frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0.04, 1, 0.915))
    out = os.path.join(run['dir'], 'maps', 'snapshot.png')
    fig.savefig(out, dpi=200, bbox_inches='tight', facecolor=SURFACE)
    plt.close(fig)
    return out


def gallery(runs, out_path):
    """Every run side by side, on one shared extent -- the trade-off at a glance."""
    runs = [r for r in runs if len(r['belief'])]
    if not runs:
        return None
    order = {'nolc': 0, 'lc': 1, 'lc_revisit': 2, 'full': 3}
    runs.sort(key=lambda r: (r['meta']['seed'],
                             order.get(r['meta']['arm'], 9)))
    allp = np.vstack([r['gt'] for r in runs] + [r['belief'] for r in runs])
    extent = (float(allp[:, 1].min() - 3), float(allp[:, 1].max() + 3),
              float(allp[:, 0].min() - 3), float(allp[:, 0].max() + 3))

    cols = min(4, len(runs))
    rows = int(np.ceil(len(runs) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(3.4 * cols, 3.8 * rows),
                             squeeze=False)
    fig.patch.set_facecolor(SURFACE)
    for ax, run in zip(axes.ravel(), runs):
        m = run['meta']
        ax.set_facecolor(SURFACE)
        final_gt = run['gt_traj'][-1, 1:8] if run['gt_traj'] is not None else None
        _draw_map(ax, run['belief'], extent, run['slam_traj'], final_gt, None,
                  f"{m['run']}\n{m['explored_voxels']:,} obs · "
                  f"{(m['coverage'] or 0):.0%} placed", 'Blues_r')
        ax.set_xlabel('')
        for txt in ax.texts:
            txt.set_visible(False)
    for ax in axes.ravel()[len(runs):]:
        ax.set_visible(False)
    fig.suptitle('Final belief map of every run', fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=180, bbox_inches='tight', facecolor=SURFACE)
    plt.close(fig)
    return out_path


def interactive(run):
    """Open the bundled map in a rotatable 3-D view."""
    matplotlib.use('TkAgg', force=True)
    import matplotlib.pyplot as ipl
    fig = ipl.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    b = run['belief'][::max(len(run['belief']) // 60000, 1)]
    ax.scatter(b[:, 1], b[:, 0], -b[:, 2], s=0.4, c=-b[:, 2], cmap='Blues_r')
    if run['gt_traj'] is not None:
        t = run['gt_traj']
        ax.plot(t[:, 2], t[:, 1], -t[:, 3], color='k', lw=1.2)
    ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)'); ax.set_zlabel('Up (m)')
    ax.set_title(run['meta']['run'])
    ipl.show()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run_dirs', nargs='+', help='run directories (globs are fine)')
    p.add_argument('--gallery', metavar='PATH', nargs='?',
                   const='auto', default=None,
                   help='also write one figure with every run side by side')
    p.add_argument('--interactive', action='store_true',
                   help='open the first run in a rotatable 3-D view')
    p.add_argument('--no-npz', action='store_true', help='render only')
    args = p.parse_args()

    dirs = []
    for pattern in args.run_dirs:
        matches = sorted(glob.glob(pattern)) if any(c in pattern for c in '*?[') \
            else [pattern]
        dirs.extend(d for d in matches if os.path.isdir(d))
    if not dirs:
        sys.exit('no run directories matched')

    runs = []
    for d in dirs:
        run = collect(d)
        if run is None:
            print(f'  [skip] {d}: no saved maps')
            continue
        runs.append(run)
        if not args.no_npz:
            write_npz(run)
        render(run)
        m = run['meta']
        print(f"  {m['run']:18} observed {m['explored_voxels']:7,} vox · "
              f"coverage {(m['coverage'] or 0):.3f} · correctly placed "
              f"{(m['correctly_placed_voxels'] or 0):7,}")

    if args.gallery and runs:
        path = (os.path.join(os.path.dirname(runs[0]['dir'].rstrip('/')),
                             'map_gallery.png')
                if args.gallery == 'auto' else args.gallery)
        print(f'  wrote {gallery(runs, path)}')
    if args.interactive and runs:
        interactive(runs[0])


if __name__ == '__main__':
    main()
