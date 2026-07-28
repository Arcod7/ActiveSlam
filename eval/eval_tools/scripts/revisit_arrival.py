#!/usr/bin/env python3
"""Did the vehicle actually reach the keyframe it was sent back to?

metrics.csv records that a revisit happened, not where it was going, so arrival
cannot be judged from it. The bag does carry both: `/frontier_slam/goal` is the
target the planner is driving to, and `/StoneFish/Odometry` is ground-truth
pose. During a `revisiting` episode the goal IS the revisit target, so the
distance between them over time answers it directly -- offline, with no change
to the running system.

This is the measurement that decides whether geodesic target selection is worth
building: reaching the target reliably means Euclidean selection is adequate.

Usage: python3 revisit_arrival.py RUNDIR [RUNDIR ...] [--arrival-radius 2.5]
"""
import argparse
import csv
import glob
import os
import subprocess
import sys

import numpy as np


def ensure_readable(bag_dir):
    """Return a directory holding a plain, indexed mcap, or None.

    Two shapes exist in the run archive. A clean recorder shutdown leaves
    `bag_0.mcap.zstd` plus metadata; an interrupted one leaves a bare
    `bag_0.mcap` with no metadata. SequentialReader handles neither directly:
    it cannot decompress file-level zstd, and it needs metadata. So decompress
    into a scratch copy when required, then reindex.
    """
    plain = glob.glob(f'{bag_dir}/*.mcap')
    if plain and os.path.exists(f'{bag_dir}/metadata.yaml'):
        return bag_dir
    if plain:
        subprocess.run(['ros2', 'bag', 'reindex', bag_dir],
                       capture_output=True, text=True)
        return bag_dir if os.path.exists(f'{bag_dir}/metadata.yaml') else None

    comp = glob.glob(f'{bag_dir}/*.mcap.zstd')
    if not comp:
        return None
    import zstandard
    scratch = os.path.join('/tmp/revisit_arrival_cache',
                           bag_dir.strip('/').replace('/', '_'))
    out = os.path.join(scratch, os.path.basename(comp[0])[:-5])
    if not os.path.exists(f'{scratch}/metadata.yaml'):
        os.makedirs(scratch, exist_ok=True)
        with open(comp[0], 'rb') as fi, open(out, 'wb') as fo:
            zstandard.ZstdDecompressor().copy_stream(fi, fo)
        subprocess.run(['ros2', 'bag', 'reindex', scratch],
                       capture_output=True, text=True)
    return scratch if os.path.exists(f'{scratch}/metadata.yaml') else None


def read_bag(bag_dir, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    # Leave storage_id empty so metadata.yaml drives it: batches differ in
    # whether the recorder finished its zstd pass, and forcing 'mcap' makes the
    # reader try to parse a compressed file as raw mcap.
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = {t for t in topics if t in types}
    if not want:
        return {}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(want)))
    out = {t: [] for t in want}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in want:
            msg = deserialize_message(data, get_message(types[topic]))
            out[topic].append((stamp * 1e-9, msg))
    return out


def episodes_from_metrics(run_dir):
    p = f'{run_dir}/metrics.csv'
    if not os.path.exists(p):
        return [], 0.0
    rows = list(csv.DictReader(open(p)))
    t = np.array([float(r['t']) for r in rows])
    st = [r.get('revisit_state', '') for r in rows]
    ex = [r.get('revisit_last_exit', '') for r in rows]
    eps, s = [], None
    for i, v in enumerate(st):
        if v == 'revisiting' and s is None:
            s = i
        elif v != 'revisiting' and s is not None:
            eps.append((t[s], t[i - 1], ex[min(i, len(ex) - 1)]))
            s = None
    return [e for e in eps if e[1] - e[0] > 5.0], t[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rundirs', nargs='+')
    ap.add_argument('--arrival-radius', type=float, default=2.5)
    a = ap.parse_args()

    print(f'{"run":24}{"ep":>3}{"dur":>5}{"d_start":>9}{"d_min":>8}{"d_end":>8}'
          f'{"reached":>9}  exit')
    print('-' * 78)
    stats = []
    for rd in a.rundirs:
        for run in sorted(glob.glob(f'{rd}/*_s*')):
            eps, _ = episodes_from_metrics(run)
            if not eps:
                continue
            bag = ensure_readable(f'{run}/bag')
            if bag is None:
                print(f'{os.path.basename(run):24}  (no readable bag)')
                continue
            try:
                data = read_bag(bag, ['/frontier_slam/goal', '/StoneFish/Odometry'])
            except Exception as exc:
                print(f'{os.path.basename(run):24}  (bag read failed: {exc})')
                continue
            goals = data.get('/frontier_slam/goal', [])
            odom = data.get('/StoneFish/Odometry', [])
            if not goals or not odom:
                print(f'{os.path.basename(run):24}  (missing goal or odometry)')
                continue

            ot = np.array([x[0] for x in odom])
            oxy = np.array([[x[1].pose.pose.position.x, x[1].pose.pose.position.y]
                            for x in odom])
            gt_ = np.array([x[0] for x in goals])
            gxy = np.array([[x[1].point.x, x[1].point.y] for x in goals])

            for j, (t0, t1, ex) in enumerate(eps):
                sel = (ot >= t0) & (ot <= t1)
                if sel.sum() < 5:
                    continue
                tt, pp = ot[sel], oxy[sel]
                gi = np.searchsorted(gt_, tt) - 1
                gi = np.clip(gi, 0, len(gxy) - 1)
                d = np.linalg.norm(pp - gxy[gi], axis=1)
                reached = bool(np.min(d) <= a.arrival_radius)
                stats.append((reached, float(np.min(d)), ex))
                print(f'{os.path.basename(run):24}{j:3d}{t1-t0:5.0f}'
                      f'{d[0]:9.1f}{d.min():8.1f}{d[-1]:8.1f}'
                      f'{"YES" if reached else "no":>9}  {ex}')

    if not stats:
        print('\nno episodes measured')
        return
    n = len(stats)
    hit = sum(s[0] for s in stats)
    print('-' * 78)
    print(f'\n  {hit}/{n} episodes brought the vehicle within '
          f'{a.arrival_radius} m of its target ({100*hit/n:.0f}%)')
    print(f'  closest approach: median {np.median([s[1] for s in stats]):.1f} m, '
          f'worst {max(s[1] for s in stats):.1f} m')
    print('\n  High reach rate  -> Euclidean target selection is adequate; the')
    print('                      geodesic/Dijkstra rework is not the bottleneck.')
    print('  Low reach rate   -> targets are being chosen that cannot be reached,')
    print('                      which is exactly what geodesic selection fixes.')


if __name__ == '__main__':
    main()
