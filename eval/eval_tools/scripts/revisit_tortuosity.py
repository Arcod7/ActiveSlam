#!/usr/bin/env python3
"""Does the revisit selector pick targets whose true swim exceeds their score?

`select_revisit_target` ranks candidates on straight-line distance. The vehicle
swims the A* route. Where the hull sits between the two, the score flatters the
candidate and the transit budget is spent without arriving.

Tortuosity = (A* path length) / (straight-line distance) at the moment the
episode begins. If unreached episodes carry systematically higher tortuosity
than reached ones, scoring on geodesic distance is the indicated fix.

Usage: python3 revisit_tortuosity.py RUNDIR [RUNDIR ...]
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from revisit_arrival import ensure_readable, read_bag, episodes_from_metrics

ARRIVAL_RADIUS = 2.5


def main():
    rows = []
    for rd in sys.argv[1:]:
        for run in sorted(glob.glob(f'{rd}/*_s*')):
            eps, _ = episodes_from_metrics(run)
            if not eps:
                continue
            bag = ensure_readable(f'{run}/bag')
            if bag is None:
                continue
            d = read_bag(bag, ['/frontier_slam/goal', '/frontier_slam/path',
                               '/StoneFish/Odometry'])
            goals = d.get('/frontier_slam/goal', [])
            paths = d.get('/frontier_slam/path', [])
            odom = d.get('/StoneFish/Odometry', [])
            if not (goals and paths and odom):
                continue
            gt_ = np.array([x[0] for x in goals])
            gxy = np.array([[x[1].point.x, x[1].point.y] for x in goals])
            pt = np.array([x[0] for x in paths])
            ot = np.array([x[0] for x in odom])
            oxy = np.array([[x[1].pose.pose.position.x, x[1].pose.pose.position.y]
                            for x in odom])

            for j, (t0, t1, ex) in enumerate(eps):
                pm = np.where((pt >= t0) & (pt <= t1))[0]
                if not len(pm):
                    continue
                w = [[p.pose.position.x, p.pose.position.y]
                     for p in paths[pm[0]][1].poses]
                if len(w) < 2:
                    continue
                w = np.array(w)
                plen = float(np.sum(np.linalg.norm(np.diff(w, axis=0), axis=1)))
                k = int(np.argmin(np.abs(ot - t0)))
                gi = int(np.clip(np.searchsorted(gt_, t0) - 1, 0, len(gxy) - 1))
                euc = float(np.linalg.norm(oxy[k] - gxy[gi]))
                sel = (ot >= t0) & (ot <= t1)
                dmin = float(np.min(np.linalg.norm(oxy[sel] - gxy[gi], axis=1)))
                rows.append((f'{os.path.basename(run)} ep{j}', euc, plen,
                             plen / max(euc, 1e-6), dmin, dmin <= ARRIVAL_RADIUS))

    if not rows:
        print('no episodes measured')
        return
    print(f'{"episode":30}{"euclid":>8}{"pathlen":>9}{"tortuos":>9}'
          f'{"closest":>9}  reached')
    print('-' * 74)
    for r in sorted(rows, key=lambda r: r[3]):
        print(f'{r[0]:30}{r[1]:8.1f}{r[2]:9.1f}{r[3]:9.2f}{r[4]:9.1f}'
              f'  {"YES" if r[5] else "no"}')

    a = np.array([[r[1], r[2], r[3], r[4]] for r in rows])
    ok = np.array([r[5] for r in rows])
    print(f'\n  reached     (n={int(ok.sum())}): euclid {a[ok,0].mean():5.1f} m, '
          f'path {a[ok,1].mean():5.1f} m, tortuosity {a[ok,2].mean():.2f}')
    print(f'  not reached (n={int((~ok).sum())}): euclid {a[~ok,0].mean():5.1f} m, '
          f'path {a[~ok,1].mean():5.1f} m, tortuosity {a[~ok,2].mean():.2f}')

    # Rank-biserial via Mann-Whitney on tortuosity: does it separate the groups?
    x, y = a[ok, 2], a[~ok, 2]
    allv = np.concatenate([x, y])
    r = np.argsort(np.argsort(allv)) + 1.0
    U = r[:len(x)].sum() - len(x) * (len(x) + 1) / 2
    if len(x) and len(y):
        auc = U / (len(x) * len(y))
        print(f'\n  P(reached episode has LOWER tortuosity than an unreached one) '
              f'= {1-auc:.2f}   (0.5 = no separation)')
    for name, col in (('euclidean distance', 0), ('true path length', 1),
                      ('tortuosity', 2)):
        print(f'  {name:22} separates reached/unreached by '
              f'{a[~ok,col].mean()/a[ok,col].mean():.2f}x')
    print('\n  The selector scores on euclidean distance. Whichever column '
          'separates\n  best is what it should be scoring on.')


if __name__ == '__main__':
    main()
