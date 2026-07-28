#!/usr/bin/env python3
"""Episode-level analysis of the revisit mechanism.

A run yields one drift number; a run yields 3-5 revisit episodes. Power analysis
(campaign ledger, night 1) showed the run-level tail claim needs n ~ 60 pairs,
which is out of reach, while the episode level multiplies effective n by 3-5x at
no extra compute. So RQ2 is answered in mechanism terms here.

Per episode, measured against the same run-relative window in a matched
no-revisit run of the same seed:

    reached       did the vehicle get within arrival_radius of its target
    closures      constraints harvested between entry and exit
    d_err         change in absolute error, minus the control's change
    d_ur          change in the uncertainty ratio the trigger reads
    exit          why the state machine left

Usage:
    python3 analyse_episodes.py RUNDIR [RUNDIR ...] [--control-arm lc]
"""
import argparse
import csv
import glob
import os

import numpy as np

W = 20.0          # seconds averaged either side of an episode boundary
MIN_EPISODE_S = 5.0


def load(d):
    p = f'{d}/metrics.csv'
    if not os.path.exists(p):
        return None
    rows = list(csv.DictReader(open(p)))
    if len(rows) < 50:
        return None

    def c(k):
        return np.array([float(r[k]) if r.get(k) not in (None, '', 'nan') else np.nan
                         for r in rows])
    t = c('t')
    arm, seed = os.path.basename(d).rsplit('_s', 1)
    return dict(arm=arm, seed=int(seed), name=os.path.basename(d),
                batch=os.path.basename(os.path.dirname(d)),
                t=t - t[0], err=c('abs_error'), ur=c('u_ratio'),
                lc=c('lc_count'), kf=c('kf_count'), sig=c('sigma_xy'),
                state=[r.get('revisit_state', '') for r in rows],
                exits=[r.get('revisit_last_exit', '') for r in rows])


def episodes(r):
    out, s = [], None
    for i, v in enumerate(r['state']):
        if v == 'revisiting' and s is None:
            s = i
        elif v != 'revisiting' and s is not None:
            out.append((s, i - 1))
            s = None
    if s is not None:
        out.append((s, len(r['state']) - 1))
    return [(a, b) for a, b in out if r['t'][b] - r['t'][a] > MIN_EPISODE_S]


def win(r, key, t_c, lo, hi):
    sel = (r['t'] >= t_c + lo) & (r['t'] <= t_c + hi)
    v = r[key][sel]
    v = v[np.isfinite(v)]
    return float(np.mean(v)) if len(v) else np.nan


def collect(runs, controls):
    eps = []
    for r in runs:
        ctl = controls.get(r['seed'])
        for j, (a, b) in enumerate(episodes(r)):
            t_in, t_out = r['t'][a], r['t'][b]
            d_err = win(r, 'err', t_out, 0, W) - win(r, 'err', t_in, -W, 0)
            c_err = (win(ctl, 'err', t_out, 0, W) - win(ctl, 'err', t_in, -W, 0)
                     if ctl is not None else np.nan)
            eps.append(dict(
                arm=r['arm'], run=r['name'], seed=r['seed'], idx=j,
                t_in=t_in, dur=t_out - t_in,
                closures=float(r['lc'][b] - r['lc'][a]),
                keyframes=float(r['kf'][b] - r['kf'][a]),
                d_err=d_err, net_err=d_err - c_err,
                ur_in=float(r['ur'][a]) if np.isfinite(r['ur'][a]) else np.nan,
                d_ur=win(r, 'ur', t_out, 0, W) - win(r, 'ur', t_in, -W, 0),
                exit=r['exits'][min(b + 1, len(r['exits']) - 1)] or 'NONE'))
    return eps


def boot_ci(v, n=10000, seed=0):
    v = np.asarray([x for x in v if np.isfinite(x)], float)
    if len(v) < 3:
        return (np.nan, np.nan)
    rng = np.random.default_rng(seed)
    m = [np.mean(rng.choice(v, len(v), replace=True)) for _ in range(n)]
    return tuple(np.percentile(m, [2.5, 97.5]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rundirs', nargs='+')
    ap.add_argument('--control-arm', default='lc')
    a = ap.parse_args()

    runs, controls = [], {}
    for rd in a.rundirs:
        for d in sorted(glob.glob(f'{rd}/*_s*')):
            r = load(d)
            if not r:
                continue
            if r['arm'] == a.control_arm:
                controls.setdefault(r['seed'], r)
            runs.append(r)

    rev = [r for r in runs if episodes(r)]
    eps = collect(rev, controls)
    if not eps:
        print('no revisit episodes found')
        return

    print(f'{len(eps)} episodes across {len({e["run"] for e in eps})} runs, '
          f'control arm = {a.control_arm} (n={len(controls)} seeds)\n')
    print(f'{"run":26}{"ep":>3}{"t_in":>6}{"dur":>5}{"LC":>4}{"KF":>5}'
          f'{"U_r in":>8}{"dU_r":>7}{"d_err":>8}{"net":>8}  exit')
    print('-' * 96)
    for e in sorted(eps, key=lambda e: (e['arm'], e['seed'], e['idx'])):
        print(f'{e["run"]:26}{e["idx"]:3d}{e["t_in"]:6.0f}{e["dur"]:5.0f}'
              f'{e["closures"]:4.0f}{e["keyframes"]:5.0f}{e["ur_in"]:8.2f}'
              f'{e["d_ur"]:+7.2f}{e["d_err"]:+8.3f}{e["net_err"]:+8.3f}  {e["exit"]}')

    print('\n' + '=' * 96)
    print(f'{"arm":26}{"eps":>5}{"reached*":>10}{"closures/ep":>13}'
          f'{"dU_r":>9}{"net d_err":>22}')
    print('-' * 96)
    for arm in sorted({e['arm'] for e in eps}):
        g = [e for e in eps if e['arm'] == arm]
        net = [e['net_err'] for e in g if np.isfinite(e['net_err'])]
        lo, hi = boot_ci(net)
        reached = sum(e['exit'] in ('ARRIVED_STERILE', 'ARRIVED_SATURATED',
                                    'RESUMED_DOPT') for e in g)
        print(f'{arm:26}{len(g):5d}{reached:6d}/{len(g):<3d}'
              f'{np.mean([e["closures"] for e in g]):13.1f}'
              f'{np.nanmean([e["d_ur"] for e in g]):+9.2f}'
              f'{np.mean(net) if net else np.nan:+10.3f} m  [{lo:+.3f}, {hi:+.3f}]')
    print('\n  * exits implying the vehicle actually got to its target. A `CLOSED`')
    print('    exit cannot make that claim: min_closures is tested before arrival.')

    arms = sorted({e['arm'] for e in eps})
    if len(arms) == 2:
        x = [e['closures'] for e in eps if e['arm'] == arms[0]]
        y = [e['closures'] for e in eps if e['arm'] == arms[1]]
        try:
            from scipy import stats
            u = stats.mannwhitneyu(x, y, alternative='two-sided')
            print(f'\n  closures per episode, {arms[0]} vs {arms[1]}: '
                  f'{np.mean(x):.1f} vs {np.mean(y):.1f}, '
                  f'Mann-Whitney p = {u.pvalue:.4f}  (n={len(x)} vs {len(y)} episodes)')
            nx = [e['net_err'] for e in eps if e['arm'] == arms[0] and np.isfinite(e['net_err'])]
            ny = [e['net_err'] for e in eps if e['arm'] == arms[1] and np.isfinite(e['net_err'])]
            if nx and ny:
                u2 = stats.mannwhitneyu(nx, ny, alternative='two-sided')
                print(f'  net error change per episode: {np.mean(nx):+.3f} vs '
                      f'{np.mean(ny):+.3f} m, Mann-Whitney p = {u2.pvalue:.4f}')
        except Exception as e:
            print(f'  (scipy unavailable: {e})')

    ex = {}
    for e in eps:
        ex.setdefault(e['arm'], {}).setdefault(e['exit'], 0)
        ex[e['arm']][e['exit']] += 1
    print('\n  exit reasons:')
    for arm, d in sorted(ex.items()):
        print(f'    {arm:26}{d}')


if __name__ == '__main__':
    main()
