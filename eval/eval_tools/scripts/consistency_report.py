#!/usr/bin/env python3
"""Is the reported uncertainty an honest description of the actual error?

ATE says how wrong the estimate is. This says whether the estimator knew.

Sampling design. Consecutive keyframes inside one run are not independent
samples of the estimator's consistency: the DVL scale and velocity-bias terms
are each ONE draw for the whole run, so a run's whole error trace is dominated
by that single realisation. Pooling every keyframe and testing against a
chi-square band for n = total keyframes therefore uses an effective sample size
far smaller than n claims, and reads "inconsistent" whatever the model does.
That is also why per-run ANEES on a fixed configuration scatters over an order
of magnitude between seeds.

So the primary statistic here takes ONE sample per run per milestone --
milestones being fixed distances along the ground-truth path -- and pools those
across seeds. Samples at a given milestone are independent by construction (one
per seed), so the chi-square band for n = number of seeds is legitimate, and the
result is a calibration curve against distance travelled rather than a single
number.

Reads keyframe_consistency.csv (exact: full XYH marginal per keyframe) when a
run has one, and falls back to metrics.csv + the TUM trajectories (approximate:
isotropic sigma_xy) for runs recorded before that file existed.
"""
import argparse
import csv
import glob
import json
import os

import numpy as np
from scipy.stats import chi2

XY_DOF, YAW_DOF, XYH_DOF = 2, 1, 3


def _finite(rows, key):
    out = []
    for r in rows:
        v = r.get(key, '')
        if v in ('', 'nan', 'None', None):
            out.append(np.nan)
        else:
            try:
                out.append(float(v))
            except ValueError:
                out.append(np.nan)
    return np.asarray(out)


def yaw_of(quat):
    x, y, z, w = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def load_tum(path):
    a = np.loadtxt(path)
    if a.ndim == 1:
        a = a[None, :]
    return a[:, 0], a[:, 1:4], a[:, 4:8]


class Run:
    """One evaluation run, resampled onto the ground-truth timeline."""

    def __init__(self, run_dir):
        self.dir = run_dir
        self.name = os.path.basename(run_dir.rstrip('/'))
        self.seed = None
        manifest = os.path.join(run_dir, 'manifest.json')
        if os.path.exists(manifest):
            m = json.load(open(manifest))
            self.seed = m.get('seed')
            self.args = m.get('args', {})
        else:
            self.args = {}

        tg, pg, qg = load_tum(os.path.join(run_dir, 'gt_traj.tum'))
        ts, ps, qs = load_tum(os.path.join(run_dir, 'slam_traj.tum'))
        idx = np.searchsorted(tg, ts).clip(0, len(tg) - 1)

        self.t = ts - ts[0]
        self.err_xy = np.linalg.norm(pg[idx, :2] - ps[:, :2], axis=1)
        self.err_yaw = np.abs(wrap(yaw_of(qg[idx]) - yaw_of(qs)))
        # GT path length at each estimate timestamp: the x-axis every growth
        # law is stated against, and the only one comparable across scenes.
        step = np.r_[0.0, np.linalg.norm(np.diff(pg, axis=0), axis=1)]
        self.dist = np.cumsum(step)[idx]

        rows = list(csv.DictReader(open(os.path.join(run_dir, 'metrics.csv'))))
        mt = _finite(rows, 't')
        mt = mt - mt[0]
        for col in ('sigma_xy', 'sigma_yaw', 'dopt', 'u_ratio', 'abs_error',
                    'anees_robust', 'anees', 'lc_count'):
            series = _finite(rows, col)
            j = np.searchsorted(mt, self.t).clip(0, len(mt) - 1)
            setattr(self, col, series[j])

        self.exact = self._load_exact(run_dir, tg, pg, qg)

    def _load_exact(self, run_dir, tg, pg, qg):
        """Per-keyframe error vector and XYH marginal, when recorded."""
        path = os.path.join(run_dir, 'keyframe_consistency.csv')
        if not os.path.exists(path):
            return None
        rows = list(csv.DictReader(open(path)))
        if not rows:
            return None
        t = _finite(rows, 't')
        err = np.column_stack([_finite(rows, k)
                               for k in ('err_x', 'err_y', 'err_yaw')])
        cov = np.zeros((len(rows), 3, 3))
        for a, b, key in ((0, 0, 'cov_xx'), (0, 1, 'cov_xy'), (0, 2, 'cov_xh'),
                          (1, 1, 'cov_yy'), (1, 2, 'cov_yh'), (2, 2, 'cov_hh')):
            cov[:, a, b] = cov[:, b, a] = _finite(rows, key)
        step = np.r_[0.0, np.linalg.norm(np.diff(pg, axis=0), axis=1)]
        cum = np.cumsum(step)
        j = np.searchsorted(tg, t).clip(0, len(tg) - 1)
        return {'t': t - tg[0], 'err': err, 'cov': cov, 'dist': cum[j]}

    def at_distance(self, s):
        """Index of the sample nearest s metres of GT path, or None if the run
        never got that far."""
        if s > self.dist[-1]:
            return None
        return int(np.argmin(np.abs(self.dist - s)))

    def exact_at_distance(self, s):
        e = self.exact
        if e is None or len(e['dist']) == 0 or s > e['dist'][-1]:
            return None
        return int(np.argmin(np.abs(e['dist'] - s)))


def nees_split(err, cov):
    """(xy, yaw, xyh) normalised squared errors from one 3-DoF sample."""
    out = []
    for sl, dof in ((slice(0, 2), XY_DOF), (slice(2, 3), YAW_DOF),
                    (slice(0, 3), XYH_DOF)):
        block, e = cov[sl, sl], err[sl]
        try:
            out.append(float(e @ np.linalg.solve(block, e)))
        except np.linalg.LinAlgError:
            out.append(np.nan)
    return tuple(out)


def consistency_verdict(samples, dof):
    """Pooled ANEES over independent samples, with its chi-square band."""
    s = np.asarray([v for v in samples if np.isfinite(v)])
    n = len(s)
    if n < 2:
        return None
    anees = float(np.mean(s))
    lo = chi2.ppf(0.025, dof * n) / n
    hi = chi2.ppf(0.975, dof * n) / n
    if anees > hi:
        verdict = f'OVERCONFIDENT {np.sqrt(anees / dof):.2f}x'
    elif anees < lo:
        verdict = f'conservative {np.sqrt(anees / dof):.2f}x'
    else:
        verdict = 'consistent'
    return {'n': n, 'anees': anees, 'lo': lo, 'hi': hi, 'dof': dof,
            'verdict': verdict}


def milestone_table(runs, milestones):
    """One independent sample per run per milestone, pooled across seeds."""
    table = []
    for s in milestones:
        exact_xy, exact_yaw, exact_xyh = [], [], []
        approx_xy, approx_yaw = [], []
        errs, sigmas, ratios = [], [], []
        for run in runs:
            k = run.exact_at_distance(s)
            if k is not None:
                a, b, c = nees_split(run.exact['err'][k], run.exact['cov'][k])
                exact_xy.append(a)
                exact_yaw.append(b)
                exact_xyh.append(c)
            i = run.at_distance(s)
            if i is None:
                continue
            sx, sy = run.sigma_xy[i], run.sigma_yaw[i]
            if np.isfinite(sx) and sx > 0:
                approx_xy.append((run.err_xy[i] / sx) ** 2)
                errs.append(run.err_xy[i])
                sigmas.append(sx)
                ratios.append(run.err_xy[i] / sx)
            if np.isfinite(sy) and sy > 0:
                approx_yaw.append((run.err_yaw[i] / sy) ** 2)
        row = {'s': s, 'n': len(ratios),
               'err': float(np.median(errs)) if errs else np.nan,
               'sigma': float(np.median(sigmas)) if sigmas else np.nan,
               'ratio': float(np.median(ratios)) if ratios else np.nan,
               'xy': consistency_verdict(exact_xy or approx_xy, XY_DOF),
               'yaw': consistency_verdict(exact_yaw or approx_yaw, YAW_DOF),
               'xyh': consistency_verdict(exact_xyh, XYH_DOF) if exact_xyh else None,
               'exact': bool(exact_xy)}
        table.append(row)
    return table


def growth_fit(runs, milestones):
    """Least-squares exponent p in err ~ a * s^p, pooled over runs.

    p ~ 0.5 is a random walk, p ~ 1.0 is a coherent bias or scale error. The
    same fit on sigma says which of the two the model believes it is -- the
    single most diagnostic comparison in this report.
    """
    def fit(getter):
        xs, ys = [], []
        for run in runs:
            for s in milestones:
                i = run.at_distance(s)
                if i is None:
                    continue
                v = getter(run, i)
                if np.isfinite(v) and v > 0 and s > 0:
                    xs.append(np.log(s))
                    ys.append(np.log(v))
        if len(xs) < 3:
            return None, None
        p, c = np.polyfit(xs, ys, 1)
        return float(p), float(np.exp(c))

    p_err, a_err = fit(lambda r, i: r.err_xy[i])
    p_sig, a_sig = fit(lambda r, i: r.sigma_xy[i])
    return {'err': (p_err, a_err), 'sigma': (p_sig, a_sig)}


def report(runs, milestones, out_png=None):
    print(f'\n{len(runs)} runs\n')
    print(f'{"run":30s}{"seed":>6}{"path_m":>9}{"err_end":>9}'
          f'{"sig_end":>9}{"ratio":>7}{"ANEES_f":>9}{"kf":>5}')
    for run in runs:
        n_kf = len(run.exact['t']) if run.exact else 0
        ratio = run.err_xy[-1] / run.sigma_xy[-1] if run.sigma_xy[-1] > 0 else np.nan
        print(f'{run.name[:30]:30s}{str(run.seed):>6}{run.dist[-1]:9.1f}'
              f'{run.err_xy[-1]:9.3f}{run.sigma_xy[-1]:9.3f}{ratio:7.2f}'
              f'{run.anees_robust[-1]:9.2f}{n_kf:5d}')

    table = milestone_table(runs, milestones)
    kind = 'exact' if any(r['exact'] for r in table) else 'approx (isotropic)'
    print(f'\nPer-milestone consistency, one sample per run [{kind}]')
    print(f'{"dist_m":>7}{"n":>4}{"err_med":>9}{"sig_med":>9}{"e/s":>6}'
          f'   {"XY":34s}{"YAW":24s}')
    for r in table:
        if r['n'] == 0:
            continue
        def fmt(v, width):
            if v is None:
                return ' ' * width
            return f'{v["anees"]:6.2f} [{v["lo"]:.2f},{v["hi"]:.2f}] {v["verdict"]}'.ljust(width)
        print(f'{r["s"]:7.0f}{r["n"]:4d}{r["err"]:9.3f}{r["sigma"]:9.3f}'
              f'{r["ratio"]:6.2f}   {fmt(r["xy"], 34)}{fmt(r["yaw"], 24)}')

    g = growth_fit(runs, milestones)
    print(f'\nGrowth with distance   err ~ {g["err"][1]:.4f} * s^{g["err"][0]:.2f}'
          f'   |   sigma ~ {g["sigma"][1]:.4f} * s^{g["sigma"][0]:.2f}')
    print('  p~0.5 random walk, p~1.0 coherent bias/scale. A gap between the '
          'two exponents is a\n  structural error in the model, not a '
          'mis-scaled constant.')

    if out_png:
        plot(runs, table, out_png)
        print(f'\nwrote {out_png}')
    return table


def plot(runs, table, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    ax = axes[0]
    for run in runs:
        ax.plot(run.dist, run.err_xy, lw=1.0, alpha=0.8)
        ax.plot(run.dist, run.sigma_xy, lw=1.0, ls='--', alpha=0.8,
                color=ax.lines[-1].get_color())
    ax.set_xlabel('GT path length (m)')
    ax.set_ylabel('metres')
    ax.set_title('solid: |error_xy|   dashed: reported sigma_xy')
    ax.grid(alpha=0.3)

    ax = axes[1]
    for run in runs:
        ax.scatter(run.sigma_xy, run.err_xy, s=2, alpha=0.25)
    lim = max(ax.get_xlim()[1], ax.get_ylim()[1])
    ax.plot([0, lim], [0, lim], 'k-', lw=1, label='perfectly calibrated')
    ax.set_xlabel('reported sigma_xy (m)')
    ax.set_ylabel('|error_xy| (m)')
    ax.set_title('calibration')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[2]
    xs = [r['s'] for r in table if r['xy']]
    ys = [r['xy']['anees'] for r in table if r['xy']]
    lo = [r['xy']['lo'] for r in table if r['xy']]
    hi = [r['xy']['hi'] for r in table if r['xy']]
    if xs:
        ax.fill_between(xs, lo, hi, alpha=0.2, label='95% consistent band')
        ax.plot(xs, ys, 'o-', label='pooled ANEES (XY)')
        ax.axhline(XY_DOF, color='k', lw=1, ls=':')
        ax.set_yscale('log')
    ax.set_xlabel('GT path length (m)')
    ax.set_ylabel('ANEES (XY, 2 DoF)')
    ax.set_title('consistency vs distance travelled')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=130)


def collect(paths):
    runs = []
    for p in paths:
        candidates = ([p] if os.path.exists(os.path.join(p, 'metrics.csv'))
                      else sorted(glob.glob(os.path.join(p, '*', 'metrics.csv'))))
        for c in candidates:
            d = c if os.path.isdir(c) else os.path.dirname(c)
            try:
                runs.append(Run(d))
            except (OSError, ValueError, IndexError) as exc:
                print(f'skipped {d}: {exc}')
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='run dirs or batch dirs')
    ap.add_argument('--milestones', default='',
                    help='comma-separated GT path lengths in metres')
    ap.add_argument('--png', default='')
    args = ap.parse_args()

    runs = collect(args.paths)
    if not runs:
        raise SystemExit('no runs found')
    if args.milestones:
        milestones = [float(x) for x in args.milestones.split(',')]
    else:
        reach = min(r.dist[-1] for r in runs)
        milestones = list(np.arange(10.0, reach + 1e-6, 10.0))
    report(runs, milestones, args.png or None)


if __name__ == '__main__':
    main()
