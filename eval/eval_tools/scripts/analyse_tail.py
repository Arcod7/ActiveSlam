#!/usr/bin/env python3
"""Pool runs across batches and apply the pre-registered tail analysis.

The claim under test is that active revisit reduces the *spread* and *upper
tail* of drift rate rather than its mean, so the tests here are dispersion and
tail tests, with the location test reported alongside for completeness.

Usage:
    python3 analyse_tail.py RUNDIR [RUNDIR ...] [--out DIR]

Primary endpoint is drift rate = ATE / path_length * 100 (%/m); raw ATE is not
comparable across arms whose path lengths differ.
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

BAD_DRIFT = 0.15        # %/m, fixed in the pre-registration
BAD_ANEES = 30.0


def _col(path, key):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for row in csv.DictReader(f):
            v = row.get(key)
            if v not in (None, '', 'nan'):
                try:
                    out.append(float(v))
                except ValueError:
                    pass
    return out


def _last(path, key, default=np.nan):
    v = _col(path, key)
    return v[-1] if v else default


def load_run(d):
    mpath, mm = f'{d}/metrics.csv', f'{d}/map_metrics.csv'
    if not os.path.exists(f'{d}/manifest.json') or not os.path.exists(mpath):
        return None
    mf = json.load(open(f'{d}/manifest.json'))
    name = os.path.basename(d)
    arm, seed = name.rsplit('_s', 1)
    path_m = float(mf.get('gt_path_m') or np.nan)
    ate = _last(mpath, 'ate')
    an = _col(mpath, 'anees_robust') or _col(mpath, 'anees')
    states = []
    with open(mpath) as f:
        states = [r.get('revisit_state', '') for r in csv.DictReader(f)]
    n_rev_ticks = sum(s == 'revisiting' for s in states)
    return dict(
        run=name, arm=arm, seed=int(seed), batch=os.path.basename(os.path.dirname(d)),
        path_m=path_m, ate=ate,
        drift=100.0 * ate / path_m if path_m and np.isfinite(ate) else np.nan,
        anees=float(np.median(an)) if an else np.nan,
        sigma=float(np.median(_col(mpath, 'sigma_xy') or [np.nan])),
        lc=_last(mpath, 'lc_count'), kf=_last(mpath, 'kf_count'),
        revisits=_last(mpath, 'revisit_count', 0.0),
        rev_frac=n_rev_ticks / len(states) if states else np.nan,
        coverage=_last(mm, 'coverage'), chamfer=_last(mm, 'chamfer'),
        dvl=mf.get('dvl_scale_error_pct'),
    )


# ---------------------------------------------------------------- statistics
def levene(groups):
    """Brown-Forsythe (median-centred Levene). Returns (W, p) or (nan, nan)."""
    try:
        from scipy import stats
        return stats.levene(*groups, center='median')
    except Exception:
        return (np.nan, np.nan)


def wilcoxon_paired(a, b):
    try:
        from scipy import stats
        return stats.wilcoxon(a, b).pvalue
    except Exception:
        return np.nan


def spearman(a, b):
    try:
        from scipy import stats
        r = stats.spearmanr(a, b)
        return r.statistic, r.pvalue
    except Exception:
        return (np.nan, np.nan)


def mcnemar_exact(b, c):
    """Exact binomial two-sided McNemar on discordant counts b and c."""
    n = b + c
    if n == 0:
        return 1.0
    try:
        from scipy import stats
        return float(min(1.0, 2.0 * stats.binom.cdf(min(b, c), n, 0.5)))
    except Exception:
        return np.nan


def perm_p75(x, y, n_perm=10000, seed=0):
    """One-sided permutation test: is p75(y) < p75(x)?"""
    rng = np.random.default_rng(seed)
    obs = np.percentile(x, 75) - np.percentile(y, 75)
    pool = np.concatenate([x, y])
    nx = len(x)
    cnt = 0
    for _ in range(n_perm):
        rng.shuffle(pool)
        if np.percentile(pool[:nx], 75) - np.percentile(pool[nx:], 75) >= obs:
            cnt += 1
    return obs, (cnt + 1) / (n_perm + 1)


def summarise(rows, arms=None):
    arms = arms or sorted({r['arm'] for r in rows})
    print(f'\n{"arm":24}{"n":>3}{"drift %/m":>22}{"max":>7}{"p75":>7}'
          f'{"ATE":>8}{"ANEES":>8}{"cov":>7}{"rev":>6}{"BAD":>6}')
    print('-' * 100)
    for a in arms:
        g = [r for r in rows if r['arm'] == a and np.isfinite(r['drift'])]
        if not g:
            continue
        d = np.array([r['drift'] for r in g])
        cov = np.array([r['coverage'] for r in g], float)
        print(f'{a:24}{len(g):3d}{np.mean(d):9.3f} +-{np.std(d, ddof=1) if len(d) > 1 else 0:6.3f}'
              f'{np.max(d):7.3f}{np.percentile(d, 75):7.3f}'
              f'{np.mean([r["ate"] for r in g]):8.3f}'
              f'{np.nanmean([r["anees"] for r in g]):8.1f}'
              f'{np.nanmean(cov):7.3f}'
              f'{np.mean([r["revisits"] for r in g]):6.1f}'
              f'{sum(x > BAD_DRIFT for x in d):4d}/{len(d):<2d}')
    return arms


def _by_seed(rows, arm):
    """seed -> drift, averaging replicates of the same (arm, seed).

    Two batches can both carry seed 101 for the same arm; keying a dict on seed
    alone silently drops one. Replicates are averaged and reported.
    """
    acc = {}
    for r in rows:
        if r['arm'] == arm and np.isfinite(r['drift']):
            acc.setdefault(r['seed'], []).append(r)
    dup = {s: len(v) for s, v in acc.items() if len(v) > 1}
    if dup:
        print(f'  note: {arm} has replicate runs per seed {dup}; averaging them')
    return {s: (v[0] if len(v) == 1 else
                {**v[0], **{k: float(np.nanmean([x[k] for x in v]))
                            for k in ('drift', 'ate', 'coverage', 'path_m', 'anees')}})
            for s, v in acc.items()}


def compare(rows, base, treat):
    """Pre-registered tests for one baseline/treatment contrast."""
    B = _by_seed(rows, base)
    T = _by_seed(rows, treat)
    seeds = sorted(set(B) & set(T))
    if len(seeds) < 3:
        print(f'\n  {base} vs {treat}: only {len(seeds)} paired seeds, skipping tests')
        return
    b = np.array([B[s]['drift'] for s in seeds])
    t = np.array([T[s]['drift'] for s in seeds])

    print(f'\n=== {base}  ->  {treat}   (n = {len(seeds)} paired seeds) ===')
    print(f'  {"seed":>6}{"base":>9}{"treat":>9}{"effect":>9}')
    for s, x, y in zip(seeds, b, t):
        print(f'  {s:6d}{x:9.3f}{y:9.3f}{100*(y-x)/x:+8.0f}%')

    sb, st = b.std(ddof=1), t.std(ddof=1)
    print(f'\n  H1 dispersion : SD {sb:.3f} -> {st:.3f} ({100*(st-sb)/sb:+.0f}%)   '
          f'range-ratio {b.max()/b.min():.1f}x -> {t.max()/t.min():.1f}x')
    W, p = levene(list(map(np.log, [b, t])))
    print(f'                  Brown-Forsythe on log drift: W = {W:.2f}, p = {p:.3f}')

    print(f'  H2 tail       : max {b.max():.3f} -> {t.max():.3f} ({100*(t.max()-b.max())/b.max():+.0f}%)   '
          f'p75 {np.percentile(b,75):.3f} -> {np.percentile(t,75):.3f}')
    obs, pp = perm_p75(b, t)
    print(f'                  permutation on p75 difference: p = {pp:.3f} (one-sided)')

    bb = sum(x > BAD_DRIFT for x in b)
    tb = sum(x > BAD_DRIFT for x in t)
    disc_bt = sum((x > BAD_DRIFT) and not (y > BAD_DRIFT) for x, y in zip(b, t))
    disc_tb = sum((y > BAD_DRIFT) and not (x > BAD_DRIFT) for x, y in zip(b, t))
    print(f'  H3 bad runs   : {bb}/{len(b)} -> {tb}/{len(t)}   '
          f'discordant {disc_bt} rescued vs {disc_tb} broken, '
          f'McNemar p = {mcnemar_exact(disc_bt, disc_tb):.3f}')

    print(f'  location      : mean {b.mean():.3f} -> {t.mean():.3f} '
          f'({100*(t.mean()-b.mean())/b.mean():+.0f}%), median '
          f'{np.median(b):.3f} -> {np.median(t):.3f}, '
          f'paired Wilcoxon p = {wilcoxon_paired(b, t):.3f}')

    slope = np.polyfit(b, t - b, 1)[0] if len(b) >= 3 else np.nan
    print(f'  H4 crossover  : slope of (treat-base) on base = {slope:+.2f}   '
          f'(needs the lc_repeat control to separate from regression to the mean)')

    cb = np.array([B[s]['coverage'] for s in seeds], float)
    ct = np.array([T[s]['coverage'] for s in seeds], float)
    pb = np.array([B[s]['path_m'] for s in seeds], float)
    pt = np.array([T[s]['path_m'] for s in seeds], float)
    print(f'  H5 cost       : coverage {np.nanmean(cb):.3f} -> {np.nanmean(ct):.3f} '
          f'({100*(np.nanmean(ct)-np.nanmean(cb))/np.nanmean(cb):+.1f}%),  '
          f'path {np.nanmean(pb):.0f} -> {np.nanmean(pt):.0f} m '
          f'({100*(np.nanmean(pt)-np.nanmean(pb))/np.nanmean(pb):+.1f}%)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rundirs', nargs='+')
    ap.add_argument('--out', default=None)
    ap.add_argument('--contrast', action='append', default=[],
                    help='base:treat, repeatable')
    ap.add_argument('--alias', action='append', default=[],
                    help='from=to, fold one arm into another (e.g. lc_repeat=lc '
                         'since they are the same configuration)')
    a = ap.parse_args()

    rows = []
    for rd in a.rundirs:
        for d in sorted(glob.glob(f'{rd}/*_s*')):
            r = load_run(d)
            if r:
                rows.append(r)
    if not rows:
        sys.exit('no runs found')

    alias = dict(x.split('=') for x in a.alias)
    for r in rows:
        r['arm'] = alias.get(r['arm'], r['arm'])

    print(f'{len(rows)} runs from {len({r["batch"] for r in rows})} batches')
    print(f'\n{"run":26}{"batch":30}{"drift":>8}{"ATE":>8}{"path":>7}'
          f'{"ANEES":>8}{"rev":>5}{"%rev":>7}')
    for r in sorted(rows, key=lambda r: (r['arm'], r['seed'])):
        print(f'{r["run"]:26}{r["batch"][:29]:30}{r["drift"]:8.3f}{r["ate"]:8.3f}'
              f'{r["path_m"]:7.0f}{r["anees"]:8.1f}{r["revisits"]:5.0f}'
              f'{100*r["rev_frac"]:6.0f}%')

    summarise(rows)

    for c in a.contrast:
        base, treat = c.split(':')
        compare(rows, base, treat)

    ok = [r for r in rows if np.isfinite(r['anees']) and np.isfinite(r['drift'])]
    if len(ok) >= 5:
        rho, p = spearman(np.array([r['anees'] for r in ok]),
                          np.array([r['drift'] for r in ok]))
        print(f'\nH6 (RQ3): Spearman ANEES vs drift rate over {len(ok)} runs: '
              f'rho = {rho:+.2f}, p = {p:.4f}')

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        with open(f'{a.out}/pooled_runs.csv', 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f'\nwrote {a.out}/pooled_runs.csv')


if __name__ == '__main__':
    main()
