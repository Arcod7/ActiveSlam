#!/usr/bin/env python3
"""Figures for the revisit tail analysis.

fig1_crossover     per-seed treatment effect against baseline drift, with the
                   regression-to-the-mean control overlaid -- the key figure
fig2_dispersion    drift rate by arm; the claim is about spread, so show it
fig3_trigger       reported sigma_xy against true error (RQ3 calibration)
fig4_anatomy       revisit exit reasons and closures harvested per episode
fig5_traces        error over time with revisit episodes shaded

Usage: python3 plot_tail.py RUNDIR [RUNDIR ...] --out DIR [--alias a=b]
"""
import argparse
import csv
import glob
import json
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BAD_DRIFT = 0.15
C = {'nolc': '#888888', 'lc': '#1f77b4', 'lc_repeat': '#9ecae1',
     'lc_revisit': '#d62728', 'lc_revisit_complete': '#2ca02c'}


def col(p, k):
    if not os.path.exists(p):
        return []
    out = []
    for r in csv.DictReader(open(p)):
        v = r.get(k)
        if v not in (None, '', 'nan'):
            try:
                out.append(float(v))
            except ValueError:
                pass
    return out


def load(d):
    if not os.path.exists(f'{d}/manifest.json'):
        return None
    mf = json.load(open(f'{d}/manifest.json'))
    m = f'{d}/metrics.csv'
    if not os.path.exists(m):
        return None
    rows = list(csv.DictReader(open(m)))
    def c(k):
        return np.array([float(r[k]) if r.get(k) not in (None, '', 'nan') else np.nan
                         for r in rows])
    arm, seed = os.path.basename(d).rsplit('_s', 1)
    t = c('t')
    ate = col(m, 'ate')
    path = float(mf.get('gt_path_m') or np.nan)
    an = col(m, 'anees_robust') or col(m, 'anees')
    return dict(arm=arm, seed=int(seed), dir=d, path=path,
                batch=os.path.basename(os.path.dirname(d)),
                ate=ate[-1] if ate else np.nan,
                drift=100 * ate[-1] / path if ate and path else np.nan,
                anees=float(np.median(an)) if an else np.nan,
                t=t - t[0], err=c('abs_error'), sig=c('sigma_xy'),
                lc=c('lc_count'),
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
    return [(a, b) for a, b in out if r['t'][b] - r['t'][a] > 5.0]


def by_seed(rows, arm):
    return {r['seed']: r for r in rows if r['arm'] == arm and np.isfinite(r['drift'])}


def fig1(rows, out, base='lc', treats=('lc_revisit', 'lc_revisit_complete'),
         ctrl='lc_repeat'):
    B = by_seed(rows, base)
    fig, ax = plt.subplots(figsize=(7, 5.2))
    lo = hi = None
    for arm in list(treats) + [ctrl]:
        T = by_seed(rows, arm)
        seeds = sorted(set(B) & set(T))
        if not seeds:
            continue
        x = np.array([B[s]['drift'] for s in seeds])
        y = np.array([T[s]['drift'] - B[s]['drift'] for s in seeds])
        style = dict(color=C.get(arm, 'k'), s=55,
                     marker='s' if arm == ctrl else 'o',
                     alpha=0.55 if arm == ctrl else 0.9)
        lab = f'{arm} (n={len(seeds)})' + (' — RTM null' if arm == ctrl else '')
        ax.scatter(x, y, label=lab, zorder=3, **style)
        if len(seeds) >= 3:
            k, b = np.polyfit(x, y, 1)
            xs = np.linspace(x.min(), x.max(), 20)
            ax.plot(xs, k * xs + b, '--' if arm == ctrl else '-',
                    color=C.get(arm, 'k'), lw=2 if arm != ctrl else 1.5,
                    alpha=0.9, zorder=2)
            ax.annotate(f'slope {k:+.2f}', (xs[-1], k * xs[-1] + b),
                        color=C.get(arm, 'k'), fontsize=9,
                        xytext=(4, 0), textcoords='offset points', va='center')
        lo = min(lo, x.min()) if lo is not None else x.min()
        hi = max(hi, x.max()) if hi is not None else x.max()
    ax.axhline(0, color='k', lw=1)
    if lo is not None:
        ax.axvspan(BAD_DRIFT, max(hi * 1.05, BAD_DRIFT * 1.05), color='red',
                   alpha=0.06, zorder=0)
        ax.text(BAD_DRIFT, ax.get_ylim()[1], ' BAD runs', color='red',
                fontsize=8, va='top')
    ax.set_xlabel(f'baseline drift rate of the same seed ({base}) [%/m]')
    ax.set_ylabel('change in drift rate vs baseline [%/m]')
    ax.set_title('Revisit helps runs that were going badly and mildly costs ones\n'
                 'that were fine — a negative slope is the insurance signature')
    ax.legend(fontsize=8, loc='best')
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{out}/fig1_crossover.png', dpi=150)
    plt.close(fig)


def fig2(rows, out):
    arms = [a for a in ('nolc', 'lc', 'lc_repeat', 'lc_revisit',
                        'lc_revisit_complete')
            if any(r['arm'] == a for r in rows)]
    fig, ax = plt.subplots(figsize=(7.5, 5.4))
    labels = []
    for i, a in enumerate(arms):
        d = np.array([r['drift'] for r in rows
                      if r['arm'] == a and np.isfinite(r['drift'])])
        if not len(d):
            labels.append(a)
            continue
        ax.scatter(np.random.default_rng(i).normal(i, 0.055, len(d)), d,
                   color=C.get(a, 'k'), s=48, alpha=0.85, zorder=3)
        ax.hlines(np.median(d), i - 0.26, i + 0.26, color='k', lw=2.2, zorder=4)
        ax.vlines(i, d.min(), d.max(), color=C.get(a, 'k'), lw=1.2, alpha=0.5)
        # n and SD go under the tick, not over the title
        labels.append(f'{a}\nn={len(d)}  SD {d.std(ddof=1) if len(d) > 1 else 0:.3f}')
    ax.axhline(BAD_DRIFT, color='red', ls=':', lw=1.4)
    ax.text(len(arms) - 0.5, BAD_DRIFT, ' BAD threshold', color='red',
            fontsize=8, va='bottom', ha='right')
    ax.set_xticks(range(len(arms)))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel('drift rate [%/m]')
    ax.set_title('Drift rate by arm — the claim is about spread, not the median')
    ax.grid(alpha=0.3, axis='y')
    nb = len({r['batch'] for r in rows})
    if nb > 1:
        ax.text(0.99, 0.02, f'pooled across {nb} batches of differing duration '
                            'and sigma_allow;\nSD is unstable at these n '
                            '(identical configs differed by 147% at n=3)',
                transform=ax.transAxes, ha='right', va='bottom', fontsize=7,
                style='italic', color='#555555')
    fig.tight_layout()
    fig.savefig(f'{out}/fig2_dispersion.png', dpi=150)
    plt.close(fig)


def fig3(rows, out):
    fig, ax = plt.subplots(figsize=(7, 5))
    for r in rows:
        ok = np.isfinite(r['err']) & np.isfinite(r['sig'])
        if ok.sum() < 20:
            continue
        ax.scatter(r['err'][ok][::25], r['sig'][ok][::25], s=4, alpha=0.18,
                   color=C.get(r['arm'], 'k'))
    e = np.concatenate([r['err'][np.isfinite(r['err'])] for r in rows])
    s = np.concatenate([r['sig'][np.isfinite(r['sig'])] for r in rows])
    lo, hi = np.nanpercentile(e, 1), np.nanpercentile(e, 99)
    ax.plot([lo, hi], [lo, hi], 'k--', lw=1.3, label='perfectly calibrated')
    ax.axhspan(np.nanpercentile(s, 5), np.nanpercentile(s, 95), color='orange',
               alpha=0.18, label='reported sigma, 5–95%')
    ax.set_xlabel('true error [m]')
    ax.set_ylabel('reported sigma_xy [m]')
    ax.set_title('RQ3 — the reported uncertainty barely moves while true error\n'
                 'varies several-fold, so it cannot identify a run in trouble')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f'{out}/fig3_trigger.png', dpi=150)
    plt.close(fig)


def fig4(rows, out):
    data = {}
    for r in rows:
        eps = episodes(r)
        if not eps:
            continue
        for a, b in eps:
            ex = r['exits'][min(b + 1, len(r['exits']) - 1)] or 'NONE'
            gained = r['lc'][b] - r['lc'][a]
            data.setdefault(r['arm'], []).append((ex, gained, r['t'][b] - r['t'][a]))
    if not data:
        return
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    arms = sorted(data)
    reasons = sorted({e for v in data.values() for e, _, _ in v})
    w = 0.8 / max(len(arms), 1)
    for i, a in enumerate(arms):
        cnt = [sum(1 for e, _, _ in data[a] if e == rr) for rr in reasons]
        axes[0].bar(np.arange(len(reasons)) + i * w, cnt, w,
                    label=f'{a} ({len(data[a])} ep)', color=C.get(a, 'k'))
    axes[0].set_xticks(np.arange(len(reasons)) + 0.4 - w / 2)
    axes[0].set_xticklabels(reasons, rotation=20, ha='right', fontsize=8)
    axes[0].set_ylabel('episodes')
    axes[0].set_title('How revisits end')
    axes[0].legend(fontsize=8)
    for i, a in enumerate(arms):
        g = [x for _, x, _ in data[a]]
        axes[1].scatter(np.random.default_rng(i).normal(i, 0.05, len(g)), g,
                        color=C.get(a, 'k'), s=45, alpha=0.85)
        axes[1].hlines(np.median(g), i - 0.25, i + 0.25, color='k', lw=2)
    axes[1].set_xticks(range(len(arms)))
    axes[1].set_xticklabels(arms, fontsize=8)
    axes[1].set_ylabel('loop closures harvested per revisit')
    axes[1].set_title('What each revisit actually collects')
    for ax in axes:
        ax.grid(alpha=0.3, axis='y')
    fig.tight_layout()
    fig.savefig(f'{out}/fig4_anatomy.png', dpi=150)
    plt.close(fig)


def fig5(rows, out, n=6):
    sel = [r for r in rows if episodes(r)][:n]
    if not sel:
        return
    fig, axes = plt.subplots(len(sel), 1, figsize=(9, 2.0 * len(sel)),
                             sharex=True, squeeze=False)
    for ax, r in zip(axes[:, 0], sel):
        ax.plot(r['t'], r['err'], color=C.get(r['arm'], 'k'), lw=1.1)
        for a, b in episodes(r):
            ax.axvspan(r['t'][a], r['t'][b], color='orange', alpha=0.3)
        ax.set_ylabel('|err| m', fontsize=8)
        ax.text(0.01, 0.88, f"{r['arm']}_s{r['seed']}  drift {r['drift']:.3f} %/m",
                transform=ax.transAxes, fontsize=8, va='top')
        ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel('time [s]   (shaded = revisiting)')
    fig.suptitle('Error against time — revisit resets the level, then it re-accumulates',
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(f'{out}/fig5_traces.png', dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rundirs', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--alias', action='append', default=[])
    a = ap.parse_args()
    rows = []
    for rd in a.rundirs:
        for d in sorted(glob.glob(f'{rd}/*_s*')):
            r = load(d)
            if r:
                rows.append(r)
    alias = dict(x.split('=') for x in a.alias)
    for r in rows:
        r['arm'] = alias.get(r['arm'], r['arm'])
    os.makedirs(a.out, exist_ok=True)
    for f in (fig1, fig2, fig3, fig4, fig5):
        try:
            f(rows, a.out)
        except Exception as e:
            print(f'{f.__name__} failed: {type(e).__name__}: {e}')
    print(f'figures -> {a.out}  ({len(rows)} runs)')


if __name__ == '__main__':
    main()
