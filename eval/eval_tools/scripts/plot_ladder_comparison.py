#!/usr/bin/env python3
"""Comparison figures for the ladder10 batch and its 20-minute lc_revisit rerun.

  python3 eval/eval_tools/scripts/plot_ladder_comparison.py \
      --ladder eval/runs/ladder10_20260801_0337 \
      --long eval/runs/lc_revisit_20min_20260801_1041 \
      --out eval/figures/0801_lc_revisit_20min

  figA_metrics_vs_volume   ATE / coverage / chamfer against explored volume,
                           every seed as a trace
  figB_12v20_paired        lc_revisit final values, 12 vs 20 min, paired by seed
  figC_matched_volume      boxes at final values, matched 34k, matched 48k
  figD_delta_vs_no_lc      per-seed paired difference of each arm against no_lc
  figE_uncertainty_vs_err  dopt / sigma_xy / U_r against GT error, lc_revisit

lc_revisit seed 901 never converged inside 12 minutes (coverage 0.33, chamfer
5.1 at cutoff; same seed at 20 min: 0.80 / 0.78). It is excluded from every
quality aggregate (n=9) and drawn dashed where its trace still appears.
"""
import argparse
import collections
import csv
import glob
import os

import numpy as np

import matplotlib
matplotlib.use('Agg')
import plot_style
plot_style.apply()
import matplotlib.pyplot as plt

ARM_COLOR = plot_style.ARM_COLOR
EXCLUDED = ('lc_revisit', '901')      # 12-minute batch only
ARMS12 = ('no_lc', 'lc', 'lc_revisit')


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def read(path):
    return list(csv.DictReader(open(path)))


def runs(base):
    out = {}
    for d in sorted(glob.glob(os.path.join(base, '*_s*'))):
        arm, seed = os.path.basename(d).rsplit('_s', 1)
        out[(arm, seed)] = d
    return out


def at_threshold(d, thr):
    """Metrics at the moment a run first reaches `thr` explored voxels."""
    mm = read(os.path.join(d, 'map_metrics.csv'))
    if not mm:
        return None
    t0 = num(mm[0]['t'])
    hit = next((r for r in mm if (num(r.get('explored')) or 0) >= thr), None)
    if hit is None:
        return None
    t = num(hit['t'])
    me = read(os.path.join(d, 'metrics.csv'))
    near = min(me, key=lambda r: abs((num(r.get('t')) or 0) - t))
    return {'dt': t - t0, 'ate': num(near.get('ate')),
            'cov': num(hit.get('coverage')), 'cham': num(hit.get('chamfer'))}


def spearman(x, y):
    rx = np.argsort(np.argsort(x)).astype(float)
    ry = np.argsort(np.argsort(y)).astype(float)
    if rx.std() == 0 or ry.std() == 0:
        return float('nan')
    return float(np.corrcoef(rx, ry)[0, 1])


def fig_a(r12, r20, out):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    groups = [(a, {k: v for k, v in r12.items() if k[0] == a}, a) for a in ARMS12]
    groups.append(('lc_revisit_20m', r20, 'lc_revisit_20m'))
    for label, rr, ckey in groups:
        first = True
        for (arm, seed), d in sorted(rr.items()):
            me = read(os.path.join(d, 'metrics.csv'))
            mm = read(os.path.join(d, 'map_metrics.csv'))
            xs, ate, cov, cham = [], [], [], []
            for r in mm:
                e = num(r.get('explored'))
                if e is None:
                    continue
                tt = num(r.get('t'))
                near = min(me, key=lambda x: abs((num(x.get('t')) or 0) - tt))
                xs.append(e / 1000.0)
                ate.append(num(near.get('ate')))
                cov.append(num(r.get('coverage')))
                cham.append(num(r.get('chamfer')))
            dashed = ckey == 'lc_revisit' and (arm, seed) == EXCLUDED
            kw = dict(color=ARM_COLOR[ckey], alpha=0.45, lw=1.2,
                      ls='--' if dashed else '-',
                      label=label if first else None)
            first = False
            axes[0].plot(xs, ate, **kw)
            axes[1].plot(xs, cov, **kw)
            axes[2].plot(xs, cham, **kw)
    for ax, ylab in zip(axes, ['ATE (m)', 'GT coverage', 'Chamfer (m)']):
        ax.set_xlabel('explored volume (k voxels)')
        ax.set_ylabel(ylab)
    axes[2].set_ylim(0, 2.0)
    axes[0].legend(loc='upper left')
    fig.suptitle('Quality vs explored volume — every seed '
                 '(dashed: lc_revisit s901, excluded from aggregates; '
                 'chamfer clipped at 2 m)')
    fig.tight_layout()
    fig.savefig(f'{out}/figA_metrics_vs_volume.png')
    fig.savefig(f'{out}/figA_metrics_vs_volume.pdf')
    plt.close(fig)
    print('wrote figA_metrics_vs_volume')


def fig_b(s12, s20, out):
    fig, axes = plt.subplots(1, 4, figsize=(15, 4))
    metrics = [('final_ate', 'ATE (m)'), ('final_coverage', 'GT coverage'),
               ('final_chamfer', 'Chamfer (m)'), ('final_explored', 'explored (voxels)')]
    for ax, (k, lab) in zip(axes, metrics):
        for s in sorted(s20):
            dashed = ('lc_revisit', s) == EXCLUDED
            ax.plot([0, 1], [num(s12[s][k]), num(s20[s][k])], marker='o',
                    ls='--' if dashed else '-',
                    label=f's{s}' + (' (excl.)' if dashed else ''))
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['12 min', '20 min'])
        ax.set_ylabel(lab)
    axes[0].legend()
    fig.suptitle('lc_revisit, 12 vs 20 minutes — paired by seed (final values)')
    fig.tight_layout()
    fig.savefig(f'{out}/figB_12v20_paired.png')
    fig.savefig(f'{out}/figB_12v20_paired.pdf')
    plt.close(fig)
    print('wrote figB_12v20_paired')


def fig_c(r12, r20, sum12, sum20, out):
    """Rows: final values, matched 34k, matched 48k. s901 excluded (n=9);
    the 12-minute lc_revisit arm is left out of the 48k row entirely, where
    excluding it leaves n=1."""
    rows_out = []
    fig, axes = plt.subplots(3, 3, figsize=(14, 12))

    def boxrow(ax_row, data, title):
        arms = [a for a in ('no_lc', 'lc', 'lc_revisit', 'lc_revisit_20m')
                if data.get(a)]
        for col, (k, lab) in enumerate([('ate', 'ATE (m)'), ('cov', 'GT coverage'),
                                        ('cham', 'Chamfer (m)')]):
            ax = ax_row[col]
            vals = [[x[k] for x in data[a] if x[k] is not None] for a in arms]
            bp = ax.boxplot(vals, patch_artist=True,
                            labels=[f'{a}\n(n={len(v)})' for a, v in zip(arms, vals)])
            for patch, a in zip(bp['boxes'], arms):
                patch.set_facecolor(ARM_COLOR.get(a, '#cccccc'))
                patch.set_alpha(0.55)
            ax.set_ylabel(lab)
            if col == 0:
                ax.annotate(title, xy=(0, 0.5), xycoords='axes fraction',
                            xytext=(-0.45, 0.5), rotation=90, va='center', fontsize=11)

    final = collections.defaultdict(list)
    for r in sum12:
        if (r['name'], r['seed']) == EXCLUDED:
            continue
        final[r['name']].append({'ate': num(r['final_ate']), 'cov': num(r['final_coverage']),
                                 'cham': num(r['final_chamfer'])})
    for r in sum20:
        final['lc_revisit_20m'].append({'ate': num(r['final_ate']),
                                        'cov': num(r['final_coverage']),
                                        'cham': num(r['final_chamfer'])})
    boxrow(axes[0], final, 'final values')

    for row, thr in ((1, 34000), (2, 48000)):
        data = collections.defaultdict(list)
        for (arm, seed), d in r12.items():
            if (arm, seed) == EXCLUDED or (thr == 48000 and arm == 'lc_revisit'):
                continue
            v = at_threshold(d, thr)
            if v:
                data[arm].append(v)
                rows_out.append({'threshold': thr, 'arm': arm, 'seed': seed, **v})
        for (arm, seed), d in r20.items():
            v = at_threshold(d, thr)
            if v:
                data['lc_revisit_20m'].append(v)
                rows_out.append({'threshold': thr, 'arm': 'lc_revisit_20m',
                                 'seed': seed, **v})
        boxrow(axes[row], data, f'matched at {thr // 1000}k voxels')

    fig.suptitle('Ablation comparison — final and matched-volume '
                 '(lc_revisit s901 excluded; 12-min lc_revisit omitted at 48k)')
    fig.tight_layout()
    fig.savefig(f'{out}/figC_matched_volume_boxes.png')
    fig.savefig(f'{out}/figC_matched_volume_boxes.pdf')
    plt.close(fig)
    with open(f'{out}/figC_matched_volume.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['threshold', 'arm', 'seed',
                                          'dt', 'ate', 'cov', 'cham'])
        w.writeheader()
        w.writerows(rows_out)
    print('wrote figC_matched_volume_boxes (+csv)')


def fig_d(sum12, out):
    """Per-seed paired difference against no_lc, final values."""
    by = {(r['name'], r['seed']): r for r in sum12}
    seeds = sorted({r['seed'] for r in sum12})
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    metrics = [('final_ate', 'ΔATE (m)'), ('final_coverage', 'Δcoverage'),
               ('final_chamfer', 'Δchamfer (m)')]
    for ax, (k, lab) in zip(axes, metrics):
        for j, arm in enumerate(('lc', 'lc_revisit')):
            xs, ys = [], []
            for i, s in enumerate(seeds):
                if (arm, s) == EXCLUDED or (arm, s) not in by:
                    continue
                a, b = num(by[('no_lc', s)][k]), num(by[(arm, s)][k])
                if a is None or b is None:
                    continue
                xs.append(i + (j - 0.5) * 0.22)
                ys.append(b - a)
            ax.scatter(xs, ys, s=34, color=ARM_COLOR[arm],
                       label=f'{arm} − no_lc (n={len(ys)})')
            ax.hlines(np.median(ys), -0.5, len(seeds) - 0.5,
                      color=ARM_COLOR[arm], ls=':', lw=1.2)
        ax.axhline(0.0, color='k', lw=1)
        ax.set_xticks(range(len(seeds)))
        ax.set_xticklabels([f's{s}' for s in seeds], rotation=45)
        ax.set_ylabel(lab)
    axes[0].legend()
    fig.suptitle('Paired per-seed difference against no_lc — final values '
                 '(dotted: median; below zero is better for ATE and chamfer)')
    fig.tight_layout()
    fig.savefig(f'{out}/figD_delta_vs_no_lc.png')
    fig.savefig(f'{out}/figD_delta_vs_no_lc.pdf')
    plt.close(fig)
    print('wrote figD_delta_vs_no_lc')


def fig_e(r12, out):
    """Does the reported uncertainty track the true error, for lc_revisit."""
    lcr = {k: v for k, v in r12.items() if k[0] == 'lc_revisit'}
    signals = [('dopt', 'D-opt'), ('sigma_xy', 'sigma_xy (m)'), ('u_ratio', 'U_r')]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # Representative time series: seed 900, revisit windows shaded.
    d = lcr[('lc_revisit', '900')]
    me = read(os.path.join(d, 'metrics.csv'))
    t0 = num(me[0]['t'])
    t = np.array([num(r['t']) - t0 for r in me])
    err = np.array([num(r['abs_error']) or np.nan for r in me])
    ax = axes[0][0]
    ax.plot(t, err, color='k', lw=1.6, label='GT abs error (m)')
    in_rev = [str(r.get('revisit_state')) == 'revisiting' for r in me]
    start = None
    for i, flag in enumerate(in_rev + [False]):
        if flag and start is None:
            start = t[i]
        elif not flag and start is not None:
            ax.axvspan(start, t[min(i, len(t) - 1)], color='#d62728', alpha=0.12)
            start = None
    ax2 = ax.twinx()
    for (key, lab), c in zip(signals, ('#1f77b4', '#2ca02c', '#9467bd')):
        y = np.array([num(r.get(key)) or np.nan for r in me])
        ax2.plot(t, y / np.nanmax(y), color=c, lw=1.1, alpha=0.8, label=lab)
    ax2.set_ylabel('signal / max')
    ax2.grid(False)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('GT abs error (m)')
    ax.set_title('seed 900 — error vs scaled signals (red: revisiting)')
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper left')

    # Pooled scatter per signal, one colour per seed, Spearman rho.
    for ax, (key, lab) in zip((axes[0][1], axes[1][0], axes[1][1]), signals):
        pooled_x, pooled_y = [], []
        rhos = []
        for (arm, seed), d in sorted(lcr.items()):
            me = read(os.path.join(d, 'metrics.csv'))
            x = np.array([num(r.get(key)) or np.nan for r in me])
            y = np.array([num(r.get('abs_error')) or np.nan for r in me])
            ok = ~(np.isnan(x) | np.isnan(y))
            if ok.sum() < 10:
                continue
            ax.scatter(x[ok], y[ok], s=3, alpha=0.25)
            rhos.append(spearman(x[ok], y[ok]))
            pooled_x.extend(x[ok])
            pooled_y.extend(y[ok])
        rho_all = spearman(np.array(pooled_x), np.array(pooled_y))
        ax.set_xlabel(lab)
        ax.set_ylabel('GT abs error (m)')
        ax.set_title(f'rho pooled {rho_all:+.2f}, per-seed median '
                     f'{np.median(rhos):+.2f} (10 seeds)')
    fig.suptitle('lc_revisit — reported uncertainty against true error')
    fig.tight_layout()
    fig.savefig(f'{out}/figE_uncertainty_vs_error.png')
    fig.savefig(f'{out}/figE_uncertainty_vs_error.pdf')
    plt.close(fig)
    print('wrote figE_uncertainty_vs_error')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ladder', required=True)
    ap.add_argument('--long', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    r12, r20 = runs(args.ladder), runs(args.long)
    sum12 = read(os.path.join(args.ladder, 'summary.csv'))
    sum20 = read(os.path.join(args.long, 'summary.csv'))
    s12 = {r['seed']: r for r in sum12
           if r['name'] == 'lc_revisit' and r['seed'] in {s for _, s in r20}}
    s20 = {r['seed']: r for r in sum20}

    fig_a(r12, r20, args.out)
    fig_b(s12, s20, args.out)
    fig_c(r12, r20, sum12, sum20, args.out)
    fig_d(sum12, args.out)
    fig_e(r12, args.out)
    print(f'\nFigures in {args.out}')


if __name__ == '__main__':
    main()
