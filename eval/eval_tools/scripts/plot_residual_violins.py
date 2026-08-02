#!/usr/bin/env python3
"""Residual distributions for the ablation ladder, as violins.

The ladder figures in plot_ablation.py reduce each run to one number (final
ATE, mean RPE, ANEES). That hides the shape: two arms can share a median and
differ entirely in the tail, and for a drifting estimator the tail is the
result. These figures keep every per-sample residual and compare the whole
distribution instead.

  python3 eval/eval_tools/scripts/plot_residual_violins.py eval/runs/ablation25_* \
      --out eval/figures/0729_ablation25

Two figures, PDF (vector, for LaTeX) and PNG (300 dpi), each with a CSV twin
of per-arm quantiles:

  fig7_residual_violins          absolute position residual, RPE translation,
                                 RPE rotation, per-keyframe NEES -- pooled by
                                 arm
  fig8_residual_violins_by_seed  the same position and RPE-translation
                                 residuals split per run, so between-seed
                                 spread is visible beside within-run spread

Two things the reader has to know, and both are said on the figures:

Samples are not independent. A residual at 10 Hz is strongly autocorrelated,
so a violin's width is the fraction of *time* spent at an error, not a
sampling distribution -- the width says nothing about how confident we are in
the median. That is what the per-seed medians drawn on top are for: four
seeds, four dots, and their spread is the honest uncertainty.

Runs differ in length (483-685 m of path here), so pooling raw samples would
let the longest run set the shape. Each seed is strided down to the shortest
run in its arm before pooling, which weights the four seeds equally and keeps
each one's full time span.

Density is estimated on log10 of the residual, not on the residual, because
these are positive and heavy-tailed: a linear-space kernel puts mass below
zero and flattens everything below the mode into one spike. The axis is
relabelled back to metres/degrees, so the ticks read as values.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('Agg')
import plot_style
plot_style.apply()
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FixedLocator, FuncFormatter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_ablation import (  # noqa: E402
    ARMS, ARM_LABELS, INK, INK2, MUTED, NEES_DOF, _distinct_consecutive,
    _save, _style, _titles, arm_color, by_arm, load_batches,
)

# Column, axis label, and whether the column latches its last value between
# updates. abs_error and the two RPE terms are recomputed every tick; nees is
# written once per keyframe and repeated on the ticks in between, so pooling
# it raw would weight each keyframe by how long it stayed the latest one.
PANELS = [
    ('abs_error', 'Absolute position residual (m)', False),
    ('rpe_trans', 'RPE translation (m)', False),
    ('rpe_rot_deg', 'RPE rotation (deg)', False),
    ('nees', 'NEES per keyframe', True),
]

MIN_SAMPLES = 20        # below this a violin is a rumour, not a distribution


def _series(run, column, latched):
    """One run's finite, strictly positive samples of a residual column."""
    arr = run['metrics'].get(column)
    if arr is None:
        return np.array([])
    vals = _distinct_consecutive(arr) if latched else arr[np.isfinite(arr)]
    return vals[vals > 0]


def _stride_to(vals, n):
    """Evenly thin to n samples, keeping the full time span."""
    if len(vals) <= n:
        return vals
    idx = np.linspace(0, len(vals) - 1, n).round().astype(int)
    return vals[idx]


def _pooled(runs, arm, column, latched):
    """Seed-balanced pool for one arm, plus each seed's median."""
    per_seed = [(r['seed'], _series(r, column, latched))
                for r in by_arm(runs, arm)]
    per_seed = [(s, v) for s, v in per_seed if len(v) >= MIN_SAMPLES]
    if not per_seed:
        return np.array([]), []
    n = min(len(v) for _, v in per_seed)
    pool = np.concatenate([_stride_to(v, n) for _, v in per_seed])
    medians = [(s, float(np.median(v))) for s, v in per_seed]
    return pool, medians


def _log_axis(ax, values):
    """Label a log10-space axis with the values it stands for."""
    lo, hi = np.floor(np.min(values)), np.ceil(np.max(values))
    ticks = [t for t in np.arange(lo, hi + 0.5, 0.5) if lo <= t <= hi]
    if len(ticks) > 8:
        ticks = ticks[::2]
    ax.yaxis.set_major_locator(FixedLocator(ticks))
    ax.yaxis.set_major_formatter(FuncFormatter(
        lambda v, _: f'{10 ** v:g}' if 10 ** v >= 1e-4 else f'{10 ** v:.0e}'))


def _violin(ax, position, log_vals, color, width=0.72):
    """One violin with its median and interquartile range drawn inside.

    The body is the shape; the bar and the dot are what a box plot would have
    said, kept because a violin alone makes a median hard to read off."""
    parts = ax.violinplot([log_vals], positions=[position], widths=width,
                          showextrema=False, showmedians=False)
    lo, hi = float(np.min(log_vals)), float(np.max(log_vals))
    for body in parts['bodies']:
        body.set_facecolor(color)
        body.set_alpha(0.55)
        body.set_edgecolor(color)
        body.set_linewidth(1.0)
        # The kernel spreads a bandwidth past both extremes, drawing density
        # where nothing was measured -- a metre-long tail below the smallest
        # residual ever seen. Cut the body back to the observed range.
        for path in body.get_paths():
            path.vertices[:, 1] = np.clip(path.vertices[:, 1], lo, hi)
    q1, med, q3 = np.percentile(log_vals, [25, 50, 75])
    p5, p95 = np.percentile(log_vals, [5, 95])
    ax.vlines(position, p5, p95, color=INK2, linewidth=1.0, zorder=3)
    ax.vlines(position, q1, q3, color=INK2, linewidth=4.0, zorder=4)
    ax.plot([position], [med], 'o', markersize=5, color='white',
            markeredgecolor=INK, markeredgewidth=1.0, zorder=5)
    return med


def _panel_frame(ax, positions, labels):
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.grid(True, axis='y', alpha=0.9)
    ax.set_axisbelow(True)
    ax.set_xticks(positions)
    ax.set_xticklabels(labels)
    ax.set_xlim(min(positions) - 0.65, max(positions) + 0.65)


def _legend(fig, top, with_seeds=True):
    handles = [
        Line2D([], [], marker='o', color='none', markerfacecolor='white',
               markeredgecolor=INK, markersize=5, label='median'),
        Line2D([], [], color=INK2, linewidth=4.0, label='interquartile range'),
        Line2D([], [], color=INK2, linewidth=1.0, label='5th-95th percentile'),
    ]
    if with_seeds:
        handles.append(Line2D([], [], marker='D', color='none',
                              markerfacecolor='none', markeredgecolor=INK2,
                              markersize=4, label='per-seed median'))
    fig.legend(handles=handles, loc='lower center', ncol=len(handles),
               bbox_to_anchor=(0.5, -0.02))


# ------------------------------------------------- fig 7: pooled by arm

def fig_violins(runs, arms, out_dir):
    fig, axes = plt.subplots(1, len(PANELS), figsize=(11.5, 4.4))
    positions = list(range(len(arms)))
    labels = [ARM_LABELS.get(a, a) for a in arms]
    table = []

    for ax, (column, label, latched) in zip(axes, PANELS):
        drawn = []
        for pos, arm in zip(positions, arms):
            pool, medians = _pooled(runs, arm, column, latched)
            if len(pool) == 0:
                continue
            log_vals = np.log10(pool)
            _violin(ax, pos, log_vals, arm_color(arm))
            for _, m in medians:
                ax.plot([pos + 0.30], [np.log10(m)], 'D', markersize=4,
                        markerfacecolor='none', markeredgecolor=INK2,
                        markeredgewidth=0.9, zorder=6)
            drawn.append(log_vals)
            table.append(_row(arm, column, pool, medians))
        ax.set_ylabel(label)
        _panel_frame(ax, positions, labels)
        if drawn:
            _log_axis(ax, np.concatenate(drawn))
        for tick in ax.get_xticklabels():
            tick.set_rotation(20)
            tick.set_ha('right')

    # NEES has a known expectation; the other three do not, so only it gets a
    # reference line -- a line on the rest would invite reading one in.
    nees_ax = axes[[c for c, _, _ in PANELS].index('nees')]
    nees_ax.axhline(np.log10(NEES_DOF), color=MUTED, linestyle='--',
                    linewidth=1.0, zorder=1)
    nees_ax.annotate(f'consistent = {NEES_DOF}', xy=(0.02, np.log10(NEES_DOF)),
                     xycoords=('axes fraction', 'data'), va='bottom',
                     fontsize=7, color=MUTED)

    top = _titles(
        fig, 'Residual distributions, not just their endpoints',
        'each violin pools all four seeds, strided to equal counts · '
        'samples are autocorrelated, so width is time spent at an error, '
        'not confidence · density estimated in log space')
    fig.tight_layout(rect=(0, 0.06, 1, top))
    _legend(fig, top)
    _save(fig, out_dir, 'fig7_residual_violins')
    _write_table(table, os.path.join(out_dir, 'fig7_residual_violins.csv'))


# The lower tail of the position residual is a result in its own right, and
# the violin shows it as thin ink that is easy to miss: it is the share of the
# run spent well localised. Not a startup transient -- under lc_revisit those
# samples fall late in the run, when a revisit has just corrected the pose.
LOW_TAIL = {'abs_error': 0.1}


def _row(arm, column, pool, medians):
    q = np.percentile(pool, [5, 25, 50, 75, 95, 99])
    seed_meds = [m for _, m in medians]
    thr = LOW_TAIL.get(column)
    frac = f'{float(np.mean(pool < thr)):.4f}' if thr else ''
    return [arm, column, len(medians), len(pool)] + \
        [f'{v:.4g}' for v in q] + \
        [f'{np.mean(pool):.4g}', f'{np.max(pool):.4g}',
         f'{np.min(seed_meds):.4g}', f'{np.max(seed_meds):.4g}', frac]


def _write_table(rows, path):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['arm', 'metric', 'n_seeds', 'n_samples_pooled',
                    'p05', 'p25', 'median', 'p75', 'p95', 'p99',
                    'mean', 'max', 'seed_median_min', 'seed_median_max',
                    'frac_below_0.1m'])
        w.writerows(rows)
    print(f'  wrote {os.path.basename(path)}')


# ---------------------------------------------- fig 8: split per seed

BY_SEED_PANELS = [
    ('abs_error', 'Absolute position residual (m)', False),
    ('rpe_trans', 'RPE translation (m)', False),
]


def fig_violins_by_seed(runs, arms, out_dir):
    fig, axes = plt.subplots(len(BY_SEED_PANELS), 1, figsize=(10.0, 6.4),
                             sharex=True)
    layout, pos = [], 0
    for arm in arms:
        for r in by_arm(runs, arm):
            layout.append((pos, arm, r))
            pos += 1
        pos += 0.8                      # a gap so the arms read as groups

    table = []
    for ax, (column, label, latched) in zip(axes, BY_SEED_PANELS):
        drawn = []
        for p, arm, r in layout:
            vals = _series(r, column, latched)
            if len(vals) < MIN_SAMPLES:
                continue
            log_vals = np.log10(vals)
            med = _violin(ax, p, log_vals, arm_color(arm), width=0.68)
            drawn.append(log_vals)
            if ax is axes[0]:
                table.append([arm, r['seed'], column, len(vals)] +
                             [f'{v:.4g}' for v in
                              np.percentile(vals, [5, 25, 50, 75, 95])] +
                             [f"{r['ate']:.4g}", f"{r['path_m']:.4g}"])
        ax.set_ylabel(label)
        _panel_frame(ax, [p for p, _, _ in layout],
                     [f"s{r['seed']}" for _, _, r in layout])
        if drawn:
            _log_axis(ax, np.concatenate(drawn))

    # Arm names once, under the seed labels, rather than repeated per violin.
    for arm in arms:
        ps = [p for p, a, _ in layout if a == arm]
        axes[-1].annotate(ARM_LABELS.get(arm, arm),
                          xy=(float(np.mean(ps)), -0.17),
                          xycoords=('data', 'axes fraction'), ha='center',
                          fontsize=9, color=INK)

    top = _titles(
        fig, 'The same residuals, one violin per run',
        'four seeds per arm · a shape that changes between seeds of one arm '
        'is noise, not a property of the arm')
    fig.tight_layout(rect=(0, 0.06, 1, top))
    _legend(fig, top, with_seeds=False)
    _save(fig, out_dir, 'fig8_residual_violins_by_seed')

    path = os.path.join(out_dir, 'fig8_residual_violins_by_seed.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['arm', 'seed', 'metric', 'n_samples', 'p05', 'p25',
                    'median', 'p75', 'p95', 'final_ate_m', 'path_m'])
        w.writerows(table)
    print(f'  wrote {os.path.basename(path)}')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('batch_dirs', nargs='+',
                   help='run_matrix.py batch directories (globs are fine)')
    p.add_argument('--out', help='output directory (default: <first batch>/figures)')
    p.add_argument('--exclude', nargs='*', default=[], metavar='ARM:SEED',
                   help='drop specific runs, e.g. lc_revisit:901')
    args = p.parse_args()

    batch_dirs = []
    for pattern in args.batch_dirs:
        matches = sorted(glob.glob(pattern)) if any(c in pattern for c in '*?[') \
            else [pattern]
        batch_dirs.extend(d for d in matches if os.path.isdir(d))
    if not batch_dirs:
        sys.exit('no batch directories matched')

    runs = load_batches(batch_dirs)
    if not runs:
        sys.exit('no usable runs found')
    for spec in args.exclude:
        arm, _, seed = spec.partition(':')
        before = len(runs)
        runs = [r for r in runs
                if not (r['arm'] == arm and str(r['seed']) == seed)]
        print(f'  [exclude] {spec}: dropped {before - len(runs)} run(s)')
    arms = [a for a in ARMS if any(r['arm'] == a for r in runs)]
    runs = [r for r in runs if r['arm'] in arms]
    seeds = sorted({r['seed'] for r in runs})
    print(f'{len(runs)} runs · arms {arms} · seeds {seeds}')

    out_dir = args.out or os.path.join(batch_dirs[0], 'figures')
    os.makedirs(out_dir, exist_ok=True)
    _style()

    fig_violins(runs, arms, out_dir)
    fig_violins_by_seed(runs, arms, out_dir)
    print(f'\nFigures in {out_dir}')


if __name__ == '__main__':
    main()
