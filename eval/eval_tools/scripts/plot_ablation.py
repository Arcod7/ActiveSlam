#!/usr/bin/env python3
"""Dissertation figures for the active-SLAM ablation ladder.

Pools any number of run_matrix.py batch directories by arm name, so a sweep
extended later with `run_matrix.py <config> --seeds 105 106` is folded in by
re-running this over both directories -- nothing needs recomputing.

  python3 eval/eval_tools/scripts/plot_ablation.py eval/runs/shipwreck_ablation_*
  python3 eval/eval_tools/scripts/plot_ablation.py <dirs...> --out eval/figures

Six figures, each written as PDF (vector, for LaTeX) and PNG (300 dpi), plus
the table-view twin of every figure as CSV:

  fig1_ablation_metrics  ATE, drift, chamfer, and the three that must be read
                         together: surface observed, fraction placed, product
  fig2_trajectories      ground truth vs SLAM vs dead reckoning, one seed,
                         with the error over time that makes drift legible
  fig3_uncertainty       D-optimality, closures, revisits, rebuilds over time
  fig4_consistency       ANEES / NIS / normalised chi-square vs expectation
  fig5_repeatability     every contrast against the measured noise floor
  fig6_tradeoff          explored extent vs placement accuracy, with
                         iso-curves of their product

Read fig5 first: two runs of the *same* configuration differed by a median
32% on ATE, so any contrast below that is not a result. Read fig1's coverage
panel only beside its neighbours -- coverage is a fraction of what was
observed, and an arm can raise it by observing less, which is exactly what
active revisit does.

Colour: the configurations take one hue light->dark (an ordinal ramp, which
survives greyscale printing) and the repeatability control takes a
contrasting categorical slot so it never reads as a point on that ramp.
Marker shape repeats the distinction, so identity never rests on colour.
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
import matplotlib.transforms as transforms
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator

try:
    from scipy.stats import chi2 as _chi2
except ImportError:
    _chi2 = None

# Display order. `lc_repeat` is byte-identical to `lc`: not a rung but the
# measured noise floor, so it sits beside `lc` and is styled apart from the
# ordinal ramp rather than pretending to be a step along it.
ARMS = ['nolc', 'lc', 'lc_repeat', 'lc_rebuild', 'lc_revisit', 'full']
ARM_LABELS = {
    'nolc': 'Open loop',
    'lc': 'Loop closure',
    'lc_repeat': 'Repeat (control)',
    'lc_rebuild': '+ rebuild',
    'lc_revisit': '+ revisit',
    'full': '+ both',
}

# Ordinal ramp over the five real configurations, one hue light->dark
# (validated: monotone lightness, adjacent dL >= 0.06, light end 2.06:1 on the
# surface, hue spread 4 deg). The control takes categorical slot 2 so it never
# reads as a point on that ramp.
_CONTROL_COLOR = '#eb6834'
ARM_STYLE = {
    'nolc':       ('#86b6ef', 'o'),
    'lc':         ('#5598e7', 's'),
    'lc_repeat':  (_CONTROL_COLOR, 'P'),
    'lc_rebuild': ('#2a78d6', 'v'),
    'lc_revisit': ('#1c5cab', '^'),
    'full':       ('#0d366b', 'D'),
}
_FALLBACK = ('#5598e7', 'o')


def arm_color(arm):
    return ARM_STYLE.get(arm, _FALLBACK)[0]


def arm_marker(arm):
    return ARM_STYLE.get(arm, _FALLBACK)[1]

# Categorical slots 1 and 2 for the two estimated trajectories; ground truth
# is the reference, so it wears primary ink rather than a competing hue.
C_SLAM, C_ODOM = '#2a78d6', '#eb6834'

INK, INK2, MUTED = '#0b0b0b', '#52514e', '#898781'
GRID, AXIS, SURFACE = '#e1e0d9', '#c3c2b7', '#fcfcfb'

NEES_DOF, NIS_DOF = 3, 6


def _style():
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
        'savefig.facecolor': SURFACE,
        'font.family': 'sans-serif',
        'font.sans-serif': ['DejaVu Sans'],
        'font.size': 9, 'axes.titlesize': 10, 'axes.labelsize': 9,
        'text.color': INK, 'axes.labelcolor': INK2, 'axes.titlecolor': INK,
        'xtick.color': MUTED, 'ytick.color': MUTED,
        'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
        'axes.edgecolor': AXIS, 'axes.linewidth': 0.8,
        'grid.color': GRID, 'grid.linewidth': 0.8, 'grid.linestyle': '-',
        'legend.frameon': False, 'legend.fontsize': 8,
        'xtick.direction': 'out', 'ytick.direction': 'out',
        'xtick.major.size': 3, 'ytick.major.size': 3,
    })


def _titles(fig, title, subtitle):
    """Title and subtitle spaced in points, not figure fractions.

    A fixed fraction puts them on top of each other on a tall figure and
    leaves a gap on a short one, since the gap needed is a text height.
    Returns the top of the area left for the axes."""
    height_in = fig.get_size_inches()[1]
    line = 1.0 / 72.0 / height_in           # one point, as a figure fraction
    top = 1.0 - 8 * line
    # suptitle hangs below y, so the subtitle clears the title's own height
    # (11pt) plus a gap before it starts.
    fig.suptitle(title, fontsize=11, color=INK, y=top)
    fig.text(0.5, top - 24 * line, subtitle, ha='center', fontsize=8,
             color=MUTED)
    return top - 40 * line


def _fmt_compact(v):
    """Short enough that six of them fit across one panel."""
    if not np.isfinite(v):
        return ''
    a = abs(v)
    if a >= 1e6:
        return f'{v / 1e6:.2f}M'
    if a >= 1e4:
        return f'{v / 1e3:.0f}k'
    if a >= 1e3:
        return f'{v / 1e3:.1f}k'
    return f'{v:.3g}'


def _despine(ax):
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.grid(True, axis='y', alpha=0.9)
    ax.set_axisbelow(True)


# ---------------------------------------------------------------- loading

def _load_csv(path):
    """CSV as a dict of float arrays; blank cells become NaN."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    out = {}
    for key in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        out[key] = np.array(vals)
    return out


def _load_tum(path):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    try:
        data = np.loadtxt(path, ndmin=2)
    except Exception:
        return None
    return data if len(data) >= 2 and data.shape[1] >= 8 else None


def _last_finite(arr):
    if arr is None or len(arr) == 0:
        return np.nan
    finite = arr[np.isfinite(arr)]
    return float(finite[-1]) if len(finite) else np.nan


def _distinct_consecutive(arr):
    """Values from a column that repeats the latest sample every tick.

    metrics.csv is written at the benchmark's tick rate but NIS and the
    normalised chi-square only update when a closure fires, so the raw column
    over-weights whichever value was latched longest."""
    if arr is None or len(arr) == 0:
        return np.array([])
    finite = arr[np.isfinite(arr)]
    if len(finite) == 0:
        return np.array([])
    keep = np.concatenate(([True], finite[1:] != finite[:-1]))
    return finite[keep]


def load_run(run_dir):
    """One run's metrics, or None if it produced nothing usable."""
    metrics = _load_csv(os.path.join(run_dir, 'metrics.csv'))
    if not metrics or 't' not in metrics:
        return None
    gt = _load_tum(os.path.join(run_dir, 'gt_traj.tum'))
    if gt is None:
        return None

    manifest = {}
    manifest_path = os.path.join(run_dir, 'manifest.json')
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            manifest = json.load(f)

    name = manifest.get('name') or os.path.basename(run_dir).rsplit('_s', 1)[0]
    seed = manifest.get('seed')
    if seed is None:
        tail = os.path.basename(run_dir).rsplit('_s', 1)
        seed = int(tail[1]) if len(tail) == 2 and tail[1].isdigit() else -1

    path_m = float(np.linalg.norm(np.diff(gt[:, 1:3], axis=0), axis=1).sum())
    ate = _last_finite(metrics.get('ate'))
    map_metrics = _load_csv(os.path.join(run_dir, 'map_metrics.csv'))

    nees_series = _distinct_consecutive(metrics.get('nees'))
    nis_series = _distinct_consecutive(metrics.get('nis'))
    chi2_series = _distinct_consecutive(metrics.get('chi2_norm'))

    return {
        'dir': run_dir, 'arm': name, 'seed': int(seed),
        'status': manifest.get('status', 'unknown'),
        'duration_s': float(metrics['t'][-1] - metrics['t'][0]),
        'path_m': path_m,
        'ate': ate,
        # ATE alone is not comparable across arms: an arm that stalls near
        # structure both travels less and drifts less. Per metre travelled is.
        'drift_pct': 100.0 * ate / path_m if path_m > 0 else np.nan,
        'rpe_trans': float(np.nanmean(metrics['rpe_trans']))
                     if 'rpe_trans' in metrics else np.nan,
        'coverage': _last_finite(map_metrics.get('coverage')),
        'chamfer': _last_finite(map_metrics.get('chamfer')),
        'lc_count': _last_finite(metrics.get('lc_count')),
        'revisit_count': _last_finite(metrics.get('revisit_count')),
        'rebuild_count': _last_finite(metrics.get('rebuild_count')),
        'anees': _last_finite(metrics.get('anees')),
        # Coverage is a fraction of what was observed, and the arms do not
        # observe the same amount -- carry the denominator and the product.
        'explored': _explored_voxels(run_dir),
        'correct': _explored_voxels(run_dir) * _last_finite(
            map_metrics.get('coverage')),
        'n_keyframes': len(nees_series),
        'nis_median': float(np.median(nis_series)) if len(nis_series) else np.nan,
        'chi2_median': float(np.median(chi2_series)) if len(chi2_series) else np.nan,
        'metrics': metrics,
        'map_metrics': map_metrics,
    }


def load_batches(batch_dirs):
    runs = []
    for batch in batch_dirs:
        for entry in sorted(os.listdir(batch)):
            run_dir = os.path.join(batch, entry)
            if not os.path.isdir(run_dir):
                continue
            run = load_run(run_dir)
            if run is None:
                print(f'  [skip] {run_dir}: no usable metrics')
                continue
            if run['status'] not in ('ok', 'short', 'unknown'):
                print(f"  [skip] {run_dir}: status={run['status']}")
                continue
            runs.append(run)
    return runs


def by_arm(runs, arm):
    return sorted((r for r in runs if r['arm'] == arm), key=lambda r: r['seed'])


# ------------------------------------------------------- fig 1: the ladder

def _paired_panel(ax, runs, arms, seeds, field, label, *, logy=False,
                  higher_better=None, reference=None, reference_note=''):
    """Per-arm distribution with every sample drawn and seeds joined.

    A box over four samples would imply quartiles it cannot support, so the
    box is the median and range only, and the paired lines carry the fact
    that the seeds are matched across arms.

    `reference` is a calibration expectation to draw and to keep inside the
    limits even when the data sits nowhere near it -- how far off it is IS
    the result, so it must not be cropped out."""
    offsets = np.linspace(-0.17, 0.17, max(len(seeds), 1))

    # Seed threads first, so the markers sit on top of them.
    for si, seed in enumerate(seeds):
        xs, ys = [], []
        for ai, arm in enumerate(arms):
            match = [r for r in runs if r['arm'] == arm and r['seed'] == seed]
            if match and np.isfinite(match[0][field]):
                xs.append(ai + offsets[si])
                ys.append(match[0][field])
        if len(xs) > 1:
            ax.plot(xs, ys, '-', color=MUTED, lw=0.7, alpha=0.55, zorder=1)

    medians = []
    empty = []
    for ai, arm in enumerate(arms):
        vals = np.array([r[field] for r in by_arm(runs, arm)
                         if np.isfinite(r[field])])
        if len(vals) == 0:
            medians.append(np.nan)
            empty.append(ai)
            continue
        med = float(np.median(vals))
        medians.append(med)
        # Range bar + median rule: honest for n of this size.
        ax.plot([ai, ai], [vals.min(), vals.max()], '-',
                color=arm_color(arm), lw=1.2, alpha=0.45, zorder=2)
        ax.plot([ai - 0.26, ai + 0.26], [med, med], '-',
                color=arm_color(arm), lw=2.2, solid_capstyle='butt', zorder=4)
        for si, seed in enumerate(seeds):
            match = [r for r in runs if r['arm'] == arm and r['seed'] == seed]
            if not match or not np.isfinite(match[0][field]):
                continue
            ax.plot(ai + offsets[si], match[0][field], arm_marker(arm),
                    color=arm_color(arm), markersize=5.5,
                    markeredgecolor=SURFACE, markeredgewidth=1.2, zorder=3)

    if logy:
        ax.set_yscale('log')

    if reference is not None:
        ax.axhline(reference, color=MUTED, lw=1.0, zorder=0)

    # Headroom for a single label row, so a median value never lands on a
    # marker or a seed thread.
    values = [r[field] for r in runs if r['arm'] in arms and np.isfinite(r[field])]
    if values:
        lo, hi = float(np.min(values)), float(np.max(values))
        if reference is not None:
            lo, hi = min(lo, reference), max(hi, reference)
        if logy and lo > 0:
            span = np.log10(hi) - np.log10(lo) or 1.0
            ax.set_ylim(10 ** (np.log10(lo) - span * 0.10),
                        10 ** (np.log10(hi) + span * 0.28))
        else:
            span = (hi - lo) or (abs(hi) or 1.0)
            ax.set_ylim(lo - span * 0.10, hi + span * 0.28)

    # One direct label per rung -- the median -- never a number per point.
    blend = transforms.blended_transform_factory(ax.transData, ax.transAxes)
    for ai, med in enumerate(medians):
        if not np.isfinite(med):
            continue
        ax.text(ai, 0.985, _fmt_compact(med), ha='center', va='top',
                fontsize=7.5, color=INK, transform=blend, zorder=6)
    # An arm with no samples is a fact about the arm, not a rendering gap.
    for ai in empty:
        ax.text(ai, 0.5, 'no closures\nto score', ha='center', va='center',
                fontsize=7.5, color=MUTED, style='italic',
                transform=blend, zorder=6)

    if reference is not None and reference_note:
        ax.text(0.99, reference, f' {reference_note}', ha='right', va='bottom',
                fontsize=7.5, color=INK2, zorder=5,
                transform=transforms.blended_transform_factory(
                    ax.transAxes, ax.transData))

    ax.set_xticks(range(len(arms)))
    rot = 0 if len(arms) <= 4 else 30
    ax.set_xticklabels([ARM_LABELS.get(a, a) for a in arms], rotation=rot,
                       ha='right' if rot else 'center')
    ax.set_xlim(-0.55, len(arms) - 0.45)
    if higher_better is None:
        ax.set_ylabel(label)
    else:
        arrow = 'higher is better' if higher_better else 'lower is better'
        ax.set_ylabel(f'{label}\n({arrow})')
    _despine(ax)
    return medians


def fig_metrics(runs, arms, seeds, out_dir):
    fig, axes = plt.subplots(2, 3, figsize=(max(12.0, 2.1 * len(arms) + 5.4), 7.6))
    panels = [
        ('ate', 'Absolute trajectory error (m)', False, False),
        ('drift_pct', 'Drift per metre travelled (%)', False, False),
        ('chamfer', 'TSDF Chamfer distance (m)', False, False),
        # These three belong together: coverage is the middle one divided by
        # the left one, and reading it alone inverts the conclusion.
        ('explored', 'Surface observed (GT voxels)', False, True),
        ('coverage', 'Fraction placed within 0.4 m', False, True),
        ('correct', 'Correctly mapped (voxels)', False, True),
    ]
    table = {}
    for ax, (field, label, logy, higher) in zip(axes.ravel(), panels):
        table[field] = _paired_panel(ax, runs, arms, seeds, field, label,
                                      logy=logy, higher_better=higher)

    top = _titles(fig, 'Active-SLAM ablation on the shipwreck scene',
                  f'{len(seeds)} seeds · thin lines join the same seed across '
                  'arms · bar = median · orange = repeatability control')
    fig.tight_layout(rect=(0, 0, 1, top))
    _save(fig, out_dir, 'fig1_ablation_metrics')

    with open(os.path.join(out_dir, 'fig1_ablation_metrics.csv'), 'w',
              newline='') as f:
        w = csv.writer(f)
        w.writerow(['arm', 'seed', 'path_m', 'ate_m', 'drift_pct',
                    'explored_voxels', 'coverage', 'correct_voxels',
                    'chamfer_m', 'lc_count', 'revisit_count',
                    'rebuild_count'])
        for arm in arms:
            for r in by_arm(runs, arm):
                w.writerow([arm, r['seed'], f"{r['path_m']:.1f}",
                            f"{r['ate']:.4f}", f"{r['drift_pct']:.4f}",
                            f"{r['explored']:.0f}", f"{r['coverage']:.4f}",
                            f"{r['correct']:.0f}", f"{r['chamfer']:.4f}",
                            r['lc_count'], r['revisit_count'],
                            r['rebuild_count']])
    return table


# ------------------------------------------------ fig 2: what drift looks like

def _representative_seed(runs, arms, seeds):
    """The seed whose full-system ATE is the median of its arm.

    Restricted to seeds every rung actually ran, so the panels differ by
    capability rather than by which noise realisation happened to be there."""
    complete = [s for s in seeds
                if all(any(r['arm'] == a and r['seed'] == s for r in runs)
                       for a in arms)]
    if not complete:
        complete = seeds
    if not complete:
        return None
    reference = 'full' if any(r['arm'] == 'full' for r in runs) else arms[-1]
    candidates = [r for r in by_arm(runs, reference)
                  if np.isfinite(r['ate']) and r['seed'] in complete]
    if not candidates:
        return complete[0]
    candidates.sort(key=lambda r: r['ate'])
    return candidates[len(candidates) // 2]['seed']


def _error_against_gt(gt, est):
    """Horizontal distance between an estimate and truth, on truth's clock."""
    if gt is None or est is None:
        return None, None
    t = gt[:, 0]
    inside = (t >= est[0, 0]) & (t <= est[-1, 0])
    if inside.sum() < 2:
        return None, None
    t = t[inside]
    ex = np.interp(t, est[:, 0], est[:, 1])
    ey = np.interp(t, est[:, 0], est[:, 2])
    err = np.hypot(ex - gt[inside, 1], ey - gt[inside, 2])
    return t - gt[0, 0], err


def fig_trajectories(runs, arms, seeds, out_dir):
    seed = _representative_seed(runs, arms, seeds)
    if seed is None:
        return None
    present = [a for a in arms
               if any(r['arm'] == a and r['seed'] == seed for r in runs)]
    if not present:
        return None

    # Two rows: the paths themselves, and the error that separates them. At
    # sub-metre drift over a scene tens of metres across the three paths are
    # visually coincident, so the overlay alone would show nothing -- the
    # error row is what makes each rung's behaviour legible.
    fig, axes = plt.subplots(2, len(present),
                             figsize=(2.9 * len(present), 6.4),
                             sharey='row',
                             gridspec_kw={'height_ratios': [2.1, 1]})
    axes = axes.reshape(2, len(present))

    bounds, err_max = [], 0.0
    for col, arm in enumerate(present):
        ax, eax = axes[0, col], axes[1, col]
        run = [r for r in runs if r['arm'] == arm and r['seed'] == seed][0]
        gt = _load_tum(os.path.join(run['dir'], 'gt_traj.tum'))
        slam = _load_tum(os.path.join(run['dir'], 'slam_traj.tum'))
        odom = _load_tum(os.path.join(run['dir'], 'odom_traj.tum'))

        # East-north view: TUM x is NED north, y is east. Truth underneath,
        # then dead reckoning, then the estimate on top -- the estimate is
        # the subject, and where it coincides with truth that must be visible.
        for data, color, style, lw, z in ((gt, INK, '-', 1.1, 2),
                                          (odom, C_ODOM, '--', 1.2, 3),
                                          (slam, C_SLAM, '-', 1.2, 4)):
            if data is None:
                continue
            ax.plot(data[:, 2], data[:, 1], style, color=color, lw=lw,
                    solid_capstyle='round', zorder=z)
            bounds.append((data[:, 2].min(), data[:, 2].max(),
                           data[:, 1].min(), data[:, 1].max()))

        if gt is not None:
            ax.plot(gt[0, 2], gt[0, 1], 'o', color=INK, markersize=6,
                    markeredgecolor=SURFACE, markeredgewidth=1.2, zorder=5)
            # The contrast-relief label: the anchor point is named, not
            # left to colour alone.
            ax.annotate('start', (gt[0, 2], gt[0, 1]),
                        textcoords='offset points', xytext=(7, 4),
                        fontsize=7.5, color=INK2)

        for est, color, style, lw in ((odom, C_ODOM, '--', 1.2),
                                      (slam, C_SLAM, '-', 1.4)):
            t, err = _error_against_gt(gt, est)
            if t is None:
                continue
            eax.plot(t / 60.0, err, style, color=color, lw=lw)
            err_max = max(err_max, float(np.nanmax(err)))

        ax.set_title(f"{ARM_LABELS[arm].replace(chr(10), ' ')}\n"
                     f"ATE {run['ate']:.2f} m · {run['path_m']:.0f} m travelled",
                     fontsize=9)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('East (m)')
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        ax.grid(True, alpha=0.9)
        ax.set_axisbelow(True)
        eax.set_xlabel('Time (min)')
        _despine(eax)

    if bounds:
        xlo = min(b[0] for b in bounds); xhi = max(b[1] for b in bounds)
        ylo = min(b[2] for b in bounds); yhi = max(b[3] for b in bounds)
        mx, my = (xhi - xlo) * 0.08 + 1, (yhi - ylo) * 0.08 + 1
        for ax in axes[0]:
            ax.set_xlim(xlo - mx, xhi + mx)
            ax.set_ylim(ylo - my, yhi + my)
    for eax in axes[1]:
        eax.set_ylim(0, err_max * 1.12 if err_max > 0 else 1)
    axes[0, 0].set_ylabel('North (m)')
    axes[1, 0].set_ylabel('Position error\nvs ground truth (m)')

    handles = [Line2D([], [], color=INK, lw=1.6, label='Ground truth'),
               Line2D([], [], color=C_SLAM, lw=1.4, label='SLAM estimate'),
               Line2D([], [], color=C_ODOM, lw=1.3, ls='--',
                      label='Dead reckoning')]
    fig.legend(handles=handles, loc='lower center', ncol=3,
               bbox_to_anchor=(0.5, 0.0))
    top = _titles(fig, f'Estimated versus true trajectory, seed {seed}',
                  'top: paths, near-coincident at this scale · '
                  'bottom: the error between them')
    fig.tight_layout(rect=(0, 0.045, 1, top))
    _save(fig, out_dir, 'fig2_trajectories')
    return seed


# ------------------------------------------- fig 3: the mechanism over time

def _resample(t, y, grid):
    """A run's series on the common time grid, NaN outside its own span."""
    mask = np.isfinite(t) & np.isfinite(y)
    if mask.sum() < 2:
        return np.full_like(grid, np.nan, dtype=float)
    out = np.interp(grid, t[mask], y[mask], left=np.nan, right=np.nan)
    out[grid > t[mask].max()] = np.nan
    return out


def _band_panel(ax, runs, arms, field, grid, label, *, logy=False, band=True):
    for ai, arm in enumerate(arms):
        stack = []
        for r in by_arm(runs, arm):
            m = r['metrics']
            if field not in m:
                continue
            t = m['t'] - m['t'][0]
            stack.append(_resample(t, m[field], grid))
        if not stack:
            continue
        stack = np.vstack(stack)
        if np.all(np.isnan(stack)):
            continue
        with np.errstate(invalid='ignore'):
            med = np.nanmedian(stack, axis=0)
            lo = np.nanpercentile(stack, 25, axis=0)
            hi = np.nanpercentile(stack, 75, axis=0)
        if band and len(stack) > 2:
            ax.fill_between(grid / 60.0, lo, hi, color=arm_color(arm),
                            alpha=0.13, linewidth=0)
        ax.plot(grid / 60.0, med, '-', color=arm_color(arm), lw=1.6,
                marker=arm_marker(arm), markevery=max(len(grid) // 9, 1),
                markersize=4.5, markeredgecolor=SURFACE, markeredgewidth=0.8,
                label=ARM_LABELS[arm].replace('\n', ' '))
    if logy:
        ax.set_yscale('log')
    ax.set_ylabel(label)
    _despine(ax)


def fig_uncertainty(runs, arms, out_dir):
    span = min((r['duration_s'] for r in runs), default=0)
    if span <= 0:
        return
    grid = np.linspace(0, span, 240)

    # Each rung's mechanism gets a panel showing it firing: closures for `lc`,
    # revisits for `lc_revisit`, re-integrations for `full`.
    panels = [('dopt', 'D-optimality of the\npose marginal', True),
              ('lc_count', 'Loop closures\n(cumulative)', False),
              ('revisit_count', 'Revisit engagements\n(cumulative)', False)]
    if any(np.isfinite(r['rebuild_count']) and r['rebuild_count'] > 0
           for r in runs):
        panels.append(('rebuild_count', 'Map re-integrations\n(cumulative)',
                       False))

    fig, axes = plt.subplots(len(panels), 1,
                             figsize=(7.4, 2.7 * len(panels)), sharex=True)
    for ax, (field, label, logy) in zip(axes, panels):
        _band_panel(ax, runs, arms, field, grid, label, logy=logy)
        if not logy:
            ax.yaxis.set_major_locator(MaxNLocator(integer=True))

    axes[-1].set_xlabel('Time into run (minutes)')
    top = _titles(fig, 'What each capability does over the course of a run',
                  'line = median across seeds · band = inter-quartile range')
    # One legend above everything it scopes, never sitting on the data.
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=len(labels),
               bbox_to_anchor=(0.5, top))
    line = 1.0 / 72.0 / fig.get_size_inches()[1]
    fig.tight_layout(rect=(0, 0, 1, top - 16 * line))
    _save(fig, out_dir, 'fig3_uncertainty')

    with open(os.path.join(out_dir, 'fig3_uncertainty.csv'), 'w',
              newline='') as f:
        fields = [field for field, _, _ in panels]
        w = csv.writer(f)
        w.writerow(['t_min'] + [f'{a}_{field}_median' for a in arms
                                for field in fields])
        series = {}
        for arm in arms:
            for field in fields:
                stack = []
                for r in by_arm(runs, arm):
                    m = r['metrics']
                    if field in m:
                        stack.append(_resample(m['t'] - m['t'][0], m[field], grid))
                with np.errstate(invalid='ignore'):
                    series[(arm, field)] = (np.nanmedian(np.vstack(stack), axis=0)
                                             if stack else np.full_like(grid, np.nan))
        for i, t in enumerate(grid):
            row = [f'{t / 60.0:.2f}']
            for arm in arms:
                for field in fields:
                    row.append(f'{series[(arm, field)][i]:.6g}')
            w.writerow(row)


# --------------------------------------------------- fig 4: is it calibrated

def fig_consistency(runs, arms, seeds, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(10.0, 4.2))

    # Log scales throughout: on every one of these the measured values sit an
    # order of magnitude off the expectation, and how far off is the result --
    # so the expectation stays inside the limits rather than being cropped.
    _paired_panel(axes[0], runs, arms, seeds, 'anees', 'ANEES (pose marginal)',
                  logy=True, reference=float(NEES_DOF),
                  reference_note='consistent = 3')
    _paired_panel(axes[1], runs, arms, seeds, 'nis_median',
                  'Median NIS per closure', logy=True,
                  reference=float(NIS_DOF), reference_note='expected = 6')
    _paired_panel(axes[2], runs, arms, seeds, 'chi2_median',
                  'Normalised graph $\\chi^2$', logy=True,
                  reference=1.0, reference_note='expected = 1')

    # The two-sided 95% acceptance region the ANEES is actually judged against.
    n_kf = int(np.median([r['n_keyframes'] for r in runs
                          if r['n_keyframes'] > 0] or [0]))
    if _chi2 is not None and n_kf > 0:
        lo = _chi2.ppf(0.025, NEES_DOF * n_kf) / n_kf
        hi = _chi2.ppf(0.975, NEES_DOF * n_kf) / n_kf
        axes[0].axhspan(lo, hi, color=MUTED, alpha=0.16, zorder=0)

    top = _titles(fig, 'Estimator consistency: is the reported covariance honest?',
                  'above the line = overconfident, below = conservative · '
                  f'shaded band = 95% acceptance region at {n_kf} keyframes')
    fig.tight_layout(rect=(0, 0, 1, top))
    _save(fig, out_dir, 'fig4_consistency')

    with open(os.path.join(out_dir, 'fig4_consistency.csv'), 'w',
              newline='') as f:
        w = csv.writer(f)
        w.writerow(['arm', 'seed', 'anees', 'n_keyframes', 'nis_median',
                    'chi2_median'])
        for arm in arms:
            for r in by_arm(runs, arm):
                w.writerow([arm, r['seed'], f"{r['anees']:.4f}",
                            r['n_keyframes'], f"{r['nis_median']:.4f}",
                            f"{r['chi2_median']:.4f}"])


# ------------------------------------------------ fig 5: the noise floor

def _paired_deltas(runs, lower, upper, field):
    """Per-seed |relative change| between two arms, in percent."""
    out = []
    for r_lo in by_arm(runs, lower):
        r_hi = next((x for x in by_arm(runs, upper)
                     if x['seed'] == r_lo['seed']), None)
        if r_hi is None or not np.isfinite(r_lo[field]) \
                or not np.isfinite(r_hi[field]) or not r_lo[field]:
            continue
        out.append(abs(100.0 * (r_hi[field] - r_lo[field]) / abs(r_lo[field])))
    return out


# Contrasts drawn against the noise floor. The control must come first: it
# is the yardstick every other bar is read against.
_CONTRAST_PAIRS = [
    ('lc', 'lc_repeat', 'Repeat\n(control)'),
    ('nolc', 'lc', '+ loop\nclosure'),
    ('lc', 'lc_rebuild', '+ map\nrebuild'),
    ('lc', 'lc_revisit', '+ active\nrevisit'),
]

# The control pair, oldest-first, used to shade the noise floor on each panel.
CONTROL_PAIR = ('lc', 'lc_repeat')


def fig_repeatability(runs, arms, seeds, out_dir):
    """How big a difference has to be before it means anything.

    `lc_repeat` is byte-identical to `lc`, so the spread between them is this
    stack's fixed-seed run-to-run noise, measured rather than assumed: on
    2026-07-26 it was a median 32% on ATE. A contrast that does not clear it
    is not a result, however large it looks.

    Before an explicit control arm existed this used `full` vs `lc_revisit`,
    valid only while map rebuild never fired. It now does, so that pair is a
    real comparison and cannot double as the control."""
    lo_c, hi_c = CONTROL_PAIR
    if not {lo_c, hi_c} <= set(arms):
        # Batches predating the explicit control arm: `full` differs from
        # `lc_revisit` only by re-integration, so while no rebuild ever fired
        # the two ARE the same configuration and their spread is still a valid
        # noise floor. Only usable when that condition actually held.
        rebuilds = [r['rebuild_count'] for r in by_arm(runs, 'full')
                    if np.isfinite(r['rebuild_count'])]
        if {'lc_revisit', 'full'} <= set(arms) and not any(c > 0 for c in rebuilds):
            lo_c, hi_c = 'lc_revisit', 'full'
            print('  [note] fig5 control: no lc_repeat arm, falling back to '
                  'lc_revisit vs full (rebuild never fired, so they are '
                  'configuration-identical)')
        else:
            print(f'  [note] fig5 skipped: no control pair in this batch, so '
                  'there is no measured noise floor to draw')
            return None

    pairs = [(lo, hi, label, arm_color(hi if hi != hi_c else lo_c),
              arm_marker(hi))
             for lo, hi, label in _CONTRAST_PAIRS]
    pairs = [p for p in pairs if p[0] in arms and p[1] in arms]
    metrics = [('ate', 'ATE'), ('drift_pct', 'Drift per metre'),
               ('coverage', 'Map coverage'), ('chamfer', 'Chamfer')]

    # Not sharey: the comparison that matters is control-vs-step inside one
    # panel, and a single ATE outlier on a shared scale flattens the coverage
    # panel to a line.
    fig, axes = plt.subplots(1, len(metrics), figsize=(3.5 * len(metrics), 4.0))
    control_medians = {}
    for ax, (field, label) in zip(axes, metrics):
        control = _paired_deltas(runs, lo_c, hi_c, field)
        if control:
            control_medians[field] = float(np.median(control))
            ax.axhspan(0, float(np.median(control)), color=_CONTROL_COLOR,
                       alpha=0.10, zorder=0)
        for xi, (lo, hi, _, color, marker) in enumerate(pairs):
            deltas = _paired_deltas(runs, lo, hi, field)
            if not deltas:
                continue
            ax.plot([xi, xi], [min(deltas), max(deltas)], '-', color=color,
                    lw=1.2, alpha=0.45, zorder=2)
            ax.plot([xi - 0.26, xi + 0.26],
                    [np.median(deltas)] * 2, '-', color=color, lw=2.2,
                    solid_capstyle='butt', zorder=4)
            jitter = np.linspace(-0.15, 0.15, len(deltas))
            ax.plot(xi + jitter, deltas, marker, color=color, markersize=5.5,
                    markeredgecolor=SURFACE, markeredgewidth=1.2, zorder=3,
                    linestyle='none')
        ax.set_xticks(range(len(pairs)))
        ax.set_xticklabels([p[2] for p in pairs])
        ax.set_xlim(-0.55, len(pairs) - 0.45)
        ax.set_title(label, fontsize=9.5)
        ax.set_ylim(bottom=0)
        _despine(ax)
    for ax in axes:
        ax.set_ylabel('Absolute change between\npaired runs (%)')

    top = _titles(fig, 'How large must a difference be to be real?',
                  'shaded = median change between two runs of the *same* '
                  'configuration · a rung step below it is not distinguishable')
    fig.tight_layout(rect=(0, 0, 1, top))
    _save(fig, out_dir, 'fig5_repeatability')

    with open(os.path.join(out_dir, 'fig5_repeatability.csv'), 'w',
              newline='') as f:
        w = csv.writer(f)
        w.writerow(['comparison', 'metric', 'seed', 'abs_change_pct'])
        for lo, hi, name, _, _ in pairs:
            for field, _label in metrics:
                for seed, d in zip([r['seed'] for r in by_arm(runs, lo)],
                                   _paired_deltas(runs, lo, hi, field)):
                    w.writerow([f'{lo} -> {hi}', field, seed, f'{d:.2f}'])
    return control_medians


# ---------------------------------------- fig 6: what the ratio hides

def _explored_voxels(run_dir, voxel_size=0.2):
    """Distinct voxels in the ground-truth cloud: what the sonar observed.

    Coverage is a *fraction of* this, so an arm can raise coverage simply by
    observing less. The absolute quantity has to be carried alongside it."""
    path = os.path.join(run_dir, 'maps', 'gt_tsdf.npy')
    if not os.path.exists(path):
        return np.nan
    try:
        pts = np.load(path)
    except Exception:
        return np.nan
    if len(pts) == 0:
        return np.nan
    return float(len(np.unique(np.floor(pts / voxel_size).astype(np.int32),
                               axis=0)))


def fig_tradeoff(runs, arms, out_dir):
    """Explored extent against placement accuracy, with iso-curves of their
    product -- the figure that shows coverage alone is not the result."""
    pts = []
    for ai, arm in enumerate(arms):
        for r in by_arm(runs, arm):
            explored = _explored_voxels(r['dir'])
            if not np.isfinite(explored) or not np.isfinite(r['coverage']):
                continue
            pts.append((ai, arm, r['seed'], explored, r['coverage'],
                        explored * r['coverage']))
    if not pts:
        print('  [note] fig6 skipped: no saved maps to measure explored extent')
        return None

    fig, ax = plt.subplots(figsize=(7.6, 5.6))
    xs = np.array([p[3] for p in pts]) / 1000.0
    ys = np.array([p[4] for p in pts])

    # Iso-curves of constant correctly-placed surface: explored x coverage = k.
    grid = np.linspace(max(xs.min() * 0.75, 1e-3), xs.max() * 1.12, 200)
    for k in np.linspace(np.nanpercentile([p[5] for p in pts], 10) / 1000.0,
                         np.nanpercentile([p[5] for p in pts], 90) / 1000.0, 4):
        with np.errstate(divide='ignore', invalid='ignore'):
            iso = k / grid
        keep = (iso >= 0) & (iso <= 1.02)
        if keep.sum() < 2:
            continue
        ax.plot(grid[keep], iso[keep], '-', color=GRID, lw=1.0, zorder=0)
        j = int(np.argmax(grid[keep]))
        ax.annotate(f'{k:.0f}k', (grid[keep][j], iso[keep][j]), fontsize=7,
                    color=MUTED, textcoords='offset points', xytext=(3, 0),
                    va='center', zorder=0)

    for ai, arm in enumerate(arms):
        sel = [p for p in pts if p[0] == ai]
        if not sel:
            continue
        ax.plot([p[3] / 1000.0 for p in sel], [p[4] for p in sel],
                arm_marker(arm), color=arm_color(arm), markersize=8,
                markeredgecolor=SURFACE, markeredgewidth=1.3, linestyle='none',
                label=ARM_LABELS[arm].replace('\n', ' '), zorder=3)

    ax.set_xlabel('Surface observed (thousands of ground-truth voxels)')
    ax.set_ylabel('Fraction placed within 0.4 m (map coverage)')
    # A scatter, not a bar length, so the axis may start at the data: the
    # comparison is between point positions, and 0->0.3 is dead space.
    pad = (ys.max() - ys.min()) * 0.35 or 0.05
    ax.set_ylim(max(0.0, ys.min() - pad), min(1.0, ys.max() + pad))
    ax.legend(loc='upper right', ncol=2)
    _despine(ax)
    ax.grid(True, axis='x', alpha=0.9)

    top = _titles(fig, 'Active revisit trades explored extent for placement accuracy',
                  'grey curves join equal amounts of correctly-mapped surface · '
                  'the rungs sit on the same curve')
    fig.tight_layout(rect=(0, 0, 1, top))
    _save(fig, out_dir, 'fig6_tradeoff')

    with open(os.path.join(out_dir, 'fig6_tradeoff.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['arm', 'seed', 'explored_voxels', 'coverage',
                    'correctly_placed_voxels'])
        for _, arm, seed, explored, cov, product in pts:
            w.writerow([arm, seed, f'{explored:.0f}', f'{cov:.4f}',
                        f'{product:.0f}'])
    return pts


# ------------------------------------------------------------------- output

def _save(fig, out_dir, stem):
    for ext, kwargs in (('pdf', {}), ('png', {'dpi': 300})):
        path = os.path.join(out_dir, f'{stem}.{ext}')
        fig.savefig(path, bbox_inches='tight', **kwargs)
    plt.close(fig)
    print(f'  wrote {stem}.pdf / .png')


def write_summary(runs, arms, out_dir):
    """Per-arm medians, and each rung's change against the one below it."""
    fields = ['path_m', 'ate', 'drift_pct', 'explored', 'coverage', 'correct',
              'chamfer', 'lc_count', 'revisit_count', 'anees', 'nis_median']
    path = os.path.join(out_dir, 'summary_by_arm.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['arm', 'n'] + [f'{k}_median' for k in fields])
        meds = {}
        for arm in arms:
            vals = by_arm(runs, arm)
            row = [arm, len(vals)]
            meds[arm] = {}
            for k in fields:
                v = np.array([r[k] for r in vals if np.isfinite(r[k])])
                m = float(np.median(v)) if len(v) else np.nan
                meds[arm][k] = m
                row.append(f'{m:.4g}')
            w.writerow(row)
        w.writerow([])
        # Paired, per seed. Comparing two medians throws away the fact that
        # the same noise realisation ran on both rungs, which is the only
        # thing that makes a difference visible at this sample size.
        w.writerow(['rung change', 'metric', 'median_from', 'median_to',
                    'median_change_pct', 'paired_median_change_pct',
                    'seeds_improved', 'n_paired'])
        for lower, upper in zip(arms, arms[1:]):
            for k in ('ate', 'drift_pct', 'explored', 'coverage', 'correct',
                      'chamfer', 'anees'):
                a, b = meds[lower][k], meds[upper][k]
                pct = 100.0 * (b - a) / a if np.isfinite(a) and a else np.nan
                better_is_higher = k in ('coverage', 'explored', 'correct')
                deltas, wins = [], 0
                for r_lo in by_arm(runs, lower):
                    r_hi = next((x for x in by_arm(runs, upper)
                                 if x['seed'] == r_lo['seed']), None)
                    if r_hi is None or not (np.isfinite(r_lo[k])
                                            and np.isfinite(r_hi[k])) or not r_lo[k]:
                        continue
                    deltas.append(100.0 * (r_hi[k] - r_lo[k]) / abs(r_lo[k]))
                    wins += int(r_hi[k] > r_lo[k] if better_is_higher
                                else r_hi[k] < r_lo[k])
                paired = float(np.median(deltas)) if deltas else np.nan
                w.writerow([f'{lower} -> {upper}', k, f'{a:.4g}', f'{b:.4g}',
                            f'{pct:+.1f}', f'{paired:+.1f}', wins, len(deltas)])
    print(f'  wrote summary_by_arm.csv')
    return path


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('batch_dirs', nargs='+',
                   help='run_matrix.py batch directories (globs are fine)')
    p.add_argument('--out', help='output directory (default: <first batch>/figures)')
    args = p.parse_args()

    batch_dirs = []
    for pattern in args.batch_dirs:
        matches = sorted(glob.glob(pattern)) if any(c in pattern for c in '*?[') \
            else [pattern]
        batch_dirs.extend(d for d in matches if os.path.isdir(d))
    if not batch_dirs:
        sys.exit('no batch directories matched')

    print(f'Pooling {len(batch_dirs)} batch director'
          f"{'y' if len(batch_dirs) == 1 else 'ies'}:")
    for d in batch_dirs:
        print(f'  {d}')

    runs = load_batches(batch_dirs)
    if not runs:
        sys.exit('no usable runs found')

    arms = [a for a in ARMS if any(r['arm'] == a for r in runs)]
    extra = sorted({r['arm'] for r in runs} - set(ARMS))
    if extra:
        print(f'  [note] ignoring arms outside the ladder: {extra}')
        runs = [r for r in runs if r['arm'] in arms]
    seeds = sorted({r['seed'] for r in runs})
    print(f'{len(runs)} runs · arms {arms} · seeds {seeds}')

    out_dir = args.out or os.path.join(batch_dirs[0], 'figures')
    os.makedirs(out_dir, exist_ok=True)
    _style()

    fig_metrics(runs, arms, seeds, out_dir)
    fig_trajectories(runs, arms, seeds, out_dir)
    fig_uncertainty(runs, arms, out_dir)
    fig_consistency(runs, arms, seeds, out_dir)
    fig_repeatability(runs, arms, seeds, out_dir)
    fig_tradeoff(runs, arms, out_dir)
    write_summary(runs, arms, out_dir)
    print(f'\nFigures in {out_dir}')


if __name__ == '__main__':
    main()
