#!/usr/bin/env python3
"""Session analysis tool for frontier_slam.

Usage:
    python3 tools/analyze_session.py                        # latest session
    python3 tools/analyze_session.py 2026-05-19_16-15-40    # specific session
    python3 tools/analyze_session.py --compare A B          # compare two sessions

This script is meant to evolve alongside the system. As new CSV columns are added
(lateral obstacles, cluster sizes, odometer…) add the corresponding sections below.
Current coverage: controller + extractor CSV format as of Change 19 (frontier scoring).

Column reference:
  controller: t_ros, rx, ry, rz, gx, gy, gz, dist_m, hdg_err_deg, depth_err_m,
               surge, yaw_cmd, heave, obs_m, path_len, wp_idx, event
  extractor:  t_ros, rx, ry, rz, gx, gy, dist_m, clusters, stuck_pct,
               blacklist_n, free_cells, occ_cells, mapped_cells, event
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict
from typing import Optional


# ── paths ─────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
_LOG_DIR    = os.path.join(_SCRIPT_DIR, '..', 'logs')


# ── data loading ──────────────────────────────────────────────────────────────

def _load(prefix: str, kind: str) -> list[dict]:
    path = os.path.join(_LOG_DIR, f'{prefix}_{kind}.csv')
    if not os.path.exists(path):
        sys.exit(f'[error] not found: {path}')
    with open(path) as f:
        return list(csv.DictReader(f))


def _latest_prefix() -> str:
    files = [f for f in os.listdir(_LOG_DIR) if f.endswith('_controller.csv')]
    if not files:
        sys.exit('[error] no session logs found in logs/')
    return sorted(files)[-1].replace('_controller.csv', '')


def _load_session(prefix: str) -> tuple[list, list]:
    return _load(prefix, 'controller'), _load(prefix, 'extractor')


# ── helpers ───────────────────────────────────────────────────────────────────

def _f(row: dict, key: str, fallback: float = float('nan')) -> float:
    v = row.get(key, '')
    try:
        return float(v)
    except (ValueError, TypeError):
        return fallback


def _t(row: dict) -> float:
    return _f(row, 't_ros')


def _odometer(ctrl: list) -> float:
    total = 0.0
    for i in range(1, len(ctrl)):
        dx = _f(ctrl[i], 'rx') - _f(ctrl[i-1], 'rx')
        dy = _f(ctrl[i], 'ry') - _f(ctrl[i-1], 'ry')
        total += math.hypot(dx, dy)
    return total



def _goal_commits(ext: list) -> list[dict]:
    """Return rows where a new goal commitment happened (stuck_pct reset to 0)."""
    commits = []
    prev_pct = 100
    t0 = _t(ext[0])
    for r in ext:
        pct = int(_f(r, 'stuck_pct', 0))
        if pct < prev_pct:
            commits.append({'t_rel': _t(r) - t0, 'gx': _f(r, 'gx'), 'gy': _f(r, 'gy'),
                            'rx': _f(r, 'rx'), 'ry': _f(r, 'ry')})
        prev_pct = pct
    return commits


def _stuck_events(ext: list) -> list[dict]:
    t0 = _t(ext[0])
    return [{'t_rel': _t(r) - t0, 'gx': _f(r, 'gx'), 'gy': _f(r, 'gy'),
             'rx': _f(r, 'rx'), 'ry': _f(r, 'ry')}
            for r in ext if r.get('event') == 'STUCK_BLACKLIST']


def _coverage_phases(ext: list, n_phases: int = 4) -> list[tuple]:
    """Split session into n_phases equal time windows; return (t_start, t_end, delta_cells) each."""
    t0 = _t(ext[0])
    dur = _t(ext[-1]) - t0
    phase_len = dur / n_phases
    phases = []
    for i in range(n_phases):
        t_start = t0 + i * phase_len
        t_end   = t0 + (i + 1) * phase_len
        rows_in = [r for r in ext if t_start <= _t(r) < t_end]
        if not rows_in:
            phases.append((i * phase_len, (i+1) * phase_len, 0))
            continue
        gain = int(_f(rows_in[-1], 'mapped_cells', 0)) - int(_f(rows_in[0], 'mapped_cells', 0))
        phases.append((i * phase_len, (i+1) * phase_len, gain))
    return phases


def _depth_stats(ctrl: list) -> dict:
    errs = [_f(r, 'depth_err_m') for r in ctrl
            if r.get('depth_err_m') not in ('', 'nan', None)]
    errs = [e for e in errs if not math.isnan(e)]
    if not errs:
        return {'min': float('nan'), 'max': float('nan'), 'mean': float('nan'), 'final': float('nan'),
                'frac_over_0_3': float('nan')}
    return {
        'min':   min(errs),
        'max':   max(errs),
        'mean':  sum(errs) / len(errs),
        'final': errs[-1],
        'frac_over_0_3': sum(1 for e in errs if abs(e) > 0.3) / len(errs),
    }


# ── report ────────────────────────────────────────────────────────────────────

def _col(w: int, s: str) -> str:
    return str(s)[:w].ljust(w)


def _fmt(v, fmt='.1f', unit='') -> str:
    if isinstance(v, float) and math.isnan(v):
        return 'n/a'
    if isinstance(v, float):
        return f'{v:{fmt}}{unit}'
    return str(v) + unit


def report(prefix: str, ctrl: list, ext: list) -> dict:
    """Print a full session report and return a metrics dict for comparison."""

    t0       = _t(ctrl[0])
    duration = _t(ctrl[-1]) - t0
    m_start  = int(_f(ext[0],  'mapped_cells', 0))
    m_end    = int(_f(ext[-1], 'mapped_cells', 0))
    m_gain   = m_end - m_start

    emerg_stops  = sum(1 for r in ctrl if r.get('event') == 'EMERG_STOP')
    ctrl_stucks  = sum(1 for r in ctrl if r.get('event') == 'CTRL_STUCK')
    odo            = _odometer(ctrl)
    commits        = _goal_commits(ext)
    stuck_evs      = _stuck_events(ext)
    max_bl_n       = max(int(_f(r, 'blacklist_n', 0)) for r in ext)
    depth          = _depth_stats(ctrl)
    phases         = _coverage_phases(ext, n_phases=4)

    rx_vals = [_f(r, 'rx') for r in ctrl]
    ry_vals = [_f(r, 'ry') for r in ctrl]

    print(f'\n{"═"*62}')
    print(f'  Session: {prefix}')
    print(f'{"═"*62}')

    # ── Overview ──────────────────────────────────────────────────
    print(f'\n── Overview ──────────────────────────────────────────────')
    print(f'  Duration              {duration:.0f}s')
    print(f'  Controller rows       {len(ctrl)}')
    print(f'  Extractor rows        {len(ext)}')

    # ── Coverage ──────────────────────────────────────────────────
    print(f'\n── Coverage ──────────────────────────────────────────────')
    print(f'  Mapped cells          {m_start} → {m_end}  (+{m_gain})')
    print(f'  Free cells            {_f(ext[0],"free_cells",0):.0f} → {_f(ext[-1],"free_cells",0):.0f}')
    print(f'  Occ  cells            {_f(ext[0],"occ_cells",0):.0f} → {_f(ext[-1],"occ_cells",0):.0f}')
    print(f'  Cells/minute          {m_gain / duration * 60:.0f}')
    print(f'  Cells/meter           {m_gain / max(1.0, odo):.1f}')
    print()
    print(f'  Coverage by quarter:')
    for t_a, t_b, gain in phases:
        bar_len = max(0, int(gain / 200))
        bar = '█' * bar_len
        print(f'    t={t_a:5.0f}–{t_b:5.0f}s  +{gain:5d} cells  {bar}')

    # ── Navigation ────────────────────────────────────────────────
    print(f'\n── Navigation ────────────────────────────────────────────')
    print(f'  Odometer              {odo:.1f}m')
    print(f'  Robot X range         [{min(rx_vals):.1f}, {max(rx_vals):.1f}]')
    print(f'  Robot Y range         [{min(ry_vals):.1f}, {max(ry_vals):.1f}]')

    # ── Goal behaviour ────────────────────────────────────────────
    print(f'\n── Goal behaviour ────────────────────────────────────────')
    print(f'  Goal commits          {len(commits)}  ({len(commits)/duration*60:.1f}/min)')
    print(f'  STUCK_BLACKLIST       {len(stuck_evs)}')
    print(f'  Max blacklist size    {max_bl_n}')
    if stuck_evs:
        for ev in stuck_evs:
            print(f'    t={ev["t_rel"]:5.0f}s  robot=({ev["rx"]:.1f},{ev["ry"]:.1f})'
                  f'  blacklisted=({ev["gx"]:.1f},{ev["gy"]:.1f})')

    # ── Obstacle events ────────────────────────────────────────────
    print(f'\n── Obstacle events ───────────────────────────────────────')
    print(f'  EMERG_STOP events     {emerg_stops}')
    print(f'  CTRL_STUCK events     {ctrl_stucks}')

    # Lateral sonar columns (future: obs_left_m, obs_right_m)
    has_lateral = 'obs_left_m' in (ctrl[0] if ctrl else {})
    if has_lateral:
        left_close  = sum(1 for r in ctrl if _f(r,'obs_left_m',  float('inf')) < 0.8)
        right_close = sum(1 for r in ctrl if _f(r,'obs_right_m', float('inf')) < 0.8)
        print(f'  Left wall (<0.8m)     {left_close} ticks  ({left_close/len(ctrl)*100:.1f}%)')
        print(f'  Right wall (<0.8m)    {right_close} ticks  ({right_close/len(ctrl)*100:.1f}%)')

    # ── Depth control ─────────────────────────────────────────────
    print(f'\n── Depth control ─────────────────────────────────────────')
    print(f'  Depth error  min={_fmt(depth["min"],"+.2f","m")}  '
          f'max={_fmt(depth["max"],"+.2f","m")}  '
          f'mean={_fmt(depth["mean"],"+.2f","m")}  '
          f'final={_fmt(depth["final"],"+.2f","m")}')
    print(f'  Time |err|>0.3m       {_fmt(depth["frac_over_0_3"],".1%")}')

    print(f'\n{"═"*62}\n')

    return {
        'prefix':        prefix,
        'duration_s':    duration,
        'mapped_gain':   m_gain,
        'cells_per_min': m_gain / duration * 60,
        'cells_per_m':   m_gain / max(1.0, odo),
        'odometer_m':    odo,
        'rx_min':        min(rx_vals),  'rx_max': max(rx_vals),
        'ry_min':        min(ry_vals),  'ry_max': max(ry_vals),
        'goal_commits':  len(commits),
        'stuck_events':  len(stuck_evs),
        'emerg_stops':   emerg_stops,
        'ctrl_stucks':   ctrl_stucks,
        'depth_max_err': depth['max'],
        'depth_frac_03': depth['frac_over_0_3'],
    }


def compare(m1: dict, m2: dict) -> None:
    rows = [
        ('Duration (s)',          'm', 'duration_s',    '.0f'),
        ('Cells explored',        '',  'mapped_gain',   'd'),
        ('Cells/minute',          '',  'cells_per_min', '.0f'),
        ('Cells/meter',           '',  'cells_per_m',   '.1f'),
        ('Odometer (m)',          'm', 'odometer_m',    '.1f'),
        ('Robot X range min',     '',  'rx_min',        '.1f'),
        ('Robot X range max',     '',  'rx_max',        '.1f'),
        ('Robot Y range min',     '',  'ry_min',        '.1f'),
        ('Robot Y range max',     '',  'ry_max',        '.1f'),
        ('Goal commits',          '',  'goal_commits',  'd'),
        ('STUCK_BLACKLIST',       '',  'stuck_events',  'd'),
        ('EMERG_STOP events',      '',  'emerg_stops',   'd'),
        ('CTRL_STUCK events',      '',  'ctrl_stucks',   'd'),
        ('Depth max error (m)',   '',  'depth_max_err', '+.2f'),
        ('Depth >0.3m fraction',  '%', 'depth_frac_03', '.1%'),
    ]

    p1 = m1['prefix'][-19:]
    p2 = m2['prefix'][-19:]
    print(f'\n{"═"*72}')
    print(f'  Comparison')
    print(f'{"═"*72}')
    print(f'  {"Metric":<28}  {p1:>19}  {p2:>19}')
    print(f'  {"─"*28}  {"─"*19}  {"─"*19}')
    for label, unit, key, fmt in rows:
        v1, v2 = m1.get(key, float('nan')), m2.get(key, float('nan'))
        def _fv(v):
            if isinstance(v, float) and math.isnan(v): return 'n/a'
            if fmt == 'd': return f'{int(v)}'
            return f'{v:{fmt}}'
        print(f'  {label:<28}  {_fv(v1):>19}  {_fv(v2):>19}')
    print(f'{"═"*72}\n')


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sessions', nargs='*',
                        help='session prefix(es) e.g. 2026-05-19_16-15-40')
    parser.add_argument('--compare', metavar='PREFIX', nargs=2,
                        help='compare two sessions by prefix')
    parser.add_argument('--list', action='store_true',
                        help='list available sessions')
    args = parser.parse_args()

    if args.list:
        files = sorted(f.replace('_controller.csv','')
                       for f in os.listdir(_LOG_DIR)
                       if f.endswith('_controller.csv'))
        print('\nAvailable sessions:')
        for f in files:
            ctrl = _load(f, 'controller')
            dur = _t(ctrl[-1]) - _t(ctrl[0])
            print(f'  {f}  ({dur:.0f}s)')
        return

    prefixes = args.compare if args.compare else (args.sessions or [_latest_prefix()])

    metrics = []
    for p in prefixes:
        ctrl, ext = _load_session(p)
        m = report(p, ctrl, ext)
        metrics.append(m)

    if len(metrics) == 2:
        compare(metrics[0], metrics[1])


if __name__ == '__main__':
    main()
