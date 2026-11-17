#!/usr/bin/env python3
"""Confirmatory analysis for docs/PREREG_TRIGGER.md. Fixed before the data.

Primary endpoint is final_abs_error, paired by seed, two-sided paired t-test at
alpha=0.05, with a Wilcoxon signed-rank alongside as a distribution-free check.
Everything else is descriptive and carries no inferential claim.

Run unmodified:
  python3 eval/eval_tools/scripts/trigger_confirm.py 'eval/runs/trigger_confirm_*/summary.csv'
"""
import csv
import glob
import sys

import numpy as np
from scipy import stats

PRIMARY = 'final_abs_error'
SECONDARY = ('final_ate', 'final_coverage', 'final_chamfer', 'lc_count',
             'revisit_count', 'gt_path_m')
ARM_A, ARM_B = 'uncertainty', 'scheduled'
ALPHA = 0.05


def load(pattern):
    rows = []
    for f in glob.glob(pattern):
        rows += list(csv.DictReader(open(f)))
    by = {}
    for r in rows:
        by.setdefault(int(r['seed']), {})[r['name']] = r
    return rows, by


def gates(rows, by, seeds):
    """Declared in advance; a failure voids the comparison."""
    out = []
    bad = [r for r in rows if r['status'] != 'ok'
           or float(r['gt_path_m'] or 0) <= 5.0]
    out.append(('all runs valid', not bad,
                f'{len(bad)} invalid' if bad else f'{len(rows)} runs ok'))

    def mean(arm, key):
        return float(np.mean([float(by[s][arm][key]) for s in seeds]))

    ra, rb = mean(ARM_A, 'revisit_count'), mean(ARM_B, 'revisit_count')
    dev = abs(ra - rb) / max(ra, rb) if max(ra, rb) else 1.0
    out.append(('matched cadence within 15%', dev <= 0.15,
                f'{ra:.2f} vs {rb:.2f} ({100*dev:.1f}% apart)'))

    pa, pb = mean(ARM_A, 'gt_path_m'), mean(ARM_B, 'gt_path_m')
    devp = abs(pa - pb) / max(pa, pb) if max(pa, pb) else 1.0
    out.append(('comparable travel within 10%', devp <= 0.10,
                f'{pa:.1f} vs {pb:.1f} m ({100*devp:.1f}% apart)'))
    return out


def main():
    pattern = (sys.argv[1] if len(sys.argv) > 1
               else 'eval/runs/trigger_confirm_*/summary.csv')
    rows, by = load(pattern)
    seeds = sorted(k for k, v in by.items() if ARM_A in v and ARM_B in v)
    if not seeds:
        raise SystemExit(f'no paired seeds found in {pattern}')
    print(f'Confirmatory analysis, {len(seeds)} paired seeds: '
          f'{seeds[0]}-{seeds[-1]}\n')

    print('Validity gates (declared in advance; a failure voids the run)')
    ok = True
    for name, passed, detail in gates(rows, by, seeds):
        ok &= passed
        print(f'  [{"PASS" if passed else "FAIL"}] {name:32s} {detail}')
    if not ok:
        print('\nVOID: a declared gate failed. Report the failure; do not '
              'switch endpoint or re-derive the control on these seeds.')

    a = np.array([float(by[s][ARM_A][PRIMARY]) for s in seeds])
    b = np.array([float(by[s][ARM_B][PRIMARY]) for s in seeds])
    d = a - b
    t, p = stats.ttest_rel(a, b)
    _, pw = stats.wilcoxon(a, b)
    ci = stats.t.ppf(1 - ALPHA / 2, len(d) - 1) * d.std(ddof=1) / np.sqrt(len(d))
    print(f'\nPRIMARY ENDPOINT: {PRIMARY}')
    print(f'  {ARM_A:12s} {a.mean():.4f} +- {a.std(ddof=1):.4f}')
    print(f'  {ARM_B:12s} {b.mean():.4f} +- {b.std(ddof=1):.4f}')
    print(f'  paired difference {d.mean():+.4f} m, 95% CI '
          f'[{d.mean()-ci:+.4f}, {d.mean()+ci:+.4f}]')
    print(f'  relative change {100*d.mean()/b.mean():+.1f}%, '
          f'{int((d < 0).sum())}/{len(d)} seeds favour {ARM_A}')
    print(f'  paired t-test  p = {p:.4f}  ({"REJECT H0" if p < ALPHA else "fail to reject H0"})')
    print(f'  Wilcoxon       p = {pw:.4f}')
    if (p < ALPHA) != (pw < ALPHA):
        print('  NOTE: the two tests disagree. That disagreement is the '
              'finding; neither is promoted over the other.')

    print('\nSecondary (descriptive only, no inferential claim)')
    for m in SECONDARY:
        try:
            x = np.array([float(by[s][ARM_A][m]) for s in seeds])
            y = np.array([float(by[s][ARM_B][m]) for s in seeds])
        except ValueError:
            continue
        print(f'  {m:16s} {ARM_A} {x.mean():8.3f} +-{x.std(ddof=1):6.3f} | '
              f'{ARM_B} {y.mean():8.3f} +-{y.std(ddof=1):6.3f} | '
              f'diff {x.mean()-y.mean():+8.3f}')


if __name__ == '__main__':
    main()
