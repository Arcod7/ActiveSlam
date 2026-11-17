#!/usr/bin/env python3
"""Backfill 3-D map-overlap metrics for TSDF batches from saved surface clouds.

The live tsdf backend records coverage (recall) and chamfer but no bounded
penalty for hallucinated surface. This recomputes, per run, from
maps/belief_tsdf.npy and maps/gt_tsdf.npy (same frame, same session — no
alignment needed; do NOT substitute the static eval/ground_truth references,
their object pose differs):

  iou_vox    occupied-voxel IoU at the run's voxel size (shell-vs-shell;
             grid-origin sensitivity measured ~±0.04 — report beside fscore,
             never alone)
  precision  fraction of belief points within tau of GT surface
  recall     fraction of GT points within tau of belief
  fscore     harmonic mean of the two

Recall here is NOT the live coverage column: the live metric subsamples the
belief reference to 20k points (map_metrics.py), which depresses it in a
map-size-dependent way; this script queries against the full-resolution
reference. The GT reference is the run's own observed cloud, so its extent is
policy-endogenous — compare arms only alongside explored volume, or rebuild
against a common union reference. Values are only comparable within one mapper
configuration; they are not the octomap backend's 2-D iou_occ. Pure numpy:
neighbours are found by binning one cloud into tau-sized cells and testing
exact distances against the 27 surrounding cells of each (subsampled) query
point.

Usage: map_overlap_offline.py <batch_dir> [--tau 0.4] [--voxel 0.25]
Writes <batch_dir>/map_overlap_offline.csv and prints per-arm medians.
"""

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path

import numpy as np

SUBSAMPLE = 20000
NEIGHBOUR_OFFSETS = [(dx, dy, dz) for dx in (-1, 0, 1)
                     for dy in (-1, 0, 1) for dz in (-1, 0, 1)]


def _cells(pts: np.ndarray, size: float) -> np.ndarray:
    return np.floor(pts / size).astype(np.int64)


def _cell_keys(cells: np.ndarray) -> np.ndarray:
    # 21 bits per axis, offset to stay positive; collision-free for |cell| < 2^20
    c = cells + (1 << 20)
    return (c[:, 0] << 42) | (c[:, 1] << 21) | c[:, 2]


def voxel_iou(a: np.ndarray, b: np.ndarray, voxel: float) -> float:
    ka = np.unique(_cell_keys(_cells(a, voxel)))
    kb = np.unique(_cell_keys(_cells(b, voxel)))
    inter = np.intersect1d(ka, kb, assume_unique=True).size
    union = ka.size + kb.size - inter
    return inter / union if union else float('nan')


def fraction_within(query: np.ndarray, ref: np.ndarray, tau: float,
                    rng: np.random.Generator) -> float:
    """Exact fraction of (subsampled) query points with a ref point within tau."""
    if len(query) == 0 or len(ref) == 0:
        return float('nan')
    if len(query) > SUBSAMPLE:
        query = query[rng.choice(len(query), SUBSAMPLE, replace=False)]
    bins = defaultdict(list)
    for i, key in enumerate(_cell_keys(_cells(ref, tau))):
        bins[key].append(i)
    bins = {k: ref[v] for k, v in bins.items()}
    tau2 = tau * tau
    qcells = _cells(query, tau)
    hits = 0
    for p, cell in zip(query, qcells):
        for off in NEIGHBOUR_OFFSETS:
            c = cell + off
            cand = bins.get(((c[0] + (1 << 20)) << 42)
                            | ((c[1] + (1 << 20)) << 21) | (c[2] + (1 << 20)))
            if cand is not None:
                d = cand - p
                if (np.einsum('ij,ij->i', d, d) <= tau2).any():
                    hits += 1
                    break
    return hits / len(query)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('batch_dir', type=Path)
    ap.add_argument('--tau', type=float, default=0.4,
                    help='match radius, default tsdf_coverage_radius_m')
    ap.add_argument('--voxel', type=float, default=None,
                    help='voxel size for IoU; default: read from manifest.json')
    args = ap.parse_args()

    rows = []
    for run_dir in sorted(args.batch_dir.iterdir()):
        belief_p = run_dir / 'maps' / 'belief_tsdf.npy'
        gt_p = run_dir / 'maps' / 'gt_tsdf.npy'
        if not (belief_p.exists() and gt_p.exists()):
            continue
        belief, gt = np.load(belief_p), np.load(gt_p)
        voxel = args.voxel
        if voxel is None:
            manifest_p = run_dir / 'manifest.json'
            if not manifest_p.exists():
                print(f'{run_dir.name}: skipped (no manifest.json; pass --voxel)')
                continue
            manifest = json.loads(manifest_p.read_text())
            voxel = float(manifest.get('args', {}).get('voxel_size', 0.25))
        # independent generators: each number is a pure function of its inputs
        precision = fraction_within(belief, gt, args.tau, np.random.default_rng(0))
        recall = fraction_within(gt, belief, args.tau, np.random.default_rng(1))
        fscore = (2 * precision * recall / (precision + recall)
                  if precision + recall else 0.0)
        arm, _, seed = run_dir.name.rpartition('_s')
        rows.append({'run': run_dir.name, 'arm': arm, 'seed': seed,
                     'n_belief': len(belief), 'n_gt': len(gt),
                     'iou_vox': round(voxel_iou(belief, gt, voxel), 6),
                     'precision': round(precision, 6),
                     'recall': round(recall, 6), 'fscore': round(fscore, 6)})
        print(f"{run_dir.name}: P={precision:.3f} R={recall:.3f} "
              f"F={fscore:.3f} IoU={rows[-1]['iou_vox']:.3f}")

    if not rows:
        raise SystemExit(f'no runs with maps/*.npy under {args.batch_dir}')

    out = args.batch_dir / 'map_overlap_offline.csv'
    with out.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f'\nwrote {out}')

    by_arm = defaultdict(list)
    for r in rows:
        by_arm[r['arm']].append(r)
    print('\narm medians (n): precision / recall / fscore / iou_vox')
    for arm, group in by_arm.items():
        med = lambda k: statistics.median(r[k] for r in group)
        print(f"  {arm} ({len(group)}): {med('precision'):.3f} / "
              f"{med('recall'):.3f} / {med('fscore'):.3f} / {med('iou_vox'):.3f}")


if __name__ == '__main__':
    main()
