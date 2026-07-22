"""Tests for run_matrix.py's run-validity checks — the guards that catch a
batch which is producing well-formed files for a vehicle that never moved.

This class of failure (orphaned nodes from a previous run, a motion gate that
was never armed, a stalled simulator) writes complete-looking metrics.csv,
map_metrics.csv and TUM files, so it stays invisible until someone reads a
trajectory. That cost a full 6-run batch once; these tests pin the detection.

Run: python3 -m pytest eval/eval_tools/test/ -q
"""
import importlib.util
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'run_matrix.py')
_spec = importlib.util.spec_from_file_location('run_matrix', os.path.abspath(_SCRIPT))
run_matrix = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(run_matrix)


def _write_tum(path, xy, yaw_deg):
    """Minimal TUM file: timestamp x y z qx qy qz qw."""
    rows = []
    for i, ((x, y), yaw) in enumerate(zip(xy, yaw_deg)):
        half = np.radians(yaw) / 2.0
        rows.append([float(i), x, y, 8.0, 0.0, 0.0, np.sin(half), np.cos(half)])
    np.savetxt(path, np.array(rows))
    return str(path)


def _frozen_tum(tmp_path, n=200):
    """A vehicle sitting still: sub-centimetre jitter, sub-degree yaw noise."""
    rng = np.random.default_rng(0)
    xy = rng.normal(0, 0.002, size=(n, 2))
    yaw = rng.normal(0, 0.4, size=n)
    return _write_tum(tmp_path / 'gt_traj.tum', xy, yaw)


def _moving_tum(tmp_path, n=200):
    """A vehicle driving a 20 m line while turning through 180°."""
    xy = np.column_stack([np.linspace(0, 20, n), np.zeros(n)])
    yaw = np.linspace(-90, 90, n)
    return _write_tum(tmp_path / 'gt_traj.tum', xy, yaw)


# ----------------------------------------------------------------------
# trajectory_motion
# ----------------------------------------------------------------------

def test_moving_trajectory_reports_real_motion(tmp_path):
    m = run_matrix.trajectory_motion(_moving_tum(tmp_path))
    assert m['path_m'] == pytest.approx(20.0, rel=1e-3)
    assert m['net_m'] == pytest.approx(20.0, rel=1e-3)
    assert m['yaw_deg'] == pytest.approx(180.0, abs=1.0)


def test_frozen_trajectory_reports_no_motion(tmp_path):
    m = run_matrix.trajectory_motion(_frozen_tum(tmp_path))
    assert m['path_m'] < run_matrix.MIN_PATH_M
    assert m['yaw_deg'] < run_matrix.MIN_YAW_DEG


def test_missing_or_empty_trajectory_is_not_an_error(tmp_path):
    assert run_matrix.trajectory_motion(str(tmp_path / 'nope.tum'))['samples'] == 0
    empty = tmp_path / 'empty.tum'
    empty.write_text('')
    assert run_matrix.trajectory_motion(str(empty))['samples'] == 0


def test_scanning_in_place_counts_as_motion(tmp_path):
    # The floors must not flag a run that is still doing its initial scan:
    # no translation at all, but a full sweep of yaw.
    path = _write_tum(tmp_path / 'gt_traj.tum',
                      np.zeros((100, 2)), np.linspace(0, 170, 100))
    m = run_matrix.trajectory_motion(path)
    assert m['path_m'] < run_matrix.MIN_PATH_M
    assert m['yaw_deg'] > run_matrix.MIN_YAW_DEG


# ----------------------------------------------------------------------
# _check_validity status classification
# ----------------------------------------------------------------------

def _run_dir(tmp_path, tum_writer, metrics_rows=100, map_rows=20):
    d = tmp_path / 'run'
    d.mkdir()
    tum_writer(d)
    with open(d / 'metrics.csv', 'w') as f:
        f.write('t,abs_error,ate,lc_count\n')
        for i in range(metrics_rows):
            f.write(f'{i},0.1,0.1,0\n')
    with open(d / 'map_metrics.csv', 'w') as f:
        f.write('t,coverage\n')
        for i in range(map_rows):
            f.write(f'{i},0.9\n')
    (d / 'launch.log').write_text('nothing interesting\n')
    return str(d), str(d / 'launch.log')


def test_moving_run_with_full_csvs_is_ok(tmp_path):
    d, log = _run_dir(tmp_path, _moving_tum)
    assert run_matrix._check_validity(d, log)['status'] == 'ok'


def test_frozen_run_with_full_csvs_is_motionless(tmp_path):
    # The regression this whole file exists for: every file is well-formed and
    # long enough, and the run is still worthless.
    d, log = _run_dir(tmp_path, _frozen_tum)
    result = run_matrix._check_validity(d, log)
    assert result['status'] == 'motionless'
    assert result['metrics_rows'] == 100
    assert result['gt_path_m'] < run_matrix.MIN_PATH_M


def test_motion_fields_are_reported_for_every_run(tmp_path):
    d, log = _run_dir(tmp_path, _moving_tum)
    result = run_matrix._check_validity(d, log)
    assert result['gt_path_m'] == pytest.approx(20.0, rel=1e-2)
    assert result['gt_yaw_deg'] == pytest.approx(180.0, abs=1.0)


def test_empty_metrics_still_reports_no_data(tmp_path):
    d, log = _run_dir(tmp_path, _frozen_tum, metrics_rows=0, map_rows=0)
    assert run_matrix._check_validity(d, log)['status'] == 'no_data'


# ----------------------------------------------------------------------
# orphan detection
# ----------------------------------------------------------------------

def test_orphan_patterns_cover_the_nodes_that_froze_a_batch():
    # Every one of these was found alive after a killed run and, duplicated,
    # is enough to stop the vehicle moving.
    for node in ('stonefish_simulator', 'octomap_server_node', 'odom_tf_sync',
                 'motion_safety_gate', 'heavy_sim_mixer'):
        assert node in run_matrix.ORPHAN_PATTERNS


def test_orphan_scan_ignores_the_orchestrator_itself():
    # find_orphan_nodes() greps ps output; matching its own command line would
    # make every batch refuse to start.
    assert all('run_matrix.py' not in entry
               for entry in run_matrix.find_orphan_nodes())
