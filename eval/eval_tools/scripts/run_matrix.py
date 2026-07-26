#!/usr/bin/env python3
"""Batch evaluation orchestrator: runs bringup/demo.launch.py over a matrix of
(config x seed) combinations, headless, one at a time, and aggregates the
results. ``status`` is structural validity only: it catches missing/short data,
frozen motion and process failures, but deliberately does not impose an ATE or
map-quality threshold.

Not a ROS node / console_script -- a plain script, run directly with python3
inside the activeslam-jazzy distrobox (needs `source install/setup.bash` first, so
`eval_tools.plot_results` is importable):

  python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_full.yaml
  python3 eval/eval_tools/scripts/run_matrix.py eval/eval_tools/config/matrix_smoke.yaml --dry-run
  python3 eval/eval_tools/scripts/run_matrix.py --aggregate-only eval/runs/full_20260706_2200

There is no natural end-of-run signal anywhere in the stack (frontier
exploration keeps spinning after "no frontiers found" rather than publishing
completion) -- duration_s IS the termination mechanism. benchmark.py and
map_metrics.py flush every row as it's written, so a SIGINT/SIGKILL-truncated
run still leaves valid, partial CSV/TUM files.

Matrix YAML schema:
  batch_name: <str>                  # default 'batch'
  duration_s: <float>                # per-run wall-clock budget, default 480
  settle_s: <float>                  # pause between runs, default 15
  seeds: [<int>, ...]                # forwarded as noise_seed:=<seed>
  common_args: {<demo.launch.py arg>: <value>, ...}
  runs:
    - name: <str>
      args: {<demo.launch.py arg>: <value>, ...}   # overrides common_args
"""
import argparse
import csv
import json
import os
import shlex
import signal
import subprocess
import sys
import time

import yaml
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
DEFAULT_BATCH_ROOT = os.path.join(REPO_ROOT, 'eval', 'runs')

# demo.launch.py arguments the matrix is allowed to set (bringup/launch/demo.launch.py).
VALID_KEYS = {
    'slam', 'mode', 'motion', 'mapper', 'noise_profile', 'loop_closure',
    'noise_profile_pressure', 'noise_profile_imu', 'noise_profile_compass',
    'noise_profile_dvl', 'noise_profile_sonar',
    'map_rebuild', 'rviz', 'revisit', 'scenario', 'scenario_out_dx', 'scenario_out_dy',
    'mission_waypoints', 'mission_loop',
    # The motion gate is fail-closed (safety_gate.py start_enabled=False) and
    # zeroes every command until armed — sim batches must opt in explicitly.
    'safety_start_enabled', 'scan_style', 'scan_sweep_deg', 'carve_no_return',
    'tsdf_octomap',
    'near_cutoff', 'near_fade', 'near_fade_range',
    'scene', 'obj_mesh', 'obj_x', 'obj_y', 'obj_z', 'obj_scale',
    'obj_roll', 'obj_pitch', 'obj_yaw',
    'robot_x', 'robot_y', 'robot_z', 'robot_roll', 'robot_pitch', 'robot_yaw', 'depth',
    # Motion tuning and the revisit trigger, so a batch can pin the same demo
    # config an interactive session was tuned to.
    'speed_factor', 'turn_factor', 'thrust_boost',
    'sigma_allow_xy_m', 'sigma_allow_yaw_rad',
    'ratio_trigger', 'ratio_resume',
    'wall_orientation_offset_deg', 'wall_orientation_lookahead_m',
    'tsdf_frontier_standoff_m',
    # Map resolution and wall thresholds, and the A* inflation radii: a batch
    # that cannot set these cannot reproduce a tuned interactive config.
    'voxel_size', 'voxel_min_weight', 'voxel_min_solid_confidence',
    'hard_inflation_m', 'inflation_m', 'plan_inflation_m',
    # The rest of the wall-oriented executor's geometry.
    'wall_standoff', 'wall_switch_goal_distance', 'wall_switch_scan_angle',
    'wall_switch_scan_yaw', 'wall_path_influence', 'wall_path_look_offset_deg',
    'wall_normal_offset_deg', 'wall_path_heading_weight',
    # Survey working area: bounds frontier exploration to the structure.
    'survey_radius_m', 'survey_center_x', 'survey_center_y',
}


# Node executables a previous run leaves behind if it was killed without its
# process group. Any of these still alive means duplicate publishers on
# /motion/body_command_safe and /tf, which silently freezes the vehicle.
ORPHAN_PATTERNS = (
    'stonefish_simulator', 'octomap_server_node', 'component_container',
    'odom_tf_sync', 'cloud_relabel', 'motion_safety_gate', 'heavy_sim_mixer',
    'pose_graph', 'tsdf_mapper', 'waypoint_controller', 'frontier_extractor',
    'benchmark', 'map_metrics',
)

# A run that never moves produces plausible-looking CSVs, so the numbers alone
# do not reveal it. These are floors, not targets: even a scan-in-place start
# clears them within the first minute.
MIN_PATH_M = 1.0
MIN_YAW_DEG = 20.0
MOTION_CHECK_AT_S = 90.0


def _command_executables(cmd: str) -> set:
    """Executable basenames represented by a process command line.

    ROS Python console scripts appear as ``python3 /.../pose_graph ...`` while
    C++ nodes appear directly as ``/.../component_container ...``.  Looking at
    only those positions avoids treating an unrelated ``rg pose_graph`` or an
    editor command line as a live ROS node.
    """
    try:
        words = shlex.split(cmd)
    except ValueError:
        words = cmd.split()
    if not words:
        return set()
    names = {os.path.basename(words[0])}
    if names & {'python', 'python2', 'python3', 'python3.12'} and len(words) > 1:
        names.add(os.path.basename(words[1]))
    return names


def _pid_ros_domain(pid: str):
    """ROS_DOMAIN_ID for a process, or None when its environment is unreadable."""
    try:
        with open(f'/proc/{pid}/environ', 'rb') as f:
            entries = f.read().split(b'\0')
    except (OSError, ValueError):
        return None
    prefix = b'ROS_DOMAIN_ID='
    for entry in entries:
        if entry.startswith(prefix):
            return entry[len(prefix):].decode(errors='replace') or '0'
    return '0'


def find_orphan_nodes() -> list:
    """Return same-domain leftover ROS nodes as ``['pid cmdline', ...]``."""
    try:
        out = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    except Exception:
        return []
    mine = str(os.getpid())
    found = []
    current_domain = str(os.environ.get('ROS_DOMAIN_ID', '0'))
    for line in out.splitlines()[1:]:
        pid, _, cmd = line.strip().partition(' ')
        if pid == mine or 'run_matrix.py' in cmd:
            continue
        if not (_command_executables(cmd) & set(ORPHAN_PATTERNS)):
            continue
        process_domain = _pid_ros_domain(pid)
        # An unreadable environment stays a conservative match.  A known
        # different ROS domain cannot exchange topics with this batch and is
        # therefore safe to ignore.
        if process_domain is not None and process_domain != current_domain:
            continue
        found.append(f'{pid} {cmd[:100]}')
    return found


def trajectory_motion(tum_path: str) -> dict:
    """Path length, net displacement and yaw range from a TUM trajectory."""
    zero = {'samples': 0, 'path_m': 0.0, 'net_m': 0.0, 'yaw_deg': 0.0}
    if not os.path.exists(tum_path):
        return zero
    if os.path.getsize(tum_path) == 0:
        return zero
    try:
        data = np.loadtxt(tum_path, ndmin=2)
    except Exception:
        return zero
    if len(data) < 2 or data.shape[1] < 8:
        return zero
    xy = data[:, 1:3]
    qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy),
                               1 - 2 * (qy ** 2 + qz ** 2)))
    return {
        'samples': len(data),
        'path_m': float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
        'net_m': float(np.linalg.norm(xy[-1] - xy[0])),
        'yaw_deg': float(np.degrees(yaw.max() - yaw.min())),
    }


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ['git', '-C', REPO_ROOT, 'rev-parse', 'HEAD'], text=True).strip()
    except Exception:
        return 'unknown'


def load_matrix(config_path: str) -> dict:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    cfg.setdefault('batch_name', 'batch')
    cfg.setdefault('duration_s', 480)
    cfg.setdefault('settle_s', 15)
    cfg.setdefault('seeds', [-1])
    cfg.setdefault('common_args', {})
    cfg.setdefault('runs', [])
    _validate_matrix(cfg)
    return cfg


def _validate_matrix(cfg: dict) -> None:
    if not cfg['runs']:
        raise ValueError('matrix config has no runs')
    for run in cfg['runs']:
        if 'name' not in run:
            raise ValueError(f"run missing 'name': {run}")
        args = resolve_args(cfg, run)
        unknown = set(args) - VALID_KEYS
        if unknown:
            raise ValueError(f"run '{run['name']}': unknown arg keys {unknown}")
        if args.get('motion') == 'wallfollow' and args.get('mapper') != 'tsdf':
            raise ValueError(
                f"run '{run['name']}': motion=wallfollow requires mapper=tsdf")


def resolve_args(cfg: dict, run: dict) -> dict:
    return {**cfg['common_args'], **run.get('args', {})}


def build_command(args: dict, output_dir: str, seed: int) -> list:
    cmd = ['ros2', 'launch', 'bringup', 'demo.launch.py']
    for k, v in args.items():
        cmd.append(f'{k}:={v}')
    cmd.append(f'output_dir:={output_dir}')
    cmd.append(f'noise_seed:={seed}')
    return cmd


def _terminate(proc: subprocess.Popen) -> None:
    """SIGINT -> SIGTERM -> SIGKILL escalation on the whole process group
    (Stonefish's GL context can be slow to tear down cleanly)."""
    for sig, grace in ((signal.SIGINT, 30), (signal.SIGTERM, 15), (signal.SIGKILL, 10)):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
            proc.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def _launch_and_wait(cmd: list, log_path: str, duration_s: float,
                     output_dir: str = None):
    """Run one launch, aborting early if the vehicle never starts moving.

    The motion check is the whole point of the early abort: a frozen run still
    writes metrics.csv, map_metrics.csv and TUM files, so it is indistinguishable
    from a real one until someone reads the trajectory. Checking at
    MOTION_CHECK_AT_S turns an 8-minute waste into a 90-second one."""
    start = time.strftime('%Y-%m-%dT%H:%M:%S')
    motionless = None
    with open(log_path, 'w') as log_file:
        proc = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            if output_dir and duration_s > MOTION_CHECK_AT_S * 1.5:
                proc.wait(timeout=MOTION_CHECK_AT_S)
            else:
                proc.wait(timeout=duration_s)
        except subprocess.TimeoutExpired:
            if output_dir and duration_s > MOTION_CHECK_AT_S * 1.5:
                motion = trajectory_motion(os.path.join(output_dir, 'gt_traj.tum'))
                if (motion['path_m'] < MIN_PATH_M
                        and motion['yaw_deg'] < MIN_YAW_DEG):
                    motionless = motion
                    print(f"  [abort] no motion after {MOTION_CHECK_AT_S:.0f}s "
                          f"(path {motion['path_m']:.2f}m, yaw "
                          f"{motion['yaw_deg']:.1f}deg) — killing this run")
                    _terminate(proc)
                else:
                    try:
                        proc.wait(timeout=duration_s - MOTION_CHECK_AT_S)
                    except subprocess.TimeoutExpired:
                        _terminate(proc)
            else:
                _terminate(proc)
    end = time.strftime('%Y-%m-%dT%H:%M:%S')
    return start, end, proc.returncode, motionless


def _count_data_rows(csv_path: str) -> int:
    if not os.path.exists(csv_path):
        return 0
    with open(csv_path) as f:
        return max(0, sum(1 for _ in f) - 1)   # minus header


def _count_tracebacks(log_path: str) -> tuple:
    """Return (harmful, benign) traceback counts from a launch.log.

    A traceback whose final line is KeyboardInterrupt is benign: a second
    SIGINT arriving during shutdown can land anywhere (even inside a
    library call in a callback) and does not indicate a fault -- see the
    node teardown fix in pose_graph.py and friends. Any traceback whose
    header appears after launch's "user interrupted with ctrl-c (SIGINT)"
    line is also benign regardless of exception type: teardown races that
    live upstream (e.g. rclpy's pybind11 layer) can raise other exception
    types too, and phase (before/after the SIGINT was issued) is the
    principled signal, not the exception class. Everything else, or a
    traceback that never resolves before EOF, is counted harmful
    (conservative). Log lines are grouped by their `[proc-name-N]` prefix
    so interleaved output from concurrent processes doesn't confuse the
    per-traceback state machine.
    """
    if not os.path.exists(log_path):
        return 0, 0

    harmful = 0
    benign = 0
    in_traceback: dict = {}         # prefix -> bool
    after_sigint: dict = {}         # prefix -> bool (traceback header seen post-SIGINT)
    sigint_seen = False

    with open(log_path, errors='replace') as f:
        for raw_line in f:
            line = raw_line.rstrip('\n')
            if line.startswith('[') and '] ' in line:
                prefix, content = line.split('] ', 1)
            else:
                prefix, content = '', line

            if 'user interrupted with ctrl-c (SIGINT)' in content:
                sigint_seen = True

            if in_traceback.get(prefix):
                # Blank lines occur mid-traceback too (e.g. a frame whose
                # source line can't be looked up, such as a C-implemented
                # property) -- only a non-empty, non-indented line is the
                # actual exception summary.
                if content == '' or content.startswith((' ', '\t')):
                    continue
                in_traceback[prefix] = False   # first non-indented line = exception summary
                if content.startswith('KeyboardInterrupt') or after_sigint.get(prefix):
                    benign += 1
                else:
                    harmful += 1
                continue

            if content == 'Traceback (most recent call last):':
                in_traceback[prefix] = True
                after_sigint[prefix] = sigint_seen

    harmful += sum(1 for unresolved in in_traceback.values() if unresolved)
    return harmful, benign


def _count_process_deaths(log_path: str) -> int:
    """Count ROS child-process deaths before the orchestrator starts teardown."""
    if not os.path.exists(log_path):
        return 0
    deaths = 0
    with open(log_path, errors='replace') as f:
        for line in f:
            if 'user interrupted with ctrl-c (SIGINT)' in line:
                break
            if 'process has died' in line:
                deaths += 1
    return deaths


def _check_validity(output_dir: str, log_path: str, exit_code=None) -> dict:
    metrics_rows = _count_data_rows(os.path.join(output_dir, 'metrics.csv'))
    map_metrics_rows = _count_data_rows(os.path.join(output_dir, 'map_metrics.csv'))
    tracebacks, benign_tracebacks = _count_tracebacks(log_path)
    process_deaths = _count_process_deaths(log_path)
    motion = trajectory_motion(os.path.join(output_dir, 'gt_traj.tum'))

    # Ordered by how badly the run is broken. `motionless` outranks a traceback:
    # a vehicle that never moved produces no usable data whatever else happened.
    if metrics_rows == 0:
        status = 'no_data'
    elif motion['path_m'] < MIN_PATH_M and motion['yaw_deg'] < MIN_YAW_DEG:
        status = 'motionless'
    elif (tracebacks > 0 or process_deaths > 0
          or exit_code not in (None, 0, -signal.SIGINT)):
        status = 'crashed_soft'
    elif metrics_rows < 30 or map_metrics_rows < 5:
        status = 'short'
    else:
        status = 'ok'

    return {'metrics_rows': metrics_rows, 'map_metrics_rows': map_metrics_rows,
            'tracebacks': tracebacks, 'benign_tracebacks': benign_tracebacks,
            'process_deaths': process_deaths,
            'gt_path_m': round(motion['path_m'], 3),
            'gt_net_m': round(motion['net_m'], 3),
            'gt_yaw_deg': round(motion['yaw_deg'], 1),
            'status': status}


def run_one(args: dict, output_dir: str, seed: int, duration_s: float,
            dry_run: bool = False) -> dict:
    cmd = build_command(args, output_dir, seed)
    manifest = {'args': args, 'seed': seed, 'git_sha': git_sha(),
                'command': cmd, 'duration_s': duration_s}

    if dry_run:
        print(' '.join(cmd))
        return manifest

    os.makedirs(output_dir, exist_ok=True)
    log_path = os.path.join(output_dir, 'launch.log')
    manifest['start'], manifest['end'], manifest['exit_code'], motionless = \
        _launch_and_wait(cmd, log_path, duration_s, output_dir)
    manifest.update(_check_validity(output_dir, log_path, manifest['exit_code']))
    manifest['status_scope'] = 'structural_validity_only'
    manifest['aborted_motionless'] = motionless is not None
    manifest['retried'] = False

    # One retry, only when the sim produced literally nothing (failed to
    # start) -- SLAM divergence or a short-but-nonzero run is signal, not a
    # transient fault, so it is NOT retried.
    if manifest['metrics_rows'] == 0:
        print(f'  [retry] {output_dir}: 0 metrics rows, retrying once')
        manifest['retried'] = True
        manifest['start'], manifest['end'], manifest['exit_code'], _ = \
            _launch_and_wait(cmd, log_path, duration_s, output_dir)
        manifest.update(_check_validity(output_dir, log_path, manifest['exit_code']))

    with open(os.path.join(output_dir, 'manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2, default=str)

    try:
        from eval_tools.plot_results import plot_run
        plot_run(output_dir)
    except Exception as exc:
        print(f'  [warn] plot_results failed for {output_dir}: {exc}')

    return manifest


def run_matrix(cfg: dict, batch_root: str, dry_run: bool = False) -> str:
    batch_dir = os.path.join(
        batch_root, f"{cfg['batch_name']}_{time.strftime('%Y%m%d_%H%M')}")

    # Before creating anything: a refused batch should leave no empty directory
    # behind to be mistaken later for a run that produced nothing.
    if not dry_run:
        orphans = find_orphan_nodes()
        if orphans:
            raise SystemExit(
                'Refusing to start: ROS nodes from an earlier run are still '
                'alive. Duplicate motion gates and TF broadcasters freeze the '
                'vehicle, and every run in the batch would be wasted.\n  '
                + '\n  '.join(orphans)
                + '\nKill them (by PID, not the ros2 launch wrapper) and re-run.')
        os.makedirs(batch_dir, exist_ok=True)

    total = len(cfg['runs']) * len(cfg['seeds'])
    est_hours = total * (cfg['duration_s'] + 50 + cfg['settle_s']) / 3600
    print(f"Batch '{cfg['batch_name']}' -> {batch_dir}")
    print(f"{len(cfg['runs'])} configs x {len(cfg['seeds'])} seeds = {total} runs, "
          f"duration_s={cfg['duration_s']}, est. wall time ~{est_hours:.1f}h")

    manifests = []
    i = 0
    for run in cfg['runs']:
        args = resolve_args(cfg, run)
        for seed in cfg['seeds']:
            i += 1
            run_dir = os.path.join(batch_dir, f"{run['name']}_s{seed}")
            print(f"[{i}/{total}] {run['name']} seed={seed} -> {run_dir}")
            manifest = run_one(args, run_dir, seed, cfg['duration_s'], dry_run=dry_run)
            manifest['name'] = run['name']
            manifest['run_dir'] = run_dir
            manifests.append(manifest)
            # Stop the whole batch on the first frozen run: the cause is always
            # environmental (orphans, a closed motion gate, a stalled sim) and
            # applies to every run that would follow.
            if not dry_run and manifest.get('status') == 'motionless':
                print(f"  [stop] {run_dir} never moved "
                      f"(path {manifest.get('gt_path_m')}m, "
                      f"yaw {manifest.get('gt_yaw_deg')}deg). "
                      f"Aborting the batch after {i}/{total} runs.")
                leftover = find_orphan_nodes()
                if leftover:
                    print('  Leftover nodes are the likely cause:\n  '
                          + '\n  '.join(leftover))
                break
            if not dry_run and i < total:
                time.sleep(cfg['settle_s'])
        else:
            continue
        break

    if not dry_run:
        aggregate(batch_dir, manifests)
    return batch_dir


def _csv_column(csv_path: str, col: str):
    if not os.path.exists(csv_path):
        return []
    vals = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            v = row.get(col, '')
            if v not in ('', None):
                vals.append(float(v))
    return vals


def _last(csv_path: str, col: str):
    vals = _csv_column(csv_path, col)
    return vals[-1] if vals else None


def _mean(csv_path: str, col: str):
    vals = _csv_column(csv_path, col)
    return float(np.mean(vals)) if vals else None


def aggregate(batch_dir: str, manifests: list = None) -> None:
    if manifests is None:
        manifests = []
        for entry in sorted(os.listdir(batch_dir)):
            run_dir = os.path.join(batch_dir, entry)
            manifest_path = os.path.join(run_dir, 'manifest.json')
            if not os.path.exists(manifest_path):
                continue
            with open(manifest_path) as f:
                m = json.load(f)
            m.setdefault('name', entry.rsplit('_s', 1)[0])
            m['run_dir'] = run_dir
            manifests.append(m)

    fieldnames = ['name', 'seed', 'status', 'gt_path_m', 'gt_yaw_deg',
                  'final_ate', 'mean_rpe_trans',
                  'final_abs_error', 'final_coverage', 'final_iou', 'final_chamfer',
                  'lc_count', 'rebuild_count', 'revisit_count', 'tracebacks',
                  'process_deaths']
    rows = []
    for m in manifests:
        run_dir = m['run_dir']
        metrics_csv = os.path.join(run_dir, 'metrics.csv')
        map_csv = os.path.join(run_dir, 'map_metrics.csv')
        rows.append({
            'name': m['name'], 'seed': m['seed'], 'status': m.get('status', 'unknown'),
            'gt_path_m': m.get('gt_path_m'), 'gt_yaw_deg': m.get('gt_yaw_deg'),
            'final_ate': _last(metrics_csv, 'ate'),
            'mean_rpe_trans': _mean(metrics_csv, 'rpe_trans'),
            'final_abs_error': _last(metrics_csv, 'abs_error'),
            'final_coverage': _last(map_csv, 'coverage'),
            'final_iou': _last(map_csv, 'iou_occ'),
            'final_chamfer': _last(map_csv, 'chamfer'),
            'lc_count': _last(metrics_csv, 'lc_count'),
            'rebuild_count': _last(metrics_csv, 'rebuild_count'),
            'revisit_count': _last(metrics_csv, 'revisit_count'),
            'tracebacks': m.get('tracebacks', 0),
            'process_deaths': m.get('process_deaths', 0),
        })

    summary_path = os.path.join(batch_dir, 'summary.csv')
    with open(summary_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f'Wrote {summary_path} ({len(rows)} rows)')

    _plot_comparisons(batch_dir, rows)


def _plot_comparisons(batch_dir: str, rows: list) -> None:
    if not rows:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    names = sorted({r['name'] for r in rows})
    metrics_to_plot = [
        ('final_ate', 'Final ATE (m)'),
        ('final_coverage', 'Final map coverage'),
        ('final_chamfer', 'Final TSDF chamfer (m)'),
    ]

    fig, axes = plt.subplots(1, len(metrics_to_plot), figsize=(6 * len(metrics_to_plot), 5))
    for ax, (metric, label) in zip(axes, metrics_to_plot):
        data = [[r[metric] for r in rows if r['name'] == n and r[metric] is not None]
                for n in names]
        if any(len(d) for d in data):
            ax.boxplot(data, labels=names)
        ax.set_title(label)
        ax.tick_params(axis='x', rotation=45)
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out_path = os.path.join(batch_dir, 'comparison.png')
    fig.savefig(out_path, dpi=150)
    print(f'Wrote {out_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('config', nargs='?', help='Path to a matrix YAML config')
    parser.add_argument('--batch-root', default=DEFAULT_BATCH_ROOT,
                         help=f'default: {DEFAULT_BATCH_ROOT}')
    parser.add_argument('--dry-run', action='store_true',
                         help='Print the resolved ros2 launch commands without running them')
    parser.add_argument('--aggregate-only', metavar='BATCH_DIR',
                         help='Re-aggregate an existing batch directory (reads manifest.json '
                              'files, no new runs)')
    parser.add_argument('--seeds', type=int, nargs='+', metavar='SEED',
                         help="Override the config's seed list. Extends an existing "
                              'sweep with more samples without editing the YAML: the '
                              'new runs land in their own batch directory, and the '
                              'plotting script pools batch directories by arm.')
    args = parser.parse_args()

    if args.aggregate_only:
        aggregate(args.aggregate_only)
        return

    if not args.config:
        parser.error('config is required unless --aggregate-only is given')

    cfg = load_matrix(args.config)
    if args.seeds:
        cfg['seeds'] = args.seeds
    run_matrix(cfg, args.batch_root, dry_run=args.dry_run)


if __name__ == '__main__':
    main()
