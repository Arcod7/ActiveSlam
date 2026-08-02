#!/usr/bin/env python3
"""Local web viewer for evaluation runs.

Serves trajectories, error/uncertainty/map metrics and the planner's session
logs for any run under eval/runs, with seed-to-seed navigation, a side-by-side
compare mode and a button that replays a seed's rosbag in RViz.

Runs on the host with the standard library plus numpy -- no ROS, no build step:

    python3 eval/eval_tools/viewer/server.py [run_name]
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import numpy as np

VIEWER_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(VIEWER_DIR, 'static')
EVAL_TOOLS_DIR = os.path.dirname(VIEWER_DIR)
REPO_ROOT = os.path.dirname(os.path.dirname(EVAL_TOOLS_DIR))

DISTROBOX_NAME = 'activeslam-jazzy'
MAX_SERIES_POINTS = 2000        # still well above the pixel width of a panel
MAX_CLOUD_POINTS = 400000
MAX_GRID_CELLS = 400
CLOUD_CELL_M = 0.25


def runs_root():
    """Evaluation runs directory, resolved exactly as the producers resolve it."""
    sys.path.insert(0, EVAL_TOOLS_DIR)
    try:
        from eval_tools.run_paths import runs_root as _root
        return _root()
    except Exception:
        override = os.environ.get('ACTIVESLAM_EVAL_RUNS')
        if override:
            return os.path.expanduser(override)
        return os.path.join(REPO_ROOT, 'eval', 'runs')
    finally:
        if sys.path and sys.path[0] == EVAL_TOOLS_DIR:
            sys.path.pop(0)


# --- parsing ---------------------------------------------------------------

def _indices(n, max_n):
    if n <= max_n:
        return list(range(n))
    return [int(round(i * (n - 1) / (max_n - 1))) for i in range(max_n)]


def _finite(value):
    if value is None:
        return None
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return None
    return value


def load_tum(path, max_n=MAX_SERIES_POINTS):
    """Timestamps and positions from a TUM trajectory file; quaternion dropped."""
    if not os.path.isfile(path):
        return None
    t, x, y, z = [], [], [], []
    with open(path, errors='replace') as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                values = [float(p) for p in parts[:4]]
            except ValueError:
                continue
            t.append(values[0])
            x.append(values[1])
            y.append(values[2])
            z.append(values[3])
    if not t:
        return None
    idx = _indices(len(t), max_n)
    return {
        't': [t[i] for i in idx],
        'x': [x[i] for i in idx],
        'y': [y[i] for i in idx],
        'z': [z[i] for i in idx],
        'n': len(t),
    }


def load_table(path, max_n=MAX_SERIES_POINTS, keep_events=True):
    """Header-driven CSV load; numeric columns where every value parses.

    Schemas drift across runs (metrics.csv is 13 or 23 columns, map_metrics.csv
    9 or 11), so nothing here may depend on column position.
    """
    if not os.path.isfile(path):
        return None
    try:
        with open(path, newline='', errors='replace') as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if not header:
                return None
            width = len(header)
            rows = [r for r in reader if len(r) == width]
    except OSError:
        return None
    if not rows:
        return None

    columns = {}
    for j, name in enumerate(header):
        raw = [r[j].strip() for r in rows]
        parsed, numeric = [], True
        for value in raw:
            if value == '':
                parsed.append(None)
                continue
            try:
                parsed.append(float(value))
            except ValueError:
                numeric = False
                break
        columns[name] = [_finite(v) for v in parsed] if numeric else [v or None for v in raw]

    events = []
    if keep_events and 'event' in columns and 't_ros' in columns:
        for stamp, label in zip(columns['t_ros'], columns['event']):
            if label and stamp is not None:
                events.append({'t': stamp, 'event': label})

    idx = _indices(len(rows), max_n)
    table = {name: [values[i] for i in idx] for name, values in columns.items()}
    return {'columns': table, 'full': columns, 'n': len(rows), 'events': events}


LOG_TARGET_RE = re.compile(r'logging to (\S+\.csv)')


def planner_log_paths(seed_dir):
    """Planner session CSVs for this seed, recovered from its launch.log.

    The nodes write to planner/frontier_slam/logs/<stamp>_<node>.csv, outside
    the run directory; the startup line in launch.log is the only join key.
    """
    log = os.path.join(seed_dir, 'launch.log')
    found = {}
    if not os.path.isfile(log):
        return found
    fallback_dir = os.path.join(REPO_ROOT, 'planner', 'frontier_slam', 'logs')
    with open(log, errors='replace') as handle:
        for line in handle:
            match = LOG_TARGET_RE.search(line)
            if not match:
                continue
            path = match.group(1)
            if not os.path.isfile(path):
                path = os.path.join(fallback_dir, os.path.basename(path))
                if not os.path.isfile(path):
                    continue
            stem = os.path.basename(path)[:-4]
            parts = stem.split('_', 2)
            node = parts[2] if len(parts) == 3 else stem
            found[node] = path
    return found


def state_bands(times, states, skip=('exploring',)):
    """Contiguous intervals of a categorical state column, for chart shading."""
    bands, current, start, last = [], None, None, None
    for stamp, state in zip(times, states):
        if stamp is None:
            continue
        if state != current:
            if current is not None and start is not None:
                bands.append({'state': current, 't0': start, 't1': stamp})
            current, start = state, stamp
        last = stamp
    if current is not None and start is not None and last is not None:
        bands.append({'state': current, 't0': start, 't1': last})
    return [b for b in bands if b['state'] and b['state'] not in skip]


def distinct_points(table, x_name, y_name, tolerance=1e-4):
    """Successive distinct x-y pairs from a planner log.

    Goals and revisit targets are re-logged every tick, so the raw columns hold
    thousands of repeats of a handful of positions. Collapsing them keeps the
    browser from drawing a marker per row.
    """
    if not table:
        return None
    columns = table.get('full') or table['columns']
    xs, ys = columns.get(x_name), columns.get(y_name)
    if not xs or not ys:
        return None
    out_x, out_y = [], []
    for x, y in zip(xs, ys):
        if x is None or y is None:
            continue
        if out_x and abs(x - out_x[-1]) < tolerance and abs(y - out_y[-1]) < tolerance:
            continue
        out_x.append(round(x, 3))
        out_y.append(round(y, 3))
    return {'x': out_x, 'y': out_y} if out_x else None


def _cloud_to_grid(cloud, cell=CLOUD_CELL_M):
    """Bin a TSDF surface cloud into a 2-D density grid.

    One heatmap image is far cheaper for the browser to draw than tens of
    thousands of individual markers, and it reads like the occupancy map.
    """
    x, y = cloud[:, 0], cloud[:, 1]
    x0, y0 = float(x.min()), float(y.min())
    span = max(float(x.max()) - x0, float(y.max()) - y0)
    cell = max(cell, span / MAX_GRID_CELLS) if span > 0 else cell
    width = int((float(x.max()) - x0) / cell) + 1
    height = int((float(y.max()) - y0) / cell) + 1
    grid = np.zeros((height, width), dtype=np.int32)
    np.add.at(grid, (((y - y0) / cell).astype(int), ((x - x0) / cell).astype(int)), 1)
    density = np.log1p(grid)
    top = float(density.max()) or 1.0
    return {
        'kind': 'grid',
        'z': [[None if v <= 0 else round(float(v) / top * 100.0, 1) for v in row]
              for row in density.tolist()],
        'resolution': cell,
        'x0': x0 + 0.5 * cell,
        'y0': y0 + 0.5 * cell,
    }


def load_map(seed_dir):
    """Belief map for the trajectory backdrop, always as a 2-D grid."""
    maps_dir = os.path.join(seed_dir, 'maps')
    if not os.path.isdir(maps_dir):
        return None
    grid_path = os.path.join(maps_dir, 'belief_projected_map.npy')
    meta_path = os.path.join(maps_dir, 'belief_projected_map.json')
    try:
        if os.path.isfile(grid_path) and os.path.isfile(meta_path):
            grid = np.load(grid_path)
            with open(meta_path) as handle:
                meta = json.load(handle)
            step = max(1, int(max(grid.shape) // MAX_GRID_CELLS) + (1 if max(grid.shape) > MAX_GRID_CELLS else 0))
            grid = grid[::step, ::step]
            resolution = float(meta['resolution']) * step
            return {
                'kind': 'grid',
                'z': [[None if v < 0 else float(v) for v in row] for row in grid.tolist()],
                'resolution': resolution,
                'x0': float(meta['origin_x']) + 0.5 * resolution,
                'y0': float(meta['origin_y']) + 0.5 * resolution,
            }
        cloud_path = os.path.join(maps_dir, 'belief_tsdf.npy')
        if os.path.isfile(cloud_path):
            cloud = np.load(cloud_path)
            if cloud.ndim != 2 or cloud.shape[1] < 3 or not len(cloud):
                return None
            if len(cloud) > MAX_CLOUD_POINTS:
                cloud = cloud[np.linspace(0, len(cloud) - 1, MAX_CLOUD_POINTS).round().astype(int)]
            return _cloud_to_grid(cloud[:, :3].astype(float))
    except Exception:
        return None
    return None


# --- run discovery ---------------------------------------------------------

def _has_bag(seed_dir):
    bag = os.path.join(seed_dir, 'bag')
    if not os.path.isdir(bag):
        return False
    return bool(glob.glob(os.path.join(bag, '*.mcap')) or glob.glob(os.path.join(bag, '*.mcap.zstd')))


def _read_json(path):
    try:
        with open(path, errors='replace') as handle:
            return json.load(handle)
    except Exception:
        return None


def _seed_sort_key(seed):
    return (seed['arm'], seed['seed'] if seed['seed'] is not None else -1, seed['dir'])


def list_seeds(run_dir):
    """Seed directories of a batch; older flat batches expose a single seed."""
    seeds = []
    try:
        names = sorted(os.listdir(run_dir))
    except OSError:
        return seeds
    for name in names:
        path = os.path.join(run_dir, name)
        if not os.path.isdir(path) or name in ('maps', 'bag'):
            continue
        if not (os.path.isfile(os.path.join(path, 'metrics.csv'))
                or os.path.isfile(os.path.join(path, 'manifest.json'))):
            continue
        arm, marker, tail = name.rpartition('_s')
        seed_number = None
        if marker and tail.isdigit():
            seed_number = int(tail)
        else:
            arm = name
        manifest = _read_json(os.path.join(path, 'manifest.json')) or {}
        seeds.append({
            'dir': name,
            'arm': arm,
            'seed': seed_number,
            'status': manifest.get('status'),
            'mapper': (manifest.get('args') or {}).get('mapper'),
            'has_bag': _has_bag(path),
        })
    if not seeds and os.path.isfile(os.path.join(run_dir, 'metrics.csv')):
        manifest = _read_json(os.path.join(run_dir, 'manifest.json')) or {}
        seeds.append({
            'dir': '.',
            'arm': os.path.basename(run_dir),
            'seed': None,
            'status': manifest.get('status'),
            'mapper': (manifest.get('args') or {}).get('mapper'),
            'has_bag': _has_bag(run_dir),
        })
    seeds.sort(key=_seed_sort_key)
    return seeds


def list_runs():
    root = runs_root()
    runs = []
    try:
        names = os.listdir(root)
    except OSError:
        return runs
    for name in names:
        path = os.path.join(root, name)
        if not os.path.isdir(path):
            continue
        progress = _read_json(os.path.join(path, 'progress.json')) or {}
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            mtime = 0.0
        runs.append({
            'name': name,
            'mtime': mtime,
            'running': progress.get('state') == 'running',
            'phase': progress.get('phase'),
            'has_summary': os.path.isfile(os.path.join(path, 'summary.csv')),
        })
    runs.sort(key=lambda r: r['mtime'], reverse=True)
    return runs


def _last_value(columns, name):
    values = [v for v in columns.get(name, []) if v is not None]
    return values[-1] if values else None


def build_tiles(manifest, metrics, map_metrics):
    metric_cols = metrics['columns'] if metrics else {}
    map_cols = map_metrics['columns'] if map_metrics else {}
    tiles = [
        {'label': 'Final ATE', 'value': _last_value(metric_cols, 'ate'), 'unit': 'm',
         'note': 'unaligned', 'digits': 3},
        {'label': 'Final position error', 'value': _last_value(metric_cols, 'abs_error'), 'unit': 'm',
         'note': 'unaligned', 'digits': 3},
        {'label': 'Map coverage', 'value': _last_value(map_cols, 'coverage'), 'unit': '', 'digits': 3},
    ]
    if _last_value(map_cols, 'iou_occ') is not None:
        tiles.append({'label': 'Map IoU', 'value': _last_value(map_cols, 'iou_occ'), 'unit': '', 'digits': 3})
    if _last_value(map_cols, 'chamfer') is not None:
        tiles.append({'label': 'Chamfer', 'value': _last_value(map_cols, 'chamfer'), 'unit': 'm', 'digits': 3})
    tiles += [
        {'label': 'Loop closures', 'value': _last_value(metric_cols, 'lc_count'), 'unit': '', 'digits': 0},
        {'label': 'Revisits', 'value': _last_value(metric_cols, 'revisit_count'), 'unit': '', 'digits': 0},
        {'label': 'Rebuilds', 'value': _last_value(metric_cols, 'rebuild_count'), 'unit': '', 'digits': 0},
    ]
    if _last_value(metric_cols, 'anees_robust') is not None:
        tiles.append({'label': 'Robust ANEES', 'value': _last_value(metric_cols, 'anees_robust'),
                      'unit': '', 'note': '3 DoF, consistent = 3', 'digits': 2})
    if manifest.get('gt_path_m') is not None:
        tiles.append({'label': 'Path length', 'value': manifest.get('gt_path_m'), 'unit': 'm', 'digits': 1})
    return [t for t in tiles if t['value'] is not None]


_bundle_cache = {}
_bundle_lock = threading.Lock()


def seed_bundle(run, seed_dir_name):
    root = runs_root()
    run_dir = os.path.join(root, run)
    seed_dir = run_dir if seed_dir_name == '.' else os.path.join(run_dir, seed_dir_name)
    if not os.path.isdir(seed_dir):
        return None

    try:
        stamp = max(os.path.getmtime(os.path.join(seed_dir, f))
                    for f in ('metrics.csv', 'manifest.json', 'launch.log')
                    if os.path.isfile(os.path.join(seed_dir, f)))
    except ValueError:
        stamp = 0.0
    key = (seed_dir, stamp)
    with _bundle_lock:
        if key in _bundle_cache:
            return _bundle_cache[key]

    manifest = _read_json(os.path.join(seed_dir, 'manifest.json')) or {}
    metrics = load_table(os.path.join(seed_dir, 'metrics.csv'))
    map_metrics = load_table(os.path.join(seed_dir, 'map_metrics.csv'))

    planner = {}
    for node, path in planner_log_paths(seed_dir).items():
        table = load_table(path)
        if table:
            planner[node] = table

    bands = []
    if metrics and 'revisit_state' in metrics['columns']:
        bands = state_bands(metrics['columns']['t'], metrics['columns']['revisit_state'])
    elif 'revisit' in planner and 'state' in planner['revisit']['columns']:
        bands = state_bands(planner['revisit']['columns']['t_ros'],
                            planner['revisit']['columns']['state'])

    trajectories = {name: load_tum(os.path.join(seed_dir, f'{name}_traj.tum'))
                    for name in ('gt', 'slam', 'odom')}
    origin = None
    for candidate in (trajectories['gt'], trajectories['slam'], metrics):
        if candidate:
            series = candidate['t'] if 't' in candidate else candidate['columns']['t']
            if series:
                origin = series[0]
                break

    bundle = {
        'run': run,
        'seed_dir': seed_dir_name,
        'manifest': manifest,
        'args': manifest.get('args') or {},
        'status': manifest.get('status'),
        'tiles': build_tiles(manifest, metrics, map_metrics),
        'traj': trajectories,
        'metrics': metrics['columns'] if metrics else None,
        'map_metrics': map_metrics['columns'] if map_metrics else None,
        'planner': {node: {'columns': t['columns'], 'events': t['events']} for node, t in planner.items()},
        'goals': distinct_points(planner.get('extractor'), 'gx', 'gy'),
        'revisit_targets': distinct_points(planner.get('revisit'), 'tgt_x', 'tgt_y'),
        'revisit_bands': bands,
        'map': load_map(seed_dir),
        'has_bag': _has_bag(seed_dir),
        't0': origin,
    }
    with _bundle_lock:
        _bundle_cache[key] = bundle
        while len(_bundle_cache) > 64:          # the browser prefetches whole runs
            _bundle_cache.pop(next(iter(_bundle_cache)))
    return bundle


# --- rosbag replay ---------------------------------------------------------

TSDF_REPLAY_DISPLAY = {
    'Alpha': 1,
    'Autocompute Intensity Bounds': True,
    'Autocompute Value Bounds': {'Max Value': 10, 'Min Value': -10, 'Value': True},
    'Axis': 'Z',
    'Channel Name': 'intensity',
    'Class': 'rviz_default_plugins/PointCloud2',
    'Color': '200; 200; 200',
    'Color Transformer': 'AxisColor',
    'Decay Time': 0,
    'Enabled': True,
    'Invert Rainbow': False,
    'Max Color': '255; 255; 255',
    'Min Color': '0; 0; 0',
    'Name': 'ReplayTSDF',
    'Position Transformer': 'XYZ',
    'Selectable': True,
    'Size (Pixels)': 3,
    'Size (m)': 0.1,
    'Style': 'Boxes',
    'Topic': {
        'Depth': 5,
        'Durability Policy': 'Volatile',
        'History Policy': 'Keep Last',
        'Reliability Policy': 'Best Effort',
        'Value': '/tsdf/occupied_voxels',
    },
    'Use Fixed Frame': True,
    'Use rainbow': True,
    'Value': True,
}


def in_container():
    return os.path.exists('/run/.containerenv') or os.path.exists('/.dockerenv')


def workspace_root():
    """Nearest ancestor of the repository holding a built workspace."""
    override = os.environ.get('ACTIVESLAM_WS')
    if override:
        return os.path.expanduser(override)
    path = REPO_ROOT
    for _ in range(6):
        if os.path.isfile(os.path.join(path, 'install', 'setup.bash')):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def resolve_bag(seed_dir):
    """Playable bag target: the directory when indexed, else the mcap itself."""
    bag = os.path.join(seed_dir, 'bag')
    if not os.path.isdir(bag):
        return None, []
    if os.path.isfile(os.path.join(bag, 'metadata.yaml')):
        return bag, []
    plain = sorted(glob.glob(os.path.join(bag, '*.mcap')))
    if plain:
        return plain[0], ['-s', 'mcap']
    return None, []


def session_rviz_config():
    """Scratch copy of demo.rviz -- RViz rewrites its config when it exits."""
    source = os.path.join(REPO_ROOT, 'bringup', 'rviz', 'demo.rviz')
    session_dir = os.path.join(tempfile.gettempdir(), 'activeslam_rviz')
    os.makedirs(session_dir, exist_ok=True)
    target = os.path.join(session_dir, f'replay_{os.getpid()}.rviz')
    try:
        import yaml
        with open(source) as handle:
            config = yaml.safe_load(handle)
        displays = config['Visualization Manager']['Displays']
        if not any(d.get('Name') == 'ReplayTSDF' for d in displays if isinstance(d, dict)):
            displays.append(TSDF_REPLAY_DISPLAY)
        with open(target, 'w') as handle:
            yaml.safe_dump(config, handle, default_flow_style=False)
    except Exception:
        shutil.copyfile(source, target)
    return target


def build_replay_command(seed_dir, rate=1.0, loop=False):
    """Command that plays a seed's bag and shows it in RViz."""
    bag, extra = resolve_bag(seed_dir)
    if not bag:
        raise ValueError('no rosbag was recorded for this seed')
    workspace = workspace_root()
    if not workspace:
        raise ValueError('no built workspace found (install/setup.bash)')
    config = session_rviz_config()
    play = ['ros2', 'bag', 'play', bag, '--clock', '--rate', str(rate)]
    if loop:
        play.append('--loop')
    play += extra
    script = '\n'.join([
        f'source {shlex.quote(os.path.join(workspace, "install", "setup.bash"))}',
        'octomap_prefix="$(ros2 pkg prefix octomap 2>/dev/null)"',
        'preload="$(ls "$octomap_prefix"/lib/*/liboctomap.so "$octomap_prefix"/lib/liboctomap.so '
        '2>/dev/null | head -1)"',
        f'LD_PRELOAD="$preload" rviz2 -d {shlex.quote(config)} --ros-args -p use_sim_time:=true &',
        'rviz_pid=$!',
        'sleep 3',  # RViz must be subscribed before /tf_static replays
        ' '.join(shlex.quote(a) for a in play) + ' &',
        'play_pid=$!',
        'trap \'kill -INT $play_pid $rviz_pid 2>/dev/null\' INT TERM',
        'wait $play_pid',
        'kill -INT $rviz_pid 2>/dev/null',
        'wait',
    ])
    if in_container():
        return ['bash', '-lc', script]
    return ['distrobox', 'enter', DISTROBOX_NAME, '--', 'bash', '-lc', script]


class ReplayManager:
    """One replay at a time, torn down by signalling its process group."""

    def __init__(self):
        self._lock = threading.Lock()
        self._process = None
        self._info = {}

    def _reap(self):
        if self._process is not None and self._process.poll() is not None:
            self._process = None
            self._info = {}

    def status(self):
        with self._lock:
            self._reap()
            if self._process is None:
                return {'state': 'idle'}
            return dict(self._info, state='running', pid=self._process.pid)

    def start(self, run, seed_dir_name, seed_dir, rate, loop):
        with self._lock:
            self._reap()
            if self._process is not None:
                raise RuntimeError('a replay is already running')
            command = build_replay_command(seed_dir, rate, loop)
            self._process = subprocess.Popen(
                command, start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            self._info = {'run': run, 'seed': seed_dir_name, 'rate': rate, 'loop': loop}
            return dict(self._info, state='running', pid=self._process.pid)

    def stop(self):
        with self._lock:
            self._reap()
            if self._process is None:
                return {'state': 'idle'}
            process = self._process
            try:
                group = os.getpgid(process.pid)
            except OSError:
                group = None
            for sig, timeout in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
                try:
                    if group is not None:
                        os.killpg(group, sig)
                    else:
                        process.send_signal(sig)
                except OSError:
                    break
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
            self._reap()
            return {'state': 'idle' if self._process is None else 'stopping'}


REPLAY = ReplayManager()


# --- http ------------------------------------------------------------------

def find_plotly():
    """Local plotly.min.js if any environment here ships one; else the CDN."""
    candidates = []
    workspace = workspace_root()
    if workspace:
        candidates += glob.glob(os.path.join(
            workspace, '.venv', 'lib', 'python3*', 'site-packages',
            'plotly', 'package_data', 'plotly.min.js'))
    try:
        import importlib.util
        spec = importlib.util.find_spec('plotly')
        if spec and spec.submodule_search_locations:
            for location in spec.submodule_search_locations:
                candidates.append(os.path.join(location, 'package_data', 'plotly.min.js'))
    except Exception:
        pass
    return next((c for c in candidates if os.path.isfile(c)), None)


def _safe(component):
    return component and component not in ('.', '..') and '/' not in component and '\\' not in component


class Handler(BaseHTTPRequestHandler):
    server_version = 'ActiveSlamEvalViewer/1.0'

    def log_message(self, fmt, *args):
        pass

    def _send(self, status, body, content_type='application/json', extra_headers=()):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        for key, value in extra_headers:
            self.send_header(key, value)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, payload, status=200):
        self._send(status, json.dumps(payload, allow_nan=False))

    def _error(self, status, message):
        self._json({'error': message}, status)

    def _file(self, path, content_type):
        if not os.path.isfile(path):
            return self._error(404, 'not found')
        with open(path, 'rb') as handle:
            self._send(200, handle.read(), content_type)

    def do_GET(self):
        path = urlparse(self.path).path
        parts = [p for p in path.split('/') if p]

        if not parts or parts == ['index.html']:
            return self._file(os.path.join(STATIC_DIR, 'index.html'), 'text/html; charset=utf-8')
        if parts == ['plotly.min.js']:
            local = find_plotly()
            if not local:
                return self._error(404, 'no local plotly')
            return self._file(local, 'application/javascript')
        if len(parts) == 2 and parts[0] == 'static' and _safe(parts[1]):
            kinds = {'.js': 'application/javascript', '.css': 'text/css',
                     '.html': 'text/html; charset=utf-8'}
            ext = os.path.splitext(parts[1])[1]
            if ext not in kinds:
                return self._error(404, 'not found')
            return self._file(os.path.join(STATIC_DIR, parts[1]), kinds[ext])

        if parts == ['api', 'runs']:
            return self._json({'runs': list_runs(), 'root': runs_root()})
        if parts == ['api', 'replay']:
            return self._json(REPLAY.status())
        if len(parts) == 3 and parts[:2] == ['api', 'runs']:
            if not _safe(parts[2]):
                return self._error(400, 'bad run name')
            run_dir = os.path.join(runs_root(), parts[2])
            if not os.path.isdir(run_dir):
                return self._error(404, 'no such run')
            summary = load_table(os.path.join(run_dir, 'summary.csv'), max_n=10000)
            return self._json({
                'run': parts[2],
                'seeds': list_seeds(run_dir),
                'summary': summary['columns'] if summary else None,
                'progress': _read_json(os.path.join(run_dir, 'progress.json')),
            })
        if len(parts) == 4 and parts[:2] == ['api', 'runs']:
            if not _safe(parts[2]) or not (parts[3] == '.' or _safe(parts[3])):
                return self._error(400, 'bad path')
            bundle = seed_bundle(parts[2], parts[3])
            if bundle is None:
                return self._error(404, 'no such seed')
            return self._json(bundle)
        return self._error(404, 'not found')

    def do_POST(self):
        path = urlparse(self.path).path
        length = int(self.headers.get('Content-Length') or 0)
        try:
            payload = json.loads(self.rfile.read(length) or b'{}')
        except ValueError:
            return self._error(400, 'bad json')

        if path == '/api/replay/stop':
            return self._json(REPLAY.stop())
        if path == '/api/replay/start':
            run, seed = payload.get('run'), payload.get('seed')
            if not _safe(run) or not (seed == '.' or _safe(seed)):
                return self._error(400, 'bad run or seed')
            run_dir = os.path.join(runs_root(), run)
            seed_dir = run_dir if seed == '.' else os.path.join(run_dir, seed)
            if not os.path.isdir(seed_dir):
                return self._error(404, 'no such seed')
            try:
                rate = float(payload.get('rate') or 1.0)
            except (TypeError, ValueError):
                rate = 1.0
            try:
                return self._json(REPLAY.start(run, seed, seed_dir, rate, bool(payload.get('loop'))))
            except (ValueError, RuntimeError) as exc:
                return self._error(409, str(exc))
        return self._error(404, 'not found')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run', nargs='?', help='run to open in the browser')
    parser.add_argument('--port', type=int, default=8899)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--print-replay-cmd', nargs=2, metavar=('RUN', 'SEED'),
                        help='print the replay command for a seed and exit')
    args = parser.parse_args()

    if args.print_replay_cmd:
        run, seed = args.print_replay_cmd
        seed_dir = os.path.join(runs_root(), run, seed)
        if not os.path.isdir(seed_dir):
            parser.error(f'no such seed directory: {seed_dir}')
        try:
            command = build_replay_command(seed_dir)
        except ValueError as exc:
            parser.error(str(exc))
        print(' '.join(shlex.quote(a) for a in command))
        return

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    url = f'http://127.0.0.1:{args.port}/'
    if args.run:
        url += f'#run={args.run}'
    print(f'Evaluation viewer on {url}')
    print(f'Runs root: {runs_root()}')
    if not find_plotly():
        print('No local plotly.min.js found -- the page will load it from the CDN.')
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nstopping')
    finally:
        REPLAY.stop()
        server.server_close()


if __name__ == '__main__':
    main()
