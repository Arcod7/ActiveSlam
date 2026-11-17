"""Evaluation discovery, execution, and progress for the launcher TUI.

This module deliberately has no curses dependency.  The screen in launcher.py
is presentation; the process and file handling here can be tested without a
terminal or a ROS installation.
"""

import csv
from collections import deque
from dataclasses import dataclass
import json
import os
import re
import select
import signal
import subprocess
import sys
import time

import yaml


@dataclass(frozen=True)
class MatrixInfo:
    path: str
    filename: str
    name: str
    description: str
    duration_s: float
    settle_s: float
    configs: int
    seeds: int
    total_runs: int
    estimated_s: float
    record_bag: bool
    common_args: dict
    error: str = ""


def _description(path):
    """Leading YAML comments, folded into a compact screen description."""
    lines = []
    try:
        with open(path, encoding="utf-8") as stream:
            for raw in stream:
                stripped = raw.strip()
                if stripped.startswith("#"):
                    text = stripped[1:].strip()
                    if text:
                        lines.append(text)
                    continue
                if not stripped:
                    if lines:
                        break
                    continue
                break
    except OSError:
        return ""
    return " ".join(lines)


def load_matrix_info(path):
    filename = os.path.basename(path)
    fallback_name = os.path.splitext(filename)[0].removeprefix("matrix_")
    try:
        with open(path, encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream) or {}
        runs = cfg.get("runs") or []
        seeds = cfg.get("seeds") or [-1]
        duration_s = float(cfg.get("duration_s", 480))
        settle_s = float(cfg.get("settle_s", 15))
        total = len(runs) * len(seeds)
        # Keep this identical to run_matrix.py's estimate: the fixed 50 s is
        # launch/teardown/plotting overhead observed by that runner.
        estimated_s = total * (duration_s + settle_s + 50)
        error = "" if runs else "matrix has no runs"
        return MatrixInfo(
            path=os.path.abspath(path),
            filename=filename,
            name=str(cfg.get("batch_name") or fallback_name),
            description=_description(path),
            duration_s=duration_s,
            settle_s=settle_s,
            configs=len(runs),
            seeds=len(seeds),
            total_runs=total,
            estimated_s=estimated_s,
            record_bag=bool(cfg.get("record_bag", False)),
            common_args=dict(cfg.get("common_args") or {}),
            error=error,
        )
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        return MatrixInfo(
            path=os.path.abspath(path),
            filename=filename,
            name=fallback_name,
            description="",
            duration_s=0,
            settle_s=0,
            configs=0,
            seeds=0,
            total_runs=0,
            estimated_s=0,
            record_bag=False,
            common_args={},
            error=str(exc),
        )


def discover_matrices(config_dir):
    try:
        paths = [
            entry.path
            for entry in os.scandir(config_dir)
            if entry.is_file()
            and entry.name.startswith("matrix_")
            and entry.name.endswith((".yaml", ".yml"))
        ]
    except OSError:
        return []
    infos = [load_matrix_info(path) for path in sorted(paths)]
    # The two-run smoke test is the useful first selection for someone opening
    # this page to learn it; alphabetic order would put a multi-hour ablation
    # first.
    return sorted(infos, key=lambda info: (info.name != "smoke", info.name))


def format_duration(seconds):
    seconds = max(0, int(round(seconds)))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours}h {minutes:02d}m"
    if minutes:
        return f"{minutes}m {secs:02d}s"
    return f"{secs}s"


def _read_json(path):
    try:
        with open(path, encoding="utf-8") as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _csv_snapshot(path):
    """Number of flushed rows plus the final row, tolerating an active writer."""
    count = 0
    last = {}
    try:
        with open(path, encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                count += 1
                last = row
    except (OSError, csv.Error):
        pass
    return count, last


class EvaluationRunner:
    """Own one run_matrix.py process while screens are opened and closed."""

    OUTPUT_RE = re.compile(r"^Batch .* -> (.+)$")

    def __init__(self, repo_root):
        self.repo_root = os.path.abspath(repo_root)
        self.script = os.path.join(
            self.repo_root, "eval", "eval_tools", "scripts", "run_matrix.py")
        self.proc = None
        self.matrix = None
        self.batch_dir = None
        self.started_at = None
        self.finished_at = None
        self.returncode = None
        self.stop_requested = False
        self.stop_requested_at = None
        self.output = deque(maxlen=80)
        self._partial = ""

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    @property
    def state(self):
        if self.running:
            return "stopping" if self.stop_requested else "running"
        if self.returncode is None:
            return "idle"
        if self.stop_requested:
            return "stopped"
        return "complete" if self.returncode == 0 else "failed"

    def start(self, matrix, env=None):
        if self.running:
            raise RuntimeError("an evaluation is already running")
        if matrix.error:
            raise ValueError(matrix.error)
        child_env = dict(env or os.environ)
        child_env["PYTHONUNBUFFERED"] = "1"
        self.matrix = matrix
        self.batch_dir = None
        self.started_at = time.time()
        self.finished_at = None
        self.returncode = None
        self.stop_requested = False
        self.stop_requested_at = None
        self.output.clear()
        self._partial = ""
        self.proc = subprocess.Popen(
            [sys.executable, self.script, matrix.path],
            cwd=self.repo_root,
            env=child_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        os.set_blocking(self.proc.stdout.fileno(), False)

    def _consume_output(self):
        if self.proc is None or self.proc.stdout is None:
            return
        fd = self.proc.stdout.fileno()
        while True:
            ready, _, _ = select.select([fd], [], [], 0)
            if not ready:
                break
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                break
            if not chunk:
                break
            self._partial += chunk.decode("utf-8", "replace")
            pieces = self._partial.split("\n")
            self._partial = pieces.pop()
            for line in pieces:
                clean = line.rstrip()
                if clean:
                    self.output.append(clean)
                    match = self.OUTPUT_RE.match(clean)
                    if match:
                        self.batch_dir = match.group(1)

    def poll(self):
        if self.proc is None:
            return
        self._consume_output()
        rc = self.proc.poll()
        if rc is not None and self.returncode is None:
            self._consume_output()
            if self._partial.strip():
                self.output.append(self._partial.strip())
                self._partial = ""
            self.returncode = rc
            self.finished_at = time.time()

    def request_stop(self):
        self.poll()
        if not self.running or self.stop_requested:
            return False
        self.stop_requested = True
        self.stop_requested_at = time.time()
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        return True

    def close(self, timeout=55):
        """Stop the batch and allow its current ROS launch to tear down."""
        self.request_stop()
        if self.proc is None:
            return
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.poll()

    def progress(self):
        self.poll()
        if not self.batch_dir:
            return {}
        return _read_json(os.path.join(self.batch_dir, "progress.json"))

    def data_snapshot(self):
        progress = self.progress()
        run_dir = progress.get("run_dir")
        if not run_dir:
            return {"metrics_rows": 0, "map_rows": 0, "metrics": {}, "map": {}}
        metrics_rows, metrics = _csv_snapshot(
            os.path.join(run_dir, "metrics.csv"))
        map_rows, map_metrics = _csv_snapshot(
            os.path.join(run_dir, "map_metrics.csv"))
        return {
            "metrics_rows": metrics_rows,
            "map_rows": map_rows,
            "metrics": metrics,
            "map": map_metrics,
        }
