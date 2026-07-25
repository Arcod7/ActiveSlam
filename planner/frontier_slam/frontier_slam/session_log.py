"""Shared CSV session logger for frontier_slam nodes.

Each node opens one CSV at startup, named with the wall-clock timestamp:
    logs/YYYY-MM-DD_HH-MM-SS_<node>.csv

Float formatting is handled centrally so callers can pass raw numbers:
  - finite floats → fixed-point with 2 decimal places, or the column's own
    precision when one is given (a D-optimality of 0.002 logs as 0.00 at the
    default, which loses the column outright)
  - NaN          → empty string (spreadsheet-friendly)
  - +/- inf      → 'inf' / '-inf'
  - everything else → str() via csv.writer
"""
import csv
import math
import os
from datetime import datetime


def open_session_log(node_name: str, columns: list, log_dir: str,
                     precision: dict = None) -> "SessionLog":
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    path = os.path.join(log_dir, f'{ts}_{node_name}.csv')
    handle = open(path, 'w', newline='')
    writer = csv.writer(handle)
    writer.writerow(columns)
    return SessionLog(writer, handle, path, columns, precision)


class SessionLog:
    DEFAULT_PRECISION = 2

    def __init__(self, writer, file_handle, path, columns=None, precision=None):
        self._writer = writer
        self._file = file_handle
        self.path = path
        # Per-column decimal places, resolved to positions once at open time.
        self._precision = {}
        for name, digits in (precision or {}).items():
            self._precision[(columns or []).index(name)] = digits

    def write(self, row: list) -> None:
        self._writer.writerow(
            [self._format(v, self._precision.get(i, self.DEFAULT_PRECISION))
             for i, v in enumerate(row)])
        self._file.flush()

    @classmethod
    def _format(cls, v, digits=DEFAULT_PRECISION):
        if isinstance(v, float):
            if math.isnan(v):
                return ''
            if math.isinf(v):
                return 'inf' if v > 0 else '-inf'
            return f'{v:.{digits}f}'
        return v

    def close(self) -> None:
        self._file.close()
