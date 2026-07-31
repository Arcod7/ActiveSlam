"""Evaluation-panel model tests; no ROS process or real TUI required."""

import curses
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import launcher as ui  # noqa: E402
import launcher_eval as evaluation  # noqa: E402


def test_matrix_summary_counts_runs_and_matches_runner_estimate(tmp_path):
    matrix = tmp_path / "matrix_example.yaml"
    matrix.write_text(
        "# A short evaluation for the panel.\n"
        "batch_name: example\n"
        "duration_s: 120\n"
        "settle_s: 10\n"
        "seeds: [1, 2]\n"
        "common_args: {mapper: tsdf}\n"
        "runs:\n"
        "  - {name: baseline, args: {}}\n"
        "  - {name: changed, args: {loop_closure: 'false'}}\n"
    )

    info = evaluation.load_matrix_info(matrix)

    assert info.name == "example"
    assert info.configs == 2
    assert info.seeds == 2
    assert info.total_runs == 4
    assert info.estimated_s == 4 * (120 + 10 + 50)
    assert info.description == "A short evaluation for the panel."
    assert info.common_args["mapper"] == "tsdf"
    assert info.error == ""


def test_discovery_puts_smoke_first_and_reports_bad_yaml(tmp_path):
    (tmp_path / "matrix_long.yaml").write_text(
        "batch_name: long\nseeds: [1]\nruns: [{name: a}]\n")
    (tmp_path / "matrix_smoke.yaml").write_text(
        "batch_name: smoke\nseeds: [1]\nruns: [{name: a}]\n")
    (tmp_path / "matrix_broken.yaml").write_text("runs: [\n")
    (tmp_path / "notes.yaml").write_text("ignored: true\n")

    infos = evaluation.discover_matrices(tmp_path)

    assert infos[0].name == "smoke"
    assert len(infos) == 3
    assert next(info for info in infos if info.name == "broken").error


def test_data_snapshot_reads_flushed_rows_from_active_run(tmp_path):
    batch = tmp_path / "batch"
    run = batch / "baseline_s1"
    run.mkdir(parents=True)
    (batch / "progress.json").write_text(json.dumps({"run_dir": str(run)}))
    (run / "metrics.csv").write_text(
        "time,ate,abs_error\n0,1.2,0.5\n1,0.8,0.3\n")
    (run / "map_metrics.csv").write_text(
        "time,coverage\n0,0.1\n")

    runner = evaluation.EvaluationRunner(tmp_path)
    runner.batch_dir = str(batch)
    snapshot = runner.data_snapshot()

    assert snapshot["metrics_rows"] == 2
    assert snapshot["map_rows"] == 1
    assert snapshot["metrics"]["ate"] == "0.8"
    assert snapshot["map"]["coverage"] == "0.1"


def test_duration_is_compact():
    assert evaluation.format_duration(45) == "45s"
    assert evaluation.format_duration(125) == "2m 05s"
    assert evaluation.format_duration(3725) == "1h 02m"


def test_evaluation_is_directly_below_launch():
    assert ui.WELCOME_OPTIONS[:3] == ("Launch", "Evaluation", "---")


def test_evaluation_screen_renders_matrix_plan(monkeypatch):
    class Screen:
        def __init__(self):
            self.drawn = []

        def getmaxyx(self):
            return 24, 100

        def erase(self):
            self.drawn = []

        def addstr(self, y, x, text, attr=0):
            self.drawn.append(str(text))

        def refresh(self):
            pass

        def timeout(self, _milliseconds):
            pass

        def getch(self):
            return ord("q")

    monkeypatch.setattr(curses, "color_pair", lambda _number: 0)
    screen = Screen()
    runner = evaluation.EvaluationRunner(ui.REPO_ROOT)

    ui.evaluation_screen(screen, runner, built=True, ws_root=ui.REPO_ROOT)

    frame = " ".join(screen.drawn)
    assert "smoke" in frame
    assert "2 configs × 1 seeds = 2 runs" in frame
    assert "Enter run evaluation" in frame
