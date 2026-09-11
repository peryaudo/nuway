"""compare_runs: rows within tolerance agree; the first differing command tick is reported."""

from __future__ import annotations

import csv
from pathlib import Path

from compare_runs import Tolerances, compare_runs, summary

from nuway_eval.report import RouteResult, write_results


def _write_run(
    run_dir: Path, result: RouteResult, accels: list[float], k0: int
) -> None:
    route_dir = run_dir / result.run_dir_name
    route_dir.mkdir(parents=True)
    with (route_dir / "control_command.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("k", "accel", "steering_angle", "emergency_stop"))
        for i, a in enumerate(accels):
            w.writerow([k0 + i, a, 0.0, 0])
    write_results([result], run_dir)


def _result(score: float, completion: float, sim_s: float, red: int = 0) -> RouteResult:
    r = RouteResult("dev03_00", "Town03", "ClearNoon", 0, status="completed")
    r.driving_score, r.completion, r.sim_time_s, r.n_red_light = (
        score,
        completion,
        sim_s,
        red,
    )
    return r


def test_runs_within_tolerance_agree_despite_absolute_ticks(tmp_path: Path) -> None:
    accels = [0.1 * i for i in range(10)]
    _write_run(tmp_path / "a", _result(80.0, 1.0, 100.0), accels, k0=100)
    _write_run(
        tmp_path / "b", _result(80.6, 0.995, 100.8), [a + 0.01 for a in accels], k0=5000
    )
    out = compare_runs(tmp_path / "a", tmp_path / "b", Tolerances())
    assert len(out) == 1
    assert out[0].agree, out[0].detail
    assert out[0].first_diff_tick is None
    assert "1/1 runs agree" in summary(out)


def test_first_command_outside_the_spread_and_exact_columns(tmp_path: Path) -> None:
    accels = [0.1 * i for i in range(10)]
    other = list(accels)
    other[6] += 0.2
    _write_run(tmp_path / "a", _result(80.0, 1.0, 100.0), accels, k0=100)
    _write_run(tmp_path / "b", _result(80.0, 1.0, 100.0, red=1), other, k0=100)
    out = compare_runs(tmp_path / "a", tmp_path / "b", Tolerances())
    assert not out[0].agree
    assert out[0].first_diff_tick == 6
    assert "n_red_light 0 vs 1" in out[0].detail
    assert "commands differ from tick 6" in out[0].detail
    # A tick-count difference without a command difference is reported too.
    _write_run(tmp_path / "c", _result(80.0, 1.0, 100.0), accels[:8], k0=100)
    out = compare_runs(tmp_path / "a", tmp_path / "c", Tolerances())
    assert "tick count 10 vs 8" in out[0].detail
