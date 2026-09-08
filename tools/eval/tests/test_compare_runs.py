"""compare_runs: two episodes with different absolute ticks but equal state are identical."""

from __future__ import annotations

import csv
from pathlib import Path

from compare_runs import ALIGNMENT_COLUMNS, compare_route

COLUMNS = ("k", "t", "x", "y", "z", "yaw", "vx", "vy", "yaw_rate")


def _write_run(run_dir: Path, route_id: str, k0: int, xs: list[float]) -> None:
    route_dir = run_dir / "routes" / route_id
    route_dir.mkdir(parents=True)
    with (route_dir / "ego_odom.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for i, x in enumerate(xs):
            k = k0 + i
            w.writerow([k, k * 0.05, x, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    with (run_dir / "results.csv").open("w", newline="") as f:
        results = csv.DictWriter(
            f, fieldnames=["route_id", "status", "timeouts", "wall_s"]
        )
        results.writeheader()
        results.writerow(
            {"route_id": route_id, "status": "completed", "timeouts": 0, "wall_s": 1}
        )


def test_absolute_tick_offset_is_alignment_not_a_difference(tmp_path: Path) -> None:
    assert ALIGNMENT_COLUMNS == ("k", "t")
    xs = [float(i) for i in range(10)]
    _write_run(tmp_path / "a", "r0", k0=100, xs=xs)
    _write_run(tmp_path / "b", "r0", k0=3000, xs=xs)
    c = compare_route(tmp_path / "a", tmp_path / "b", "r0")
    assert c.identical, c.detail
    assert c.first_diff_tick is None


def test_first_differing_state_is_reported_as_ticks_since_the_reset(
    tmp_path: Path,
) -> None:
    xs = [float(i) for i in range(10)]
    ys = list(xs)
    ys[7] += 0.01
    _write_run(tmp_path / "a", "r0", k0=100, xs=xs)
    _write_run(tmp_path / "b", "r0", k0=3000, xs=ys)
    c = compare_route(tmp_path / "a", tmp_path / "b", "r0")
    assert not c.identical
    assert c.first_diff_tick == 7
    assert "['x']" in c.detail
