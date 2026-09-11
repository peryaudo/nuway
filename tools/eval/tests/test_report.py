"""report: the results.csv schema round-trips and report.md links what exists."""

from __future__ import annotations

from pathlib import Path

from nuway_eval.driving_score import InfractionCounts, ScoringConfig
from nuway_eval.report import (
    KEY_COLUMNS,
    RESULT_COLUMNS,
    RouteResult,
    percentile,
    read_results,
    write_report,
    write_results,
)

ROOT = Path(__file__).resolve().parents[3]


def test_schema_starts_with_the_protocol_key_and_round_trips(tmp_path: Path) -> None:
    assert RESULT_COLUMNS[:4] == KEY_COLUMNS
    cfg = ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")
    r = RouteResult(
        "dev03_00", "Town03", "ClearNoon", 0, profile="m1_classical", git_sha="abc"
    )
    r.status = "completed"
    r.route_length_m = 1641.5
    r.completion = 0.987
    counts = InfractionCounts()
    counts.add(12, "red_light")
    counts.add_min_speed(3000, 50.0)
    counts.outside_lanes_frac = 0.1
    r.apply_counts(counts, cfg)
    r.degraded_sources = "agents;occupancy"
    r.first_timeout_tick = 400
    r.non_deterministic = True
    r.bag_path = "dev03_00_ClearNoon_0/run.mcap"
    write_results([r], tmp_path)
    back = read_results(tmp_path / "results.csv")
    assert len(back) == 1
    b = back[0]
    assert b.key == r.key
    assert b.n_red_light == 1
    assert b.non_deterministic is True
    assert b.blocked is False
    assert b.first_timeout_tick == 400
    assert b.n_min_speed == 1
    assert b.min_speed_pct == 50.0
    assert abs(b.driving_score - 98.7 * 0.9 * 0.7 * 0.85) < 1e-2
    assert b.degraded_sources == "agents;occupancy"
    assert read_results(tmp_path / "missing.csv") == []


def test_report_lists_scores_and_incident_sheets(tmp_path: Path) -> None:
    cfg = ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")
    a = RouteResult(
        "dev03_00", "Town03", "ClearNoon", 0, run_id="r", profile="p", git_sha="s"
    )
    a.status, a.completion, a.driving_score = "completed", 1.0, 100.0
    a.bag_path = "dev03_00_ClearNoon_0/run.mcap"
    b = RouteResult("dev05_00", "Town05", "ClearNoon", 0)
    b.status, b.completion, b.driving_score, b.blocked = "blocked", 0.5, 50.0, True
    inc = tmp_path / a.run_dir_name / "incidents"
    inc.mkdir(parents=True)
    (inc / "000120_red_light.png").write_bytes(b"")
    (inc / "sheet_not_an_incident.png").write_bytes(b"")
    path = write_report([a, b], tmp_path, cfg)
    text = path.read_text()
    assert "Overall: **75.0**" in text
    assert "| Town03 | 1 | 100.000 | 1.000 | 1 |" in text
    assert "| blocked | 1 |" in text
    assert "[mcap](dev03_00_ClearNoon_0/run.mcap)" in text
    assert (
        "tick 120 `red_light`: [sheet](dev03_00_ClearNoon_0/incidents/000120_red_light.png)"
        in text
    )
    assert "sheet_not_an_incident" not in text


def test_percentile_is_nearest_rank() -> None:
    assert percentile([], 50) == 0.0
    assert percentile([3.0, 1.0, 2.0], 50) == 2.0
    assert percentile([3.0, 1.0, 2.0], 99) == 3.0
    assert percentile([3.0, 1.0, 2.0], 0) == 1.0
