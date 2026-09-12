"""report: the results.csv schema round-trips and report.md links what exists."""

from __future__ import annotations

import itertools
from pathlib import Path

from nuway_eval.driving_score import InfractionCounts, ScoringConfig
from nuway_eval.report import (
    KEY_COLUMNS,
    RESULT_COLUMNS,
    RouteResult,
    coalesce_incidents,
    percentile,
    read_results,
    render_incidents,
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
    (inc / "000300_collision_vehicle.png").write_bytes(b"")
    a.collisions = [(300, "collision_vehicle", "vehicle.audi.a2", 5.6, True)]
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
    assert (
        "tick 300 `collision_vehicle`: [sheet](dev03_00_ClearNoon_0/incidents/"
        "000300_collision_vehicle.png) — other actor `vehicle.audi.a2` at 5.6 m/s, visible"
        in text
    )
    assert "sheet_not_an_incident" not in text


def test_incidents_coalesce_within_the_window_and_cap_per_kind() -> None:
    events = [
        (100, "safety_intervention"),
        (105, "safety_intervention"),
        (121, "safety_intervention"),
    ]
    events += [(100, "red_light"), (3000, "mpc_failure")]
    kept, dropped = coalesce_incidents(events, window_after=20)
    assert kept == [
        (100, "red_light"),
        (100, "safety_intervention"),
        (121, "safety_intervention"),
        (3000, "mpc_failure"),
    ]
    assert dropped == {"safety_intervention": 1}
    flapping = [(k, "safety_intervention") for k in range(0, 10000, 50)]
    kept, dropped = coalesce_incidents(flapping, window_after=20, cap=20)
    assert len(kept) == 20
    assert dropped == {"safety_intervention": 180}


def test_render_incidents_calls_render_bag_once_per_route(tmp_path: Path) -> None:
    cfg = ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")
    r = RouteResult("dev03_00", "Town03", "ClearNoon", 0, run_id="r")
    r.status = "completed"
    r.bag_path = "dev03_00_ClearNoon_0/run.mcap"
    (tmp_path / r.run_dir_name).mkdir(parents=True)
    counts = InfractionCounts()
    counts.add(120, "red_light")
    r.incidents = [(400, "safety_intervention"), (402, "safety_intervention")]
    r.apply_counts(counts, cfg)
    assert r.incidents == [
        (120, "red_light"),
        (400, "safety_intervention"),
        (402, "safety_intervention"),
    ]
    calls: list[list[str]] = []

    def fake_run(cmd: list[str]) -> None:
        calls.append(cmd)
        out = Path(cmd[cmd.index("--out") + 1]) / "incidents"
        out.mkdir(parents=True, exist_ok=True)
        for flag, item in itertools.pairwise(cmd):
            if flag == "--incident":
                tick, _, kind = item.partition(":")
                (out / f"{int(tick):06d}_{kind}.png").write_bytes(b"")

    sheets = render_incidents(tmp_path, r, (40, 20), run=fake_run)
    assert len(calls) == 1
    assert calls[0][1].endswith("tools/viz/render_bag.py")
    assert calls[0][calls[0].index("--window") + 1] == "40:20"
    assert [p.name for p in sheets] == [
        "000120_red_light.png",
        "000400_safety_intervention.png",
    ]
    assert all(p.exists() for p in sheets)
    summary = (tmp_path / r.run_dir_name / "incidents" / "summary.txt").read_text()
    assert "safety_intervention: 1 rendered, 1 more" in summary
    text = write_report([r], tmp_path, cfg).read_text()
    assert "[2 sheets](#dev03_00_clearnoon_0)" in text
    assert "- red_light: 1 rendered, 0 more" in text
    # Without a bag or without events nothing is rendered.
    assert (
        render_incidents(
            tmp_path, RouteResult("x", "Town03", "ClearNoon", 0), (40, 20), run=fake_run
        )
        == []
    )


def test_percentile_is_nearest_rank() -> None:
    assert percentile([], 50) == 0.0
    assert percentile([3.0, 1.0, 2.0], 50) == 2.0
    assert percentile([3.0, 1.0, 2.0], 99) == 3.0
    assert percentile([3.0, 1.0, 2.0], 0) == 1.0
