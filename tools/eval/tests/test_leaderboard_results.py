"""nuway_eval.leaderboard_results: the route split and the results merge."""

from __future__ import annotations

import csv
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from nuway_eval import leaderboard_results as lr

ROUTES = Path(__file__).resolve().parents[1] / "routes" / "dev_town03.xml"


def test_split_writes_one_file_per_route_keeping_attributes(tmp_path: Path) -> None:
    out = lr.split_routes(ROUTES, tmp_path, ["dev03_01", "dev03_03"])
    assert [r[0] for r in out] == ["dev03_01", "dev03_03"]
    assert all(town == "Town03" for _, town, _ in out)
    root = ET.parse(out[0][2]).getroot()
    routes = root.findall("route")
    assert len(routes) == 1
    assert routes[0].attrib["id"] == "dev03_01"
    assert (
        routes[0].attrib.get("protocol") == "m1"
    )  # kept: the runner ignores it (M1 §3.10)
    assert len(routes[0].findall("waypoints/position")) > 2
    # The 2.x parser needs the element; route_gen files have none.
    assert routes[0].find("scenarios") is not None
    assert len(routes[0].findall("scenarios/scenario")) == 0
    everything = lr.split_routes(ROUTES, tmp_path / "all")
    assert len(everything) == 10


def test_merge_joins_checkpoints_sidecars_and_our_scores(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    lb = run_dir / "leaderboard"
    lr.split_routes(ROUTES, lb, ["dev03_00", "dev03_01"])
    record = {
        "route_id": "RouteScenario_dev03_00_rep0",
        "status": "Completed",
        "scores": {
            "score_composed": 71.5,
            "score_route": 100.0,
            "score_penalty": 0.715,
        },
        "meta": {"duration_game": 123.4, "route_length": 1708.9},
        "infractions": {
            "collisions_pedestrian": [],
            "collisions_vehicle": ["hit"],
            "collisions_layout": [],
            "red_light": [],
            "stop_infraction": [],
            "min_speed_infractions": ["slow"],
        },
    }
    (lb / "dev03_00.json").write_text(
        json.dumps({"_checkpoint": {"records": [record]}})
    )
    (lb / "dev03_00_agent.json").write_text(
        json.dumps(
            {
                "ticks": 2468,
                "tick_timeouts": 2468,
                "serialize_ms_mean": 12.5,
                "serialize_ms_max": 40.0,
            }
        )
    )
    with (run_dir / "results.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["route_id", "driving_score"])
        w.writeheader()
        w.writerow({"route_id": "dev03_00", "driving_score": "98.1"})
    out = lr.merge(run_dir)
    with out.open(newline="") as f:
        rows = list(csv.DictReader(f))
    assert [r["route_id"] for r in rows] == ["dev03_00", "dev03_01"]
    first = rows[0]
    assert first["status"] == "Completed"
    assert float(first["score_composed"]) == 71.5
    assert first["n_collisions_vehicle"] == "1"
    assert first["n_min_speed"] == "1"
    assert first["agent_tick_timeouts"] == "2468"
    assert first["nuway_driving_score"] == "98.1"
    # A route the evaluator never finished has an empty record but keeps its row.
    assert rows[1]["status"] == ""
    assert rows[1]["town"] == "Town03"
