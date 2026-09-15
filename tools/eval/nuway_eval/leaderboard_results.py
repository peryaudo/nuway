"""Split a route file for the Leaderboard evaluator and merge its results (M1 §3.12).

``split <routes.xml> <out_dir> [route_id ...]`` writes one single-route XML per
route (the evaluator restarts game time per route and a file may span towns;
the stack needs one town and monotonic sim time) and prints
``route_id,town,path`` lines for ``run_leaderboard.sh``. ``merge <run_dir>``
reads the evaluator's ``<run_dir>/leaderboard/<route_id>.json`` checkpoints
and the agent sidecars ``<route_id>_agent.json`` into
``<run_dir>/leaderboard_results.csv``, one row per route, next to our own
``results.csv`` whose ``driving_score`` is joined in for the comparison.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from pathlib import Path
from typing import Any

COLUMNS = (
    "route_id",
    "town",
    "status",
    "score_composed",
    "score_route",
    "score_penalty",
    "duration_game_s",
    "route_length_m",
    "n_collisions_pedestrian",
    "n_collisions_vehicle",
    "n_collisions_layout",
    "n_red_light",
    "n_stop_infraction",
    "n_min_speed",
    "agent_ticks",
    "agent_tick_timeouts",
    "serialize_ms_mean",
    "serialize_ms_max",
    "nuway_driving_score",
)


def split_routes(
    routes_xml: Path, out_dir: Path, route_ids: Sequence[str] = ()
) -> list[tuple[str, str, Path]]:
    """Write one XML per ``<route>`` and return (id, town, path).

    Every attribute and child is kept (the runner's parser ignores our
    ``protocol``); a route without a ``<scenarios>`` element gets an empty
    one, which the Leaderboard 2.x parser requires even when no scenario
    is triggered (route_gen writes none: an M1 route has only traffic).
    """
    tree = ET.parse(routes_xml)
    root = tree.getroot()
    out: list[tuple[str, str, Path]] = []
    out_dir.mkdir(parents=True, exist_ok=True)
    for route in root.findall("route"):
        route_id = route.attrib["id"]
        if route_ids and route_id not in route_ids:
            continue
        single = ET.Element(root.tag, root.attrib)
        if route.find("scenarios") is None:
            ET.SubElement(route, "scenarios")
        single.append(route)
        path = out_dir / f"{route_id}.xml"
        ET.ElementTree(single).write(path, encoding="unicode", xml_declaration=True)
        out.append((route_id, route.attrib["town"], path))
    return out


def _record_row(
    route_id: str,
    town: str,
    record: dict[str, Any] | None,
    agent: dict[str, Any] | None,
) -> dict[str, Any]:
    """One CSV row from the evaluator's route record and the agent sidecar (either may be missing)."""
    row: dict[str, Any] = dict.fromkeys(COLUMNS, "")
    row["route_id"] = route_id
    row["town"] = town
    if record is not None:
        scores = record.get("scores", {})
        meta = record.get("meta", {})
        infractions = record.get("infractions", {})
        row["status"] = record.get("status", "")
        row["score_composed"] = scores.get("score_composed", "")
        row["score_route"] = scores.get("score_route", "")
        row["score_penalty"] = scores.get("score_penalty", "")
        row["duration_game_s"] = meta.get("duration_game", "")
        row["route_length_m"] = meta.get("route_length", "")
        for column, key in (
            ("n_collisions_pedestrian", "collisions_pedestrian"),
            ("n_collisions_vehicle", "collisions_vehicle"),
            ("n_collisions_layout", "collisions_layout"),
            ("n_red_light", "red_light"),
            ("n_stop_infraction", "stop_infraction"),
            ("n_min_speed", "min_speed_infractions"),
        ):
            value = infractions.get(key, [])
            row[column] = len(value) if isinstance(value, list) else value
    if agent is not None:
        row["agent_ticks"] = agent.get("ticks", "")
        row["agent_tick_timeouts"] = agent.get("tick_timeouts", "")
        row["serialize_ms_mean"] = agent.get("serialize_ms_mean", "")
        row["serialize_ms_max"] = agent.get("serialize_ms_max", "")
    return row


def merge(run_dir: Path) -> Path:
    """Write ``<run_dir>/leaderboard_results.csv`` from the checkpoints and sidecars under ``leaderboard/``."""
    lb_dir = run_dir / "leaderboard"
    ours: dict[str, str] = {}
    results_csv = run_dir / "results.csv"
    if results_csv.is_file():
        with results_csv.open(newline="") as f:
            for r in csv.DictReader(f):
                ours[r["route_id"]] = r.get("driving_score", "")
    rows: list[dict[str, Any]] = []
    for xml in sorted(lb_dir.glob("*.xml")):
        route_id = xml.stem
        town = ET.parse(xml).getroot().find("route").attrib.get("town", "")  # type: ignore[union-attr]
        record: dict[str, Any] | None = None
        checkpoint = lb_dir / f"{route_id}.json"
        if checkpoint.is_file():
            data = json.loads(checkpoint.read_text())
            records = data.get("_checkpoint", {}).get("records", [])
            record = records[0] if records else None
        agent_json = lb_dir / f"{route_id}_agent.json"
        agent = json.loads(agent_json.read_text()) if agent_json.is_file() else None
        row = _record_row(route_id, town, record, agent)
        row["nuway_driving_score"] = ours.get(route_id, "")
        rows.append(row)
    out = run_dir / "leaderboard_results.csv"
    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    return out


def main(argv: Sequence[str] | None = None) -> int:
    """CLI: ``split`` for run_leaderboard.sh, ``merge`` at its end."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p_split = sub.add_parser("split")
    p_split.add_argument("routes")
    p_split.add_argument("out_dir")
    p_split.add_argument("route_ids", nargs="*")
    p_merge = sub.add_parser("merge")
    p_merge.add_argument("run_dir")
    args = parser.parse_args(argv)
    if args.command == "split":
        for route_id, town, path in split_routes(
            Path(args.routes), Path(args.out_dir), args.route_ids
        ):
            print(f"{route_id},{town},{path}")
        return 0
    out = merge(Path(args.run_dir))
    with out.open(newline="") as f:
        for row in csv.DictReader(f):
            print(
                f"{row['route_id']}: {row['status'] or 'no record'} score {row['score_composed']} "
                f"(ours {row['nuway_driving_score'] or '-'}), agent timeouts {row['agent_tick_timeouts']}, "
                f"serialize {row['serialize_ms_mean']} ms"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
