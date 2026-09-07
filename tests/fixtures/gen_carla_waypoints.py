"""Generate the CARLA waypoint fixtures for the nuway_map parser test (M0 task 7).

For each town: export ``data/maps/<town>/map.xodr`` if absent (gitignored, large)
and write two committed CSV fixtures (CSV rather than JSON so the gtest reads
them without a JSON dependency): ``tests/fixtures/<town>_waypoints.csv`` with a
subsample of ``world.get_map().generate_waypoints(2.0)`` converted to ROS
convention through ``carla_conv``, and ``<town>_traffic_lights.csv`` with every
traffic light's stop and affected-lane waypoints (``kind`` column).

Usage (CARLA running): ``uv run tests/fixtures/gen_carla_waypoints.py --towns Town03 Town05``.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import carla

from nuway_ml.common.carla_conv import location_to_ros, yaw_to_ros

REPO_ROOT = Path(__file__).resolve().parents[2]
FIXTURE_DIR = Path(__file__).resolve().parent


COLUMNS = [
    "road_id",
    "section_id",
    "lane_id",
    "s",
    "x",
    "y",
    "z",
    "yaw",
    "width",
    "type",
    "junction",
]


def waypoint_record(wp: carla.Waypoint) -> list[float | int | str]:
    """One waypoint in ROS convention, in COLUMNS order."""
    xyz = location_to_ros(wp.transform.location)
    return [
        int(wp.road_id),
        int(wp.section_id),
        int(wp.lane_id),
        round(float(wp.s), 3),
        round(float(xyz[0]), 3),
        round(float(xyz[1]), 3),
        round(float(xyz[2]), 3),
        round(yaw_to_ros(float(wp.transform.rotation.yaw)), 4),
        round(float(wp.lane_width), 3),
        str(wp.lane_type).split(".")[-1].lower(),
        int(wp.is_junction),
    ]


def generate(client: carla.Client, town: str, stride: int, spacing: float) -> None:
    """Export the xodr (if absent) and write the fixture for one town."""
    world = client.get_world()
    if town not in world.get_map().name:
        world = client.load_world(town)
    carla_map = world.get_map()
    xodr_path = REPO_ROOT / "data" / "maps" / town / "map.xodr"
    if not xodr_path.is_file():
        xodr_path.parent.mkdir(parents=True, exist_ok=True)
        xodr_path.write_text(carla_map.to_opendrive())
        print(f"exported {xodr_path}")
    waypoints = carla_map.generate_waypoints(spacing)
    records = [waypoint_record(wp) for i, wp in enumerate(waypoints) if i % stride == 0]
    out = FIXTURE_DIR / f"{town.lower()}_waypoints.csv"
    with out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        writer.writerows(records)
    lights = world.get_actors().filter("traffic.traffic_light")
    out_tl = FIXTURE_DIR / f"{town.lower()}_traffic_lights.csv"
    with out_tl.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["actor_id", "kind", *COLUMNS])
        for tl in lights:
            for wp in tl.get_stop_waypoints():
                writer.writerow([int(tl.id), "stop", *waypoint_record(wp)])
            for wp in tl.get_affected_lane_waypoints():
                writer.writerow([int(tl.id), "affected", *waypoint_record(wp)])
    print(
        f"wrote {out}: {len(records)} of {len(waypoints)} waypoints (stride {stride}); "
        f"{out_tl}: {len(lights)} lights"
    )


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--towns", nargs="+", default=["Town03", "Town05"])
    parser.add_argument("--stride", type=int, default=4, help="keep every Nth waypoint")
    parser.add_argument("--spacing", type=float, default=2.0)
    args = parser.parse_args()
    client = carla.Client(args.host, args.port)
    client.set_timeout(180.0)
    for town in args.towns:
        generate(client, town, args.stride, args.spacing)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
