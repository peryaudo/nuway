"""Tests for nuway_ml.common.routes (M0 task 8)."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from nuway_ml.common.carla_conv import CarlaLocation
from nuway_ml.common.routes import (
    RouteSpec,
    Waypoint,
    load_route_xml,
    parse_route_xml,
    path_payload,
    route_length_m,
    waypoints_from_global_plan,
)

ROUTES_DIR = Path(__file__).resolve().parents[2] / "tools" / "eval" / "routes"

MINI_XML = """<?xml version="1.0"?>
<routes>
  <route id="7" town="Town05" protocol="m1">
    <weathers><weather preset="ClearNoon" route_percentage="0"/></weathers>
    <waypoints>
      <position x="1.0" y="2.0" z="0.5"/>
      <position x="11.0" y="2.0"/>
      <position x="11.0" y="-8.0" z="0.0"/>
    </waypoints>
    <scenarios/>
  </route>
  <route id="8" town="Town05">
    <waypoint x="0" y="0" z="0"/>
    <waypoint x="3" y="4" z="0"/>
  </route>
</routes>
"""


def test_parse_converts_to_ros_convention() -> None:
    routes = parse_route_xml(MINI_XML)
    assert [r.route_id for r in routes] == ["7", "8"]
    first = routes[0]
    assert first.town == "Town05"
    assert first.protocol == "m1"
    assert first.weathers == ("ClearNoon",)
    # CARLA y flips sign; a missing z is 0.
    assert first.waypoints[0] == Waypoint(1.0, -2.0, 0.5)
    assert first.waypoints[1] == Waypoint(11.0, -2.0, 0.0)
    assert first.waypoints[2] == Waypoint(11.0, 8.0, 0.0)
    assert first.start == first.waypoints[0]
    assert first.goal == first.waypoints[-1]
    # Leaderboard 1.x layout is accepted too.
    assert routes[1].protocol == ""
    assert route_length_m(routes[1].waypoints) == pytest.approx(5.0)


def test_parse_rejects_bad_input() -> None:
    with pytest.raises(ValueError, match="root"):
        parse_route_xml("<nope/>")
    with pytest.raises(ValueError, match="fewer than 2"):
        parse_route_xml(
            '<routes><route id="1" town="T"><waypoints><position x="0" y="0"/></waypoints></route></routes>'
        )


def test_committed_dev_town03_routes() -> None:
    routes = load_route_xml(ROUTES_DIR / "dev_town03.xml")
    assert routes, "dev_town03.xml has no routes"
    for route in routes:
        assert isinstance(route, RouteSpec)
        assert route.town == "Town03"
        assert len(route.waypoints) >= 2
        assert route_length_m(route.waypoints) >= 1500.0, route.route_id
        # Consecutive waypoints are a sensible stride apart (no duplicates).
        for a, b in zip(route.waypoints, route.waypoints[1:], strict=False):
            assert math.hypot(b.x - a.x, b.y - a.y) > 1.0


def test_global_plan_and_payload() -> None:
    class _Transform:
        def __init__(self, x: float, y: float, z: float) -> None:
            self.location = CarlaLocation(x, y, z)

    plan = [
        (_Transform(0.0, 0.0, 0.0), "LANEFOLLOW"),
        (_Transform(10.0, -10.0, 1.0), "LEFT"),
    ]
    wps = waypoints_from_global_plan(plan)
    assert wps == (Waypoint(0.0, 0.0, 0.0), Waypoint(10.0, 10.0, 1.0))
    payload = path_payload(wps)
    assert payload["frame_id"] == "map"
    assert [p["x"] for p in payload["poses"]] == [0.0, 10.0]
    # Each pose faces the next waypoint; the last one keeps the incoming heading.
    assert payload["poses"][0]["yaw"] == pytest.approx(math.pi / 4)
    assert payload["poses"][1]["yaw"] == pytest.approx(math.pi / 4)
    assert path_payload([Waypoint(1.0, 1.0)])["poses"][0]["yaw"] == 0.0
    assert route_length_m([]) == 0.0
