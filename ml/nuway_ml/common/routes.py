"""Route definitions -> ordered waypoints (M0 §2.5).

Two sources feed ``/nuway/route/waypoints`` (``nav_msgs/Path``, map frame):
Leaderboard-format route XML files under ``tools/eval/routes/`` and the
Leaderboard's ``global_plan`` (M1 §3.12). Both are CARLA-convention positions,
converted here through ``carla_conv`` (the one place with that arithmetic);
everything this module returns is ROS convention. No ``rclpy``: the message is
assembled by the callers (``run_routes.py``, ``leaderboard_agent.py``) from the
plain ``PathPayload``.

Route XML (Leaderboard 2.x)::

    <routes>
      <route id="0" town="Town03" protocol="m1">
        <weathers><weather route_percentage="0" .../></weathers>
        <waypoints><position x="..." y="..." z="..."/>...</waypoints>
        <scenarios/>
      </route>
    </routes>
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, TypedDict

import numpy as np

from nuway_ml.common.carla_conv import CarlaLocation, LocationLike, location_to_ros
from nuway_ml.common.frames import FRAME_MAP


@dataclass(frozen=True, slots=True)
class Waypoint:
    """A route waypoint in the ROS map frame (metres)."""

    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True, slots=True)
class RouteSpec:
    """One ``<route>`` of a route file."""

    route_id: str
    town: str
    waypoints: tuple[Waypoint, ...]
    protocol: str = ""  # e.g. "m1" (M1 §3.10); empty when unmarked
    weathers: tuple[str, ...] = ()  # weather preset names in file order, if given

    @property
    def start(self) -> Waypoint:
        """First waypoint (the reset pose is derived from it, M0 §2.11)."""
        return self.waypoints[0]

    @property
    def goal(self) -> Waypoint:
        """Last waypoint."""
        return self.waypoints[-1]


class PoseDict(TypedDict):
    """One pose of the ``nav_msgs/Path`` payload."""

    x: float
    y: float
    z: float
    yaw: float


class PathPayload(TypedDict):
    """Plain ``nav_msgs/Path`` payload: ``header.frame_id`` and one pose per waypoint."""

    frame_id: str
    poses: list[PoseDict]


class _HasLocation(Protocol):
    @property
    def location(self) -> LocationLike: ...  # protocol member


def _carla_to_waypoint(loc: LocationLike) -> Waypoint:
    xyz = location_to_ros(loc)
    return Waypoint(float(xyz[0]), float(xyz[1]), float(xyz[2]))


def parse_route_xml(text: str) -> list[RouteSpec]:
    """Parse the routes of a Leaderboard route file; positions -> ROS convention."""
    root = ET.fromstring(text)  # trusted repo files, not network input
    if root.tag != "routes":
        msg = f"expected <routes> root, got <{root.tag}>"
        raise ValueError(msg)
    routes: list[RouteSpec] = []
    for node in root.findall("route"):
        positions = node.findall("./waypoints/position")
        if not positions:  # Leaderboard 1.x layout
            positions = node.findall("waypoint")
        waypoints = tuple(
            _carla_to_waypoint(
                CarlaLocation(
                    float(p.attrib["x"]),
                    float(p.attrib["y"]),
                    float(p.attrib.get("z", 0.0)),
                )
            )
            for p in positions
        )
        if len(waypoints) < 2:
            msg = f"route {node.attrib.get('id', '?')} has fewer than 2 waypoints"
            raise ValueError(msg)
        weathers = tuple(
            w.attrib["preset"]
            for w in node.findall("./weathers/weather")
            if "preset" in w.attrib
        )
        routes.append(
            RouteSpec(
                route_id=str(node.attrib.get("id", str(len(routes)))),
                town=str(node.attrib["town"]),
                waypoints=waypoints,
                protocol=str(node.attrib.get("protocol", "")),
                weathers=weathers,
            )
        )
    return routes


def load_route_xml(path: Path | str) -> list[RouteSpec]:
    """Read and parse a route file."""
    return parse_route_xml(Path(path).read_text())


def waypoints_from_global_plan(
    plan: Iterable[tuple[_HasLocation, object]],
) -> tuple[Waypoint, ...]:
    """Leaderboard ``global_plan_world_coord`` (``(carla.Transform, RoadOption)`` pairs) -> waypoints."""
    return tuple(_carla_to_waypoint(transform.location) for transform, _option in plan)


def route_length_m(waypoints: Iterable[Waypoint]) -> float:
    """Polyline length through the waypoints."""
    pts = np.array([(w.x, w.y) for w in waypoints], dtype=np.float64)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def path_payload(waypoints: Iterable[Waypoint]) -> PathPayload:
    """``nav_msgs/Path`` payload; each pose's yaw points at the next waypoint (the last repeats)."""
    wps = list(waypoints)
    poses: list[PoseDict] = []
    for i, w in enumerate(wps):
        nxt = wps[min(i + 1, len(wps) - 1)]
        prev = wps[max(i - 1, 0)] if i + 1 >= len(wps) else w
        yaw = math.atan2(nxt.y - prev.y, nxt.x - prev.x) if len(wps) > 1 else 0.0
        poses.append(PoseDict(x=w.x, y=w.y, z=w.z, yaw=yaw))
    return PathPayload(frame_id=FRAME_MAP, poses=poses)
