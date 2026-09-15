"""Map sidecars exported by ``world_manager`` (M0 §2.1). Module, not a node.

CARLA's towns place stop signs as level props with a trigger volume; the
OpenDRIVE the server hands out lists only some of them as ``<signal
type="206">`` (Town05: 5 of 33). ``map_server_node`` builds the lane graph
from the OpenDRIVE, so the props reach it through ``stop_signs.csv`` written
next to ``map.xodr``: one map-frame box per ``traffic.stop`` actor. The
governed lanes are found by the map server from position and heading, so
nothing here depends on lane ids.
"""

from __future__ import annotations

import csv
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from nuway_ml.common.carla_conv import location_to_ros, yaw_to_ros


# The slice of carla.Actor this module reads (carla ships no stubs); read-only
# properties so that any object with these attributes satisfies them.
class _VectorLike(Protocol):
    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def z(self) -> float: ...


class _RotationLike(Protocol):
    @property
    def yaw(self) -> float: ...


class _BoxLike(Protocol):
    @property
    def location(self) -> _VectorLike: ...
    @property
    def extent(self) -> _VectorLike: ...
    @property
    def rotation(self) -> _RotationLike: ...


class _TransformLike(Protocol):
    @property
    def rotation(self) -> _RotationLike: ...
    def transform(self, local: _VectorLike) -> _VectorLike: ...


class _ActorLike(Protocol):
    @property
    def type_id(self) -> str: ...
    @property
    def trigger_volume(self) -> _BoxLike: ...
    def get_transform(self) -> _TransformLike: ...


STOP_SIGNS_CSV = "stop_signs.csv"
STOP_SIGN_COLUMNS = ("x_m", "y_m", "yaw_rad", "half_length_m", "half_width_m")


@dataclass(frozen=True, slots=True)
class StopSignRecord:
    """A stop sign's trigger volume as a map-frame box (ROS convention)."""

    x_m: float  # box centre
    y_m: float
    yaw_rad: float  # the box's x axis; CARLA rotates it arbitrarily per prop
    half_length_m: float  # along yaw
    half_width_m: float  # across


def stop_sign_records(actors: Iterable[_ActorLike]) -> list[StopSignRecord]:
    """One record per ``traffic.stop`` actor of a ``world.get_actors()`` list.

    The trigger volume is a bounding box in the actor frame (its own
    ``location`` offset and ``rotation``); the actor transform maps the box
    centre into the world and the box heading is the actor yaw plus the box
    yaw. That heading is the box's, not the lane's: Town03's props carry box
    rotations of 0, 90 or 180 deg, so the map server matches lanes by
    position and by axis (heading modulo pi), never by the sign's facing.
    Conversion arithmetic stays in ``carla_conv``.
    """
    out: list[StopSignRecord] = []
    for actor in actors:
        if not str(actor.type_id).startswith("traffic.stop"):
            continue
        box = actor.trigger_volume
        transform = actor.get_transform()
        centre = transform.transform(box.location)
        xyz = location_to_ros(centre)
        yaw = yaw_to_ros(float(transform.rotation.yaw) + float(box.rotation.yaw))
        out.append(
            StopSignRecord(
                x_m=float(xyz[0]),
                y_m=float(xyz[1]),
                yaw_rad=float(yaw),
                half_length_m=float(box.extent.x),
                half_width_m=float(box.extent.y),
            )
        )
    out.sort(key=lambda r: (r.x_m, r.y_m))
    return out


def write_stop_signs_csv(path: Path, records: list[StopSignRecord]) -> None:
    """Write the sidecar (a header line even when the town has no sign)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(STOP_SIGN_COLUMNS)
        for r in records:
            writer.writerow(
                [
                    f"{r.x_m:.3f}",
                    f"{r.y_m:.3f}",
                    f"{r.yaw_rad:.4f}",
                    f"{r.half_length_m:.3f}",
                    f"{r.half_width_m:.3f}",
                ]
            )


def read_stop_signs_csv(path: Path) -> list[StopSignRecord]:
    """Read a sidecar back (tests and tools; the map server parses it in C++)."""
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return [
        StopSignRecord(
            x_m=float(r["x_m"]),
            y_m=float(r["y_m"]),
            yaw_rad=float(r["yaw_rad"]),
            half_length_m=float(r["half_length_m"]),
            half_width_m=float(r["half_width_m"]),
        )
        for r in rows
    ]
