"""map_export: the stop-sign sidecar written beside the OpenDRIVE (M0 §2.1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from nuway_carla_bridge.map_export import (
    STOP_SIGN_COLUMNS,
    StopSignRecord,
    read_stop_signs_csv,
    stop_sign_records,
    write_stop_signs_csv,
)


@dataclass
class _Vec:
    x: float
    y: float
    z: float = 0.0


@dataclass
class _Rot:
    yaw: float
    pitch: float = 0.0
    roll: float = 0.0


@dataclass
class _Box:
    location: _Vec
    extent: _Vec
    rotation: _Rot


class _PointLike(Protocol):
    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def z(self) -> float: ...


class _Transform:
    """A yaw-only CARLA transform: rotate the local point, then translate."""

    def __init__(self, location: _Vec, rotation: _Rot) -> None:
        self.location = location
        self.rotation = rotation

    def transform(self, local: _PointLike) -> _Vec:
        c = math.cos(math.radians(self.rotation.yaw))
        s = math.sin(math.radians(self.rotation.yaw))
        return _Vec(
            self.location.x + c * local.x - s * local.y,
            self.location.y + s * local.x + c * local.y,
            self.location.z + local.z,
        )


class _Actor:
    def __init__(self, type_id: str, transform: _Transform, box: _Box) -> None:
        self.type_id = type_id
        self._transform = transform
        self.trigger_volume = box

    def get_transform(self) -> _Transform:
        return self._transform


def test_stop_sign_records_map_the_box_centre_and_heading_to_ros() -> None:
    # CARLA: sign at (10, -5) facing +y (yaw 90 deg); the box sits 2 m ahead
    # of the sign along its x axis. ROS negates y and the yaw.
    sign = _Actor(
        "traffic.stop",
        _Transform(_Vec(10.0, -5.0, 0.3), _Rot(90.0)),
        _Box(_Vec(2.0, 0.0, 0.0), _Vec(1.7, 1.2, 0.5), _Rot(0.0)),
    )
    light = _Actor(
        "traffic.traffic_light",
        _Transform(_Vec(0.0, 0.0), _Rot(0.0)),
        _Box(_Vec(0.0, 0.0), _Vec(1.0, 1.0, 1.0), _Rot(0.0)),
    )
    records = stop_sign_records([light, sign])
    assert len(records) == 1
    r = records[0]
    assert r.x_m == 10.0
    assert math.isclose(r.y_m, -(-5.0 + 2.0))
    assert math.isclose(r.yaw_rad, -math.pi / 2)
    assert (r.half_length_m, r.half_width_m) == (1.7, 1.2)


def test_sidecar_round_trips_and_keeps_a_header_when_empty(tmp_path: Path) -> None:
    path = tmp_path / "Town05" / "stop_signs.csv"
    records = [
        StopSignRecord(-174.4, 95.0, 3.1416, 1.7, 1.7),
        StopSignRecord(40.1, -141.4, -1.5708, 1.1, 1.0),
    ]
    write_stop_signs_csv(path, records)
    assert path.read_text().splitlines()[0] == ",".join(STOP_SIGN_COLUMNS)
    back = read_stop_signs_csv(path)
    assert [(b.x_m, b.y_m, b.half_length_m) for b in back] == [
        (-174.4, 95.0, 1.7),
        (40.1, -141.4, 1.1),
    ]
    write_stop_signs_csv(path, [])
    assert path.read_text().strip() == ",".join(STOP_SIGN_COLUMNS)
