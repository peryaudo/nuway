"""The one parser of the sensor-rig JSON (M0).

``configs/sensors/*.json`` lists sensors in **CARLA convention relative to the
CARLA actor origin** (what ``carla.Transform`` wants). This module turns them
into ROS ``base_link`` extrinsics through ``carla_conv`` and computes pinhole
intrinsics from size and FOV, so ``sensor_rig.py`` (``/tf_static``,
``camera_info``), the collectors and ``CameraCalib`` (M2) share one source for
every K and every extrinsic (``docs/01_directory_structure.md``).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

from nuway_ml.common.carla_conv import CarlaLocation, CarlaRotation, transform_to_ros
from nuway_ml.common.geometry import SE3

Array = NDArray[np.float64]

CAMERA_TYPES = (
    "sensor.camera.rgb",
    "sensor.camera.depth",
    "sensor.camera.semantic_segmentation",
)

# Rotation from the sensor frame (x forward, y left, z up: the frame CARLA's
# native topics and our /tf_static use) to the optical frame (z forward,
# x right, y down) that pinhole projection expects. R_optical_from_sensor.
R_OPTICAL_FROM_SENSOR: Array = np.array(
    [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]], dtype=np.float64
)


@dataclass(frozen=True, slots=True)
class VehicleGeometry:
    """The geometry block of ``configs/vehicle/<vehicle>.yaml`` (ROS meters)."""

    wheelbase: float
    length: float
    width: float
    height: float
    rear_axle_offset_x: float  # base_link relative to the CARLA actor origin, ROS x
    bbox_center_z: float  # bounding-box center above the actor origin
    actor_origin_height: float  # actor origin above the ground contact
    max_steer_angle: float  # rad

    @classmethod
    def from_yaml(cls, path: Path) -> VehicleGeometry:
        """Read the geometry fields of a vehicle YAML."""
        with path.open() as f:
            cfg = yaml.safe_load(f)
        return cls(
            wheelbase=float(cfg["wheelbase"]),
            length=float(cfg["length"]),
            width=float(cfg["width"]),
            height=float(cfg["height"]),
            rear_axle_offset_x=float(cfg["rear_axle_offset_x"]),
            bbox_center_z=float(cfg["bbox_center_z"]),
            actor_origin_height=float(cfg["actor_origin_height"]),
            max_steer_angle=float(cfg["max_steer_angle"]),
        )

    @property
    def base_link_in_actor(self) -> Array:
        """Translation of base_link in the actor frame, ROS convention ``(3,)``."""
        return np.array([self.rear_axle_offset_x, 0.0, -self.actor_origin_height])


@dataclass(frozen=True, slots=True)
class SensorSpec:
    """One rig entry, CARLA convention relative to the actor origin."""

    id: str
    type: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0  # degrees
    pitch: float = 0.0
    yaw: float = 0.0
    attributes: dict[str, str] = field(default_factory=dict)
    label_only: bool = False
    viz_only: bool = False

    @property
    def is_camera(self) -> bool:
        """True for camera blueprints (have image size and FOV)."""
        return self.type in CAMERA_TYPES

    @property
    def image_size(self) -> tuple[int, int]:
        """``(width, height)`` in pixels; cameras only."""
        return int(self.attributes["image_size_x"]), int(
            self.attributes["image_size_y"]
        )

    @property
    def fov_deg(self) -> float:
        """Horizontal field of view in degrees; cameras only."""
        return float(self.attributes.get("fov", 90.0))


@dataclass(frozen=True, slots=True)
class Rig:
    """A parsed rig: vehicle blueprint id and sensor entries."""

    vehicle: str
    sensors: tuple[SensorSpec, ...]

    def get(self, sensor_id: str) -> SensorSpec:
        """Return the entry with id ``sensor_id``; raise KeyError otherwise."""
        for spec in self.sensors:
            if spec.id == sensor_id:
                return spec
        msg = f"no sensor {sensor_id!r} in rig"
        raise KeyError(msg)

    def select(self, *, label_only: bool, viz_only: bool) -> tuple[SensorSpec, ...]:
        """Entries to spawn given whether label-only and viz-only sensors are wanted."""
        return tuple(
            s
            for s in self.sensors
            if (label_only or not s.label_only) and (viz_only or not s.viz_only)
        )


def _parse_entry(raw: dict[str, Any]) -> SensorSpec:
    attributes = {k: str(v) for k, v in raw.get("attributes", {}).items()}
    return SensorSpec(
        id=str(raw["id"]),
        type=str(raw["type"]),
        x=float(raw.get("x", 0.0)),
        y=float(raw.get("y", 0.0)),
        z=float(raw.get("z", 0.0)),
        roll=float(raw.get("roll", 0.0)),
        pitch=float(raw.get("pitch", 0.0)),
        yaw=float(raw.get("yaw", 0.0)),
        attributes=attributes,
        label_only=bool(raw.get("label_only", False)),
        viz_only=bool(raw.get("viz_only", False)),
    )


def load_rig(path: Path) -> Rig:
    """Parse a rig JSON file."""
    with Path(path).open() as f:
        raw = json.load(f)
    return parse_rig(raw)


def parse_rig(raw: dict[str, Any]) -> Rig:
    """Parse an already loaded rig JSON object."""
    sensors = tuple(_parse_entry(e) for e in raw.get("sensors", []))
    ids = [s.id for s in sensors]
    if len(set(ids)) != len(ids):
        msg = f"duplicate sensor ids in rig: {ids}"
        raise ValueError(msg)
    return Rig(vehicle=str(raw["vehicle"]), sensors=sensors)


def sensor_in_actor(spec: SensorSpec) -> SE3:
    """Extrinsics of the sensor in the CARLA actor frame, converted to ROS."""
    return transform_to_ros(
        CarlaLocation(spec.x, spec.y, spec.z),
        CarlaRotation(pitch=spec.pitch, yaw=spec.yaw, roll=spec.roll),
    )


def sensor_in_base_link(spec: SensorSpec, vehicle: VehicleGeometry) -> SE3:
    """Extrinsics of the sensor in ``base_link`` (rear axle on the ground), ROS.

    ``T_base_sensor = T_base_actor * T_actor_sensor`` where the actor frame and
    ``base_link`` share their orientation and differ by
    :attr:`VehicleGeometry.base_link_in_actor`.
    """
    in_actor = sensor_in_actor(spec)
    return SE3(in_actor.translation - vehicle.base_link_in_actor, in_actor.rotation)


def intrinsics(spec: SensorSpec) -> Array:
    """Pinhole ``K`` ``(3, 3)`` from image size and horizontal FOV (no distortion).

    ``fx = fy = (width / 2) / tan(fov / 2)``, principal point at the image center.
    """
    width, height = spec.image_size
    fx = (width / 2.0) / math.tan(math.radians(spec.fov_deg) / 2.0)
    return np.array(
        [[fx, 0.0, width / 2.0], [0.0, fx, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
