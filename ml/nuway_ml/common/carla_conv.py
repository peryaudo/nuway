"""CARLA (left-handed, degrees) <-> ROS REP-103 conversion (M0).

Mirrors ``nuway_common/carla_conv.hpp``. This module and that header are the
only code allowed to know CARLA's convention (``docs/01_directory_structure.md``
Rules, ``docs/02_interfaces.md`` §1)::

    x_ros = x_c;  y_ros = -y_c;  z_ros = z_c
    roll_ros = roll_c;  pitch_ros = -pitch_c;  yaw_ros = -yaw_c  (deg -> rad)

Applies only to data read through the CARLA Python API. CARLA's native ROS 2
topics are already ROS-convention (M0 finding, 2026-09-06). The inputs are duck
typed so ``carla.Location`` / ``carla.Rotation`` objects can be passed directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from nuway_ml.common.geometry import SE3, rpy_to_quaternion, wrap_angle

Array = NDArray[np.float64]

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


class LocationLike(Protocol):
    """Anything with CARLA-convention ``x``, ``y``, ``z`` in meters."""

    @property
    def x(self) -> float: ...  # noqa: D102  -- protocol member
    @property
    def y(self) -> float: ...  # noqa: D102  -- protocol member
    @property
    def z(self) -> float: ...  # noqa: D102  -- protocol member


class RotationLike(Protocol):
    """Anything with CARLA-convention ``pitch``, ``yaw``, ``roll`` in degrees."""

    @property
    def pitch(self) -> float: ...  # noqa: D102  -- protocol member
    @property
    def yaw(self) -> float: ...  # noqa: D102  -- protocol member
    @property
    def roll(self) -> float: ...  # noqa: D102  -- protocol member


@dataclass(frozen=True, slots=True)
class CarlaLocation:
    """A ``carla.Location``: meters, CARLA convention."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass(frozen=True, slots=True)
class CarlaRotation:
    """A ``carla.Rotation``: degrees, CARLA convention (yaw clockwise-positive)."""

    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


def location_to_ros(loc: LocationLike) -> Array:
    """CARLA location or free vector (velocity, acceleration) -> ROS ``(3,)`` vector."""
    return np.array([loc.x, -loc.y, loc.z], dtype=np.float64)


def location_from_ros(ros: Array) -> CarlaLocation:
    """ROS ``(3,)`` vector -> CARLA location."""
    return CarlaLocation(float(ros[0]), -float(ros[1]), float(ros[2]))


def rotation_to_ros(rot: RotationLike) -> Array:
    """CARLA rotation (degrees) -> ROS ``(roll, pitch, yaw)`` radians, wrapped."""
    return np.array(
        [
            wrap_angle(rot.roll * DEG_TO_RAD),
            wrap_angle(-rot.pitch * DEG_TO_RAD),
            wrap_angle(-rot.yaw * DEG_TO_RAD),
        ]
    )


def rotation_from_ros(rpy_rad: Array) -> CarlaRotation:
    """ROS ``(roll, pitch, yaw)`` radians -> CARLA rotation (degrees)."""
    return CarlaRotation(
        pitch=-float(rpy_rad[1]) * RAD_TO_DEG,
        yaw=-float(rpy_rad[2]) * RAD_TO_DEG,
        roll=float(rpy_rad[0]) * RAD_TO_DEG,
    )


def yaw_to_ros(yaw_deg: float) -> float:
    """CARLA yaw in degrees -> ROS yaw in radians."""
    return wrap_angle(-yaw_deg * DEG_TO_RAD)


def yaw_from_ros(yaw_rad: float) -> float:
    """ROS yaw in radians -> CARLA yaw in degrees."""
    return -yaw_rad * RAD_TO_DEG


def transform_to_ros(loc: LocationLike, rot: RotationLike) -> SE3:
    """``carla.Transform`` (location, rotation) -> ROS SE3."""
    rpy = rotation_to_ros(rot)
    return SE3(
        location_to_ros(loc),
        rpy_to_quaternion(float(rpy[0]), float(rpy[1]), float(rpy[2])),
    )


def angular_velocity_to_ros(omega_deg: LocationLike) -> Array:
    """CARLA angular velocity (deg/s, CARLA axes) -> ROS rad/s ``(3,)``."""
    return np.array(
        [
            omega_deg.x * DEG_TO_RAD,
            -omega_deg.y * DEG_TO_RAD,
            -omega_deg.z * DEG_TO_RAD,
        ]
    )


def steer_from_ros(steering_angle_rad: float, max_steer_rad: float) -> float:
    """ROS front-wheel angle (rad, CCW positive) -> CARLA ``VehicleControl.steer`` in [-1, 1].

    The native ``vehicle_control_cmd`` topic keeps CARLA's convention, positive =
    right (``docs/02_interfaces.md`` §3.1), hence the sign flip.
    """
    return max(-1.0, min(1.0, -steering_angle_rad / max_steer_rad))


def steer_to_ros(steer: float, max_steer_rad: float) -> float:
    """CARLA ``VehicleControl.steer`` -> ROS front-wheel angle in radians."""
    return -steer * max_steer_rad
