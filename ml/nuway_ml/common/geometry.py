"""SE2/SE3 helpers and angle wrapping, ROS convention (M0).

Mirrors ``nuway_common/geometry.hpp``: x forward, y left, z up, yaw
counter-clockwise positive, radians. Quaternions are ``(x, y, z, w)`` arrays, the
order of ``geometry_msgs/Quaternion`` and of Eigen's coefficient storage.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

PI = math.pi

Array = NDArray[np.float64]


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle in radians into (-pi, pi]."""
    wrapped = math.fmod(angle_rad + PI, 2.0 * PI)
    if wrapped <= 0.0:
        wrapped += 2.0 * PI
    return wrapped - PI


def wrap_angles(angles_rad: Array) -> Array:
    """Vectorised :func:`wrap_angle` over an array of any shape."""
    wrapped = np.fmod(np.asarray(angles_rad, dtype=np.float64) + PI, 2.0 * PI)
    wrapped = np.where(wrapped <= 0.0, wrapped + 2.0 * PI, wrapped)
    return np.asarray(wrapped - PI, dtype=np.float64)


@dataclass(frozen=True, slots=True)
class SE2:
    """Planar pose: translation (x, y) in meters, heading yaw in radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class SE3:
    """Spatial pose: translation (3,) in meters and unit quaternion (x, y, z, w)."""

    translation: Array = field(default_factory=lambda: np.zeros(3))
    rotation: Array = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))


def _quat_multiply(a: Array, b: Array) -> Array:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def _quat_rotate(q: Array, v: Array) -> Array:
    qv = np.asarray(q[:3], dtype=np.float64)
    qw = float(q[3])
    t = 2.0 * np.cross(qv, v)
    return np.asarray(v + qw * t + np.cross(qv, t), dtype=np.float64)


def compose(a: SE2 | SE3, b: SE2 | SE3) -> SE2 | SE3:
    """Return ``a * b`` (apply b in a's frame, then a); yaw wrapped for SE2."""
    if isinstance(a, SE2) and isinstance(b, SE2):
        cos_a = math.cos(a.yaw)
        sin_a = math.sin(a.yaw)
        return SE2(
            a.x + cos_a * b.x - sin_a * b.y,
            a.y + sin_a * b.x + cos_a * b.y,
            wrap_angle(a.yaw + b.yaw),
        )
    if isinstance(a, SE3) and isinstance(b, SE3):
        rotation = _quat_multiply(a.rotation, b.rotation)
        rotation = rotation / np.linalg.norm(rotation)
        return SE3(a.translation + _quat_rotate(a.rotation, b.translation), rotation)
    msg = "compose() needs two SE2 or two SE3 poses"
    raise TypeError(msg)


def inverse(a: SE2 | SE3) -> SE2 | SE3:
    """Return ``a^-1``."""
    if isinstance(a, SE2):
        cos_a = math.cos(a.yaw)
        sin_a = math.sin(a.yaw)
        return SE2(
            -(cos_a * a.x) - sin_a * a.y,
            sin_a * a.x - cos_a * a.y,
            wrap_angle(-a.yaw),
        )
    conj = np.array([-a.rotation[0], -a.rotation[1], -a.rotation[2], a.rotation[3]])
    return SE3(-_quat_rotate(conj, a.translation), conj)


def between(a: SE2, b: SE2) -> SE2:
    """Return ``a^-1 * b``: the pose of b expressed in a's frame."""
    out = compose(inverse(a), b)
    assert isinstance(out, SE2)
    return out


def apply(pose: SE2 | SE3, point: Array) -> Array:
    """Transform point(s) from the frame of ``pose`` into the parent frame.

    ``point`` is ``(2,)`` / ``(N, 2)`` for SE2 and ``(3,)`` / ``(N, 3)`` for SE3.
    """
    pts = np.asarray(point, dtype=np.float64)
    if isinstance(pose, SE2):
        cos_a = math.cos(pose.yaw)
        sin_a = math.sin(pose.yaw)
        x = pts[..., 0]
        y = pts[..., 1]
        return np.stack(
            [pose.x + cos_a * x - sin_a * y, pose.y + sin_a * x + cos_a * y], axis=-1
        )
    if pts.ndim == 1:
        return pose.translation + _quat_rotate(pose.rotation, pts)
    return np.stack(
        [pose.translation + _quat_rotate(pose.rotation, p) for p in pts], axis=0
    )


def rotate(pose: SE2, vec: Array) -> Array:
    """Rotate vector(s) ``(2,)`` / ``(N, 2)`` from the frame of ``pose`` into the parent."""
    vecs = np.asarray(vec, dtype=np.float64)
    cos_a = math.cos(pose.yaw)
    sin_a = math.sin(pose.yaw)
    x = vecs[..., 0]
    y = vecs[..., 1]
    return np.stack([cos_a * x - sin_a * y, sin_a * x + cos_a * y], axis=-1)


def rpy_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Array:
    """Roll, pitch, yaw (Z-Y-X intrinsic, radians) to a unit quaternion (x, y, z, w)."""
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    q = np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )
    return np.asarray(q / np.linalg.norm(q), dtype=np.float64)


def quaternion_to_rpy(quat: Array) -> Array:
    """Quaternion (x, y, z, w) to (roll, pitch, yaw) radians, each in (-pi, pi]."""
    q = np.asarray(quat, dtype=np.float64)
    q = q / np.linalg.norm(q)
    qx, qy, qz, qw = (float(v) for v in q)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


def yaw_to_quaternion(yaw_rad: float) -> Array:
    """Yaw-only quaternion (rotation about z)."""
    return rpy_to_quaternion(0.0, 0.0, yaw_rad)


def quaternion_to_yaw(quat: Array) -> float:
    """Heading about z of a quaternion (x, y, z, w), in (-pi, pi]."""
    return float(quaternion_to_rpy(quat)[2])


def to_se2(pose: SE3) -> SE2:
    """Planar projection of an SE3 pose: (x, y, yaw)."""
    return SE2(
        float(pose.translation[0]),
        float(pose.translation[1]),
        quaternion_to_yaw(pose.rotation),
    )


def distance(a: Array, b: Array) -> float:
    """Euclidean distance between two planar points."""
    return float(np.linalg.norm(np.asarray(a, dtype=np.float64) - np.asarray(b)))
