"""Small conversions between nuway_ml geometry types and ROS messages (M0).

No coordinate-convention arithmetic here: everything is already ROS convention
(``carla_conv`` does the CARLA side).
"""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3

from nuway_ml.common.geometry import SE3, rpy_to_quaternion
from nuway_ml.common.tick import NANOS_PER_SECOND


def stamp_from_seconds(seconds: float) -> Time:
    """Sim time in seconds -> builtin_interfaces/Time (nanosecond rounding)."""
    total_ns = round(seconds * NANOS_PER_SECOND)
    return Time(sec=total_ns // NANOS_PER_SECOND, nanosec=total_ns % NANOS_PER_SECOND)


def seconds_from_stamp(stamp: Time) -> float:
    """builtin_interfaces/Time -> seconds."""
    return float(stamp.sec) + float(stamp.nanosec) / NANOS_PER_SECOND


def pose_from_se3(pose: SE3) -> Pose:
    """SE3 -> geometry_msgs/Pose."""
    t = pose.translation
    q = pose.rotation
    return Pose(
        position=Point(x=float(t[0]), y=float(t[1]), z=float(t[2])),
        orientation=Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        ),
    )


def se3_from_pose(pose: Pose) -> SE3:
    """geometry_msgs/Pose -> SE3."""
    return SE3(
        np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64),
        np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float64,
        ),
    )


def transform_from_se3(pose: SE3) -> Transform:
    """SE3 -> geometry_msgs/Transform."""
    t = pose.translation
    q = pose.rotation
    return Transform(
        translation=Vector3(x=float(t[0]), y=float(t[1]), z=float(t[2])),
        rotation=Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])),
    )


def pose_from_xyz_yaw(x: float, y: float, z: float, yaw: float) -> Pose:
    """Planar pose with height -> geometry_msgs/Pose."""
    q = rpy_to_quaternion(0.0, 0.0, yaw)
    return Pose(
        position=Point(x=float(x), y=float(y), z=float(z)),
        orientation=Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        ),
    )
