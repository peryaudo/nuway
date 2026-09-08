"""nuway_rclpy: ROS <-> nuway_ml conversions and QoS profiles (no ROS graph)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy

from nuway_ml.common.geometry import SE3, rpy_to_quaternion
from nuway_rclpy.ros_conv import (
    pose_from_se3,
    pose_from_xyz_yaw,
    se3_from_pose,
    seconds_from_stamp,
    stamp_from_seconds,
)
from nuway_rclpy.ros_qos import CLOCK_QOS, qos


def test_stamp_round_trip() -> None:
    for t in (0.0, 1.05, 12345.678901234):
        assert seconds_from_stamp(stamp_from_seconds(t)) == pytest.approx(t, abs=1e-9)
    assert stamp_from_seconds(2.5).sec == 2
    assert stamp_from_seconds(2.5).nanosec == 500_000_000


def test_pose_round_trip() -> None:
    pose = SE3(np.array([1.0, -2.0, 0.5]), rpy_to_quaternion(0.1, -0.2, 0.3))
    back = se3_from_pose(pose_from_se3(pose))
    np.testing.assert_allclose(back.translation, pose.translation, atol=1e-12)
    np.testing.assert_allclose(back.rotation, pose.rotation, atol=1e-12)


def test_pose_from_xyz_yaw_is_counter_clockwise_positive() -> None:
    pose = pose_from_xyz_yaw(3.0, 4.0, 0.0, math.pi / 2)
    se3 = se3_from_pose(pose)
    np.testing.assert_allclose(se3.translation, [3.0, 4.0, 0.0])
    # A quarter turn to the left about +z: qz = qw = +sqrt(0.5) (a sign-blind
    # check would also accept a right turn or a negated quaternion).
    assert pose.orientation.z == pytest.approx(math.sqrt(0.5), abs=1e-12)
    assert pose.orientation.w == pytest.approx(math.sqrt(0.5), abs=1e-12)
    right = pose_from_xyz_yaw(0.0, 0.0, 0.0, -math.pi / 2)
    assert right.orientation.z == pytest.approx(-math.sqrt(0.5), abs=1e-12)


def test_named_profiles_map_onto_rclpy_policies() -> None:
    latched = qos("latched")
    assert latched.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert latched.reliability == QoSReliabilityPolicy.RELIABLE
    stream = qos("stream")
    assert stream.durability == QoSDurabilityPolicy.VOLATILE
    assert CLOCK_QOS.reliability == QoSReliabilityPolicy.RELIABLE
    with pytest.raises(KeyError):
        qos("no_such_profile")
