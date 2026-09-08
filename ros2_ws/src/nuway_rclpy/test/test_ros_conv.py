"""nuway_rclpy: ROS <-> nuway_ml conversions and QoS profiles (no ROS graph)."""

# ruff: noqa: PT009, PT027 -- colcon runs these with unittest discovery (no pytest in the system interpreter)
from __future__ import annotations

import math
import unittest

import numpy as np
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


class RosConvTest(unittest.TestCase):
    """Round trips through the message types."""

    def test_stamp_round_trip(self) -> None:
        for t in (0.0, 1.05, 12345.678901234):
            stamp = stamp_from_seconds(t)
            self.assertAlmostEqual(seconds_from_stamp(stamp), t, places=9)
        self.assertEqual(stamp_from_seconds(2.5).sec, 2)
        self.assertEqual(stamp_from_seconds(2.5).nanosec, 500_000_000)

    def test_pose_round_trip(self) -> None:
        pose = SE3(np.array([1.0, -2.0, 0.5]), rpy_to_quaternion(0.1, -0.2, 0.3))
        back = se3_from_pose(pose_from_se3(pose))
        np.testing.assert_allclose(back.translation, pose.translation, atol=1e-12)
        np.testing.assert_allclose(back.rotation, pose.rotation, atol=1e-12)

    def test_pose_from_xyz_yaw(self) -> None:
        pose = pose_from_xyz_yaw(3.0, 4.0, 0.0, math.pi / 2)
        se3 = se3_from_pose(pose)
        np.testing.assert_allclose(se3.translation, [3.0, 4.0, 0.0])
        # A quarter turn about z: |qz| = |qw| = sqrt(0.5).
        self.assertAlmostEqual(abs(pose.orientation.z), math.sqrt(0.5), places=12)
        self.assertAlmostEqual(abs(pose.orientation.w), math.sqrt(0.5), places=12)


class RosQosTest(unittest.TestCase):
    """Named profiles map onto rclpy policies (docs/02 §2)."""

    def test_named_profiles(self) -> None:
        latched = qos("latched")
        self.assertEqual(latched.durability, QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.assertEqual(latched.reliability, QoSReliabilityPolicy.RELIABLE)
        stream = qos("stream")
        self.assertEqual(stream.durability, QoSDurabilityPolicy.VOLATILE)
        self.assertEqual(CLOCK_QOS.reliability, QoSReliabilityPolicy.RELIABLE)
        with self.assertRaises(KeyError):
            qos("no_such_profile")


if __name__ == "__main__":
    unittest.main()
