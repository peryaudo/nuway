"""nuway_carla_bridge.control_adapter: constants and the pedal split it relies on (no CARLA)."""

# ruff: noqa: PT009 -- colcon runs these with unittest discovery (no pytest in the system interpreter)
from __future__ import annotations

import unittest
from pathlib import Path

import yaml
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy

from nuway_carla_bridge import control_adapter as ca
from nuway_ml.common.carla_conv import steer_from_ros
from nuway_ml.common.longitudinal_map import LongitudinalMap

ROOT = Path(__file__).resolve().parents[4]


class ControlAdapterTest(unittest.TestCase):
    """The adapter publishes raw CARLA control on the topic the server subscribes to."""

    def test_topic_and_qos(self) -> None:
        self.assertEqual(ca.TOPIC_CARLA_CONTROL, "/carla/hero/vehicle_control_cmd")
        self.assertEqual(
            ca.CARLA_CONTROL_QOS.reliability, QoSReliabilityPolicy.RELIABLE
        )
        self.assertEqual(ca.CARLA_CONTROL_QOS.durability, QoSDurabilityPolicy.VOLATILE)
        self.assertEqual(ca.NODE_NAME, "control_adapter")

    def test_pedal_split_from_the_committed_vehicle(self) -> None:
        with (ROOT / "configs/vehicle/lincoln_mkz_2020.yaml").open() as f:
            cfg = yaml.safe_load(f)
        lon = LongitudinalMap.from_dict(cfg["longitudinal_map"])
        throttle, brake = lon.inverse(10.0, 1.0)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        # Engine braking alone is -6.8 m/s^2 at 10 m/s (sysid, task 9), so the
        # brake pedal appears only below the coast deceleration.
        coast = lon.coast(10.0)
        self.assertLess(coast, -3.0)
        throttle, brake = lon.inverse(10.0, coast - 1.0)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)
        # Steering: ROS left-positive radians -> CARLA [-1, 1] with left negative.
        max_steer = float(cfg["max_steer_angle"])
        self.assertAlmostEqual(steer_from_ros(max_steer, max_steer), -1.0)
        self.assertAlmostEqual(steer_from_ros(0.0, max_steer), 0.0)


if __name__ == "__main__":
    unittest.main()
