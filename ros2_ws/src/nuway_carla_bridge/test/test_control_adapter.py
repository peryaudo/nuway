"""nuway_carla_bridge.control_adapter: ControlCommand -> CarlaEgoVehicleControl (no CARLA, no graph)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from nuway_msgs.msg import ControlCommand
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy

from nuway_carla_bridge import control_adapter as ca
from nuway_ml.common.longitudinal_map import LongitudinalMap

ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture(scope="module")
def vehicle() -> tuple[LongitudinalMap, float]:
    with (ROOT / "configs/vehicle/lincoln_mkz_2020.yaml").open() as f:
        cfg = yaml.safe_load(f)
    return LongitudinalMap.from_dict(cfg["longitudinal_map"]), float(
        cfg["max_steer_angle"]
    )


def test_topic_and_qos_match_the_carla_subscriber() -> None:
    assert ca.TOPIC_CARLA_CONTROL == "/carla/hero/vehicle_control_cmd"
    assert ca.CARLA_CONTROL_QOS.reliability == QoSReliabilityPolicy.RELIABLE
    assert ca.CARLA_CONTROL_QOS.durability == QoSDurabilityPolicy.VOLATILE
    assert ca.NODE_NAME == "control_adapter"


def test_emergency_stop_is_full_brake_and_straight(
    vehicle: tuple[LongitudinalMap, float],
) -> None:
    lon, max_steer = vehicle
    cmd = ControlCommand()
    cmd.header.stamp.sec = 3
    cmd.emergency_stop = True
    cmd.accel = 2.0  # ignored
    cmd.steering_angle = 0.3  # ignored
    out = ca.carla_control_from_command(cmd, lon, max_steer, 10.0)
    assert out.header.stamp.sec == 3
    assert (out.throttle, out.brake, out.steer) == (0.0, 1.0, 0.0)
    assert not out.hand_brake
    assert not out.reverse
    assert out.gear == 0


def test_accel_splits_into_pedals_at_the_given_speed(
    vehicle: tuple[LongitudinalMap, float],
) -> None:
    lon, max_steer = vehicle
    cmd = ControlCommand()
    cmd.accel = 1.0
    cmd.steering_angle = max_steer  # full left in ROS
    out = ca.carla_control_from_command(cmd, lon, max_steer, 10.0)
    assert out.throttle > 0.0
    assert out.brake == 0.0
    assert out.steer == pytest.approx(-1.0)  # CARLA: left is negative
    # Below the coast deceleration the brake pedal appears.
    cmd.accel = lon.coast(10.0) - 1.0
    cmd.steering_angle = 0.0
    out = ca.carla_control_from_command(cmd, lon, max_steer, 10.0)
    assert out.throttle == 0.0
    assert out.brake > 0.0
    assert out.steer == 0.0


def test_hold_at_rest_brakes_instead_of_creeping(
    vehicle: tuple[LongitudinalMap, float],
) -> None:
    lon, max_steer = vehicle
    hold = ca.HoldAtRest(speed_mps=0.3, accel_mps2=0.05, brake=0.3)
    cmd = ControlCommand()
    cmd.accel = 0.0  # "stay put": the pedal map alone would give rolling throttle
    cmd.steering_angle = max_steer / 2.0
    out = ca.carla_control_from_command(cmd, lon, max_steer, 0.1, hold)
    assert out.throttle == 0.0
    assert out.brake == pytest.approx(0.3)
    assert out.steer == pytest.approx(-0.5)  # the wheel is still converted
    # A positive command releases the hold; so does any speed.
    cmd.accel = 0.5
    assert ca.carla_control_from_command(cmd, lon, max_steer, 0.1, hold).throttle > 0.0
    cmd.accel = 0.0
    assert ca.carla_control_from_command(cmd, lon, max_steer, 1.0, hold).brake == 0.0
    # Without a hold the old behaviour stands.
    assert ca.carla_control_from_command(cmd, lon, max_steer, 0.1).brake == 0.0
