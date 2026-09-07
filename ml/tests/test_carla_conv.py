import math

import numpy as np
import pytest

from nuway_ml.common.carla_conv import (
    DEG_TO_RAD,
    CarlaLocation,
    CarlaRotation,
    angular_velocity_to_ros,
    location_from_ros,
    location_to_ros,
    rotation_from_ros,
    rotation_to_ros,
    steer_from_ros,
    steer_to_ros,
    transform_to_ros,
    yaw_from_ros,
    yaw_to_ros,
)
from nuway_ml.common.geometry import quaternion_to_yaw

pytestmark = pytest.mark.import_light


def test_location_flips_y():
    ros = location_to_ros(CarlaLocation(1.0, 2.0, 3.0))
    np.testing.assert_allclose(ros, [1.0, -2.0, 3.0])
    assert location_from_ros(ros).y == pytest.approx(2.0)


def test_yaw_clockwise_degrees_becomes_counter_clockwise_radians():
    assert yaw_to_ros(90.0) == pytest.approx(-math.pi / 2.0)
    assert yaw_from_ros(-math.pi / 2.0) == pytest.approx(90.0)
    assert yaw_to_ros(-180.0) == pytest.approx(math.pi)


def test_rotation_round_trip():
    rot = CarlaRotation(pitch=10.0, yaw=-30.0, roll=5.0)
    rpy = rotation_to_ros(rot)
    np.testing.assert_allclose(
        rpy, [5.0 * DEG_TO_RAD, -10.0 * DEG_TO_RAD, 30.0 * DEG_TO_RAD]
    )
    back = rotation_from_ros(rpy)
    assert back.pitch == pytest.approx(rot.pitch)
    assert back.yaw == pytest.approx(rot.yaw)
    assert back.roll == pytest.approx(rot.roll)


def test_transform_to_ros_right_mounted_sensor_has_negative_y():
    # docs/02_interfaces.md §1 M0 finding: CARLA y=+1.5 (right) is ROS y=-1.5.
    pose = transform_to_ros(CarlaLocation(1.0, 1.5, 2.0), CarlaRotation(0.0, 90.0, 0.0))
    assert pose.translation[1] == pytest.approx(-1.5)
    assert quaternion_to_yaw(pose.rotation) == pytest.approx(-math.pi / 2.0)


def test_duck_typed_carla_objects_are_accepted():
    class Loc:
        x, y, z = 0.5, -1.0, 2.0

    np.testing.assert_allclose(location_to_ros(Loc()), [0.5, 1.0, 2.0])
    np.testing.assert_allclose(
        angular_velocity_to_ros(CarlaLocation(0.0, 0.0, 90.0)),
        [0.0, 0.0, -math.pi / 2.0],
    )


def test_steer_from_ros_flips_sign_and_clamps():
    assert steer_from_ros(0.61, 1.22) == pytest.approx(-0.5)
    assert steer_from_ros(-5.0, 1.22) == pytest.approx(1.0)
    assert steer_to_ros(-0.5, 1.22) == pytest.approx(0.61)
