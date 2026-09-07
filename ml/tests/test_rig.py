import json
import math

import numpy as np
import pytest

from nuway_ml.common.geometry import quaternion_to_rpy
from nuway_ml.common.rig import (
    R_OPTICAL_FROM_SENSOR,
    VehicleGeometry,
    intrinsics,
    load_rig,
    parse_rig,
    sensor_in_actor,
    sensor_in_base_link,
)

pytestmark = pytest.mark.import_light

VEHICLE = VehicleGeometry(
    wheelbase=2.86,
    length=4.892,
    width=1.837,
    height=1.49,
    rear_axle_offset_x=-1.389,
    bbox_center_z=0.749,
    actor_origin_height=0.091,
    max_steer_angle=1.2217,
)


def test_parse_rig_dev_json(repo_root_ml):
    rig = load_rig(repo_root_ml / "configs" / "sensors" / "rig_dev.json")
    assert rig.vehicle == "vehicle.lincoln.mkz_2020"
    ids = [s.id for s in rig.sensors]
    assert ids[:7] == [
        "lidar_top",
        "cam_front",
        "cam_left",
        "cam_right",
        "cam_rear",
        "imu",
        "gnss",
    ]
    assert rig.get("cam_chase").viz_only
    assert rig.get("lidar_top_semantic").label_only
    runtime = rig.select(label_only=False, viz_only=False)
    assert [s.id for s in runtime] == ids[:7]
    assert len(rig.select(label_only=True, viz_only=True)) == len(ids)
    assert rig.get("lidar_top").attributes["rotation_frequency"] == "20"
    assert rig.get("cam_front").image_size == (704, 256)


def test_duplicate_ids_are_rejected():
    raw = {
        "vehicle": "v",
        "sensors": [
            {"id": "a", "type": "sensor.other.imu"},
            {"id": "a", "type": "sensor.other.gnss"},
        ],
    }
    with pytest.raises(ValueError, match="duplicate"):
        parse_rig(raw)


def test_intrinsics_match_hand_computed_k():
    spec = parse_rig(
        {
            "vehicle": "v",
            "sensors": [
                {
                    "id": "c",
                    "type": "sensor.camera.rgb",
                    "attributes": {"image_size_x": 800, "image_size_y": 450, "fov": 90},
                }
            ],
        }
    ).get("c")
    k = intrinsics(spec)
    # 800 px wide at FOV 90: fx = 400 / tan(45 deg) = 400.
    np.testing.assert_allclose(
        k, [[400.0, 0.0, 400.0], [0.0, 400.0, 225.0], [0.0, 0.0, 1.0]]
    )
    spec2 = parse_rig(
        {
            "vehicle": "v",
            "sensors": [
                {
                    "id": "c",
                    "type": "sensor.camera.rgb",
                    "attributes": {"image_size_x": 704, "image_size_y": 256, "fov": 90},
                }
            ],
        }
    ).get("c")
    assert intrinsics(spec2)[0, 0] == pytest.approx(352.0)


def test_right_mounted_sensor_has_negative_y_in_base_link():
    # CARLA y = +0.9 is the right side; ROS base_link y must be -0.9. The rear
    # axle is 1.389 behind the actor origin, so x grows by 1.389; the origin sits
    # 0.091 above ground, so z grows by 0.091.
    spec = parse_rig(
        {
            "vehicle": "v",
            "sensors": [
                {
                    "id": "cam_right",
                    "type": "sensor.camera.rgb",
                    "x": 0.5,
                    "y": 0.9,
                    "z": 2.0,
                    "yaw": 90,
                    "attributes": {"image_size_x": 10, "image_size_y": 10},
                }
            ],
        }
    ).get("cam_right")
    in_actor = sensor_in_actor(spec)
    np.testing.assert_allclose(in_actor.translation, [0.5, -0.9, 2.0])
    pose = sensor_in_base_link(spec, VEHICLE)
    np.testing.assert_allclose(pose.translation, [0.5 + 1.389, -0.9, 2.0 + 0.091])
    # CARLA yaw +90 (right) is ROS yaw -pi/2.
    assert quaternion_to_rpy(pose.rotation)[2] == pytest.approx(-math.pi / 2.0)
    np.testing.assert_allclose(VEHICLE.base_link_in_actor, [-1.389, 0.0, -0.091])


def test_optical_rotation_maps_forward_to_z():
    forward = np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(R_OPTICAL_FROM_SENSOR @ forward, [0.0, 0.0, 1.0])
    left = np.array([0.0, 1.0, 0.0])
    np.testing.assert_allclose(R_OPTICAL_FROM_SENSOR @ left, [-1.0, 0.0, 0.0])
    assert np.linalg.det(R_OPTICAL_FROM_SENSOR) == pytest.approx(1.0)


def test_vehicle_geometry_from_yaml(repo_root_ml):
    geom = VehicleGeometry.from_yaml(
        repo_root_ml / "configs" / "vehicle" / "lincoln_mkz_2020.yaml"
    )
    assert geom.wheelbase == pytest.approx(2.86)
    assert geom.max_steer_angle == pytest.approx(math.radians(70.0), abs=1e-3)


def test_json_round_trip(tmp_path):
    raw = {
        "vehicle": "v",
        "sensors": [
            {
                "id": "imu",
                "type": "sensor.other.imu",
                "attributes": {"sensor_tick": 0.05},
            }
        ],
    }
    path = tmp_path / "rig.json"
    path.write_text(json.dumps(raw))
    assert load_rig(path).get("imu").attributes == {"sensor_tick": "0.05"}
