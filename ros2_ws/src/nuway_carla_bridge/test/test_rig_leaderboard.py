"""configs/sensors/rig_leaderboard.json: the dev rig's spawnable entries, same extrinsics (M1 §3.12)."""

from __future__ import annotations

import math
from pathlib import Path

from nuway_ml.common.rig import load_rig

REPO_ROOT = Path(__file__).resolve().parents[4]
MAX_ALLOWED_RADIUS_M = 3.0  # leaderboard/autoagents/agent_wrapper.py
SENSOR_LIMITS = {  # the qualifier limits of the pinned Leaderboard (the strictest set; MAP and SENSORS allow 8 / 2)
    "sensor.camera.rgb": 4,
    "sensor.lidar.ray_cast": 1,
    "sensor.other.gnss": 1,
    "sensor.other.imu": 1,
}


def test_leaderboard_rig_matches_the_dev_rig() -> None:
    dev = load_rig(REPO_ROOT / "configs/sensors/rig_dev.json")
    lb = load_rig(REPO_ROOT / "configs/sensors/rig_leaderboard.json")
    assert lb.vehicle == dev.vehicle
    expected = dev.select(label_only=False, viz_only=False)
    assert [s.id for s in lb.sensors] == [s.id for s in expected]
    for ours, theirs in zip(lb.sensors, expected, strict=True):
        assert ours.type == theirs.type
        assert (ours.x, ours.y, ours.z) == (theirs.x, theirs.y, theirs.z)
        assert (ours.roll, ours.pitch, ours.yaw) == (
            theirs.roll,
            theirs.pitch,
            theirs.yaw,
        )
        if ours.is_camera:
            assert ours.image_size == theirs.image_size
            assert ours.fov_deg == theirs.fov_deg


def test_leaderboard_rig_fits_the_runner_limits() -> None:
    lb = load_rig(REPO_ROOT / "configs/sensors/rig_leaderboard.json")
    counts: dict[str, int] = {}
    for s in lb.sensors:
        counts[s.type] = counts.get(s.type, 0) + 1
        assert math.sqrt(s.x**2 + s.y**2 + s.z**2) <= MAX_ALLOWED_RADIUS_M, s.id
    for sensor_type, n in counts.items():
        assert sensor_type in SENSOR_LIMITS, sensor_type
        assert n <= SENSOR_LIMITS[sensor_type], (sensor_type, n)
