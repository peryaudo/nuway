"""nuway_carla_bridge.leaderboard_agent: the pure parts (no evaluator, no ROS)."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import pytest

from nuway_carla_bridge import leaderboard_agent as la
from nuway_ml.common.longitudinal_map import LongitudinalMap
from nuway_ml.common.rig import load_rig

REPO_ROOT = Path(__file__).resolve().parents[4]
VEHICLE = REPO_ROOT / "configs/vehicle/lincoln_mkz_2020.yaml"


def test_runner_sensors_are_the_rig_plus_the_pseudo_sensors() -> None:
    rig = load_rig(REPO_ROOT / "configs/sensors/rig_leaderboard.json")
    specs = la.runner_sensors(rig)
    ids = [s["id"] for s in specs]
    assert ids[-2:] == [la.OPENDRIVE_ID, la.SPEED_ID]
    assert ids[:-2] == [s.id for s in rig.sensors]
    cams = [s for s in specs if s["type"] == "sensor.camera.rgb"]
    assert len(cams) == 4
    assert all({"width", "height", "fov"} <= set(c) for c in cams)
    front = next(c for c in cams if c["id"] == "cam_front")
    assert (front["width"], front["height"], front["fov"]) == (704, 256, 90.0)
    assert (front["x"], front["y"], front["z"]) == (1.5, 0.0, 2.0)


def test_lockstep_gate_releases_on_the_tick_or_later_and_times_out() -> None:
    gate = la.LockstepGate()
    waited, timed_out = gate.wait(5, 0.05)
    assert timed_out
    assert waited >= 0.05
    threading.Timer(0.05, gate.offer, args=(7,)).start()
    waited, timed_out = gate.wait(5, 2.0)  # a newer tick's command releases k = 5 too
    assert not timed_out
    assert waited < 1.0
    gate.offer(3)  # an older one never lowers the latest
    assert gate.latest_k == 7


def test_pedals_match_the_control_adapter_mapping() -> None:
    lon = LongitudinalMap.from_yaml(VEHICLE)
    max_steer = 1.2217
    p = la.pedals_from_command(
        1.0, max_steer, False, speed_mps=10.0, lon_map=lon, max_steer_rad=max_steer
    )
    assert p.throttle > 0.0
    assert p.brake == 0.0
    assert p.steer == pytest.approx(-1.0)  # CARLA: left is negative
    stop = la.pedals_from_command(
        1.0, 0.3, True, speed_mps=10.0, lon_map=lon, max_steer_rad=max_steer
    )
    assert (stop.throttle, stop.brake) == (0.0, 1.0)
    assert stop.steer == pytest.approx(
        -0.3 / max_steer
    )  # the wheel is kept under e-stop
    held = la.brake_pedals(0.2)
    assert (held.throttle, held.brake, held.steer) == (0.0, 1.0, 0.2)


def test_opendrive_is_written_once(tmp_path: Path) -> None:
    path = la.write_opendrive(
        tmp_path, la.town_name("Carla/Maps/Town03"), "<OpenDRIVE/>"
    )
    assert path == tmp_path / "Town03" / "map.xodr"
    assert path.read_text() == "<OpenDRIVE/>"
    la.write_opendrive(tmp_path, "Town03", "changed")
    assert path.read_text() == "<OpenDRIVE/>"


def test_agent_config_refuses_gt(tmp_path: Path) -> None:
    prof = tmp_path / "p.yaml"
    prof.write_text(
        "sensors: configs/sensors/rig_leaderboard.json\n"
        "vehicle: configs/vehicle/lincoln_mkz_2020.yaml\n"
        "use_gt: {localization: true}\n"
    )
    with pytest.raises(ValueError, match=r"use_gt\.localization"):
        la.AgentConfig.load(prof, REPO_ROOT)
    cfg = la.AgentConfig.load(
        REPO_ROOT / "configs/profiles/leaderboard.yaml", REPO_ROOT
    )
    assert cfg.lockstep_timeout_s == 2.0
    assert cfg.vehicle.max_steer_angle == pytest.approx(1.2217)


def test_stats_summary() -> None:
    stats = la.AgentStats(ticks=3, tick_timeouts=1, serialize_ms=[1.0, 2.0, 6.0])
    d = stats.as_dict()
    assert d["ticks"] == 3
    assert d["tick_timeouts"] == 1
    assert d["serialize_ms_mean"] == pytest.approx(3.0)
    assert d["serialize_ms_max"] == pytest.approx(6.0)
    t0 = time.monotonic()
    assert la.AgentStats().as_dict()["serialize_ms_mean"] == 0.0
    assert time.monotonic() - t0 < 1.0
