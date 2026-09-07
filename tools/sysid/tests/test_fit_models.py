"""Tests for tools/sysid/fit_models.py on synthetic sweeps of a known model."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import fit_models as fm  # type: ignore[import-not-found]  # tools/ scripts are not a package

from nuway_ml.common.longitudinal_map import LongitudinalMap
from nuway_ml.common.tick import TICK_DT_S

TAU = 0.2
WHEELBASE = 2.86
K_US = 0.002


def drag(v: float) -> float:
    return -(0.1 + 0.005 * v + 0.0006 * v * v)


def accel_true(v: float, u: float) -> float:
    return 4.5 * u * (1.0 - v / 45.0) + drag(v)


def decel_true(v: float, b: float) -> float:
    return -8.0 * b + drag(v)


def simulate(mode: str, run: int, v0: float, level: float, seconds: float) -> fm.Run:
    """First-order actuator lag on the pedal, Euler integration of v."""
    n = int(seconds / TICK_DT_S)
    v = np.zeros(n)
    a = np.zeros(n)
    u = 0.0
    speed = v0
    for i in range(n):
        u += (level - u) * (TICK_DT_S / TAU)
        if mode == "throttle":
            acc = accel_true(speed, u)
        elif mode == "brake":
            acc = decel_true(speed, u) if speed > 0.0 else 0.0
        else:
            acc = drag(speed)
        v[i] = speed
        a[i] = acc
        speed = max(0.0, speed + acc * TICK_DT_S)
    t = np.arange(n) * TICK_DT_S
    return fm.Run(
        mode, run, v0, level, t, v, fm.acceleration(v), np.zeros(n), np.zeros(n)
    )


def steer_run(run: int, v0: float, steer: float, max_steer: float = 1.2217) -> fm.Run:
    n = int(4.0 / TICK_DT_S)
    t = np.arange(n) * TICK_DT_S
    delta_ss = -steer * max_steer  # CARLA steer > 0 turns right: ROS wheel angle < 0
    delta = delta_ss * (1.0 - np.exp(-t / 0.12))
    # ROS convention: the wheel angle is +left, the yaw rate follows its sign.
    yaw_rate = v0 * np.tan(delta) / (WHEELBASE + K_US * v0 * v0)
    v = np.full(n, v0)
    return fm.Run("steer", run, v0, steer, t, v, np.zeros(n), yaw_rate, delta)


@pytest.fixture(scope="module")
def synthetic_runs() -> dict[str, list[fm.Run]]:
    runs: dict[str, list[fm.Run]] = {
        "throttle": [],
        "brake": [],
        "coast": [],
        "steer": [],
    }
    levels = [round(0.1 * i, 1) for i in range(1, 11)]
    for v0 in (0.0, 5.0, 10.0, 15.0, 20.0):
        for u in levels:
            runs["throttle"].append(
                simulate("throttle", len(runs["throttle"]), v0, u, 12.0)
            )
    for v0 in (5.0, 10.0, 20.0, 30.0):
        for b in levels:
            runs["brake"].append(simulate("brake", len(runs["brake"]), v0, b, 20.0))
    for v0 in (10.0, 20.0, 30.0):
        runs["coast"].append(simulate("coast", len(runs["coast"]), v0, 0.0, 40.0))
    for v0 in (5.0, 10.0, 15.0):
        for s in (0.02, -0.02, 0.05, -0.05, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4):
            runs["steer"].append(steer_run(len(runs["steer"]), v0, s))
    return runs


def test_acceleration_is_central_difference() -> None:
    v = np.arange(0.0, 5.0, 0.1)  # a = 2 m/s^2 at 0.05 s ticks
    a = fm.acceleration(v)
    assert a[10] == pytest.approx(2.0)
    assert len(fm.acceleration(np.array([1.0, 2.0]))) == 2


def test_longitudinal_fit_recovers_the_model(
    synthetic_runs: dict[str, list[fm.Run]],
) -> None:
    fit = fm.fit_longitudinal(synthetic_runs)
    lon = LongitudinalMap.from_dict(fm.longitudinal_dict(fit))
    lon.validate()
    # Coast drag within a few cm/s^2 over the measured range.
    for v in (5.0, 15.0, 25.0):
        assert lon.coast(v) == pytest.approx(drag(v), abs=0.03)
    # Throttle / brake tables within 0.1 m/s^2 where runs reached the bin.
    for v, u in ((3.0, 0.5), (12.0, 0.8), (20.0, 1.0), (2.0, 0.2)):
        assert lon.accel(v, u) == pytest.approx(accel_true(v, u), abs=0.1)
    for v, b in ((4.0, 0.3), (15.0, 0.6), (25.0, 0.9)):
        assert lon.decel(v, b) == pytest.approx(decel_true(v, b), abs=0.15)
    # Bins no run reached (throttle 0.1 at 25 m/s) extrapolate along the drag curve.
    assert math.isfinite(lon.accel(25.0, 0.1))
    assert fit.residual_rms["throttle"] < 0.1
    assert fit.residual_rms["brake"] < 0.15
    assert fit.tau_throttle == pytest.approx(TAU, abs=0.1)
    # Inverse map round trip.
    throttle, brake = lon.inverse(10.0, 1.5)
    assert brake == 0.0
    assert lon.accel(10.0, throttle) == pytest.approx(1.5, abs=0.05)


def test_lateral_fit_recovers_wheelbase_and_lag(
    synthetic_runs: dict[str, list[fm.Run]],
) -> None:
    lat = fm.fit_lateral(synthetic_runs["steer"], geometric_wheelbase=2.9)
    assert lat.wheelbase == pytest.approx(WHEELBASE, rel=0.02)
    assert lat.tau_steer == pytest.approx(0.12, abs=0.05)
    assert lat.understeer_gradient == pytest.approx(K_US, abs=0.001)
    # Runs with a steady lateral acceleration above 3 m/s^2 are excluded.
    assert 0 < lat.samples < 30
    assert lat.residual_rms_yaw_rate < 0.02
    empty = fm.fit_lateral([], geometric_wheelbase=2.9)
    assert empty.wheelbase == 2.9


def test_yaml_round_trip(
    tmp_path: Path, synthetic_runs: dict[str, list[fm.Run]]
) -> None:
    lon = fm.fit_longitudinal(synthetic_runs)
    lat = fm.fit_lateral(synthetic_runs["steer"], geometric_wheelbase=2.86)
    existing = {
        "name": "x",
        "wheelbase": 2.86,
        "max_steer_angle": 1.2217,
        "tau_steer": 0.0,
    }
    data = fm.build_vehicle_yaml(existing, lon, lat, source="synthetic")
    path = tmp_path / "vehicle.yaml"
    fm.write_vehicle_yaml(path, data)
    text = path.read_text()
    assert text.startswith("# configs/vehicle/lincoln_mkz_2020.yaml")
    loaded = LongitudinalMap.from_yaml(path)
    loaded.validate()
    assert data["sysid"]["status"] == "fitted"
    assert data["wheelbase"] == 2.86  # geometry kept
    assert data["wheelbase_fitted"] == pytest.approx(WHEELBASE, rel=0.02)


def test_runs_from_rows_groups_hold_phase() -> None:
    rows = []
    for k in range(30):
        rows.append(
            {
                "mode": "throttle",
                "run": "0",
                "v0": "0",
                "level": "0.5",
                "phase": "hold" if k >= 10 else "settle",
                "k": str(k),
                "t": str(k * TICK_DT_S),
                "vx": str(0.1 * max(0, k - 10)),
                "yaw_rate": "0",
                "wheel_angle": "0",
            }
        )
    runs = fm.runs_from_rows(rows)
    assert len(runs) == 1
    assert len(runs[0].t) == 20
    assert runs[0].t[0] == 0.0
    assert runs[0].level == 0.5
