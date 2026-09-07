"""Cross-language parity of nuway_ml.common against nuway_common via nuway_py (M0).

Every function pair is exercised on the same random inputs; the two sides must
agree to floating-point tolerance. Skipped when ``nuway_py`` is not importable
(no colcon build sourced).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
import pytest

from nuway_ml.common import carla_conv, frenet, geometry, occupancy, tick

nuway_py = pytest.importorskip("nuway_py")

RNG = np.random.default_rng(0)
ATOL = 1e-9


def test_wrap_angle_and_se2_parity() -> None:
    for _ in range(200):
        ax, ay, ayaw, bx, by, byaw = RNG.uniform(-50, 50, 6)
        a = geometry.SE2(float(ax), float(ay), float(ayaw))
        b = geometry.SE2(float(bx), float(by), float(byaw))
        assert geometry.wrap_angle(a.yaw) == pytest.approx(
            nuway_py.wrap_angle(a.yaw), abs=ATOL
        )
        py = geometry.compose(a, b)
        cpp = nuway_py.compose(
            nuway_py.SE2(a.x, a.y, a.yaw), nuway_py.SE2(b.x, b.y, b.yaw)
        )
        assert isinstance(py, geometry.SE2)
        assert (py.x, py.y, py.yaw) == pytest.approx((cpp.x, cpp.y, cpp.yaw), abs=ATOL)
        inv_py = geometry.inverse(a)
        inv_cpp = nuway_py.inverse(nuway_py.SE2(a.x, a.y, a.yaw))
        assert isinstance(inv_py, geometry.SE2)
        assert (inv_py.x, inv_py.y, inv_py.yaw) == pytest.approx(
            (inv_cpp.x, inv_cpp.y, inv_cpp.yaw), abs=ATOL
        )
        bt_py = geometry.between(a, b)
        bt_cpp = nuway_py.between(
            nuway_py.SE2(a.x, a.y, a.yaw), nuway_py.SE2(b.x, b.y, b.yaw)
        )
        assert (bt_py.x, bt_py.y, bt_py.yaw) == pytest.approx(
            (bt_cpp.x, bt_cpp.y, bt_cpp.yaw), abs=ATOL
        )
        point = RNG.uniform(-10, 10, 2)
        np.testing.assert_allclose(
            geometry.apply(a, point).tolist(),
            nuway_py.apply(nuway_py.SE2(a.x, a.y, a.yaw), tuple(point)),
            atol=ATOL,
        )
        np.testing.assert_allclose(
            geometry.rotate(a, point).tolist(),
            nuway_py.rotate(nuway_py.SE2(a.x, a.y, a.yaw), tuple(point)),
            atol=ATOL,
        )


def test_quaternion_and_se3_parity() -> None:
    for _ in range(200):
        roll, pitch, yaw = RNG.uniform(-math.pi, math.pi, 3)
        pitch = float(np.clip(pitch, -1.5, 1.5))
        q_py = geometry.rpy_to_quaternion(roll, pitch, yaw)
        q_cpp = np.array(nuway_py.rpy_to_quaternion(roll, pitch, yaw))
        # Same rotation: q and -q are equal up to sign.
        assert min(np.abs(q_py - q_cpp).max(), np.abs(q_py + q_cpp).max()) < ATOL
        np.testing.assert_allclose(
            geometry.quaternion_to_rpy(q_py),
            nuway_py.quaternion_to_rpy(list(q_py)),
            atol=ATOL,
        )
        assert geometry.quaternion_to_yaw(q_py) == pytest.approx(
            nuway_py.quaternion_to_yaw(list(q_py)), abs=ATOL
        )
        ta = RNG.uniform(-10, 10, 3)
        tb = RNG.uniform(-10, 10, 3)
        qb = geometry.rpy_to_quaternion(*RNG.uniform(-1, 1, 3))
        a = geometry.SE3(ta, q_py)
        b = geometry.SE3(tb, qb)
        out_py = geometry.compose(a, b)
        assert isinstance(out_py, geometry.SE3)
        t_cpp, q_cpp = nuway_py.compose_se3(list(ta), list(q_py), list(tb), list(qb))
        np.testing.assert_allclose(out_py.translation, t_cpp, atol=ATOL)
        assert (
            min(
                np.abs(out_py.rotation - q_cpp).max(),
                np.abs(out_py.rotation + q_cpp).max(),
            )
            < ATOL
        )
        inv_py = geometry.inverse(a)
        assert isinstance(inv_py, geometry.SE3)
        t_cpp, q_cpp = nuway_py.inverse_se3(list(ta), list(q_py))
        np.testing.assert_allclose(inv_py.translation, t_cpp, atol=ATOL)
        assert (
            min(
                np.abs(inv_py.rotation - q_cpp).max(),
                np.abs(inv_py.rotation + q_cpp).max(),
            )
            < ATOL
        )


def test_carla_conv_parity() -> None:
    for _ in range(200):
        loc = carla_conv.CarlaLocation(*(float(v) for v in RNG.uniform(-100, 100, 3)))
        rot = carla_conv.CarlaRotation(*(float(v) for v in RNG.uniform(-360, 360, 3)))
        cl = nuway_py.CarlaLocation(loc.x, loc.y, loc.z)
        cr = nuway_py.CarlaRotation(rot.pitch, rot.yaw, rot.roll)
        np.testing.assert_allclose(
            carla_conv.location_to_ros(loc), nuway_py.location_to_ros(cl), atol=ATOL
        )
        np.testing.assert_allclose(
            carla_conv.rotation_to_ros(rot), nuway_py.rotation_to_ros(cr), atol=ATOL
        )
        assert carla_conv.yaw_to_ros(rot.yaw) == pytest.approx(
            nuway_py.yaw_to_ros(rot.yaw), abs=ATOL
        )
        assert carla_conv.yaw_from_ros(rot.yaw / 57.0) == pytest.approx(
            nuway_py.yaw_from_ros(rot.yaw / 57.0), abs=ATOL
        )
        pose_py = carla_conv.transform_to_ros(loc, rot)
        t_cpp, q_cpp = nuway_py.transform_to_ros(cl, cr)
        np.testing.assert_allclose(pose_py.translation, t_cpp, atol=ATOL)
        assert (
            min(
                np.abs(pose_py.rotation - q_cpp).max(),
                np.abs(pose_py.rotation + q_cpp).max(),
            )
            < ATOL
        )
        back_py = carla_conv.rotation_from_ros(carla_conv.rotation_to_ros(rot))
        back_cpp = nuway_py.rotation_from_ros(nuway_py.rotation_to_ros(cr))
        assert (back_py.pitch, back_py.yaw, back_py.roll) == pytest.approx(
            (back_cpp.pitch, back_cpp.yaw, back_cpp.roll), abs=1e-7
        )
        np.testing.assert_allclose(
            carla_conv.angular_velocity_to_ros(loc),
            nuway_py.angular_velocity_to_ros([loc.x, loc.y, loc.z]),
            atol=ATOL,
        )


def _random_line() -> tuple[frenet.ReferenceLine, Any]:
    # A smooth wiggly curve: heading integrates a low-frequency random signal.
    n = 400
    kappa = 0.03 * np.sin(np.linspace(0, 4 * math.pi, n) + RNG.uniform(0, 6))
    heading = np.cumsum(kappa) * 0.5
    pts = np.cumsum(np.stack([np.cos(heading), np.sin(heading)], axis=1) * 0.5, axis=0)
    pts += RNG.uniform(-100, 100, 2)
    return frenet.ReferenceLine.from_points(pts), nuway_py.ReferenceLine.from_points(
        [tuple(p) for p in pts]
    )


def test_frenet_parity() -> None:
    for _ in range(5):
        line_py, line_cpp = _random_line()
        np.testing.assert_allclose(line_py.s, line_cpp.s, atol=ATOL)
        np.testing.assert_allclose(line_py.heading, line_cpp.heading, atol=ATOL)
        np.testing.assert_allclose(line_py.curvature, line_cpp.curvature, atol=ATOL)
        assert line_py.length == pytest.approx(line_cpp.length, abs=ATOL)
        for _ in range(100):
            s = RNG.uniform(0, line_py.length)
            d = RNG.uniform(-3, 3)
            assert line_py.heading_at(s) == pytest.approx(
                line_cpp.heading_at(s), abs=ATOL
            )
            assert line_py.curvature_at(s) == pytest.approx(
                line_cpp.curvature_at(s), abs=ATOL
            )
            c_py = line_py.to_cartesian(frenet.FrenetPoint(s, d))
            c_cpp = line_cpp.to_cartesian(nuway_py.FrenetPoint(s, d))
            assert (c_py.x, c_py.y, c_py.heading) == pytest.approx(
                (c_cpp.x, c_cpp.y, c_cpp.heading), abs=ATOL
            )
            f_py = line_py.to_frenet(c_py.x, c_py.y)
            f_cpp = line_cpp.to_frenet(c_py.x, c_py.y)
            assert f_py is not None
            assert f_cpp is not None
            assert (f_py.s, f_py.d) == pytest.approx((f_cpp.s, f_cpp.d), abs=1e-7)
        assert line_py.to_frenet(1e6, 1e6, 10.0) is None
        assert line_cpp.to_frenet(1e6, 1e6, 10.0) is None


def test_occupancy_parity() -> None:
    spec_py = occupancy.GridSpec(0.5, -50.0, -50.0, 200, 200)
    spec_cpp = nuway_py.GridSpec(0.5, -50.0, -50.0, 200, 200)
    assert list(occupancy.OCCUPANCY_CHANNEL_NAMES) == list(
        nuway_py.OCCUPANCY_CHANNEL_NAMES
    )
    channel = RNG.uniform(0, 1, (200, 200))
    torch = pytest.importorskip("torch", reason="bilinear_sample is torch-only")
    for _ in range(500):
        x, y = RNG.uniform(-52, 52, 2)
        np.testing.assert_allclose(
            occupancy.world_to_grid_coord(spec_py, x, y),
            nuway_py.world_to_grid_coord(spec_cpp, x, y),
            atol=ATOL,
        )
        assert occupancy.world_to_grid(spec_py, x, y) == nuway_py.world_to_grid(
            spec_cpp, x, y
        )
        row, col = RNG.integers(0, 200, 2)
        np.testing.assert_allclose(
            occupancy.grid_to_world(spec_py, int(row), int(col)),
            nuway_py.grid_to_world(spec_cpp, int(row), int(col)),
            atol=ATOL,
        )
        v_py = occupancy.bilinear_sample(
            spec_py,
            torch.as_tensor(channel),
            torch.tensor([x]),
            torch.tensor([y]),
            outside=-1.0,
        )
        v_cpp = nuway_py.bilinear_sample(spec_cpp, channel.ravel().tolist(), x, y, -1.0)
        assert float(v_py[0]) == pytest.approx(v_cpp, abs=ATOL)


def test_tick_parity() -> None:
    assert tick.TICK_DT_S == nuway_py.TICK_DT_S
    for _ in range(1000):
        stamp = RNG.uniform(0, 1e5)
        assert tick.tick_index(stamp) == nuway_py.tick_index(stamp)
        k = int(RNG.integers(0, 10_000_000))
        assert tick.tick_stamp(k) == tuple(nuway_py.tick_stamp(k))
        assert tick.tick_time_s(k) == pytest.approx(nuway_py.tick_time_s(k), abs=ATOL)
        assert tick.is_planning_tick(k) == nuway_py.is_planning_tick(k)
