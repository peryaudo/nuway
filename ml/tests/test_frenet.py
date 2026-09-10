import math

import numpy as np
import pytest

from nuway_ml.common.frenet import (
    CartesianState,
    FrenetPoint,
    FrenetState,
    ReferenceLine,
    menger_curvature,
)

pytestmark = pytest.mark.import_light


def make_arc(radius: float, step: float) -> ReferenceLine:
    total = math.pi / 2.0 * radius
    n = math.floor(total / step) + 1
    theta = np.arange(n) * step / radius
    return ReferenceLine.from_points(
        np.stack([radius * np.cos(theta), radius * np.sin(theta)], axis=1)
    )


def test_round_trip_on_arc_within_tolerance():
    line = make_arc(30.0, 0.5)
    for si in range(120):
        s = 1.0 + si * 0.37
        for di in range(-4, 5):
            d = di * 0.5
            cart = line.to_cartesian(FrenetPoint(s, d))
            back = line.to_frenet(cart.x, cart.y)
            assert back is not None
            assert back.s == pytest.approx(s, abs=1e-6)
            assert back.d == pytest.approx(d, abs=1e-6)


def test_curvature_of_circle_matches_inverse_radius():
    line = make_arc(20.0, 0.5)
    for s in range(2, 28):
        assert line.curvature_at(float(s)) == pytest.approx(1.0 / 20.0, abs=1e-4)
    assert menger_curvature(np.array([0, 0]), np.array([1, 0]), np.array([2, 0])) == 0.0


def test_left_of_line_is_positive():
    line = ReferenceLine.from_points(np.stack([np.arange(11.0), np.zeros(11)], axis=1))
    f = line.to_frenet(4.25, 1.5)
    assert f is not None
    assert f.s == pytest.approx(4.25, abs=1e-9)
    assert f.d == pytest.approx(1.5, abs=1e-9)
    assert line.heading_at(4.25) == pytest.approx(0.0)
    assert line.length == pytest.approx(10.0)
    assert line.point_at(3.0).x == pytest.approx(3.0)


def test_far_point_beyond_max_dist_is_rejected():
    line = ReferenceLine.from_points(np.array([[0.0, 0.0], [10.0, 0.0]]))
    assert line.to_frenet(5.0, 50.0, 10.0) is None
    assert line.to_frenet(5.0, 5.0, 10.0) is not None
    assert ReferenceLine.from_points(np.zeros((1, 2))).to_frenet(0.0, 0.0) is None


def make_circle(radius: float, step: float) -> ReferenceLine:
    total = 2.0 * math.pi * radius
    n = math.floor(total / step) + 1
    theta = np.arange(n) * step / radius
    return ReferenceLine.from_points(
        np.stack([radius * np.cos(theta), radius * np.sin(theta)], axis=1)
    )


def test_state_round_trip_on_an_arc():
    line = make_circle(40.0, 0.5)
    for i in range(40):
        f = FrenetState(
            s=5.0 + i * 4.0,
            s_dot=8.0 + 0.1 * i,
            s_ddot=-0.5 + 0.05 * i,
            d=-2.0 + 0.1 * i,
            d_prime=0.05 * ((i % 5) - 2),
            d_dprime=0.01 * ((i % 3) - 1),
        )
        c = line.to_cartesian_state(f)
        back = line.to_frenet_state(c)
        assert back is not None
        assert back.s == pytest.approx(f.s, abs=1e-6)
        assert back.d == pytest.approx(f.d, abs=1e-6)
        assert back.s_dot == pytest.approx(f.s_dot, abs=1e-6)
        assert back.d_prime == pytest.approx(f.d_prime, abs=1e-6)
        # Menger curvature on 0.5 m chords: the second derivatives to ~1e-5.
        assert back.s_ddot == pytest.approx(f.s_ddot, abs=1e-4)
        assert back.d_dprime == pytest.approx(f.d_dprime, abs=1e-4)


def test_state_on_the_line_has_the_line_heading_and_curvature():
    line = make_circle(40.0, 0.5)
    c = line.to_cartesian_state(FrenetState(s=30.0, s_dot=10.0, s_ddot=1.0))
    assert c.yaw == pytest.approx(line.heading_at(30.0), abs=1e-9)
    assert c.v == pytest.approx(10.0, abs=1e-9)
    assert c.a == pytest.approx(1.0, abs=1e-9)
    assert c.kappa == pytest.approx(1.0 / 40.0, abs=1e-4)
    inner = line.to_cartesian_state(FrenetState(s=30.0, s_dot=10.0, d=2.0))
    assert inner.v == pytest.approx(10.0 * (1.0 - 2.0 / 40.0), abs=1e-3)
    assert inner.kappa == pytest.approx(1.0 / 38.0, abs=1e-4)


def test_velocity_projection_on_a_straight_line():
    line = ReferenceLine.from_points(
        np.stack([0.5 * np.arange(101.0), np.zeros(101)], axis=1)
    )
    c = CartesianState(x=20.0, y=1.0, yaw=math.pi / 6.0, v=10.0)
    f = line.to_frenet_state(c)
    assert f is not None
    assert f.s_dot == pytest.approx(10.0 * math.cos(math.pi / 6.0), abs=1e-9)
    assert f.d_prime == pytest.approx(math.tan(math.pi / 6.0), abs=1e-9)
    assert f.d_dprime == pytest.approx(0.0, abs=1e-9)
    near = line.to_frenet_state_near(c, 5.0, 18.0, back_m=5.0, ahead_m=10.0)
    assert near is not None
    assert near.s == pytest.approx(20.0, abs=1e-9)
