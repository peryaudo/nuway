import math

import numpy as np
import pytest

from nuway_ml.common.frenet import FrenetPoint, ReferenceLine, menger_curvature

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
