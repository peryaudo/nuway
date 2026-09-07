import math

import numpy as np
import pytest

from nuway_ml.common.geometry import (
    PI,
    SE2,
    SE3,
    apply,
    between,
    compose,
    inverse,
    quaternion_to_rpy,
    quaternion_to_yaw,
    rotate,
    rpy_to_quaternion,
    to_se2,
    wrap_angle,
    wrap_angles,
    yaw_to_quaternion,
)

pytestmark = pytest.mark.import_light


def test_wrap_angle_maps_into_half_open_interval():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(PI) == pytest.approx(PI)
    assert wrap_angle(-PI) == pytest.approx(PI)
    assert wrap_angle(3.0 * PI) == pytest.approx(PI)
    assert wrap_angle(2.0 * PI + 0.5) == pytest.approx(0.5)
    np.testing.assert_allclose(
        wrap_angles(np.array([2.0 * PI + 0.5, -2.0 * PI - 0.5])), [0.5, -0.5]
    )


def test_compose_then_between_recovers_relative_pose():
    a = SE2(1.0, 2.0, 0.3)
    b = SE2(-0.5, 4.0, -2.0)
    ab = compose(a, b)
    assert isinstance(ab, SE2)
    rec = between(a, ab)
    assert rec.x == pytest.approx(b.x, abs=1e-9)
    assert rec.y == pytest.approx(b.y, abs=1e-9)
    assert rec.yaw == pytest.approx(b.yaw, abs=1e-9)


def test_inverse_composes_to_identity():
    a = SE2(3.0, -1.0, 2.5)
    ident = compose(a, inverse(a))
    assert isinstance(ident, SE2)
    assert abs(ident.x) < 1e-9
    assert abs(ident.y) < 1e-9
    assert abs(ident.yaw) < 1e-9


def test_apply_rotates_counter_clockwise_and_batches():
    pose = SE2(0.0, 0.0, PI / 2.0)
    np.testing.assert_allclose(apply(pose, np.array([1.0, 0.0])), [0.0, 1.0], atol=1e-9)
    out = apply(pose, np.array([[1.0, 0.0], [0.0, 1.0]]))
    np.testing.assert_allclose(out, [[0.0, 1.0], [-1.0, 0.0]], atol=1e-9)
    np.testing.assert_allclose(
        rotate(SE2(5.0, 5.0, PI / 2.0), np.array([1.0, 0.0])), [0.0, 1.0], atol=1e-9
    )


def test_rpy_quaternion_round_trip():
    rpy = quaternion_to_rpy(rpy_to_quaternion(0.1, -0.2, 2.0))
    np.testing.assert_allclose(rpy, [0.1, -0.2, 2.0], atol=1e-9)
    assert quaternion_to_yaw(yaw_to_quaternion(-2.5)) == pytest.approx(-2.5)
    q = yaw_to_quaternion(math.pi / 2.0)
    np.testing.assert_allclose(
        q, [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12
    )


def test_se3_compose_inverse_is_identity():
    a = SE3(np.array([1.0, 2.0, 3.0]), rpy_to_quaternion(0.3, -0.4, 1.2))
    ident = compose(a, inverse(a))
    assert isinstance(ident, SE3)
    assert np.linalg.norm(ident.translation) < 1e-9
    assert abs(abs(ident.rotation[3]) - 1.0) < 1e-9
    np.testing.assert_allclose(apply(a, np.zeros(3)), [1.0, 2.0, 3.0])
    planar = to_se2(a)
    assert planar.yaw == pytest.approx(1.2)
