import pytest

from nuway_ml.common.polynomial import Polynomial5

pytestmark = pytest.mark.import_light


def test_quintic_meets_both_boundary_states():
    p = Polynomial5.quintic(1.0, 0.5, -0.2, 4.0, 0.0, 0.0, 7.0)
    assert p.eval(0.0) == pytest.approx(1.0)
    assert p.eval_first(0.0) == pytest.approx(0.5)
    assert p.eval_second(0.0) == pytest.approx(-0.2)
    assert p.eval(7.0) == pytest.approx(4.0, abs=1e-9)
    assert p.eval_first(7.0) == pytest.approx(0.0, abs=1e-9)
    assert p.eval_second(7.0) == pytest.approx(0.0, abs=1e-9)


def test_quartic_reaches_the_end_velocity_with_free_position():
    p = Polynomial5.quartic(0.0, 5.0, 0.0, 12.0, 0.0, 6.0)
    assert p.eval_first(6.0) == pytest.approx(12.0, abs=1e-9)
    assert p.eval_second(6.0) == pytest.approx(0.0, abs=1e-9)
    assert p.c[5] == 0.0
    assert 30.0 < p.eval(6.0) < 72.0


def test_derivatives_match_finite_differences():
    p = Polynomial5.quintic(0.0, 1.0, 0.5, 3.0, -1.0, 0.25, 4.0)
    h = 1e-4
    for i in range(9):
        t = 0.5 * i
        assert p.eval_first(t) == pytest.approx(
            (p.eval(t + h) - p.eval(t - h)) / (2 * h), abs=1e-6
        )
        assert p.eval_second(t) == pytest.approx(
            (p.eval_first(t + h) - p.eval_first(t - h)) / (2 * h), abs=1e-6
        )
        assert p.eval_third(t) == pytest.approx(
            (p.eval_second(t + h) - p.eval_second(t - h)) / (2 * h), abs=1e-6
        )


def test_non_positive_length_holds_the_start():
    p = Polynomial5.quintic(2.0, 1.0, 1.0, 9.0, 9.0, 9.0, 0.0)
    assert p.eval(3.0) == 2.0
    assert p.eval_first(3.0) == 0.0
