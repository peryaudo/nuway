import numpy as np
import pytest

from nuway_ml.common.longitudinal_map import LongitudinalMap

pytestmark = pytest.mark.import_light


def make_map() -> LongitudinalMap:
    v = np.array([0.0, 10.0, 20.0])
    thr = np.array([0.0, 0.5, 1.0])
    brk = np.array([0.0, 0.5, 1.0])
    coast = np.array([-0.1, -0.3, -0.6])
    accel = np.array([[c, c + 2.0, c + 4.0] for c in coast])
    decel = np.array([[c, c - 3.0, c - 6.0] for c in coast])
    return LongitudinalMap(v, thr, accel, brk, decel, coast)


def test_forward_tables_interpolate_bilinearly():
    lm = make_map()
    assert lm.accel(5.0, 0.25) == pytest.approx(-0.2 + 1.0)
    assert lm.decel(15.0, 0.75) == pytest.approx(-0.45 - 4.5)
    assert lm.coast(10.0) == pytest.approx(-0.3)
    # Clamped outside the bins.
    assert lm.accel(50.0, 2.0) == pytest.approx(-0.6 + 4.0)


def test_inverse_recovers_pedal_and_saturates():
    lm = make_map()
    throttle, brake = lm.inverse(10.0, 1.7)  # -0.3 + 4 * 0.5 = 1.7
    assert throttle == pytest.approx(0.5)
    assert brake == 0.0
    throttle, brake = lm.inverse(10.0, -3.3)  # -0.3 - 6 * 0.5 = -3.3
    assert throttle == 0.0
    assert brake == pytest.approx(0.5)
    assert lm.inverse(0.0, 100.0) == (1.0, 0.0)
    assert lm.inverse(0.0, -100.0) == (0.0, 1.0)
    # Exactly the coast acceleration needs no pedal.
    assert lm.inverse(10.0, -0.3) == (0.0, 0.0)


def test_inverse_is_consistent_with_forward():
    lm = make_map()
    rng = np.random.default_rng(1)
    for _ in range(200):
        v = float(rng.uniform(0, 20))
        a = float(rng.uniform(-5, 3))
        throttle, brake = lm.inverse(v, a)
        if throttle > 0.0 and throttle < 1.0:
            assert lm.accel(v, throttle) == pytest.approx(a, abs=1e-9)
        if brake > 0.0 and brake < 1.0:
            assert lm.decel(v, brake) == pytest.approx(a, abs=1e-9)


def test_vehicle_yaml_loads(repo_root_ml):
    lm = LongitudinalMap.from_yaml(
        repo_root_ml / "configs" / "vehicle" / "lincoln_mkz_2020.yaml"
    )
    throttle, brake = lm.inverse(10.0, 1.0)
    assert 0.0 < throttle < 1.0
    assert brake == 0.0


def test_validate_rejects_bad_shapes():
    lm = make_map()
    with pytest.raises(ValueError, match="accel_table"):
        LongitudinalMap(
            lm.v_bins,
            lm.throttle_bins,
            lm.accel_table[:, :2],
            lm.brake_bins,
            lm.decel_table,
            lm.coast_accel,
        ).validate()
