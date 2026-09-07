"""Longitudinal actuator map: (speed, accel) <-> throttle / brake (M0).

Mirrors ``nuway_control/longitudinal_map.hpp`` (parity-tested through
``nuway_py``). The tables come from the sysid fit in
``configs/vehicle/<vehicle>.yaml`` (``M0_bringup.md`` §2.6)::

    longitudinal_map:
      v_bins: [...]            # m/s, increasing
      throttle_bins: [...]     # 0..1, increasing
      accel_table: [[...]]     # a[v][throttle], m/s^2
      brake_bins: [...]        # 0..1, increasing
      decel_table: [[...]]     # a[v][brake], m/s^2 (<= 0)
      coast_accel: [...]       # a[v] with no pedal

``control_adapter`` and the Leaderboard agent call :meth:`LongitudinalMap.inverse`.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

Array = NDArray[np.float64]


def _interp_row(v_bins: Array, table: Array, v: float) -> Array:
    """Row of ``table`` at speed v, linearly interpolated between v bins (clamped)."""
    v_clamped = float(np.clip(v, v_bins[0], v_bins[-1]))
    idx = int(np.searchsorted(v_bins, v_clamped, side="right")) - 1
    idx = max(0, min(idx, len(v_bins) - 2))
    span = v_bins[idx + 1] - v_bins[idx]
    alpha = (v_clamped - v_bins[idx]) / span if span > 0.0 else 0.0
    return np.asarray(
        table[idx] + alpha * (table[idx + 1] - table[idx]), dtype=np.float64
    )


def _interp_1d(bins: Array, values: Array, x: float) -> float:
    """Linear interpolation of ``values`` over ``bins`` at x (clamped)."""
    return float(np.interp(float(np.clip(x, bins[0], bins[-1])), bins, values))


def _invert_monotone(bins: Array, values: Array, target: float) -> float:
    """Return the bin at which the piecewise-linear ``values`` reaches ``target``.

    ``values`` must be monotone (either direction). Saturates at the ends.
    """
    increasing = values[-1] >= values[0]
    lo, hi = (values[0], values[-1]) if increasing else (values[-1], values[0])
    if target <= lo:
        return float(bins[0] if increasing else bins[-1])
    if target >= hi:
        return float(bins[-1] if increasing else bins[0])
    for i in range(len(bins) - 1):
        a, b = values[i], values[i + 1]
        if (a <= target <= b) or (b <= target <= a):
            if b == a:
                return float(bins[i])
            return float(bins[i] + (target - a) / (b - a) * (bins[i + 1] - bins[i]))
    return float(bins[-1])


@dataclass(frozen=True, slots=True)
class LongitudinalMap:
    """Forward tables and their inverse."""

    v_bins: Array
    throttle_bins: Array
    accel_table: Array  # [v][throttle]
    brake_bins: Array
    decel_table: Array  # [v][brake]
    coast_accel: Array  # [v]

    @classmethod
    def from_dict(cls, cfg: dict[str, Any]) -> LongitudinalMap:
        """Build from the ``longitudinal_map`` block of a vehicle YAML."""
        lm = cls(
            v_bins=np.asarray(cfg["v_bins"], dtype=np.float64),
            throttle_bins=np.asarray(cfg["throttle_bins"], dtype=np.float64),
            accel_table=np.asarray(cfg["accel_table"], dtype=np.float64),
            brake_bins=np.asarray(cfg["brake_bins"], dtype=np.float64),
            decel_table=np.asarray(cfg["decel_table"], dtype=np.float64),
            coast_accel=np.asarray(cfg["coast_accel"], dtype=np.float64),
        )
        lm.validate()
        return lm

    @classmethod
    def from_yaml(cls, path: Path) -> LongitudinalMap:
        """Build from ``configs/vehicle/<vehicle>.yaml``."""
        with Path(path).open() as f:
            cfg = yaml.safe_load(f)
        return cls.from_dict(cfg["longitudinal_map"])

    def validate(self) -> None:
        """Raise ValueError if the table shapes or bin orderings are inconsistent."""
        nv = len(self.v_bins)
        if self.accel_table.shape != (nv, len(self.throttle_bins)):
            msg = f"accel_table shape {self.accel_table.shape} != ({nv}, {len(self.throttle_bins)})"
            raise ValueError(msg)
        if self.decel_table.shape != (nv, len(self.brake_bins)):
            msg = f"decel_table shape {self.decel_table.shape} != ({nv}, {len(self.brake_bins)})"
            raise ValueError(msg)
        if self.coast_accel.shape != (nv,):
            msg = f"coast_accel shape {self.coast_accel.shape} != ({nv},)"
            raise ValueError(msg)
        for name, bins in (
            ("v_bins", self.v_bins),
            ("throttle_bins", self.throttle_bins),
            ("brake_bins", self.brake_bins),
        ):
            if len(bins) < 2 or np.any(np.diff(bins) <= 0):
                msg = f"{name} must be increasing with at least two entries"
                raise ValueError(msg)

    def accel(self, v: float, throttle: float) -> float:
        """Return the acceleration (m/s^2) at speed v with ``throttle`` applied."""
        row = _interp_row(self.v_bins, self.accel_table, v)
        return _interp_1d(self.throttle_bins, row, throttle)

    def decel(self, v: float, brake: float) -> float:
        """Return the acceleration (m/s^2, <= 0) at speed v with ``brake`` applied."""
        row = _interp_row(self.v_bins, self.decel_table, v)
        return _interp_1d(self.brake_bins, row, brake)

    def coast(self, v: float) -> float:
        """Acceleration with no pedal at speed v (drag, m/s^2)."""
        return _interp_1d(self.v_bins, self.coast_accel, v)

    def inverse(self, v: float, accel_des: float) -> tuple[float, float]:
        """Pedal command ``(throttle, brake)`` in [0, 1] that yields ``accel_des`` at v.

        Throttle is used when ``accel_des`` is at or above the coast
        acceleration, brake below it; each is found by inverting the
        interpolated monotone table row and saturates at 0 / 1.
        """
        if accel_des >= self.coast(v):
            row = _interp_row(self.v_bins, self.accel_table, v)
            return _invert_monotone(self.throttle_bins, row, accel_des), 0.0
        row = _interp_row(self.v_bins, self.decel_table, v)
        return 0.0, _invert_monotone(self.brake_bins, row, accel_des)
