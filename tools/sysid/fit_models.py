"""Fit the vehicle model from the sweep logs (M0 §2.6).

Reads ``data/sysid/{throttle,brake,coast,steer}.csv`` written by ``run_sweeps.py``
and writes ``configs/vehicle/lincoln_mkz_2020.yaml`` (geometry fields of the
existing file are kept) plus residual plots under ``data/sysid/``:

* ``a = f(v, throttle)`` and ``a = g(v, brake)`` as 2-D lookup tables (v bins of
  1 m/s, pedal bins of 0.1) with linear interpolation, ``a_coast(v)`` as the binned
  coast curve; bins no run reached are extrapolated from the nearest reached bin
  along the drag curve. The inverse ``u = h(v, a_des)`` is ``LongitudinalMap.inverse``
  (monotone row inversion), so the tables are made monotone in the pedal;
* ``tau_throttle`` from the standstill step responses (time to 63 % of the plateau);
* wheelbase ``L`` and understeer gradient ``K`` jointly from
  ``yaw_rate (L + K v^2) = v tan(delta)`` over the steer runs in the linear tyre
  regime (steady lateral acceleration below LINEAR_A_LAT_MPS2 and a yaw rate of the
  expected sign; the larger steps of the M0 grid saturate the tyres above 5 m/s),
  ``tau_steer`` from the front-wheel-angle step response.

Acceleration is the central difference of the body-frame ``vx`` over one tick,
smoothed with a 5-tick moving average; the first second after a pedal step is
skipped for the tables (actuator lag).
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

from nuway_ml.common.longitudinal_map import LongitudinalMap
from nuway_ml.common.tick import TICK_DT_S

Array = NDArray[np.float64]

MODES = ("throttle", "brake", "coast", "steer")
V_BINS = np.arange(0.0, 31.0, 1.0)
PEDAL_BINS = np.round(np.arange(0.0, 1.01, 0.1), 1)
STEP_SKIP_S = 1.0  # samples this soon after a pedal step are not steady state
# The brake hold starts the moment the reach phase's full throttle is released,
# and the throttle lag is ~0.1-0.2 s, so 0.5 s is enough; 1 s would discard the
# whole 5 m/s tier (its runs last 0.5 s).
BRAKE_STEP_SKIP_S = 0.5
# CARLA (PhysX) snaps a braking car to rest below ~5.5 m/s: the last ticks of
# every brake run lose ~1.3 m/s per tick (about -26 m/s^2) whatever the pedal.
# That is not brake authority; samples from the first such tick on are cut.
STOP_CLIFF_MPS2 = 12.0
STEADY_WINDOW_S = 1.5  # steer: steady state is the mean over the last window
SMOOTH_TICKS = 5
LINEAR_A_LAT_MPS2 = 3.0  # steer runs above this steady lateral accel are tyre-saturated


@dataclass(frozen=True, slots=True)
class Run:
    """The hold-phase samples of one sweep run."""

    mode: str
    run: int
    v0: float
    level: float
    t: Array  # s, from the start of the hold
    v: Array  # m/s (body vx)
    a: Array  # m/s^2 (smoothed central difference)
    yaw_rate: Array  # rad/s
    wheel_angle: Array  # rad, front wheels (from /nuway/sim/vehicle_state)


@dataclass(slots=True)
class LongitudinalFit:
    """The longitudinal tables and their residuals."""

    v_bins: Array
    throttle_bins: Array
    accel_table: Array
    brake_bins: Array
    decel_table: Array
    coast_accel: Array
    tau_throttle: float
    residual_rms: dict[str, float] = field(default_factory=dict)
    samples: dict[str, int] = field(default_factory=dict)


@dataclass(slots=True)
class LateralFit:
    """Wheelbase, steering lag and understeer gradient."""

    wheelbase: float
    tau_steer: float
    understeer_gradient: float
    residual_rms_yaw_rate: float
    samples: int
    wheelbase_per_speed: dict[float, float] = field(default_factory=dict)


# ----------------------------------------------------------------- loading
def smooth(x: Array, window: int = SMOOTH_TICKS) -> Array:
    """Centered moving average with shrinking windows at the ends."""
    n = len(x)
    out = np.empty_like(x)
    half = window // 2
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out[i] = float(np.mean(x[lo:hi]))
    return out


def acceleration(v: Array, dt_s: float = TICK_DT_S) -> Array:
    """Central-difference acceleration, one-sided at the ends, smoothed."""
    if len(v) < 3:
        return np.zeros_like(v)
    a = np.gradient(v, dt_s)
    return smooth(a)


def runs_from_rows(rows: list[dict[str, str]]) -> list[Run]:
    """Group CSV rows into runs (hold phase only)."""
    grouped: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if row["phase"] == "hold":
            grouped[(row["mode"], int(row["run"]))].append(row)
    runs: list[Run] = []
    for (mode, run_id), group in sorted(grouped.items()):
        group.sort(key=lambda r: int(r["k"]))
        t = np.array([float(r["t"]) for r in group])
        v = np.array([float(r["vx"]) for r in group])
        # The wheel angle of the row's own tick. Sweeps recorded before the
        # tick join carried the previous tick's value (wheel_tick = k - 1);
        # the recorded wheel_tick puts it back on the right row.
        wheel_by_tick = {
            int(r.get("wheel_tick", r["k"])): float(r["wheel_angle"]) for r in group
        }
        wheel = np.array(
            [wheel_by_tick.get(int(r["k"]), float(r["wheel_angle"])) for r in group]
        )
        runs.append(
            Run(
                mode=mode,
                run=run_id,
                v0=float(group[0]["v0"]),
                level=float(group[0]["level"]),
                t=t - t[0],
                v=v,
                a=acceleration(v),
                yaw_rate=np.array([float(r["yaw_rate"]) for r in group]),
                wheel_angle=wheel,
            )
        )
    return runs


def load_runs(sysid_dir: Path) -> dict[str, list[Run]]:
    """Load every mode CSV present in the directory."""
    out: dict[str, list[Run]] = {}
    for mode in MODES:
        path = sysid_dir / f"{mode}.csv"
        if not path.is_file():
            continue
        with path.open() as f:
            rows = list(csv.DictReader(f))
        out[mode] = runs_from_rows(rows)
    return out


# ------------------------------------------------------------ longitudinal
def brake_steady_mask(r: Run) -> Array:
    """Select the brake samples that measure the pedal: past the step, moving, before the stop cliff."""
    mask = (r.t >= BRAKE_STEP_SKIP_S) & (r.v > 0.3)
    if len(r.v) >= 2:
        raw_decel = np.diff(r.v) / TICK_DT_S
        cliff = np.where(raw_decel < -STOP_CLIFF_MPS2)[0]
        if len(cliff):
            # The smoothing window spreads the cliff over its half width.
            mask[max(0, int(cliff[0]) - SMOOTH_TICKS // 2) :] = False
    return np.asarray(mask, dtype=bool)


def fill_nearest(column: Array) -> Array:
    """Fill NaN bins with the nearest measured bin (flat extrapolation)."""
    out = column.copy()
    measured = np.where(~np.isnan(column))[0]
    if len(measured) == 0:
        return np.zeros_like(column)
    for i in range(len(column)):
        if np.isnan(out[i]):
            out[i] = column[measured[np.argmin(np.abs(measured - i))]]
    return out


def fit_coast(runs: list[Run], v_bins: Array = V_BINS) -> Array:
    """Binned mean coast acceleration per v bin (<= 0), nearest-filled.

    A polynomial extrapolates badly: CARLA's engine braking is strongest at
    low speed and steps with the automatic gearbox, so the curve is kept as
    the table the message carries.
    """
    v = (
        np.concatenate([r.v[r.t >= STEP_SKIP_S] for r in runs])
        if runs
        else np.array([])
    )
    a = (
        np.concatenate([r.a[r.t >= STEP_SKIP_S] for r in runs])
        if runs
        else np.array([])
    )
    keep = v > 0.3  # the stopped tail is not drag
    means, _ = bin_means(v[keep], a[keep], v_bins)
    if np.all(np.isnan(means)):
        return -(0.1 + 0.005 * v_bins + 0.0006 * v_bins * v_bins)  # the M0 placeholder
    return np.minimum(0.0, fill_nearest(means))


def coast_at(coast: Array, v: Array | float, v_bins: Array = V_BINS) -> Array:
    """Interpolate the binned coast curve (clamped)."""
    vv = np.clip(np.asarray(v, dtype=np.float64), v_bins[0], v_bins[-1])
    return np.asarray(np.interp(vv, v_bins, coast), dtype=np.float64)


def bin_means(v: Array, a: Array, v_bins: Array) -> tuple[Array, Array]:
    """Mean ``a`` per 1 m/s bin centred on ``v_bins``; NaN where empty."""
    means = np.full(len(v_bins), np.nan)
    counts = np.zeros(len(v_bins))
    if len(v) == 0:
        return means, counts
    # A sample past the last bin is dropped, not folded into it (throttle runs
    # from 20 m/s exceed 30 m/s within the hold).
    idx = np.round(v).astype(int)
    for i in range(len(v_bins)):
        sel = idx == i
        counts[i] = float(np.sum(sel))
        if counts[i] > 0:
            means[i] = float(np.mean(a[sel]))
    return means, counts


def fill_along_drag(column: Array, v_bins: Array, coast: Array) -> Array:
    """Fill NaN bins from the nearest measured bin, shifted by the drag difference."""
    out = column.copy()
    measured = np.where(~np.isnan(column))[0]
    if len(measured) == 0:
        return coast.copy()
    drag = coast
    for i in range(len(v_bins)):
        if np.isnan(out[i]):
            j = measured[np.argmin(np.abs(measured - i))]
            out[i] = column[j] + (drag[i] - drag[j])
    return out


def fit_pedal_table(
    runs: list[Run], pedal_bins: Array, v_bins: Array, coast: Array, *, brake: bool
) -> tuple[Array, int]:
    """``a[v][pedal]`` with column 0 = coast; monotone in the pedal."""
    table = np.zeros((len(v_bins), len(pedal_bins)))
    table[:, 0] = coast
    total = 0
    for j, level in enumerate(pedal_bins[1:], start=1):
        sel = [r for r in runs if abs(r.level - level) < 1e-6]
        masks = [brake_steady_mask(r) if brake else r.t >= STEP_SKIP_S for r in sel]
        v = (
            np.concatenate([r.v[m] for r, m in zip(sel, masks, strict=True)])
            if sel
            else np.array([])
        )
        a = (
            np.concatenate([r.a[m] for r, m in zip(sel, masks, strict=True)])
            if sel
            else np.array([])
        )
        total += len(v)
        means, _ = bin_means(v, a, v_bins)
        table[:, j] = fill_along_drag(means, v_bins, coast)
    # Monotone in the pedal so the row inversion is well defined.
    for i in range(len(v_bins)):
        row = table[i]
        table[i] = np.minimum.accumulate(row) if brake else np.maximum.accumulate(row)
    return table, total


def fit_tau_throttle(runs: list[Run]) -> float:
    """Time to 63 % of the plateau accel on standstill steps with throttle >= 0.5."""
    taus: list[float] = []
    for r in runs:
        if r.v0 > 0.0 or r.level < 0.5 or len(r.t) < 60:
            continue
        plateau_sel = (r.t >= 1.0) & (r.t <= 2.0)
        plateau = float(np.mean(r.a[plateau_sel]))
        if plateau <= 0.1:
            continue
        above = np.where(r.a >= 0.632 * plateau)[0]
        if len(above):
            taus.append(float(r.t[above[0]]))
    return float(np.median(taus)) if taus else 0.2


def residual_rms(lon: LongitudinalMap, runs: list[Run], mode: str) -> float:
    """RMS of ``a_sample - table(v, pedal)`` over the steady-state samples."""
    errs: list[float] = []
    for r in runs:
        sel = brake_steady_mask(r) if mode == "brake" else r.t >= STEP_SKIP_S
        for v, a in zip(r.v[sel], r.a[sel], strict=True):
            if mode == "throttle":
                pred = lon.accel(float(v), r.level)
            elif mode == "brake":
                pred = lon.decel(float(v), r.level)
            else:
                pred = lon.coast(float(v))
            errs.append(float(a) - pred)
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else math.nan


def fit_longitudinal(runs: dict[str, list[Run]]) -> LongitudinalFit:
    """Fit the tables, drag and throttle lag."""
    coast = fit_coast(runs.get("coast", []))
    accel_table, n_throttle = fit_pedal_table(
        runs.get("throttle", []), PEDAL_BINS, V_BINS, coast, brake=False
    )
    decel_table, n_brake = fit_pedal_table(
        runs.get("brake", []), PEDAL_BINS, V_BINS, coast, brake=True
    )
    fit = LongitudinalFit(
        v_bins=V_BINS.copy(),
        throttle_bins=PEDAL_BINS.copy(),
        accel_table=accel_table,
        brake_bins=PEDAL_BINS.copy(),
        decel_table=decel_table,
        coast_accel=coast,
        tau_throttle=fit_tau_throttle(runs.get("throttle", [])),
    )
    lon = LongitudinalMap.from_dict(longitudinal_dict(fit))
    lon.validate()
    for mode in ("throttle", "brake", "coast"):
        fit.residual_rms[mode] = residual_rms(lon, runs.get(mode, []), mode)
    fit.samples = {
        "throttle": n_throttle,
        "brake": n_brake,
        "coast": int(
            sum(int(np.sum(r.t >= STEP_SKIP_S)) for r in runs.get("coast", []))
        ),
    }
    return fit


def longitudinal_dict(fit: LongitudinalFit) -> dict[str, Any]:
    """Return the ``longitudinal_map`` YAML block."""
    return {
        "v_bins": [float(v) for v in fit.v_bins],
        "throttle_bins": [float(u) for u in fit.throttle_bins],
        "accel_table": [[round(float(a), 4) for a in row] for row in fit.accel_table],
        "brake_bins": [float(b) for b in fit.brake_bins],
        "decel_table": [[round(float(a), 4) for a in row] for row in fit.decel_table],
        "coast_accel": [round(float(a), 4) for a in fit.coast_accel],
    }


# ----------------------------------------------------------------- lateral
def steady_state(r: Run) -> tuple[float, float, float]:
    """Mean (v, yaw_rate, wheel_angle) over the last STEADY_WINDOW_S of a steer run (signed)."""
    sel = r.t >= r.t[-1] - STEADY_WINDOW_S
    return (
        float(np.mean(r.v[sel])),
        float(np.mean(r.yaw_rate[sel])),
        float(np.mean(r.wheel_angle[sel])),
    )


def linear_regime(r: Run) -> bool:
    """Return whether the run's steady state is one the kinematic model can describe."""
    v, yaw_rate, delta = steady_state(r)
    if abs(delta) < 1e-4 or abs(yaw_rate) < 1e-3:
        return False
    # ROS convention: a positive (left) wheel angle turns counter-clockwise.
    return (
        math.copysign(1.0, yaw_rate) == math.copysign(1.0, delta)
        and abs(v * yaw_rate) <= LINEAR_A_LAT_MPS2
    )


def fit_lateral(runs: list[Run], geometric_wheelbase: float) -> LateralFit:
    """Wheelbase and understeer gradient jointly from the linear-regime steady states."""
    if not runs:
        return LateralFit(geometric_wheelbase, 0.12, 0.0, math.nan, 0)
    per_speed: dict[float, list[float]] = defaultdict(list)
    points: list[tuple[float, float, float]] = []
    for r in runs:
        if not linear_regime(r):
            continue
        v, yaw_rate, delta = steady_state(r)
        yaw_rate, delta = abs(yaw_rate), abs(delta)
        per_speed[r.v0].append(v * math.tan(delta) / yaw_rate)
        points.append((v, yaw_rate, delta))
    # yaw_rate (L + K v^2) = v tan(delta): linear least squares in (L, K) over
    # every linear-regime run; the per-speed medians of v tan(delta) / yaw_rate
    # are reported alongside.
    if len(points) >= 2 and len(per_speed) >= 2:
        design = np.array([[yr, yr * v * v] for v, yr, _ in points])
        target = np.array([v * math.tan(d) for v, _, d in points])
        solution, _, _, _ = np.linalg.lstsq(design, target, rcond=None)
        wheelbase, k_us = float(solution[0]), float(solution[1])
    elif points:
        wheelbase = float(np.median([v * math.tan(d) / yr for v, yr, d in points]))
        k_us = 0.0
    else:
        wheelbase, k_us = geometric_wheelbase, 0.0
    # Steering lag: time to 63 % of the steady wheel angle (every run).
    taus: list[float] = []
    for r in runs:
        _, _, delta = steady_state(r)
        if abs(delta) < 1e-4:
            continue
        above = np.where(np.abs(r.wheel_angle) >= 0.632 * abs(delta))[0]
        if len(above):
            taus.append(float(r.t[above[0]]))
    tau_steer = float(np.median(taus)) if taus else 0.12
    errs = [yr - v * math.tan(d) / (wheelbase + k_us * v * v) for v, yr, d in points]
    return LateralFit(
        wheelbase=wheelbase,
        tau_steer=tau_steer,
        understeer_gradient=k_us,
        residual_rms_yaw_rate=float(np.sqrt(np.mean(np.square(errs))))
        if errs
        else math.nan,
        samples=len(points),
        wheelbase_per_speed={
            v0: float(np.median(ls)) for v0, ls in sorted(per_speed.items())
        },
    )


# -------------------------------------------------------------------- YAML
def build_vehicle_yaml(
    existing: dict[str, Any], lon: LongitudinalFit, lat: LateralFit, source: str
) -> dict[str, Any]:
    """Merge the fits into the existing vehicle YAML (geometry is kept)."""
    out = dict(existing)
    out["tau_steer"] = round(lat.tau_steer, 3)
    out["tau_throttle"] = round(lon.tau_throttle, 3)
    out["wheelbase_fitted"] = round(lat.wheelbase, 3)
    out["understeer_gradient"] = round(lat.understeer_gradient, 5)
    out["sysid"] = {
        "status": "fitted",
        "date": dt.date.today().isoformat(),
        "source": source,
        "residual_rms": {
            "throttle_accel_mps2": round(lon.residual_rms.get("throttle", math.nan), 4),
            "brake_accel_mps2": round(lon.residual_rms.get("brake", math.nan), 4),
            "coast_accel_mps2": round(lon.residual_rms.get("coast", math.nan), 4),
            "steer_yaw_rate_radps": round(lat.residual_rms_yaw_rate, 4),
        },
        "samples": {**lon.samples, "steer_runs_linear": lat.samples},
        "wheelbase_per_speed": {
            f"{v0:.0f}": round(l_fit, 3)
            for v0, l_fit in lat.wheelbase_per_speed.items()
        },
    }
    out["longitudinal_map"] = longitudinal_dict(lon)
    return out


VEHICLE_HEADER = """# configs/vehicle/lincoln_mkz_2020.yaml — ego vehicle model (docs/M0_bringup.md §2.6).
# Geometry (ROS convention, meters): measured from the CARLA actor on the dev box, 2026-09-07:
#   bounding_box.extent = (2.446, 0.918, 0.745), bbox center 0.749 above the actor origin,
#   rear axle 1.389 behind the actor origin, front axle 1.472 ahead (wheelbase 2.860),
#   wheel center 0.264 above the origin with radius 0.355 -> the origin sits 0.091 above ground,
#   wheels[0].max_steer_angle = 70 deg.
# Dynamics (tau_*, wheelbase_fitted, understeer_gradient, longitudinal_map): fitted by
#   tools/sysid/fit_models.py from tools/sysid/run_sweeps.py logs (see the sysid block).
"""


def write_vehicle_yaml(path: Path, data: dict[str, Any]) -> None:
    """Write the YAML with the fixed header comment."""
    body = yaml.safe_dump(data, sort_keys=False, width=120, default_flow_style=None)
    path.write_text(VEHICLE_HEADER + body)


# ------------------------------------------------------------------- plots
def _plot_pedal(runs: list[Run], lon: LongitudinalFit, mode: str, path: Path) -> None:
    import matplotlib.pyplot as plt  # noqa: PLC0415  # viz group; imported only when plotting

    lon_map = LongitudinalMap.from_dict(longitudinal_dict(lon))
    table = lon.accel_table if mode == "throttle" else lon.decel_table
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    cmap = plt.get_cmap("viridis")
    for r in runs:
        sel = r.t >= STEP_SKIP_S
        color = cmap(r.level)
        axes[0].scatter(r.v[sel], r.a[sel], s=2, color=color, alpha=0.4)
        pred = np.array(
            [
                lon_map.accel(float(v), r.level)
                if mode == "throttle"
                else lon_map.decel(float(v), r.level)
                for v in r.v[sel]
            ]
        )
        axes[1].scatter(r.v[sel], r.a[sel] - pred, s=2, color=color, alpha=0.4)
    for j, level in enumerate(PEDAL_BINS):
        axes[0].plot(V_BINS, table[:, j], color=cmap(level), lw=1)
    axes[0].set_title(f"{mode}: a vs v (lines: fitted table, colour: {mode} level)")
    axes[0].set_xlabel("v [m/s]")
    axes[0].set_ylabel("a [m/s^2]")
    axes[1].set_title(
        f"{mode}: residual (RMS {lon.residual_rms.get(mode, math.nan):.3f})"
    )
    axes[1].set_xlabel("v [m/s]")
    axes[1].set_ylabel("a - table [m/s^2]")
    axes[1].axhline(0.0, color="k", lw=0.5)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def _plot_coast(runs: list[Run], lon: LongitudinalFit, path: Path) -> None:
    import matplotlib.pyplot as plt  # noqa: PLC0415  # viz group; imported only when plotting

    fig, ax = plt.subplots(figsize=(7, 5))
    for r in runs:
        sel = r.t >= STEP_SKIP_S
        ax.scatter(
            r.v[sel], r.a[sel], s=2, alpha=0.4, label=f"coast from {r.v0:.0f} m/s"
        )
    ax.plot(V_BINS, lon.coast_accel, "k-", label="fit")
    ax.set_title(f"coast drag (RMS {lon.residual_rms.get('coast', math.nan):.3f})")
    ax.set_xlabel("v [m/s]")
    ax.set_ylabel("a [m/s^2]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def _plot_steer(runs: list[Run], lat: LateralFit, path: Path) -> None:
    import matplotlib.pyplot as plt  # noqa: PLC0415  # viz group; imported only when plotting

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    x_max = 0.1
    for r in runs:
        v, yr, d = steady_state(r)
        x_max = max(x_max, v * math.tan(abs(d)))
        color = "tab:blue" if linear_regime(r) else "tab:red"
        axes[0].scatter(v * math.tan(abs(d)), abs(yr), s=12, color=color)
        axes[1].plot(r.t, np.abs(r.wheel_angle), lw=0.8, alpha=0.6)
    x = np.linspace(0.0, x_max, 50)
    axes[0].plot(x, x / lat.wheelbase, "k-", label=f"L = {lat.wheelbase:.3f} m")
    axes[0].set_xlabel("v tan(delta) [m/s]")
    axes[0].set_ylabel("|yaw rate| [rad/s]")
    axes[0].set_title(
        f"steer steady state: blue = linear regime (RMS {lat.residual_rms_yaw_rate:.4f} rad/s), red = saturated"
    )
    axes[0].legend()
    axes[1].set_xlabel("t after step [s]")
    axes[1].set_ylabel("|front wheel angle| [rad]")
    axes[1].set_title(f"steer step responses (tau_steer {lat.tau_steer:.3f} s)")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_all(
    runs: dict[str, list[Run]], lon: LongitudinalFit, lat: LateralFit, out_dir: Path
) -> list[Path]:
    """Residual plots (matplotlib, Agg) under out_dir."""
    import matplotlib  # noqa: PLC0415  # viz group; imported only when plotting

    matplotlib.use("Agg")
    written: list[Path] = []
    for mode in ("throttle", "brake"):
        path = out_dir / f"fit_{mode}.png"
        _plot_pedal(runs.get(mode, []), lon, mode, path)
        written.append(path)
    path = out_dir / "fit_coast.png"
    _plot_coast(runs.get("coast", []), lon, path)
    written.append(path)
    if runs.get("steer"):
        path = out_dir / "fit_steer.png"
        _plot_steer(runs["steer"], lat, path)
        written.append(path)
    return written


# --------------------------------------------------------------------- CLI
def report(lon: LongitudinalFit, lat: LateralFit, geometric_wheelbase: float) -> None:
    """Print the fit summary (the numbers the decisions log records)."""
    print(
        f"coast a(v) = {np.round(lon.coast_accel[::5], 2).tolist()} at v = {lon.v_bins[::5].tolist()}; "
        f"tau_throttle {lon.tau_throttle:.3f} s; residual RMS {lon.residual_rms}; samples {lon.samples}"
    )
    print(
        f"wheelbase fitted {lat.wheelbase:.3f} m (geometric {geometric_wheelbase}) per speed "
        f"{lat.wheelbase_per_speed}; tau_steer {lat.tau_steer:.3f} s; understeer {lat.understeer_gradient:.5f}; "
        f"yaw-rate RMS {lat.residual_rms_yaw_rate:.4f} rad/s over {lat.samples} runs"
    )


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--sysid-dir", type=Path, default=Path("data/sysid"))
    parser.add_argument(
        "--vehicle", type=Path, default=Path("configs/vehicle/lincoln_mkz_2020.yaml")
    )
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="fit with some mode CSVs missing (their tables become placeholders)",
    )
    args = parser.parse_args(argv)

    runs = load_runs(args.sysid_dir)
    missing = [m for m in MODES if m not in runs]
    if missing and not (args.allow_partial and len(missing) < len(MODES)):
        # A partial directory would otherwise silently overwrite the fitted
        # vehicle YAML with placeholder tables (no throttle or brake authority).
        print(
            f"missing sweep CSVs in {args.sysid_dir}: {missing} (--allow-partial to fit anyway)"
        )
        return 1
    for mode, mode_runs in runs.items():
        print(
            f"{mode}: {len(mode_runs)} runs, {sum(len(r.t) for r in mode_runs)} hold ticks"
        )
    existing = yaml.safe_load(args.vehicle.read_text())
    lon = fit_longitudinal(runs)
    lat = fit_lateral(runs.get("steer", []), float(existing["wheelbase"]))
    report(lon, lat, float(existing["wheelbase"]))
    data = build_vehicle_yaml(existing, lon, lat, source=str(args.sysid_dir))
    write_vehicle_yaml(args.vehicle, data)
    print(f"wrote {args.vehicle}")
    LongitudinalMap.from_yaml(args.vehicle).validate()
    if not args.no_plots:
        for path in plot_all(runs, lon, lat, args.sysid_dir):
            print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
