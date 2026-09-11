"""Compare two run directories of run_routes.py (the M1 reproducibility criterion).

    uv run tools/eval/compare_runs.py data/eval_runs/<a> data/eval_runs/<b> [--accel-tol ...]

Per protocol key (route, town, weather, seed) present in both runs: the
infraction counts and flags of ``results.csv`` must agree exactly,
``driving_score`` / ``completion`` / ``sim_time_s`` within their
tolerances, neither row may carry a TickTimeout, and the per-tick
``control_command.csv`` sequences must agree within the command
tolerances (the CARLA spread measured by task 10); the first tick outside
them is reported. Timing columns (``*_ms``, ``wall_time_s``) are wall-clock
measurements: printed as deltas, never judged. Exit code 0 when every
shared run agrees, 1 otherwise.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from nuway_eval.report import RouteResult, read_results

EXACT_COLUMNS = (
    "n_collision_pedestrian",
    "n_collision_vehicle",
    "n_collision_layout",
    "n_red_light",
    "n_stop_sign",
    "n_min_speed",
    "route_deviation",
    "blocked",
    "timeout",
    "status",
)
DELTA_COLUMNS = (
    "min_speed_pct",
    "planner_p50_ms",
    "planner_p99_ms",
    "mpc_p50_ms",
    "mpc_p99_ms",
)


@dataclass(frozen=True, slots=True)
class Tolerances:
    """What still counts as the same run."""

    score: float = 1.0
    completion: float = 0.01
    sim_time_s: float = 1.0
    accel_mps2: float = 0.05
    steer_rad: float = 0.005


DEFAULT_TOLERANCES = Tolerances()


@dataclass(frozen=True, slots=True)
class RunComparison:
    """Outcome for one protocol key."""

    key: tuple[str, str, str, int]
    agree: bool
    ticks_a: int
    ticks_b: int
    first_diff_tick: int | None  # ticks since the reset, command outside tolerance
    score_delta: float
    completion_delta: float
    detail: str


def _read_commands(run_dir: Path, result: RouteResult) -> list[dict[str, str]]:
    path = run_dir / result.run_dir_name / "control_command.csv"
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def first_command_difference(
    cmds_a: list[dict[str, str]], cmds_b: list[dict[str, str]], tol: Tolerances
) -> int | None:
    """First index whose command differs beyond the tolerance (ticks since the reset)."""
    for i, (a, b) in enumerate(zip(cmds_a, cmds_b, strict=False)):
        if (
            abs(float(a["accel"]) - float(b["accel"])) > tol.accel_mps2
            or abs(float(a["steering_angle"]) - float(b["steering_angle"]))
            > tol.steer_rad
            or a["emergency_stop"] != b["emergency_stop"]
        ):
            return i
    return None


def compare_run(
    run_a: Path, run_b: Path, a: RouteResult, b: RouteResult, tol: Tolerances
) -> RunComparison:
    """Compare one protocol entry's rows and command sequences."""
    problems: list[str] = []
    for c in EXACT_COLUMNS:
        if getattr(a, c) != getattr(b, c):
            problems.append(f"{c} {getattr(a, c)} vs {getattr(b, c)}")
    if abs(a.driving_score - b.driving_score) > tol.score:
        problems.append(f"driving_score {a.driving_score:.2f} vs {b.driving_score:.2f}")
    if abs(a.completion - b.completion) > tol.completion:
        problems.append(f"completion {a.completion:.3f} vs {b.completion:.3f}")
    if abs(a.sim_time_s - b.sim_time_s) > tol.sim_time_s:
        problems.append(f"sim_time_s {a.sim_time_s:.1f} vs {b.sim_time_s:.1f}")
    if a.n_tick_timeouts or b.n_tick_timeouts:
        problems.append(f"tick timeouts {a.n_tick_timeouts} vs {b.n_tick_timeouts}")
    cmds_a, cmds_b = _read_commands(run_a, a), _read_commands(run_b, b)
    first = first_command_difference(cmds_a, cmds_b, tol)
    if first is not None:
        problems.append(f"commands differ from tick {first}")
    elif len(cmds_a) != len(cmds_b):
        problems.append(f"tick count {len(cmds_a)} vs {len(cmds_b)}")
    deltas = ", ".join(
        f"{c} {getattr(b, c) - getattr(a, c):+.2f}" for c in DELTA_COLUMNS
    )
    detail = "; ".join(problems) if problems else f"agree ({deltas})"
    return RunComparison(
        a.key,
        not problems,
        len(cmds_a),
        len(cmds_b),
        first,
        b.driving_score - a.driving_score,
        b.completion - a.completion,
        detail,
    )


def compare_runs(run_a: Path, run_b: Path, tol: Tolerances) -> list[RunComparison]:
    """Every protocol key present in both runs, in run A's order."""
    rows_b = {r.key: r for r in read_results(run_b / "results.csv")}
    out = []
    for a in read_results(run_a / "results.csv"):
        b = rows_b.get(a.key)
        if b is not None:
            out.append(compare_run(run_a, run_b, a, b, tol))
    return out


def summary(comparisons: list[RunComparison]) -> str:
    """Paired summary line."""
    n = len(comparisons)
    agree = sum(1 for c in comparisons if c.agree)
    score = [c.score_delta for c in comparisons]
    completion = [c.completion_delta for c in comparisons]
    return (
        f"{agree}/{n} runs agree; driving_score delta mean {statistics.fmean(score):+.2f} "
        f"(max |{max(abs(s) for s in score):.2f}|), completion delta mean "
        f"{statistics.fmean(completion):+.3f}"
    )


def main(argv: list[str] | None = None) -> int:
    """CLI."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("run_a", type=Path)
    p.add_argument("run_b", type=Path)
    p.add_argument("--score-tol", type=float, default=DEFAULT_TOLERANCES.score)
    p.add_argument(
        "--completion-tol", type=float, default=DEFAULT_TOLERANCES.completion
    )
    p.add_argument("--sim-time-tol", type=float, default=DEFAULT_TOLERANCES.sim_time_s)
    p.add_argument("--accel-tol", type=float, default=DEFAULT_TOLERANCES.accel_mps2)
    p.add_argument("--steer-tol", type=float, default=DEFAULT_TOLERANCES.steer_rad)
    args = p.parse_args(argv)
    tol = Tolerances(
        args.score_tol,
        args.completion_tol,
        args.sim_time_tol,
        args.accel_tol,
        args.steer_tol,
    )
    comparisons = compare_runs(args.run_a, args.run_b, tol)
    if not comparisons:
        print("no shared runs")
        return 1
    for c in comparisons:
        status = "agree" if c.agree else "DIFFERENT"
        key = "_".join(str(k) for k in (c.key[0], c.key[2], c.key[3]))
        print(f"{key}: {status} ({c.ticks_a} / {c.ticks_b} ticks) {c.detail}")
    print(summary(comparisons))
    return 0 if all(c.agree for c in comparisons) else 1


if __name__ == "__main__":
    sys.exit(main())
