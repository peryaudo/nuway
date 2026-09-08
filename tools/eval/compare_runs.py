"""Compare two run directories of run_routes.py (M0: the lockstep criterion).

    uv run tools/eval/compare_runs.py data/eval_runs/<a> data/eval_runs/<b>

Per route present in both runs, the per-tick ``ego_odom.csv`` must be
bit-identical (same tick set, same numbers as written) and neither run may
contain a TickTimeout; ``results.csv`` rows are compared field by field.
Exit code 0 when every shared route matches, 1 otherwise.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class RouteComparison:
    """Outcome for one route."""

    route_id: str
    identical: bool
    ticks_a: int
    ticks_b: int
    first_diff_k: int | None
    timeouts_a: int
    timeouts_b: int
    detail: str


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _results_by_id(run_dir: Path) -> dict[str, dict[str, str]]:
    path = run_dir / "results.csv"
    if not path.exists():
        return {}
    return {row["route_id"]: row for row in _read_rows(path)}


def compare_route(run_a: Path, run_b: Path, route_id: str) -> RouteComparison:
    """Compare one route's odometry traces and results rows."""
    rows_a = _read_rows(run_a / "routes" / route_id / "ego_odom.csv")
    rows_b = _read_rows(run_b / "routes" / route_id / "ego_odom.csv")
    res_a = _results_by_id(run_a).get(route_id, {})
    res_b = _results_by_id(run_b).get(route_id, {})
    timeouts_a = int(res_a.get("timeouts", 0) or 0)
    timeouts_b = int(res_b.get("timeouts", 0) or 0)
    first_diff: int | None = None
    detail = ""
    for ra, rb in zip(rows_a, rows_b, strict=False):
        if ra != rb:
            first_diff = int(ra["k"])
            changed = [c for c in ra if ra[c] != rb.get(c)]
            detail = f"first difference at tick {first_diff}: {changed}"
            break
    if first_diff is None and len(rows_a) != len(rows_b):
        detail = f"tick count differs: {len(rows_a)} vs {len(rows_b)}"
    identical = (
        first_diff is None
        and len(rows_a) == len(rows_b)
        and len(rows_a) > 0
        and timeouts_a == 0
        and timeouts_b == 0
    )
    if identical:
        differing = [
            c for c in res_a if c not in ("wall_s",) and res_a.get(c) != res_b.get(c)
        ]
        if differing:
            identical = False
            detail = f"results.csv differs in {differing}"
    if not detail and (timeouts_a or timeouts_b):
        detail = f"timeouts {timeouts_a} vs {timeouts_b}"
    return RouteComparison(
        route_id,
        identical,
        len(rows_a),
        len(rows_b),
        first_diff,
        timeouts_a,
        timeouts_b,
        detail,
    )


def compare_runs(run_a: Path, run_b: Path) -> list[RouteComparison]:
    """Every route present in both runs, in run A's order."""
    ids_a = [p.name for p in sorted((run_a / "routes").iterdir()) if p.is_dir()]
    shared = [r for r in ids_a if (run_b / "routes" / r / "ego_odom.csv").exists()]
    return [compare_route(run_a, run_b, r) for r in shared]


def main(argv: list[str] | None = None) -> int:
    """CLI."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("run_a", type=Path)
    p.add_argument("run_b", type=Path)
    args = p.parse_args(argv)
    comparisons = compare_runs(args.run_a, args.run_b)
    if not comparisons:
        print("no shared routes")
        return 1
    ok = True
    for c in comparisons:
        status = "identical" if c.identical else "DIFFERENT"
        print(
            f"{c.route_id}: {status} ({c.ticks_a} / {c.ticks_b} ticks, "
            f"timeouts {c.timeouts_a} / {c.timeouts_b}) {c.detail}"
        )
        ok = ok and c.identical
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
