"""Evaluation harness entry point, v0 (M0 §2.11).

    uv run tools/eval/run_routes.py --profile m0_gt_all tools/eval/routes/dev_*.xml

One stack per town (launched here unless --no-launch), one /nuway/sim/reset
per route, lateral-error statistics from /nuway/control/debug, results.csv +
report.md + per-route odometry/debug CSVs under data/eval_runs/<run_id>/.
M1 replaces the metrics with the real harness; the CLI stays.
"""

from __future__ import annotations

import argparse
import io
import sys
import time
from pathlib import Path

import rclpy

from nuway_eval.route_runner import (
    RouteResult,
    RouteRunnerNode,
    RunLimits,
    StackProcess,
    format_table,
    group_by_town,
    town_profile,
    write_results,
)
from nuway_ml.common.routes import RouteSpec, load_route_xml

REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """CLI."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "routes", nargs="+", type=Path, help="route XML files (Leaderboard format)"
    )
    p.add_argument("--profile", default="m0_gt_all", help="profile name or YAML path")
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="run directory (default data/eval_runs/<run_id>)",
    )
    p.add_argument("--run-id", default=time.strftime("%Y%m%d_%H%M%S"))
    p.add_argument(
        "--no-launch",
        action="store_true",
        help="a stack is already running (single town)",
    )
    p.add_argument("--wall-timeout-s", type=float, default=900.0)
    p.add_argument(
        "--min-avg-speed-mps",
        type=float,
        default=3.0,
        help="sim-time budget per route = length / this + 60 s",
    )
    p.add_argument("--goal-radius-m", type=float, default=5.0)
    p.add_argument("--route-ids", nargs="*", default=None, help="subset of route ids")
    p.add_argument(
        "--limit", type=int, default=None, help="run only the first N routes"
    )
    p.add_argument(
        "--stack-startup-s",
        type=float,
        default=60.0,
        help="wait after launching a stack",
    )
    return p.parse_args(argv)


def _drive_town(
    *,
    node: RouteRunnerNode,
    town: str,
    town_routes: list[RouteSpec],
    args: argparse.Namespace,
    profile: Path,
    out_dir: Path,
    limits: RunLimits,
    results: list[RouteResult],
) -> None:
    """One stack launch per town; every route of the town is a reset."""
    stack: StackProcess | None = None
    if not args.no_launch:
        stack = StackProcess(
            town_profile(profile, town, out_dir), out_dir / f"stack_{town}.log"
        )
        print(f"launched stack for {town}; waiting {args.stack_startup_s:.0f}s")
        time.sleep(args.stack_startup_s)
    try:
        for route in town_routes:
            print(f"route {route.route_id} ({town}, {len(route.waypoints)} waypoints)")
            if stack is not None and not stack.alive():
                # Otherwise every remaining route waits 200 s for the reset
                # service before coming back as reset_failed.
                print(
                    f"  stack for {town} has exited; see {out_dir / f'stack_{town}.log'}"
                )
                result = RouteResult(route.route_id, route.town)
                result.status = "stack_died"
                results.append(result)
                write_results(results, out_dir)
                continue
            result = node.drive(route, limits, out_dir / "routes")
            results.append(result)
            print(
                f"  {result.status}: {result.driven_m:.0f} m in {result.sim_s:.1f} s sim, "
                f"lat |mean| {result.lat_abs_mean_m:.3f} max {result.lat_max_m:.3f} m, "
                f"timeouts {result.timeouts}"
            )
            write_results(results, out_dir)
    finally:
        if stack is not None:
            stack.stop()


def main(argv: list[str] | None = None) -> int:
    """Run the routes and print the table."""
    args = parse_args(argv)
    if isinstance(sys.stdout, io.TextIOWrapper):
        sys.stdout.reconfigure(line_buffering=True)  # progress reaches a log file live
    profile = Path(args.profile)
    if profile.suffix != ".yaml":
        profile = REPO_ROOT / "configs/profiles" / f"{args.profile}.yaml"
    routes: list[RouteSpec] = []
    for f in args.routes:
        routes.extend(load_route_xml(f))
    if args.route_ids:
        routes = [r for r in routes if r.route_id in set(args.route_ids)]
    if args.limit is not None:
        routes = routes[: args.limit]
    out_dir = args.out or (REPO_ROOT / "data/eval_runs" / args.run_id)
    if (out_dir / "results.csv").exists():
        # Stale per-route directories of an earlier run would otherwise be
        # mixed into this one (and picked up by compare_runs.py).
        print(f"{out_dir} already holds a run; pick another --run-id or --out")
        return 2
    out_dir.mkdir(parents=True, exist_ok=True)
    limits = RunLimits(
        goal_radius_m=args.goal_radius_m,
        wall_timeout_s=args.wall_timeout_s,
        min_avg_speed_mps=args.min_avg_speed_mps,
    )
    rclpy.init()
    node = RouteRunnerNode()
    results: list[RouteResult] = []
    try:
        for town, town_routes in group_by_town(routes).items():
            _drive_town(
                node=node,
                town=town,
                town_routes=town_routes,
                args=args,
                profile=profile,
                out_dir=out_dir,
                limits=limits,
                results=results,
            )
    except KeyboardInterrupt:
        print("interrupted")
        return 130
    finally:
        node.destroy_node()
        # rclpy's own SIGINT handler already shut the context down on Ctrl-C.
        if rclpy.ok():
            rclpy.shutdown()
    print()
    print(format_table(results))
    print(f"\nresults: {out_dir / 'results.csv'}")
    return 0 if results and all(r.status == "completed" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
