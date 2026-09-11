"""Evaluation harness entry point (M1 §3.10).

    uv run tools/eval/run_routes.py --profile m1_classical --routes m1
    uv run tools/eval/run_routes.py --profile m1_classical tools/eval/routes/dev_town03.xml --limit 1
    uv run tools/eval/run_routes.py --profile m1_classical --routes m1 --resume 20260910_120000

The protocol is every route of the selected files under every weather
preset with one traffic seed. One stack per town (launched here unless
--no-launch), one /nuway/sim/reset per run, the infraction detectors and
the Leaderboard 2.0 score per run, results.csv + report.md + one directory
per run under data/eval_runs/<run_id>/ (docs/02 §8).

--resume skips every run that already has a final row; a CARLA client
timeout marks the row `crashed`, restarts the server and the stack, and
reruns that row once.
"""

from __future__ import annotations

import argparse
import io
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml

from nuway_eval.carla_probe import CarlaProbe
from nuway_eval.driving_score import ScoringConfig
from nuway_eval.report import (
    FINAL_STATUSES,
    RouteResult,
    read_results,
    render_full,
    render_incidents,
    write_report,
    write_results,
)
from nuway_eval.route_runner import (
    WEATHER_PRESETS,
    EvalConfig,
    RouteRun,
    RouteRunnerNode,
    RunLimits,
    StackProcess,
    group_by_town,
    protocol,
    town_profile,
)
from nuway_ml.common.routes import RouteSpec, load_route_xml

REPO_ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_FILES = (
    "tools/eval/routes/dev_town03.xml",
    "tools/eval/routes/dev_town05.xml",
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """CLI."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "route_files",
        nargs="*",
        type=Path,
        help="route XML files (Leaderboard format); default: the protocol files",
    )
    p.add_argument(
        "--routes",
        choices=("m1", "full"),
        default=None,
        help='m1: the routes marked protocol="m1"; full: every route of the dev files',
    )
    p.add_argument(
        "--profile", default="m1_classical", help="profile name or YAML path"
    )
    p.add_argument(
        "--weathers",
        nargs="+",
        default=list(WEATHER_PRESETS),
        help="weather presets (default: the three of M1 §3.10)",
    )
    p.add_argument(
        "--seed", type=int, default=None, help="traffic seed (default: the profile's)"
    )
    p.add_argument("--run-id", default=time.strftime("%Y%m%d_%H%M%S"))
    p.add_argument("--resume", default=None, help="run id to continue")
    p.add_argument(
        "--out",
        type=Path,
        default=None,
        help="run directory (default data/eval_runs/<run_id>)",
    )
    p.add_argument(
        "--no-launch",
        action="store_true",
        help="a stack is already running (single town)",
    )
    p.add_argument("--no-record", action="store_true", help="skip the MCAP recording")
    p.add_argument(
        "--render",
        choices=("off", "incidents", "full"),
        default=None,
        help="override the profile's eval.render (docs/02 §8)",
    )
    p.add_argument("--wall-timeout-s", type=float, default=1800.0)
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


def select_routes(args: argparse.Namespace) -> list[RouteSpec]:
    """Select the routes of the CLI: files and/or the protocol marks."""
    files = list(args.route_files) or [REPO_ROOT / f for f in PROTOCOL_FILES]
    routes: list[RouteSpec] = []
    for f in files:
        routes.extend(load_route_xml(f))
    if args.routes == "m1":
        routes = [r for r in routes if r.protocol == "m1"]
    if args.route_ids:
        routes = [r for r in routes if r.route_id in set(args.route_ids)]
    if args.limit is not None:
        routes = routes[: args.limit]
    return routes


def git_sha() -> str:
    """Short commit hash of the checkout, or "" outside git."""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
            cwd=REPO_ROOT,
        )
    except (subprocess.CalledProcessError, OSError):
        return ""
    return out.stdout.strip()


class Harness:
    """One run directory: the results so far, the stack per town, the probe."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Resolve the profile, the run directory and the earlier rows of a resume."""
        self.args = args
        profile = Path(args.profile)
        if profile.suffix != ".yaml":
            profile = REPO_ROOT / "configs/profiles" / f"{args.profile}.yaml"
        self.profile_path = profile
        with profile.open() as f:
            self.profile = yaml.safe_load(f) or {}
        self.eval = EvalConfig.from_profile(self.profile)
        self.scoring = ScoringConfig.load(REPO_ROOT / self.eval.scoring)
        self.seed = int(args.seed) if args.seed is not None else self.eval.traffic_seed
        run_id = args.resume or args.run_id
        self.run_id = run_id
        self.out_dir = args.out or (REPO_ROOT / "data/eval_runs" / run_id)
        self.results: list[RouteResult] = (
            read_results(self.out_dir / "results.csv") if args.resume else []
        )
        self.limits = RunLimits(
            goal_radius_m=args.goal_radius_m,
            wall_timeout_s=args.wall_timeout_s,
            record=self.eval.record and not args.no_record,
            record_sensors=self.eval.record_sensors,
            chase_cam=self.eval.chase_cam,
        )
        self.render = args.render or self.eval.render
        if not self.limits.record:
            self.render = "off"  # nothing to render without a bag
        self.node: RouteRunnerNode | None = None
        self.probe: CarlaProbe | None = None
        self.stack: StackProcess | None = None

    def done(self, run: RouteRun) -> bool:
        """Tell whether a resume may skip the run (its row is final)."""
        for r in self.results:
            if r.key == run.key and r.status in FINAL_STATUSES and not r.crashed:
                return True
        return False

    def render_outputs(self, result: RouteResult) -> None:
        """Incident sheets (and the full route when asked) from the run's bag."""
        if self.render == "off" or not result.bag_path:
            return
        sheets = render_incidents(self.out_dir, result, self.eval.incident_window)
        if sheets:
            print(f"  {len(sheets)} incident sheets")
        if self.render == "full":
            render_full(self.out_dir, result, self.eval.render_stride)

    def record(self, result: RouteResult) -> None:
        """Replace or append the row and rewrite the files."""
        self.results = [r for r in self.results if r.key != result.key]
        self.results.append(result)
        write_results(self.results, self.out_dir)
        write_report(self.results, self.out_dir, self.scoring)

    def connect_probe(self) -> None:
        """(Re)open the harness's own CARLA client."""
        if self.probe is not None:
            self.probe.close()
        self.probe = CarlaProbe(self.eval.host, self.eval.port)
        self.probe.cache_town()
        if self.node is not None:
            self.node._probe = self.probe

    def launch(self, town: str) -> None:
        """Launch the town's stack unless one is already running."""
        if self.args.no_launch:
            return
        self.stack = StackProcess(
            town_profile(self.profile_path, town, self.out_dir),
            self.out_dir / f"stack_{town}.log",
        )
        print(f"launched stack for {town}; waiting {self.args.stack_startup_s:.0f}s")
        time.sleep(self.args.stack_startup_s)

    def teardown(self) -> None:
        """Stop the stack we launched."""
        if self.stack is not None:
            self.stack.stop()
            self.stack = None

    def recover(self, town: str) -> bool:
        """After a CARLA crash: restart the server and the stack (only if we launched it)."""
        if self.args.no_launch:
            return False
        self.teardown()
        print("restarting the CARLA server")
        ok = subprocess.run(
            [str(REPO_ROOT / "tools/carla/start_carla.sh"), "--wait"],
            check=False,
            cwd=REPO_ROOT,
        )
        if ok.returncode != 0:
            return False
        self.launch(town)
        try:
            self.connect_probe()
        except RuntimeError as err:
            print(f"probe reconnect failed: {err}")
            return False
        return True

    def drive_one(self, run: RouteRun) -> RouteResult:
        """One run, marked ``crashed`` on a client timeout or a dead stack."""
        assert self.node is not None
        result = RouteResult(
            *run.key,
            profile=self.profile_path.stem,
            git_sha=git_sha(),
            run_id=self.run_id,
        )
        if self.stack is not None and not self.stack.alive():
            result.status = "stack_died"
            result.crashed = True
            return result
        try:
            return self.node.drive(run, self.limits, self.out_dir, self.scoring, result)
        except RuntimeError as err:  # the CARLA client timed out: the server is gone
            print(f"  crashed: {err}")
            result.status = "crashed"
            result.crashed = True
            return result

    def drive_town(self, town: str, runs: list[RouteRun]) -> None:
        """Every run of a town on one stack; one retry after a crash."""
        self.launch(town)
        try:
            for run in runs:
                if self.done(run):
                    print(f"skip {run.dir_name}: already final")
                    continue
                print(f"run {run.dir_name} ({len(run.route.waypoints)} waypoints)")
                result = self.drive_one(run)
                if result.crashed and self.recover(town):
                    print(f"  rerunning {run.dir_name} after the crash")
                    result = self.drive_one(run)
                    result.crashed = result.crashed or result.status == "crashed"
                self.render_outputs(result)
                self.record(result)
                print(
                    f"  {result.status}: completion {result.completion:.3f}, "
                    f"score {result.driving_score:.1f}, {result.sim_time_s:.0f} s sim, "
                    f"timeouts {result.n_tick_timeouts}"
                )
        finally:
            self.teardown()

    def run(self, runs: list[RouteRun]) -> int:
        """Drive every town; returns 0 when every run completed."""
        self.out_dir.mkdir(parents=True, exist_ok=True)
        rclpy.init()
        self.node = RouteRunnerNode()
        try:
            for town, town_runs in group_by_town(runs).items():
                if self.args.no_launch:
                    self.connect_probe()
                    self.drive_town(town, town_runs)
                else:
                    self.launch(town)
                    self.args.no_launch = True  # drive_town must not launch again
                    try:
                        self.connect_probe()
                        self.drive_town(town, town_runs)
                    finally:
                        self.args.no_launch = False
                        self.teardown()
        except KeyboardInterrupt:
            print("interrupted")
            return 130
        finally:
            if self.probe is not None:
                self.probe.close()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print(f"\nresults: {self.out_dir / 'results.csv'}")
        wanted = {r.key for r in runs}
        rows = [r for r in self.results if r.key in wanted]
        return 0 if rows and all(r.status == "completed" for r in rows) else 1


def main(argv: list[str] | None = None) -> int:
    """Run the protocol and print where the results are."""
    args = parse_args(argv)
    if isinstance(sys.stdout, io.TextIOWrapper):
        sys.stdout.reconfigure(line_buffering=True)  # progress reaches a log file live
    harness = Harness(args)
    if not args.resume and (harness.out_dir / "results.csv").exists():
        print(
            f"{harness.out_dir} already holds a run; pick another --run-id, or --resume it"
        )
        return 2
    runs = protocol(select_routes(args), args.weathers, harness.seed)
    if not runs:
        print("no routes selected")
        return 2
    return harness.run(runs)


if __name__ == "__main__":
    sys.exit(main())
