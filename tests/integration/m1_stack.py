"""Shared pieces of the M1 integration tests (task 18): a stack on the smoke route.

Every test here needs a CARLA server started by ``tools/carla/start_carla.sh``
and runs under ``uv run pytest -m "slow and carla" tests/integration``. The
stack is ``m1_classical`` with the traffic cut to 20 vehicles and 10 walkers
(the spec's "short route, 20 vehicles"), driven through the eval harness the
same way ``run_routes.py`` does, so the assertions read ``RouteResult`` rows.
"""

from __future__ import annotations

import csv
import math
import socket
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import yaml

CARLA_PORT = 2000  # configs/profiles/m1_classical.yaml carla.port
ROUTE_FILE = "tools/eval/routes/smoke_town03.xml"
PROFILE = "configs/profiles/m1_classical.yaml"
STACK_STARTUP_S = 60.0
N_VEHICLES = 20
N_WALKERS = 10


def carla_server_listening(port: int = CARLA_PORT) -> bool:
    """Whether a CARLA RPC server answers on ``port`` (the tests skip otherwise)."""
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            return True
    except OSError:
        return False


def write_test_profile(repo_root: Path, out_dir: Path) -> Path:
    """``m1_classical`` on Town03 with the test's traffic, written into ``out_dir``."""
    with (repo_root / PROFILE).open() as f:
        prof = yaml.safe_load(f)
    prof["carla"]["town"] = "Town03"
    prof["carla"]["traffic"]["n_vehicles"] = N_VEHICLES
    prof["carla"]["traffic"]["n_walkers"] = N_WALKERS
    prof.setdefault("eval", {})["render"] = "off"
    path = out_dir / "profile_Town03.yaml"
    with path.open("w") as f:
        yaml.safe_dump(prof, f, sort_keys=False)
    return path


def start_stack(repo_root: Path, run_dir: Path, launch_args: Sequence[str] = ()) -> Any:
    """Launch the stack (``StackProcess``) and wait for its warm-up."""
    from nuway_eval import route_runner  # noqa: PLC0415 -- needs the ROS env

    profile = write_test_profile(repo_root, run_dir)
    proc = route_runner.StackProcess(
        profile, run_dir / "stack_Town03.log", launch_args=launch_args
    )
    time.sleep(STACK_STARTUP_S)
    return proc


def drive_smoke_route(
    repo_root: Path, out_dir: Path, *, record: bool = False, on_started: Any = None
) -> Any:
    """Drive the smoke route once (seed 0, ClearNoon) and return its ``RouteResult``.

    ``on_started(node)`` runs in a thread once the drive is under way (the
    degradation test kills a node from it).
    """
    import threading  # noqa: PLC0415 -- keep the module importable without rclpy

    import rclpy  # noqa: PLC0415

    from nuway_eval import driving_score, route_runner  # noqa: PLC0415
    from nuway_ml.common import routes  # noqa: PLC0415

    route = routes.load_route_xml(repo_root / ROUTE_FILE)[0]
    run = route_runner.RouteRun(route, "ClearNoon", 0)
    scoring = driving_score.ScoringConfig.load(
        repo_root / "configs/eval/scoring_lb20.yaml"
    )
    rclpy.init()
    node = route_runner.RouteRunnerNode()
    try:
        if on_started is not None:
            threading.Thread(target=on_started, args=(node,), daemon=True).start()
        limits = route_runner.RunLimits(record=record, wall_timeout_s=900.0)
        return node.drive(run, limits, out_dir, scoring)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def route_dir(out_dir: Path) -> Path:
    """The harness's directory for the smoke route under ``out_dir``."""
    return out_dir / "short03_ClearNoon_0"


def read_csv(path: Path) -> list[dict[str, str]]:
    """The rows of one of the harness's CSVs."""
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def speeds(rows: Sequence[dict[str, str]]) -> list[float]:
    """Ego speed per ``ego_odom.csv`` row, m/s."""
    return [math.hypot(float(r["vx"]), float(r["vy"])) for r in rows]
