"""M0 integration test (task 14): the m0_gt_all stack drives a Town03 route (slow, needs CARLA).

Launches one stack on Town03 (``realtime_factor: 0``, no Foxglove), drives
the first dev route twice through the harness (one run, no recording), and asserts completion,
the lateral-error bounds of the M0 completion criteria, no TickTimeout, and
the lockstep criterion as degraded by M0 §6: the two runs' ``/nuway/gt/ego_odom``
sequences agree up to CARLA's own measured spread. Run with ``uv run pytest -m "slow and carla" tests/integration``
against a server started by ``tools/carla/start_carla.sh``.
"""

from __future__ import annotations

import csv
import math
import shutil
import socket
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

CARLA_PORT = 2000  # configs/profiles/m0_gt_all.yaml carla.port


def _carla_server_listening(port: int = CARLA_PORT) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            return True
    except OSError:
        return False


pytestmark = [
    pytest.mark.slow,
    pytest.mark.carla,
    pytest.mark.skipif(
        not _carla_server_listening(), reason=f"no CARLA server on :{CARLA_PORT}"
    ),
]

rclpy = pytest.importorskip("rclpy")
route_runner = pytest.importorskip("nuway_eval.route_runner")
driving_score = pytest.importorskip("nuway_eval.driving_score")
routes = pytest.importorskip("nuway_ml.common.routes")

ROUTE_FILE = "tools/eval/routes/dev_town03.xml"
LAT_MEAN_MAX_M = 0.3
LAT_MAX_M = 0.8
ALONG_TRACK_SPREAD_M = 2.0
CROSS_TRACK_SPREAD_M = 0.1
STACK_STARTUP_S = 60.0


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("m0_route")


@pytest.fixture(scope="module")
def stack(repo_root_module: Path, run_dir: Path) -> Iterator[Any]:
    profile = route_runner.town_profile(
        repo_root_module / "configs/profiles/m0_gt_all.yaml", "Town03", run_dir
    )
    proc = route_runner.StackProcess(profile, run_dir / "stack_Town03.log")
    time.sleep(STACK_STARTUP_S)
    yield proc
    proc.stop()


@pytest.fixture(scope="module")
def repo_root_module() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def results(repo_root_module: Path, run_dir: Path, stack: Any) -> list[Any]:
    del stack
    route = routes.load_route_xml(repo_root_module / ROUTE_FILE)[0]
    run = route_runner.RouteRun(route, "ClearNoon", 0)
    scoring = driving_score.ScoringConfig.load(
        repo_root_module / "configs/eval/scoring_lb20.yaml"
    )
    rclpy.init()
    node = route_runner.RouteRunnerNode()
    out: list[Any] = []
    try:
        for i in range(2):
            limits = route_runner.RunLimits(record=False)
            out.append(node.drive(run, limits, run_dir / f"run{i}", scoring))
            for name in ("ego_odom", "control_debug"):
                shutil.copy(
                    run_dir / f"run{i}" / run.dir_name / f"{name}.csv",
                    run_dir / f"{name}{i}.csv",
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return out


def _lateral_stats(path: Path) -> tuple[float, float]:
    """|mean| and max of the lateral error over the ticks with solver_ok."""
    with path.open(newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["solver_ok"] == "1"]
    lat = [abs(float(r["lateral_error"])) for r in rows]
    return (sum(lat) / len(lat), max(lat)) if lat else (0.0, 0.0)


def test_route_completes_within_lateral_bounds(
    results: list[Any], run_dir: Path
) -> None:
    for i, r in enumerate(results):
        assert r.status == "completed", r
        assert r.n_tick_timeouts == 0, r
        lat_mean, lat_max = _lateral_stats(run_dir / f"control_debug{i}.csv")
        assert lat_mean < LAT_MEAN_MAX_M, r
        assert lat_max < LAT_MAX_M, r


def test_lockstep_runs_are_reproducible(results: list[Any], run_dir: Path) -> None:
    """Two episodes of one route agree up to CARLA's own spread.

    Bit-identical odometry is not reachable inside one PhysX scene: even
    with the hero's physics body recreated on reset the first tick differs
    at 1e-6 m and the episodes then drift apart along the route (measured
    2026-09-07: 0.86 m along-track, 0.02 m cross-track, tick count +-1,
    lateral-error stats equal to 3 decimals; M0 decisions log, task 14).
    The bounds below are 2-5x those measurements.
    """
    assert len(results) == 2
    with (
        (run_dir / "odom0.csv").open(newline="") as f0,
        (run_dir / "odom1.csv").open(newline="") as f1,
    ):
        rows0 = list(csv.DictReader(f0))
        rows1 = list(csv.DictReader(f1))
    assert len(rows0) > 100
    # Sim time restarts per episode only on a map load; ticks differ in absolute
    # index between the two episodes, so compare tick by tick from the reset.
    assert abs(len(rows0) - len(rows1)) <= 0.005 * len(rows0), (len(rows0), len(rows1))
    along_max = 0.0
    cross_max = 0.0
    first_diff: int | None = None
    for i, (a, b) in enumerate(zip(rows0, rows1, strict=False)):
        if first_diff is None and any(a[c] != b[c] for c in ("x", "y", "yaw")):
            first_diff = i
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        yaw = float(a["yaw"])
        along_max = max(along_max, abs(dx * math.cos(yaw) + dy * math.sin(yaw)))
        cross_max = max(cross_max, abs(-dx * math.sin(yaw) + dy * math.cos(yaw)))
    print(  # noqa: T201 -- the measured spread is this test's report (run with -s)
        f"first differing tick {first_diff}, along {along_max:.3f} m, cross {cross_max:.3f} m"
    )
    assert along_max < ALONG_TRACK_SPREAD_M, along_max
    assert cross_max < CROSS_TRACK_SPREAD_M, cross_max
    stats = [_lateral_stats(run_dir / f"control_debug{i}.csv") for i in range(2)]
    assert abs(stats[0][0] - stats[1][0]) < 0.01
    assert abs(stats[0][1] - stats[1][1]) < 0.05
