"""M0 integration test (task 14): the m0_gt_all stack drives a Town03 route (slow, needs CARLA).

Launches one stack on Town03 (``realtime_factor: 0``, no Foxglove), drives
the first dev route twice through the v0 harness, and asserts completion,
the lateral-error bounds of the M0 completion criteria, no TickTimeout, and
the lockstep criterion as degraded by M0 §6: the two runs' ``/nuway/gt/ego_odom``
sequences agree up to CARLA's own measured spread. Run with ``uv run pytest -m "slow and carla" tests/integration``
against a server started by ``tools/carla/start_carla.sh``.
"""

from __future__ import annotations

import csv
import math
import shutil
import time
from collections.abc import Iterator
from pathlib import Path
from typing import Any

import pytest

pytestmark = [pytest.mark.slow, pytest.mark.carla]

rclpy = pytest.importorskip("rclpy")
route_runner = pytest.importorskip("nuway_eval.route_runner")
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
    rclpy.init()
    node = route_runner.RouteRunnerNode()
    out: list[Any] = []
    try:
        for i in range(2):
            out.append(node.drive(route, route_runner.RunLimits(), run_dir / f"run{i}"))
            shutil.copy(
                run_dir / f"run{i}" / route.route_id / "ego_odom.csv",
                run_dir / f"odom{i}.csv",
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return out


def test_route_completes_within_lateral_bounds(results: list[Any]) -> None:
    for r in results:
        assert r.status == "completed", r
        assert r.timeouts == 0, r
        assert r.lat_abs_mean_m < LAT_MEAN_MAX_M, r
        assert r.lat_max_m < LAT_MAX_M, r


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
    assert abs(results[0].lat_abs_mean_m - results[1].lat_abs_mean_m) < 0.01
    assert abs(results[0].lat_max_m - results[1].lat_max_m) < 0.05
