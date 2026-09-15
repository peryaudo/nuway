"""M1 integration test (task 18): the classical stack completes the smoke route in traffic.

One ``m1_classical`` stack on Town03 with 20 vehicles and 10 walkers drives
``tools/eval/routes/smoke_town03.xml`` through the harness; the row must say
``completed`` with no collision and no tick timeout (slow, needs CARLA).
"""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any

import m1_stack
import pytest

pytestmark = [
    pytest.mark.slow,
    pytest.mark.carla,
    pytest.mark.skipif(
        not m1_stack.carla_server_listening(),
        reason=f"no CARLA server on :{m1_stack.CARLA_PORT}",
    ),
]

pytest.importorskip("rclpy")
pytest.importorskip("nuway_eval.route_runner")


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("m1_traffic")


@pytest.fixture(scope="module")
def repo_root_module() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def result(repo_root_module: Path, run_dir: Path) -> Iterator[Any]:
    stack = m1_stack.start_stack(repo_root_module, run_dir)
    try:
        yield m1_stack.drive_smoke_route(repo_root_module, run_dir)
    finally:
        stack.stop()


def test_route_completes_without_collision(result: Any, run_dir: Path) -> None:
    assert result.status == "completed", result
    assert result.completion == pytest.approx(1.0), result
    assert result.n_collision_vehicle == 0, result
    assert result.n_collision_pedestrian == 0, result
    assert result.n_collision_layout == 0, result
    assert result.n_tick_timeouts == 0, result
    assert result.degraded_sources == "", result
    # The car drove: the odometry shows real motion, not a teleport.
    rows = m1_stack.read_csv(m1_stack.route_dir(run_dir) / "ego_odom.csv")
    assert max(m1_stack.speeds(rows)) > 3.0
