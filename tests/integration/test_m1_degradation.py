"""M1 integration test (task 18): a dead prediction node degrades the stack safely.

``const_vel_node`` is killed once the smoke-route drive is under way. The
lockstep then times out exactly once (``TickTimeout`` is the only wall-clock
event, docs/02 §2), the FSM, planner and safety layer switch to their
``samples`` fallback and stand the car still, and the harness flags the row
(``non_deterministic``, ``degraded_sources``) and ends it as blocked.
"""

from __future__ import annotations

import subprocess
import time
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

KILL_SPEED_MPS = 2.0  # kill once the drive's own trace shows the ego under way
KILL_WAIT_S = 180.0  # ... or after this much wall clock (the route is 105 m)
STILL_SPEED_MPS = 0.1


def _kill_const_vel(node: Any) -> None:
    """SIGKILL const_vel_node by exact process name (never ``-f``: it self-matches).

    Waits on the runner's odometry trace rather than a wall-clock delay: the
    smoke route is driven in ~40 s wall and a fixed delay either lands before
    the reset or after the goal.
    """
    deadline = time.monotonic() + KILL_WAIT_S
    while time.monotonic() < deadline:
        rows = node._trace.odom
        if rows and abs(rows[-1][6]) > KILL_SPEED_MPS:
            break
        time.sleep(0.25)
    subprocess.run(["pkill", "-9", "-x", "const_vel_node"], check=False)


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("m1_degradation")


@pytest.fixture(scope="module")
def repo_root_module() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def result(repo_root_module: Path, run_dir: Path) -> Iterator[Any]:
    stack = m1_stack.start_stack(repo_root_module, run_dir)
    try:
        yield m1_stack.drive_smoke_route(
            repo_root_module, run_dir, on_started=_kill_const_vel
        )
    finally:
        stack.stop()


def test_one_timeout_then_a_flagged_standstill(result: Any, run_dir: Path) -> None:
    assert result.n_tick_timeouts == 1, result
    assert result.non_deterministic, result
    # In the M1 profiles const_vel_node publishes samples *and* the fallback
    # (docs/02 §3.6), so both are gone: the planner and the safety layer name
    # the samples source they lost and the FSM, with neither, reports the
    # missing input; every one of the three is latched degraded.
    sources = result.degraded_sources.split(";")
    assert any("samples" in s for s in sources), result
    for node in ("behavior_fsm_node", "planner_node", "safety_layer_node"):
        assert any(s.startswith(f"{node}:") for s in sources), result
    assert result.status == "blocked", result
    assert result.n_collision_vehicle == 0, result
    assert result.n_collision_pedestrian == 0, result
    # The car stood still from the fallback on: the last seconds are at rest,
    # after having moved before the kill.
    rows = m1_stack.read_csv(m1_stack.route_dir(run_dir) / "ego_odom.csv")
    speeds = m1_stack.speeds(rows)
    assert max(speeds) > 1.0
    assert max(speeds[-200:]) < STILL_SPEED_MPS
