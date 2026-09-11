"""M1 integration test (task 18): the drive does not depend on delivery timing.

The smoke route is driven twice in two stacks, the second with
``callback_delay_ms:=20`` (a wall-clock sleep in ``const_vel_node``'s tick,
M1 §5). Under the current-tick barrier the late input reorders nothing, so
the two rows must carry the same outcome and the command sequences must agree
within CARLA's own spread as measured in task 10: identical commands while
the hero stands, then a divergence that the lattice's discrete choices
amplify. The first differing command tick is printed (run with ``-s``).
"""

from __future__ import annotations

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

DELAY_MS = 20
TICK_COUNT_SPREAD = (
    0.05  # +-5 % of the route's ticks (task 10: +-1 tick at rest, seconds in traffic)
)
METRIC_COLUMNS = (
    "status",
    "n_collision_pedestrian",
    "n_collision_vehicle",
    "n_collision_layout",
    "n_red_light",
    "n_stop_sign",
    "route_deviation",
    "blocked",
    "timeout",
    "n_tick_timeouts",
)


@pytest.fixture(scope="module")
def run_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("m1_determinism")


@pytest.fixture(scope="module")
def repo_root_module() -> Path:
    return Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def results(repo_root_module: Path, run_dir: Path) -> list[Any]:
    out: list[Any] = []
    for i, args in enumerate(((), (f"callback_delay_ms:={DELAY_MS}",))):
        sub = run_dir / f"run{i}"
        sub.mkdir()
        stack = m1_stack.start_stack(repo_root_module, sub, launch_args=args)
        try:
            out.append(m1_stack.drive_smoke_route(repo_root_module, sub))
        finally:
            stack.stop()
    return out


def test_metric_columns_agree(results: list[Any]) -> None:
    a, b = results
    for column in METRIC_COLUMNS:
        assert getattr(a, column) == getattr(b, column), (column, a, b)
    assert a.completion == pytest.approx(b.completion, abs=0.05)
    assert a.n_tick_timeouts == 0


def test_command_sequences_agree_within_the_spread(
    results: list[Any], run_dir: Path
) -> None:
    del results
    cmds = [
        m1_stack.read_csv(
            m1_stack.route_dir(run_dir / f"run{i}") / "control_command.csv"
        )
        for i in range(2)
    ]
    odom = [
        m1_stack.read_csv(m1_stack.route_dir(run_dir / f"run{i}") / "ego_odom.csv")
        for i in range(2)
    ]
    n0, n1 = len(cmds[0]), len(cmds[1])
    assert n0 > 100
    assert abs(n0 - n1) <= TICK_COUNT_SPREAD * n0, (n0, n1)
    first_diff: int | None = None
    for i, (a, b) in enumerate(zip(cmds[0], cmds[1], strict=False)):
        same = all(
            abs(float(a[c]) - float(b[c])) < 1e-9
            for c in ("accel", "steering_angle", "emergency_stop")
        )
        if not same:
            first_diff = i
            break
    speeds = m1_stack.speeds(odom[0])
    first_move = next((i for i, v in enumerate(speeds) if v > 0.05), len(speeds))
    print(  # noqa: T201 -- the measured spread is this test's report (run with -s)
        f"first differing command tick {first_diff} (row), first moving row {first_move}, "
        f"ticks {n0} vs {n1}"
    )
    # Identical while the hero stands: the delay cannot reorder a tick.
    assert first_diff is None or first_diff >= first_move, (first_diff, first_move)
