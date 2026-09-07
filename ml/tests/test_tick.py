from dataclasses import dataclass

import pytest

from nuway_ml.common.tick import (
    TickBarrier,
    is_planning_tick,
    tick_index,
    tick_stamp,
    tick_time_s,
)

pytestmark = pytest.mark.import_light


@dataclass
class Stamp:
    sec: int
    nanosec: int


def test_tick_index_rounds_accumulated_float_error():
    assert tick_index(0.0) == 0
    assert tick_index(0.05) == 1
    assert tick_index(1.0000000003) == 20
    assert tick_index(0.9999999997) == 20
    assert tick_index(12345.65) == 246913
    assert tick_index(Stamp(1, 50_000_000)) == 21


def test_tick_stamp_round_trips():
    for k in range(0, 100000, 997):
        sec, nanosec = tick_stamp(k)
        assert tick_index(Stamp(sec, nanosec)) == k
    assert tick_stamp(21) == (1, 50_000_000)
    assert tick_time_s(21) == pytest.approx(1.05)


def test_planning_ticks_are_even():
    assert is_planning_tick(0)
    assert not is_planning_tick(1)
    assert is_planning_tick(42)


def test_barrier_completes_when_every_input_arrived_for_the_tick():
    barrier = TickBarrier(["pose", "agents"])
    assert not barrier.is_complete(4)
    barrier.arrive("pose", 4)
    assert barrier.missing(4) == ["agents"]
    barrier.arrive("agents", 4)
    assert barrier.is_complete(4)
    assert not barrier.is_complete(5)
    barrier.degrade("agents")
    barrier.arrive("pose", 5)
    assert barrier.is_complete(5)
    assert barrier.is_degraded("agents")
    barrier.reset()
    assert not barrier.is_degraded("agents")
    assert not barrier.is_complete(5)
    assert barrier.inputs == ["pose", "agents"]
