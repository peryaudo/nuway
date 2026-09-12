"""infractions: every detector on synthetic ticks."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from nuway_eval.driving_score import ScoringConfig
from nuway_eval.infractions import (
    CollisionEvent,
    InfractionTracker,
    StopLine,
    StopSignVolume,
    TickObservation,
    point_in_polygon,
)

ROOT = Path(__file__).resolve().parents[3]
DT = 0.05


@pytest.fixture
def cfg() -> ScoringConfig:
    return ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")


def _obs(k: int, x: float, speed: float = 5.0, **kw: object) -> TickObservation:
    base: dict[str, object] = {
        "k": k,
        "t": k * DT,
        "x": x,
        "y": 0.0,
        "speed_mps": speed,
        "s": x,
        "d": 0.0,
        "left_bound_m": 1.75,
        "right_bound_m": 1.75,
    }
    base.update(kw)
    return TickObservation(**base)  # type: ignore[arg-type]  # kwargs mirror the dataclass fields


def test_collisions_are_classed_and_deduplicated(cfg: ScoringConfig) -> None:
    tr = InfractionTracker(cfg, 1000.0)
    car = CollisionEvent(7, "vehicle.audi.a2", 3.0, True)
    walker = CollisionEvent(9, "walker.pedestrian.0001", 1.0, True)
    pole = CollisionEvent(3, "static.prop.streetsign", 0.0, False)
    # The Leaderboard's folding: the second hit of the car is the same actor, the
    # walker's hit happens where the car's was counted.
    assert tr.update(_obs(0, 0.0, collisions=[car, car, walker])) == [
        "collision_vehicle",
    ]
    assert tr.update(_obs(10, 0.5, collisions=[car])) == []  # 0.5 s later, 0.5 m on
    # 6 m on: the location is forgotten, the car's id (2 s ago) is not.
    assert tr.update(_obs(41, 6.0, collisions=[car, pole])) == ["collision_layout"]
    # 6 s after the car was counted, 6 m from the pole: both forgotten.
    assert tr.update(_obs(120, 12.0, collisions=[car])) == ["collision_vehicle"]
    c = tr.counts
    assert c.collisions == [
        (0, "collision_vehicle", "vehicle.audi.a2", 3.0, True, 5.0),
        (41, "collision_layout", "static.prop.streetsign", 0.0, False, 5.0),
        (120, "collision_vehicle", "vehicle.audi.a2", 3.0, True, 5.0),
    ]
    assert (c.n_collision_vehicle, c.n_collision_pedestrian, c.n_collision_layout) == (
        2,
        0,
        1,
    )
    assert c.incidents[0] == (0, "collision_vehicle")


def test_red_light_counts_a_crossing_while_red_only(cfg: ScoringConfig) -> None:
    tr = InfractionTracker(cfg, 1000.0)
    line = StopLine(light_id=1, x=10.0, y=0.0, heading=0.0, half_width_m=1.75)
    # Approach and cross at x = 10 while red: one infraction, not one per tick past it.
    for k, x in enumerate([8.0, 9.0, 9.9, 10.1, 11.0, 12.0]):
        fired = tr.update(_obs(k, x, red_stop_lines=[line]))
        assert fired == (["red_light"] if x == 10.1 else [])
    assert tr.counts.n_red_light == 1
    # A second light crossed on green (not in the red list) counts nothing; the
    # same light crossed again after it turned green and red again counts anew.
    tr2 = InfractionTracker(cfg, 1000.0)
    tr2.update(_obs(0, 9.0))
    assert tr2.update(_obs(1, 11.0)) == []
    tr2.update(_obs(2, 9.0, red_stop_lines=[line]))
    assert tr2.update(_obs(3, 11.0, red_stop_lines=[line])) == ["red_light"]
    # Beside the lane (across > half width) the line is not this lane's.
    tr3 = InfractionTracker(cfg, 1000.0)
    tr3.update(_obs(0, 9.0, y=3.0, red_stop_lines=[line]))
    assert tr3.update(_obs(1, 11.0, y=3.0, red_stop_lines=[line])) == []


def test_stop_sign_needs_a_halt_inside_the_volume(cfg: ScoringConfig) -> None:
    poly = ((10.0, -2.0), (14.0, -2.0), (14.0, 2.0), (10.0, 2.0))
    assert point_in_polygon(12.0, 0.0, poly)
    assert not point_in_polygon(9.0, 0.0, poly)
    sign = StopSignVolume(5, poly)
    rolling = InfractionTracker(cfg, 1000.0)
    for k, x in enumerate([9.0, 11.0, 13.0, 15.0]):
        fired = rolling.update(_obs(k, x, speed=3.0, stop_signs=[sign]))
    assert fired == ["stop_sign"]
    assert rolling.counts.n_stop_sign == 1
    honoured = InfractionTracker(cfg, 1000.0)
    for k, (x, v) in enumerate(
        [(9.0, 3.0), (11.0, 1.0), (12.0, 0.1), (13.0, 2.0), (15.0, 3.0)]
    ):
        assert honoured.update(_obs(k, x, speed=v, stop_signs=[sign])) == []
    assert honoured.counts.n_stop_sign == 0


def test_outside_lanes_is_a_distance_fraction(cfg: ScoringConfig) -> None:
    tr = InfractionTracker(cfg, 1000.0)
    tr.update(_obs(0, 0.0))
    tr.update(_obs(1, 1.0))  # inside: 1 m
    tr.update(_obs(2, 2.0, d=2.0))  # 2 m left of the line, edge at 1.75: 1 m outside
    tr.update(_obs(3, 3.0, d=-1.0))  # inside again
    tr.update(_obs(4, 4.0, d=-2.5))  # outside on the right
    assert tr.counts.outside_lanes_frac == pytest.approx(0.5)


def test_route_ending_conditions(cfg: ScoringConfig) -> None:
    blocked = InfractionTracker(cfg, 1000.0)
    k = 0
    while not blocked.ended:
        blocked.update(_obs(k, 0.0, speed=0.05))
        k += 1
    assert blocked.ended == "blocked"
    assert blocked.counts.blocked
    assert k * DT == pytest.approx(cfg.blocked_s, abs=2 * DT)
    assert blocked.counts.incidents[-1][1] == "blocked"

    deviated = InfractionTracker(cfg, 1000.0)
    deviated.update(_obs(0, 0.0))
    assert deviated.update(_obs(1, 1.0, d=None)) == ["route_deviation"]
    assert deviated.counts.route_deviation

    timed = InfractionTracker(cfg, 100.0)  # budget 100 / 5 * 2 + 60 = 100 s
    timed.update(_obs(0, 0.0))
    assert timed.update(_obs(int(99.0 / DT), 5.0)) == []
    assert timed.update(_obs(int(101.0 / DT), 6.0)) == ["timeout"]
    assert timed.counts.timeout


def test_min_speed_is_one_percentage_per_checkpoint(cfg: ScoringConfig) -> None:
    assert cfg.min_speed_checkpoints == 1
    tr = InfractionTracker(cfg, 100.0)
    fired: list[str] = []
    # Driving 2 m/s past traffic doing 8 m/s over the whole route: no event until
    # the route closes, then one checkpoint at 25 %.
    for k in range(1000):
        fired += tr.update(_obs(k, k * 0.1, speed=2.0, traffic_speeds_mps=[8.0, 8.0]))
    assert fired == []
    assert tr.finish(1000) == ["min_speed"]
    assert tr.counts.n_min_speed == 1
    assert tr.counts.min_speed_pcts == [25.0]
    assert tr.counts.incidents == [(1000, "min_speed")]
    # A route that ended early carries no partial checkpoint (LB2 terminate: > 95 %).
    early = InfractionTracker(cfg, 100.0)
    for k in range(200):
        early.update(_obs(k, k * 0.1, speed=2.0, traffic_speeds_mps=[8.0]))
    assert early.finish(200) == []
    # No traffic at all, or the ego as fast as the traffic: 100 %, nothing fires.
    quiet = InfractionTracker(cfg, 100.0)
    for k in range(1000):
        assert quiet.update(_obs(k, k * 0.1, speed=1.0)) == []
    assert quiet.finish(1000) == []
    assert quiet.counts.min_speed_pct == 100.0
    fast = InfractionTracker(cfg, 100.0)
    for k in range(1000):
        assert fast.update(_obs(k, k * 0.1, speed=8.0, traffic_speeds_mps=[8.0])) == []
    assert fast.finish(1000) == []
    # Two checkpoints: the first closes mid-route as the ego passes half the length.
    two = InfractionTracker(replace(cfg, min_speed_checkpoints=2), 100.0)
    mid: list[str] = []
    for k in range(600):
        mid += two.update(_obs(k, k * 0.1, speed=4.0, traffic_speeds_mps=[8.0]))
    assert mid == ["min_speed"]
    assert two.counts.min_speed_pcts == [50.0]
