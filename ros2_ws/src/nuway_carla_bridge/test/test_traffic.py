"""nuway_carla_bridge.traffic: seeded selection and bookkeeping against a fake CARLA client."""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, cast

import carla
import pytest

from nuway_carla_bridge import traffic as tr


# The real carla.command constructors reject anything but CARLA blueprints,
# so the module's ``carla`` name is swapped for this namespace in the tests.
class _SpawnActor:
    def __init__(
        self, blueprint: Any, transform: Any, parent: int | None = None
    ) -> None:
        self.blueprint = blueprint
        self.transform = transform
        self.parent = parent
        self.chained: Any = None

    def then(self, command: Any) -> _SpawnActor:
        self.chained = command
        return self


@dataclass
class _SetAutopilot:
    actor: Any
    enabled: bool
    port: int


@dataclass
class _DestroyActor:
    actor_id: int


_FAKE_CARLA = SimpleNamespace(
    command=SimpleNamespace(
        SpawnActor=_SpawnActor,
        SetAutopilot=_SetAutopilot,
        DestroyActor=_DestroyActor,
        FutureActor=object(),
    ),
    Transform=carla.Transform,
    Location=carla.Location,
)


@pytest.fixture(autouse=True)
def _fake_carla(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(tr, "carla", _FAKE_CARLA)


@dataclass
class _Point:
    location: carla.Location


@dataclass
class _Attribute:
    values: list[str]

    @property
    def recommended_values(self) -> list[str]:
        return self.values

    def as_int(self) -> int:
        return int(self.values[0])


class _Blueprint:
    def __init__(self, bp_id: str, wheels: int) -> None:
        self.id = bp_id
        self._attrs = {
            "number_of_wheels": _Attribute([str(wheels)]),
            "color": _Attribute(["0,0,0", "255,255,255"]),
            "speed": _Attribute(["0.0", "1.4", "2.8"]),
        }
        self.set: dict[str, str] = {}

    def has_attribute(self, name: str) -> bool:
        return name in self._attrs

    def get_attribute(self, name: str) -> _Attribute:
        return self._attrs[name]

    def set_attribute(self, name: str, value: str) -> None:
        self.set[name] = value


class _Library:
    def __init__(self) -> None:
        self.blueprints = [
            _Blueprint("vehicle.audi.a2", 4),
            _Blueprint("vehicle.bh.crossbike", 2),
            _Blueprint("vehicle.tesla.model3", 4),
            _Blueprint("walker.pedestrian.0001", 0),
            _Blueprint("controller.ai.walker", 0),
        ]

    def filter(self, pattern: str) -> list[_Blueprint]:
        prefix = pattern.rstrip("*")
        return [bp for bp in self.blueprints if bp.id.startswith(prefix)]

    def find(self, bp_id: str) -> _Blueprint:
        return next(bp for bp in self.blueprints if bp.id == bp_id)


@dataclass
class _Response:
    actor_id: int
    error: str = ""


class _Actor:
    def __init__(self, actor_id: int) -> None:
        self.id = actor_id
        self.calls: list[tuple[str, Any]] = []

    def start(self) -> None:
        self.calls.append(("start", None))

    def stop(self) -> None:
        self.calls.append(("stop", None))

    def go_to_location(self, location: carla.Location) -> None:
        self.calls.append(("go_to", location))

    def set_max_speed(self, speed: float) -> None:
        self.calls.append(("speed", speed))


class _Map:
    def __init__(self, points: list[_Point]) -> None:
        self._points = points

    def get_spawn_points(self) -> list[_Point]:
        return list(self._points)


@dataclass
class _World:
    """Spawn points on a 20 m grid; navigation locations from a counter."""

    library: _Library = field(default_factory=_Library)
    seed: int | None = None
    nav_calls: int = 0
    actors: dict[int, _Actor] = field(default_factory=dict)

    def get_blueprint_library(self) -> _Library:
        return self.library

    def get_map(self) -> _Map:
        return _Map(
            [
                _Point(carla.Location(x=20.0 * i, y=20.0 * j))
                for i in range(5)
                for j in range(5)
            ]
        )

    def set_pedestrians_seed(self, seed: int) -> None:
        self.seed = seed

    def get_random_location_from_navigation(self) -> carla.Location:
        self.nav_calls += 1
        return carla.Location(x=float(self.nav_calls), y=0.0)

    def get_actors(self, ids: list[int]) -> list[_Actor]:
        return [self.actors.setdefault(i, _Actor(i)) for i in ids]


class _Client:
    """Hands out ids in order; a spawn at the origin fails (the hero is there)."""

    def __init__(self, world: _World) -> None:
        self.world = world
        self.next_id = 100
        self.batches: list[list[Any]] = []
        self.destroyed: list[int] = []

    fail_first_walkers: int = 0  # walker spawns that collide before one succeeds

    def apply_batch_sync(self, batch: list[Any], due_tick_cue: bool) -> list[_Response]:
        assert due_tick_cue is False  # the reset must not tick outside the gate
        self.batches.append(batch)
        out = []
        for cmd in batch:
            if isinstance(cmd, _DestroyActor):
                self.destroyed.append(int(cmd.actor_id))
                out.append(_Response(int(cmd.actor_id)))
            elif (
                isinstance(cmd, _SpawnActor)
                and cmd.blueprint.id.startswith("walker.")
                and self.fail_first_walkers > 0
            ):
                self.fail_first_walkers -= 1
                out.append(
                    _Response(0, "Spawn failed because of collision at spawn position")
                )
            else:
                self.next_id += 1
                out.append(_Response(self.next_id))
        return out


class _Tm:
    def __init__(self) -> None:
        self.calls: list[tuple[str, Any]] = []

    def __getattr__(self, name: str) -> Any:
        def record(*args: Any) -> None:
            self.calls.append((name, args))

        return record


class _Log:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, msg: str) -> None:
        self.lines.append(msg)

    def warning(self, msg: str) -> None:
        self.lines.append("W " + msg)


def test_spawn_points_are_seeded_and_keep_clear_of_the_hero() -> None:
    world = _World()
    points = world.get_map().get_spawn_points()
    a = tr.pick_spawn_points(points, (0.0, 0.0), 6, random.Random(3))
    b = tr.pick_spawn_points(points, (0.0, 0.0), 6, random.Random(3))
    c = tr.pick_spawn_points(points, (0.0, 0.0), 6, random.Random(4))
    assert [p.location.x for p in a] == [p.location.x for p in b]
    assert [(p.location.x, p.location.y) for p in a] != [
        (p.location.x, p.location.y) for p in c
    ]
    assert len(a) == 6
    assert all(p.location.x**2 + p.location.y**2 >= tr.HERO_CLEARANCE_M**2 for p in a)
    # Asking for more than the map holds returns what is free.
    assert len(tr.pick_spawn_points(points, (0.0, 0.0), 100, random.Random(0))) == 24


def _spawner(
    n_vehicles: int, n_walkers: int
) -> tuple[tr.TrafficSpawner, _Client, _World, _Tm]:
    world = _World()
    client = _Client(world)
    tm = _Tm()
    spawner = tr.TrafficSpawner(
        cast(carla.Client, client),
        cast(carla.World, world),
        cast(carla.TrafficManager, tm),
        tr.TrafficParams(
            n_vehicles=n_vehicles,
            n_walkers=n_walkers,
            tm_port=8000,
            hybrid_physics=False,
        ),
        _Log(),
    )
    return spawner, client, world, tm


def test_spawn_seeds_every_generator_and_uses_four_wheelers() -> None:
    spawner, client, world, tm = _spawner(4, 2)
    spawner.spawn(7, (0.0, 0.0))
    assert ("set_random_device_seed", (7,)) in tm.calls
    assert ("set_hybrid_physics_mode", (False,)) in tm.calls
    assert (
        "set_global_distance_to_leading_vehicle",
        (tr.LEADING_VEHICLE_DISTANCE_M,),
    ) in tm.calls
    assert world.seed == 7
    assert len(spawner.vehicle_ids) == 4
    assert len(spawner.walker_ids) == 2
    # The vehicle batch chains autopilot; only four-wheelers were offered.
    vehicles, walkers, controllers = client.batches
    assert len(vehicles) == 4
    assert len(walkers) == 2
    assert len(controllers) == 2
    assert all(
        bp.set["role_name"] == tr.VEHICLE_ROLE
        for bp in world.library.filter("vehicle.")
        if bp.set
    )
    assert not world.library.find("vehicle.bh.crossbike").set
    # Lights are obeyed by every vehicle.
    assert sum(1 for name, _ in tm.calls if name == "ignore_lights_percentage") == 4
    with pytest.raises(RuntimeError):
        spawner.spawn(7, (0.0, 0.0))


def test_walkers_start_after_the_next_tick_only() -> None:
    spawner, client, world, _tm = _spawner(0, 3)
    client.fail_first_walkers = 2  # collided draws are re-sampled
    spawner.spawn(1, (0.0, 0.0))
    assert len(spawner.walker_ids) == 3
    controllers = list(world.actors.values())
    assert not any(a.calls for a in controllers)
    spawner.after_tick()
    started = [a for a in world.actors.values() if ("start", None) in a.calls]
    assert len(started) == 3
    for actor in started:
        names = [c[0] for c in actor.calls]
        assert names == ["start", "go_to", "speed"]
    spawner.after_tick()  # idempotent
    assert all(len(a.calls) == 3 for a in started)


def test_clear_destroys_exactly_the_owned_actors() -> None:
    spawner, client, world, _tm = _spawner(3, 2)
    spawner.spawn(5, (0.0, 0.0))
    spawner.after_tick()
    owned = set(spawner.vehicle_ids) | set(spawner.walker_ids)
    controllers = {a.id for a in world.actors.values() if ("start", None) in a.calls}
    spawner.clear()
    assert set(client.destroyed) == owned | controllers
    assert all(("stop", None) in world.actors[c].calls for c in controllers)
    assert spawner.vehicle_ids == ()
    assert spawner.walker_ids == ()
    # Cleared, it can spawn again with another seed.
    spawner.spawn(6, (0.0, 0.0))
    assert len(spawner.vehicle_ids) == 3
