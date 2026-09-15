"""Seeded NPC traffic for ``world_manager`` (M1 §3.9).

Vehicles run under CARLA's Traffic Manager (TM): a server-side autopilot
that plans lane following, lane changes and light compliance for every
vehicle registered with it, stepped once per ``world.tick()`` in
synchronous mode. Walkers run under ``controller.ai.walker``: a per-walker
navigation-mesh agent that walks to a target location at a set speed.

Determinism (M1 §5). CARLA reproduces a traffic episode only when every
random decision is seeded: the TM's own generator
(``set_random_device_seed``), the walker navigation sampling
(``World.set_pedestrians_seed``) and the *client-side* choices this module
makes -- which blueprint, which colour, which spawn point, which walking
speed -- which all come from one ``random.Random(seed)``. Hybrid physics
(TM vehicles far from the hero teleport instead of simulating) is kept off
because it trades that reproducibility for speed.

Bookkeeping. The spawner remembers the ids of the actors *it* spawned and
destroys exactly those on ``clear()``; it never scans the world by type,
so an actor another client attached (the harness's collision sensor, M1
§3.10) survives every reset.

Every position here is CARLA's convention; nothing is converted (only
``carla_conv`` may do that, docs/01 Rules), the module only compares CARLA
locations with each other.
"""

from __future__ import annotations

import random
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any, Protocol

import carla

VEHICLE_ROLE = "autopilot"
# Spawn points closer than this to the hero stay empty: the hero is
# teleported to a base_link pose, not a spawn point, and a vehicle spawned
# across its bumper would be an instant collision.
HERO_CLEARANCE_M = 12.0
LEADING_VEHICLE_DISTANCE_M = 2.5
WALKER_RUN_FRACTION = 0.2  # the rest walk
WALKER_SPAWN_ROUNDS = 4  # re-sampling rounds for walkers that collided at spawn


@dataclass(frozen=True, slots=True)
class TrafficParams:
    """The profile's ``carla.traffic`` block (docs/02 §5)."""

    n_vehicles: int
    n_walkers: int
    tm_port: int
    hybrid_physics: bool


class _Logger(Protocol):
    def info(self, msg: str) -> None: ...

    def warning(self, msg: str) -> None: ...


def pick_spawn_points(
    points: Sequence[Any],
    hero_xy: tuple[float, float],
    count: int,
    rng: random.Random,
    clearance_m: float = HERO_CLEARANCE_M,
) -> list[Any]:
    """``count`` spawn transforms in seeded order, none within ``clearance_m`` of the hero.

    Pure so the selection can be tested: the order is ``rng``'s shuffle of
    the map's spawn points, which CARLA returns in a fixed order per town.
    """
    order = list(points)
    rng.shuffle(order)
    kept = []
    for tf in order:
        dx = float(tf.location.x) - hero_xy[0]
        dy = float(tf.location.y) - hero_xy[1]
        if dx * dx + dy * dy >= clearance_m * clearance_m:
            kept.append(tf)
        if len(kept) == count:
            break
    return kept


class TrafficSpawner:
    """Spawns, starts and destroys the NPC vehicles and walkers of one episode."""

    def __init__(
        self,
        client: carla.Client,
        world: carla.World,
        traffic_manager: carla.TrafficManager,
        params: TrafficParams,
        logger: _Logger,
    ) -> None:
        self._client = client
        self._world = world
        self._tm = traffic_manager
        self._params = params
        self._log = logger
        self._vehicle_ids: list[int] = []
        self._walker_ids: list[int] = []
        self._controller_ids: list[int] = []
        self._walker_speeds: list[float] = []
        self._walkers_pending_start = False
        self._tm.set_hybrid_physics_mode(params.hybrid_physics)
        self._tm.set_global_distance_to_leading_vehicle(LEADING_VEHICLE_DISTANCE_M)

    @property
    def vehicle_ids(self) -> tuple[int, ...]:
        """Ids of the vehicles this spawner owns."""
        return tuple(self._vehicle_ids)

    @property
    def walker_ids(self) -> tuple[int, ...]:
        """Ids of the walkers this spawner owns."""
        return tuple(self._walker_ids)

    # ---------------------------------------------------------------- spawn
    def spawn(self, seed: int, hero_xy: tuple[float, float]) -> None:
        """Spawn the profile's traffic with ``seed``; the hero's vicinity stays empty.

        Vehicles are spawned in one batch with ``SetAutopilot`` chained on
        the future actor, so a vehicle exists under the TM from its first
        frame and never idles a tick. Walkers are spawned now but their
        controllers only start on the next tick (:meth:`after_tick`): the
        controller needs the server to have placed the walker first, and
        the reset must not tick the world outside the lockstep gate.
        """
        if self._vehicle_ids or self._walker_ids:
            raise RuntimeError("traffic already spawned; clear() first")
        rng = random.Random(seed)
        self._tm.set_random_device_seed(seed)
        self._world.set_pedestrians_seed(seed)
        self._spawn_vehicles(rng, hero_xy)
        self._spawn_walkers(rng)
        self._log.info(
            f"traffic seed {seed}: {len(self._vehicle_ids)}/{self._params.n_vehicles} "
            f"vehicles, {len(self._walker_ids)}/{self._params.n_walkers} walkers"
        )

    def _spawn_vehicles(self, rng: random.Random, hero_xy: tuple[float, float]) -> None:
        """Four-wheelers only: the TM drives bikes into everything."""
        if self._params.n_vehicles <= 0:
            return
        library = self._world.get_blueprint_library()
        blueprints = [
            bp
            for bp in library.filter("vehicle.*")
            if int(bp.get_attribute("number_of_wheels").as_int()) == 4
        ]
        blueprints.sort(
            key=lambda bp: str(bp.id)
        )  # the library's order is not promised
        points = pick_spawn_points(
            self._world.get_map().get_spawn_points(),
            hero_xy,
            self._params.n_vehicles,
            rng,
        )
        if len(points) < self._params.n_vehicles:
            self._log.warning(
                f"only {len(points)} free spawn points for {self._params.n_vehicles} vehicles"
            )
        batch = []
        for tf in points:
            bp = rng.choice(blueprints)
            if bp.has_attribute("color"):
                bp.set_attribute(
                    "color", rng.choice(bp.get_attribute("color").recommended_values)
                )
            if bp.has_attribute("driver_id"):
                bp.set_attribute(
                    "driver_id",
                    rng.choice(bp.get_attribute("driver_id").recommended_values),
                )
            bp.set_attribute("role_name", VEHICLE_ROLE)
            batch.append(
                carla.command.SpawnActor(bp, tf).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True, self._params.tm_port
                    )
                )
            )
        for response in self._client.apply_batch_sync(batch, False):
            if response.error:
                self._log.warning(f"vehicle spawn failed: {response.error}")
                continue
            self._vehicle_ids.append(int(response.actor_id))
        for actor in self._world.get_actors(self._vehicle_ids):
            self._tm.ignore_lights_percentage(actor, 0.0)
            self._tm.update_vehicle_lights(actor, True)

    def _spawn_walkers(self, rng: random.Random) -> None:
        """Walkers at seeded navigation-mesh locations; controllers attached, not started.

        A sampled location is often already occupied (two thirds of a
        30-walker batch collided on Town03), so the shortfall is re-sampled
        for a few rounds; every draw still comes from the seeded sampler.
        """
        if self._params.n_walkers <= 0:
            return
        library = self._world.get_blueprint_library()
        blueprints = sorted(
            library.filter("walker.pedestrian.*"), key=lambda bp: str(bp.id)
        )
        for _round in range(WALKER_SPAWN_ROUNDS):
            missing = self._params.n_walkers - len(self._walker_ids)
            if missing <= 0:
                break
            batch = []
            speeds = []
            for _ in range(missing):
                location = self._world.get_random_location_from_navigation()
                if location is None:
                    continue
                bp = rng.choice(blueprints)
                if bp.has_attribute("is_invincible"):
                    bp.set_attribute("is_invincible", "false")
                speed_values = (
                    bp.get_attribute("speed").recommended_values
                    if bp.has_attribute("speed")
                    else ["0.0", "1.4", "2.8"]
                )
                running = rng.random() < WALKER_RUN_FRACTION
                speeds.append(float(speed_values[2 if running else 1]))
                batch.append(carla.command.SpawnActor(bp, carla.Transform(location)))
            failed = 0
            for response, speed in zip(
                self._client.apply_batch_sync(batch, False), speeds, strict=True
            ):
                if response.error:
                    failed += 1
                    continue
                self._walker_ids.append(int(response.actor_id))
                self._walker_speeds.append(speed)
            if failed:
                self._log.info(
                    f"{failed} of {len(batch)} walker spawns collided; re-sampling"
                )
        controller_bp = library.find("controller.ai.walker")
        batch = [
            carla.command.SpawnActor(controller_bp, carla.Transform(), walker_id)
            for walker_id in self._walker_ids
        ]
        for response in self._client.apply_batch_sync(batch, False):
            if response.error:
                self._log.warning(f"walker controller spawn failed: {response.error}")
                self._controller_ids.append(0)
                continue
            self._controller_ids.append(int(response.actor_id))
        self._walkers_pending_start = True

    def after_tick(self) -> None:
        """Start the walker controllers once the server has placed the walkers.

        Called by the tick loop after every ``world.tick()``; a no-op unless
        walkers were spawned since the last call. The target location comes
        from the seeded navigation sampler, so two episodes with one seed
        send every walker the same way.
        """
        if not self._walkers_pending_start:
            return
        self._walkers_pending_start = False
        controllers = {
            int(actor.id): actor
            for actor in self._world.get_actors([c for c in self._controller_ids if c])
        }
        for controller_id, speed in zip(
            self._controller_ids, self._walker_speeds, strict=True
        ):
            controller = controllers.get(controller_id)
            if controller is None:
                continue
            controller.start()
            target = self._world.get_random_location_from_navigation()
            if target is not None:
                controller.go_to_location(target)
            controller.set_max_speed(speed)

    # ---------------------------------------------------------------- clear
    def clear(self) -> None:
        """Destroy every actor this spawner spawned, and nothing else.

        Controllers are stopped and destroyed before their walkers (a
        controller outliving its walker keeps steering a ghost), then the
        walkers, then the vehicles, all in batches without a tick.
        """
        controller_ids = [c for c in self._controller_ids if c]
        for actor in self._world.get_actors(controller_ids):
            try:
                actor.stop()
            except RuntimeError as err:
                self._log.warning(f"stopping walker controller failed: {err}")
        for ids in (controller_ids, self._walker_ids, self._vehicle_ids):
            if not ids:
                continue
            batch = [carla.command.DestroyActor(actor_id) for actor_id in ids]
            for response in self._client.apply_batch_sync(batch, False):
                if response.error:
                    self._log.warning(
                        f"destroying traffic actor failed: {response.error}"
                    )
        self._vehicle_ids.clear()
        self._walker_ids.clear()
        self._controller_ids.clear()
        self._walker_speeds.clear()
        self._walkers_pending_start = False
