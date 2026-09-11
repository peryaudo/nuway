"""The harness's own non-ticking CARLA client (M1 §3.10).

``world_manager`` owns the tick; this second client only *reads* the world
(M0 §2.2 forbids two ticking clients, not two clients): it attaches the
hero's ``sensor.other.collision``, caches every traffic light's stop
waypoints and every stop sign's trigger volume once per town, and reports
the red stop lines each tick. Nothing here changes the world; every world
change goes through the ``/nuway/sim/*`` services so that the Leaderboard
path (M1 §3.12), which has no client at all, sees the same interface.

Geometry leaves this module in the ROS map frame: the CARLA locations and
yaws are converted through ``nuway_ml.common.carla_conv`` (the one place
allowed to know the convention) as they are cached.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import carla

from nuway_eval.infractions import CollisionEvent, StopLine, StopSignVolume
from nuway_ml.common.carla_conv import location_to_ros, yaw_to_ros

HERO_ROLE = "hero"


class CarlaProbe:
    """Read-only view of the world for the infraction detectors."""

    def __init__(self, host: str, port: int, timeout_s: float = 10.0) -> None:
        """Connect; raises ``RuntimeError`` when no server answers."""
        self._client = carla.Client(host, port)
        self._client.set_timeout(timeout_s)
        self._world = self._client.get_world()
        self._hero: Any = None
        self._sensor: Any = None
        self._lights: list[Any] = []
        self._stop_lines: dict[int, tuple[StopLine, ...]] = {}
        self._stop_signs: tuple[StopSignVolume, ...] = ()
        self._lock = threading.Lock()
        self._hits: list[tuple[int, str, float]] = []  # (other id, type, other speed)

    # ---------------------------------------------------------------- hero
    def attach(self) -> bool:
        """Find the hero and attach the collision sensor; False when there is no hero yet.

        The hero persists across resets (``clear_traffic`` only destroys
        the spawner's own actors), so one sensor serves a whole town; a
        relaunched stack spawns a new hero and the sensor is re-attached.
        """
        hero = None
        for actor in self._world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == HERO_ROLE:
                hero = actor
                break
        if hero is None:
            return False
        if (
            self._hero is not None
            and int(self._hero.id) == int(hero.id)
            and self._sensor
        ):
            return True
        self.detach()
        self._hero = hero
        bp = self._world.get_blueprint_library().find("sensor.other.collision")
        self._sensor = self._world.spawn_actor(bp, carla.Transform(), attach_to=hero)
        self._sensor.listen(self._on_collision)
        return True

    def detach(self) -> None:
        """Destroy the collision sensor (the hero is not ours to destroy)."""
        if self._sensor is not None:
            try:
                self._sensor.stop()
                self._sensor.destroy()
            except RuntimeError:
                pass
        self._sensor = None
        self._hero = None

    def _on_collision(self, event: carla.CollisionEvent) -> None:
        other = event.other_actor
        speed = 0.0
        type_id = "unknown"
        other_id = 0
        if other is not None:
            type_id = str(other.type_id)
            other_id = int(other.id)
            try:
                v = other.get_velocity()
                speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            except RuntimeError:
                speed = 0.0
        with self._lock:
            self._hits.append((other_id, type_id, speed))

    def drain_collisions(self, visible: Callable[[int], bool]) -> list[CollisionEvent]:
        """Hits since the last call; ``visible(id)`` answers from the GT agent list."""
        with self._lock:
            hits, self._hits = self._hits, []
        return [
            CollisionEvent(other_id, type_id, speed, visible(other_id))
            for other_id, type_id, speed in hits
        ]

    # ---------------------------------------------------------------- town
    def cache_town(self) -> None:
        """Stop lines of every light and the trigger volume of every stop sign."""
        self._lights = list(self._world.get_actors().filter("traffic.traffic_light"))
        self._stop_lines = {}
        for light in self._lights:
            lines = []
            for wp in light.get_stop_waypoints():
                xyz = location_to_ros(wp.transform.location)
                lines.append(
                    StopLine(
                        light_id=int(light.id),
                        x=float(xyz[0]),
                        y=float(xyz[1]),
                        heading=yaw_to_ros(float(wp.transform.rotation.yaw)),
                        half_width_m=0.5 * float(wp.lane_width),
                    )
                )
            self._stop_lines[int(light.id)] = tuple(lines)
        signs = []
        for sign in self._world.get_actors().filter("traffic.stop"):
            box = sign.trigger_volume
            transform = sign.get_transform()
            corners = []
            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
                local = carla.Location(
                    x=box.location.x + sx * box.extent.x,
                    y=box.location.y + sy * box.extent.y,
                    z=box.location.z,
                )
                world = transform.transform(local)
                xyz = location_to_ros(world)
                corners.append((float(xyz[0]), float(xyz[1])))
            signs.append(StopSignVolume(int(sign.id), tuple(corners)))
        self._stop_signs = tuple(signs)

    @property
    def stop_signs(self) -> tuple[StopSignVolume, ...]:
        """Every stop sign of the town (map frame polygons)."""
        return self._stop_signs

    def red_stop_lines(
        self, near_xy: tuple[float, float], radius_m: float
    ) -> list[StopLine]:
        """Stop lines of the lights that are red now, within ``radius_m`` of the ego."""
        out: list[StopLine] = []
        for light in self._lights:
            lines = self._stop_lines.get(int(light.id), ())
            if not lines:
                continue
            if math.hypot(lines[0].x - near_xy[0], lines[0].y - near_xy[1]) > radius_m:
                continue
            if light.state == carla.TrafficLightState.Red:
                out.extend(lines)
        return out

    def stop_signs_near(
        self, near_xy: tuple[float, float], radius_m: float
    ) -> list[StopSignVolume]:
        """Stop signs whose first corner is within ``radius_m``."""
        return [
            s
            for s in self._stop_signs
            if math.hypot(s.polygon[0][0] - near_xy[0], s.polygon[0][1] - near_xy[1])
            <= radius_m
        ]

    def close(self) -> None:
        """Release the sensor; the client itself needs no teardown."""
        self.detach()


def vehicle_speeds_near(
    agents: Sequence[Mapping[str, float]], near_xy: tuple[float, float], radius_m: float
) -> list[float]:
    """Speeds of the vehicle agents within ``radius_m``, stopped ones included.

    They stand in for the Leaderboard's *background activity* (the vehicles it
    spawns around the ego) in the min-speed test.
    """
    out = []
    for a in agents:
        if a["is_vehicle"] < 0.5:
            continue
        if math.hypot(a["x"] - near_xy[0], a["y"] - near_xy[1]) > radius_m:
            continue
        out.append(math.hypot(a["vx"], a["vy"]))
    return out
