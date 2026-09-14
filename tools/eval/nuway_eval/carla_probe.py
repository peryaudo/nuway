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
import time
from collections.abc import Callable, Mapping, Sequence
from typing import Any

import carla

from nuway_eval.infractions import CollisionEvent, StopLine, StopSignVolume
from nuway_ml.common.carla_conv import location_from_ros, location_to_ros, yaw_to_ros

HERO_ROLE = "hero"


# The Leaderboard scales a stop volume's extents by this in its box test.
STOP_BOX_SCALE = 1.2


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
        # (other id, type, other speed, bearing from the hero in degrees or None)
        self._hits: list[tuple[int, str, float, float | None]] = []

    # ----------------------------------------------------------------- map
    def lane_yaw_at(self, x: float, y: float, z: float = 0.0) -> float | None:
        """ROS yaw of the driving lane under a ROS map point; None off the road.

        The route XML carries positions only, and the heading toward the
        next waypoint is wrong wherever the road bends between them: the
        dev03_04 start sits at a junction whose lane runs 62 deg from the
        chord to a waypoint 43 m away, so the hero spawned across its lane,
        no lane matched its heading and the route planner rerouted forever
        (protocol v6). The Leaderboard spawns on the map waypoint's
        transform; this is the same lookup.
        """
        import numpy as np  # noqa: PLC0415  # only this method needs it

        loc = location_from_ros(np.array([x, y, z]))  # a plain struct, not carla's
        wp = self._world.get_map().get_waypoint(
            carla.Location(x=loc.x, y=loc.y, z=loc.z),
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if wp is None:
            return None
        return float(yaw_to_ros(wp.transform.rotation.yaw))

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
        bearing: float | None = None
        if other is not None:
            type_id = str(other.type_id)
            other_id = int(other.id)
            try:
                v = other.get_velocity()
                speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                # Where the other actor is, seen from the hero: the criteria
                # attribute a hit from behind to the actor that drove into
                # the ego. Computed in the map frame (left positive).
                hero = self._hero.get_transform() if self._hero is not None else None
                if hero is not None:
                    ego = location_to_ros(hero.location)
                    oth = location_to_ros(other.get_location())
                    rel = math.atan2(oth[1] - ego[1], oth[0] - ego[0]) - yaw_to_ros(
                        float(hero.rotation.yaw)
                    )
                    bearing = math.degrees(math.atan2(math.sin(rel), math.cos(rel)))
            except RuntimeError:
                speed = 0.0
        with self._lock:
            self._hits.append((other_id, type_id, speed, bearing))

    def drain_collisions(self, visible: Callable[[int], bool]) -> list[CollisionEvent]:
        """Hits since the last call; ``visible(id)`` answers from the GT agent list."""
        with self._lock:
            hits, self._hits = self._hits, []
        return [
            CollisionEvent(other_id, type_id, speed, visible(other_id), bearing)
            for other_id, type_id, speed, bearing in hits
        ]

    # ---------------------------------------------------------------- town
    def cache_town(self, attempts: int = 20) -> None:
        """Stop lines of every light and the trigger volume of every stop sign.

        A client that has just connected sees no actors until the server's
        episode state reaches it, and one query more than a minute after the
        launch still came back empty on the dev box, which left a whole town
        without stop-sign checks (protocol v7 check). So poll until the world
        reports actors (a town always has its traffic lights) and fail loudly
        otherwise: a silent empty cache disables the checks it feeds. Polling
        with sleeps, not ``wait_for_tick``: that call leaves a worker thread
        in the client whose teardown aborted the harness at the next town.
        """
        actors = self._world.get_actors()
        for _ in range(attempts):
            if len(actors) > 0:
                break
            time.sleep(0.5)
            actors = self._world.get_actors()
        if len(actors) == 0:
            raise RuntimeError("the world reports no actors; is the server ticking?")
        self._lights = list(actors.filter("traffic.traffic_light"))
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
        for sign in actors.filter("traffic.stop"):
            # The Leaderboard's RunningStopTest tests the volume's centre
            # against an axis-aligned box of 1.2x its extents (the volume's
            # rotation is ignored), so the harness builds the same polygon.
            box = sign.trigger_volume
            transform = sign.get_transform()
            centre = location_to_ros(transform.transform(box.location))
            cx, cy = float(centre[0]), float(centre[1])
            hx = STOP_BOX_SCALE * float(box.extent.x)
            hy = STOP_BOX_SCALE * float(box.extent.y)
            corners = tuple(
                (cx + sx * hx, cy + sy * hy)
                for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))
            )
            signs.append(StopSignVolume(int(sign.id), corners, (cx, cy)))
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
