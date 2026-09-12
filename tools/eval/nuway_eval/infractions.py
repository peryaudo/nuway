"""Infraction detectors of the M1 harness (M1 §3.10), pure and tick-driven.

The route runner builds one :class:`TickObservation` per tick from its own
non-ticking CARLA client (collision sensor, traffic-light stop waypoints,
stop-sign trigger volumes) and from the stack's own topics (ego odometry,
the reference line, GT agents), and feeds it to :class:`InfractionTracker`.
Everything geometric here is in the ROS map frame: the runner converts
CARLA's stop waypoints and trigger volumes once through
``nuway_ml.common.carla_conv`` when it caches them for a town.

The rules mirror the Leaderboard 2.0 criteria so that our harness and the
official runner (M1 §3.12) count the same things:

- collision: one per other actor per cooldown, classed by the other
  actor's blueprint (walker / vehicle / anything else is layout);
- red light: the hero's reference point crosses a stop line, within the
  lane's half width of it, while the light is red;
- stop sign: the hero enters a trigger volume and leaves it without ever
  falling below the honouring speed;
- outside lanes: the share of the driven distance with the hero's centre
  beyond the drivable edge on its side of the reference line;
- route deviation, blocked and timeout end the route;
- min speed (the Leaderboard's ``MinSpeedTest``): over a sliding window
  the hero averaged less than a fraction of the mean speed of the moving
  vehicles around it.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

from nuway_eval.driving_score import InfractionCounts, ScoringConfig


@dataclass(frozen=True, slots=True)
class CollisionEvent:
    """One hit reported by the hero's collision sensor."""

    other_id: int
    other_type: (
        str  # CARLA blueprint id, e.g. "vehicle.audi.a2", "walker.pedestrian.0001"
    )
    other_speed_mps: float
    visible: bool  # the other actor was in the GT agent list with visible = True

    @property
    def kind(self) -> str:
        """Penalty kind of the collision."""
        if self.other_type.startswith("walker."):
            return "collision_pedestrian"
        if self.other_type.startswith("vehicle."):
            return "collision_vehicle"
        return "collision_layout"


@dataclass(frozen=True, slots=True)
class StopLine:
    """A traffic light's stop waypoint: a point on the lane with the lane's direction."""

    light_id: int
    x: float
    y: float
    heading: float  # driving direction across the line, rad
    half_width_m: float


@dataclass(frozen=True, slots=True)
class StopSignVolume:
    """A stop sign's trigger volume as a map-frame polygon."""

    sign_id: int
    polygon: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class TickObservation:
    """Everything the detectors need about one tick."""

    k: int
    t: float  # sim seconds
    x: float
    y: float
    speed_mps: float
    # Projection on the reference line; None when farther than the deviation limit.
    s: float | None
    d: float | None  # left positive
    left_bound_m: float  # drivable edge distances at s (positive)
    right_bound_m: float
    red_stop_lines: Sequence[StopLine] = ()  # stop lines of lights that are red now
    stop_signs: Sequence[StopSignVolume] = ()
    collisions: Sequence[CollisionEvent] = ()
    # Speeds of the traffic vehicles within the min-speed radius, stopped ones
    # included: the Leaderboard averages over all of its background vehicles.
    traffic_speeds_mps: Sequence[float] = ()


def point_in_polygon(
    x: float, y: float, polygon: Sequence[tuple[float, float]]
) -> bool:
    """Even-odd rule."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


@dataclass
class _LineState:
    along_m: float | None = None  # signed distance past the line at the last tick
    fired: bool = False


@dataclass
class _SignState:
    inside: bool = False
    honoured: bool = False


@dataclass
class InfractionTracker:
    """Feeds ticks in order; ``counts`` is the running result."""

    config: ScoringConfig
    route_length_m: float
    counts: InfractionCounts = field(default_factory=InfractionCounts)
    ended: str = (
        ""  # "" while the route runs; "route_deviation" | "blocked" | "timeout"
    )
    _t0: float | None = None
    _last: TickObservation | None = None
    _driven_m: float = 0.0
    _outside_m: float = 0.0
    _cooldowns: dict[int, float] = field(default_factory=dict)  # other id -> until t
    _lines: dict[int, _LineState] = field(default_factory=dict)
    _signs: dict[int, _SignState] = field(default_factory=dict)
    _slow_since: float | None = None
    # Min-speed accumulators of the current checkpoint (LB2 MinimumSpeedRouteTest).
    _max_s: float = 0.0
    _checkpoint_start_s: float = 0.0
    _ego_speed_sum: float = 0.0
    _traffic_speed_sum: float = 0.0
    _speed_points: int = 0

    def update(self, obs: TickObservation) -> list[str]:
        """Consume one tick; returns the kinds that fired on it."""
        fired: list[str] = []
        if self._t0 is None:
            self._t0 = obs.t
        self._update_distance(obs)
        fired += self._collisions(obs)
        fired += self._red_lights(obs)
        fired += self._stop_signs(obs)
        fired += self._min_speed(obs)
        fired += self._ending(obs)
        self._last = obs
        return fired

    # ------------------------------------------------------------- detectors
    def _update_distance(self, obs: TickObservation) -> None:
        if self._last is not None:
            step = math.hypot(obs.x - self._last.x, obs.y - self._last.y)
            self._driven_m += step
            outside = obs.d is None or (
                obs.d > obs.left_bound_m if obs.d >= 0.0 else -obs.d > obs.right_bound_m
            )
            if outside:
                self._outside_m += step
        if self._driven_m > 0.0:
            self.counts.outside_lanes_frac = self._outside_m / self._driven_m

    def _collisions(self, obs: TickObservation) -> list[str]:
        fired: list[str] = []
        for hit in obs.collisions:
            until = self._cooldowns.get(hit.other_id, -math.inf)
            if obs.t < until:
                continue
            self._cooldowns[hit.other_id] = obs.t + self.config.collision_cooldown_s
            self.counts.add_collision(
                obs.k, hit.kind, hit.other_type, hit.other_speed_mps, hit.visible
            )
            fired.append(hit.kind)
        return fired

    def _red_lights(self, obs: TickObservation) -> list[str]:
        """Count a crossing: the signed along-line distance turning non-negative."""
        fired: list[str] = []
        seen: set[int] = set()
        for line in obs.red_stop_lines:
            seen.add(line.light_id)
            cos_h, sin_h = math.cos(line.heading), math.sin(line.heading)
            dx, dy = obs.x - line.x, obs.y - line.y
            along = dx * cos_h + dy * sin_h
            across = -dx * sin_h + dy * cos_h
            state = self._lines.setdefault(line.light_id, _LineState())
            if abs(across) > line.half_width_m:
                state.along_m = None
                continue
            crossed = state.along_m is not None and state.along_m < 0.0 <= along
            if crossed and not state.fired:
                state.fired = True
                self.counts.add(obs.k, "red_light")
                fired.append("red_light")
            state.along_m = along
        # A light that is no longer red (or out of range) may fire again later.
        for light_id in [i for i in self._lines if i not in seen]:
            del self._lines[light_id]
        return fired

    def _stop_signs(self, obs: TickObservation) -> list[str]:
        fired: list[str] = []
        present: set[int] = set()
        for sign in obs.stop_signs:
            present.add(sign.sign_id)
            state = self._signs.setdefault(sign.sign_id, _SignState())
            inside = point_in_polygon(obs.x, obs.y, sign.polygon)
            if inside:
                state.inside = True
                if obs.speed_mps < self.config.stop_sign_speed_mps:
                    state.honoured = True
            elif state.inside:
                if not state.honoured:
                    self.counts.add(obs.k, "stop_sign")
                    fired.append("stop_sign")
                del self._signs[sign.sign_id]
        for sign_id in [i for i in self._signs if i not in present]:
            del self._signs[sign_id]
        return fired

    def _min_speed(self, obs: TickObservation) -> list[str]:
        """Port of the Leaderboard 2.0 ``MinimumSpeedRouteTest``.

        The route is cut into ``min_speed_checkpoints`` equal arc-length
        segments. Over each one the ego's mean speed and the mean speed of the
        traffic around it (over the ticks that had any) are accumulated, and a
        checkpoint whose ego/traffic ratio is below ``min_speed_ratio`` records
        one infraction carrying the ratio in percent — ``driving_score`` scales
        the penalty by it, so a slightly slow route costs little and a crawl
        costs the full coefficient. Like the Leaderboard, the check runs before
        the tick's speeds are added.
        """
        fired: list[str] = []
        if obs.s is not None:
            self._max_s = max(self._max_s, obs.s)
        checkpoint_m = self.route_length_m / max(1, self.config.min_speed_checkpoints)
        if self._max_s - self._checkpoint_start_s > checkpoint_m:
            fired += self._min_speed_checkpoint(obs.k)
            self._checkpoint_start_s = self._max_s
        traffic = list(obs.traffic_speeds_mps)
        if traffic:
            self._ego_speed_sum += obs.speed_mps
            self._traffic_speed_sum += sum(traffic) / len(traffic)
            self._speed_points += 1
        return fired

    def _min_speed_checkpoint(self, k: int) -> list[str]:
        """Close the current checkpoint; 100 % when it saw no traffic."""
        pct = 100.0
        if self._speed_points > 0 and self._traffic_speed_sum > 0.0:
            ego_mean = self._ego_speed_sum / self._speed_points
            traffic_mean = self._traffic_speed_sum / self._speed_points
            pct = round(
                ego_mean / (self.config.min_speed_ratio * traffic_mean) * 100, 2
            )
        self._ego_speed_sum = self._traffic_speed_sum = 0.0
        self._speed_points = 0
        if pct >= 100.0:
            return []
        self.counts.add_min_speed(k, pct)
        return ["min_speed"]

    def finish(self, k: int) -> list[str]:
        """Close the route at tick ``k``; returns the kinds that fired.

        The Leaderboard's ``terminate`` records the last min-speed checkpoint
        only past 95 % of the route, so a route that ended early (blocked,
        timed out, deviated) carries no partial checkpoint.
        """
        if self.route_length_m > 0.0 and self._max_s / self.route_length_m > 0.95:
            return self._min_speed_checkpoint(k)
        return []

    def _ending(self, obs: TickObservation) -> list[str]:
        if self.ended:
            return []
        assert self._t0 is not None
        if obs.d is None or abs(obs.d) > self.config.route_deviation_m:
            self.counts.route_deviation = True
            self.ended = "route_deviation"
        elif obs.t - self._t0 > self.config.route_timeout_s(self.route_length_m):
            self.counts.timeout = True
            self.ended = "timeout"
        elif obs.speed_mps < self.config.blocked_speed_mps:
            if self._slow_since is None:
                self._slow_since = obs.t
            elif obs.t - self._slow_since >= self.config.blocked_s:
                self.counts.blocked = True
                self.ended = "blocked"
        else:
            self._slow_since = None
        if self.ended:
            self.counts.incidents.append((obs.k, self.ended))
            return [self.ended]
        return []
