"""Leaderboard 2.0 driving score (M1 §3.10) from ``configs/eval/scoring_lb20.yaml``.

The Leaderboard scores a route as ``route_completion * prod(penalty_i ** n_i)``:
completion is the share of the reference line the hero reached, every
counted infraction multiplies the score by its coefficient, the share of
the route driven off the route lanes scales the completion, and the three
route-ending conditions (deviation, blocked, timeout) need no coefficient
because the truncated completion already carries them. The coefficients
live in a YAML file so that an ``lb21`` variant is a sibling file, not a
code change.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import yaml

PENALTY_KINDS = (
    "collision_pedestrian",
    "collision_vehicle",
    "collision_layout",
    "red_light",
    "stop_sign",
    "min_speed",
)


@dataclass(frozen=True, slots=True)
class ScoringConfig:
    """The coefficients of one scoring variant."""

    name: str
    penalties: Mapping[str, float]
    blocked_speed_mps: float = 0.1
    blocked_s: float = 90.0
    route_deviation_m: float = 30.0
    timeout_speed_mps: float = 5.0
    timeout_factor: float = 2.0
    timeout_base_s: float = 60.0
    collision_cooldown_s: float = 2.0
    stop_sign_speed_mps: float = 0.2
    min_speed_ratio: float = 1.0
    min_speed_radius_m: float = 50.0
    min_speed_checkpoints: int = 1

    @classmethod
    def load(cls, path: Path | str) -> ScoringConfig:
        """Read a scoring YAML; every penalty kind must be present."""
        with Path(path).open() as f:
            raw: dict[str, Any] = yaml.safe_load(f) or {}
        penalties = {k: float(v) for k, v in dict(raw.get("penalties", {})).items()}
        missing = [k for k in PENALTY_KINDS if k not in penalties]
        if missing:
            msg = f"{path}: missing penalties {missing}"
            raise ValueError(msg)
        numbers: dict[str, Any] = {}
        for section in ("ending", "detectors"):
            for k, v in dict(raw.get(section, {})).items():
                numbers[str(k)] = float(v)
        if "min_speed_checkpoints" in numbers:
            numbers["min_speed_checkpoints"] = int(numbers["min_speed_checkpoints"])
        known = {f.name for f in fields(cls)} - {"name", "penalties"}
        unknown = sorted(set(numbers) - known)
        if unknown:
            msg = f"{path}: unknown keys {unknown}"
            raise ValueError(msg)
        return cls(
            name=str(raw.get("name", Path(path).stem)), penalties=penalties, **numbers
        )

    def route_timeout_s(self, route_length_m: float) -> float:
        """Return the per-route sim-time budget: length / 5 m/s * 2 + 60 s by default."""
        return (
            route_length_m / self.timeout_speed_mps * self.timeout_factor
            + self.timeout_base_s
        )


@dataclass(slots=True)
class InfractionCounts:
    """What a route run accumulated; the ``results.csv`` infraction columns."""

    n_collision_pedestrian: int = 0
    n_collision_vehicle: int = 0
    n_collision_layout: int = 0
    n_red_light: int = 0
    n_stop_sign: int = 0
    n_min_speed: int = 0
    min_speed_pcts: list[float] = field(
        default_factory=list
    )  # one per failed checkpoint
    outside_lanes_frac: float = 0.0
    route_deviation: bool = False
    blocked: bool = False
    timeout: bool = False
    incidents: list[tuple[int, str]] = field(default_factory=list)  # (tick, kind)

    def count(self, kind: str) -> int:
        """Occurrences of a penalty kind."""
        return int(getattr(self, f"n_{kind}"))

    def add(self, tick: int, kind: str) -> None:
        """Count one infraction of ``kind`` at ``tick``."""
        setattr(self, f"n_{kind}", self.count(kind) + 1)
        self.incidents.append((tick, kind))

    def add_min_speed(self, tick: int, pct: float) -> None:
        """Record a failed min-speed checkpoint: ego speed as ``pct`` of the traffic's."""
        self.min_speed_pcts.append(pct)
        self.add(tick, "min_speed")

    @property
    def min_speed_pct(self) -> float:
        """The Leaderboard's reported value: mean over the checkpoints, 100 when none failed."""
        if not self.min_speed_pcts:
            return 100.0
        return sum(self.min_speed_pcts) / len(self.min_speed_pcts)


def driving_score(
    completion: float, counts: InfractionCounts, config: ScoringConfig
) -> float:
    """Leaderboard 2.0 score in [0, 100].

    ``100 * completion * (1 - outside_lanes_frac) * Π penalty^n`` for the counted
    kinds; a min-speed checkpoint is the Leaderboard's *percentage* event, whose
    factor ``1 - (1 - penalty) * (1 - pct/100)`` runs from the coefficient at
    0 % of the traffic's speed to 1 at parity.
    """
    score = 100.0 * min(1.0, max(0.0, completion))
    score *= 1.0 - min(1.0, max(0.0, counts.outside_lanes_frac))
    for kind in PENALTY_KINDS:
        if kind != "min_speed":
            score *= config.penalties[kind] ** counts.count(kind)
    for pct in counts.min_speed_pcts:
        shortfall = 1.0 - min(100.0, max(0.0, pct)) / 100.0
        score *= 1.0 - (1.0 - config.penalties["min_speed"]) * shortfall
    return score
