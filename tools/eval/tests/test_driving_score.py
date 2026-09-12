"""driving_score: the Leaderboard 2.0 formula and the scoring YAML."""

from __future__ import annotations

from pathlib import Path

import pytest

from nuway_eval.driving_score import (
    PENALTY_KINDS,
    InfractionCounts,
    ScoringConfig,
    driving_score,
)

ROOT = Path(__file__).resolve().parents[3]


def test_lb20_config_loads_every_kind() -> None:
    cfg = ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")
    assert cfg.name == "lb20"
    assert set(PENALTY_KINDS) <= set(cfg.penalties)
    assert cfg.penalties["collision_pedestrian"] == 0.5
    assert cfg.route_timeout_s(1000.0) == pytest.approx(1000.0 / 5.0 * 2.0 + 60.0)
    assert cfg.blocked_s == 180.0


def test_score_multiplies_penalties_and_scales_by_outside_lanes() -> None:
    cfg = ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")
    counts = InfractionCounts()
    assert driving_score(1.0, counts, cfg) == pytest.approx(100.0)
    counts.add(10, "red_light")
    counts.add(20, "collision_vehicle")
    counts.add(30, "collision_vehicle")
    assert counts.n_collision_vehicle == 2
    assert counts.incidents == [
        (10, "red_light"),
        (20, "collision_vehicle"),
        (30, "collision_vehicle"),
    ]
    assert driving_score(0.5, counts, cfg) == pytest.approx(50.0 * 0.7 * 0.6 * 0.6)
    counts.outside_lanes_frac = 0.25
    assert driving_score(0.5, counts, cfg) == pytest.approx(50.0 * 0.75 * 0.7 * 0.36)
    # Completion is clipped to [0, 1] and a blocked run keeps its truncated completion.
    counts.blocked = True
    assert driving_score(1.3, InfractionCounts(), cfg) == pytest.approx(100.0)


def test_min_speed_scales_its_penalty_by_the_percentage() -> None:
    cfg = ScoringConfig.load(ROOT / "configs/eval/scoring_lb20.yaml")
    counts = InfractionCounts()
    counts.add_min_speed(500, 40.0)  # ego at 40 % of the traffic's speed
    assert counts.n_min_speed == 1
    assert counts.min_speed_pct == 40.0
    # 1 - (1 - 0.7) * (1 - 0.4) = 0.82
    assert driving_score(1.0, counts, cfg) == pytest.approx(82.0)
    counts.add_min_speed(900, 0.0)  # a second checkpoint at a standstill: the full 0.7
    assert counts.min_speed_pct == pytest.approx(20.0)
    assert driving_score(1.0, counts, cfg) == pytest.approx(82.0 * 0.7)


def test_missing_or_unknown_keys_are_errors(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("penalties: {red_light: 0.7}\n")
    with pytest.raises(ValueError, match="missing penalties"):
        ScoringConfig.load(bad)
    bad.write_text(
        "penalties: {collision_pedestrian: 0.5, collision_vehicle: 0.6, collision_layout: 0.65, "
        "red_light: 0.7, stop_sign: 0.8, min_speed: 0.7}\nending: {typo: 1}\n"
    )
    with pytest.raises(ValueError, match="unknown keys"):
        ScoringConfig.load(bad)
