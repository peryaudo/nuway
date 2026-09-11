"""nuway_ml.viz: every layer has a draw function; a fixture scene renders byte-identically."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("matplotlib")

from nuway_ml.viz import bev_draw, style  # after the importorskip
from nuway_ml.viz.contact_sheet import FRAMES_PER_SHEET, tile, write_png
from nuway_ml.viz.panel import header_lines, render_frame
from nuway_ml.viz.scene import (
    AgentBox,
    Candidates,
    EgoPose,
    LaneMap,
    LocalizationTrails,
    OccupancyRaster,
    Predictions,
    RefLine,
    Scene,
    StopLine,
    TlCrops,
)
from nuway_ml.viz.scene import Path as VizPath


def fixture_scene() -> Scene:
    """A synthetic tick with every layer populated, later milestones' ones included."""
    s = np.linspace(0.0, 120.0, 241)
    line = np.stack([s, 2.0 * np.sin(s / 20.0)], axis=1)
    heading = np.gradient(line[:, 1], line[:, 0])
    heading = np.arctan(heading)
    ref = RefLine(line, heading, np.full(241, 1.75), np.full(241, 1.75))
    lanes = LaneMap(
        centerlines=(
            line,
            line + np.array([0.0, 3.5]),
            np.array([[60.0, -30.0], [60.0, 30.0]]),
        ),
        drivable=(True, True, False),
        stop_lines=(
            StopLine(70.0, 0.0, 0.0, "light"),
            StopLine(90.0, 0.0, 0.0, "sign"),
        ),
        crosswalks=(np.array([[50.0, -3.0], [52.0, -3.0], [52.0, 6.0], [50.0, 6.0]]),),
    )
    channels = np.zeros((6, 200, 200))
    channels[3, :, 80:120] = 1.0  # drivable band
    channels[2, :30, :] = 1.0  # unknown behind
    channels[0, 140:150, 95:105] = 1.0  # an occupied block ahead
    channels[5, 140:150, 95:105] = 1.0
    ego = EgoPose(40.0, 1.8, 0.05, 7.2)
    occ = OccupancyRaster(0.5, -50.0, -50.0, channels, ego)
    agents = (
        AgentBox(7, 1, 55.0, 2.0, 0.05, 4.5, 2.0, 6.0),
        AgentBox(9, 5, 48.0, -4.0, 1.5, 0.6, 0.6, 1.2),
        AgentBox(11, 2, 30.0, 5.5, 0.0, 7.0, 2.5, 0.0, visible=False),
    )
    gt = tuple(
        AgentBox(
            a.agent_id,
            a.class_id,
            a.x + 0.3,
            a.y,
            a.yaw,
            a.length,
            a.width,
            a.speed_mps,
        )
        for a in agents
    )
    t = np.arange(1, 17) * 0.5
    xy = np.zeros((3, 2, 16, 2))
    for sample in range(3):
        xy[sample, 0, :, 0] = 55.0 + 6.0 * t + 0.3 * sample * t
        xy[sample, 0, :, 1] = 2.0 + 0.2 * sample * t
        xy[sample, 1, :, 0] = 48.0 + 0.1 * t
        xy[sample, 1, :, 1] = -4.0 + 1.2 * t * (1 - 0.2 * sample)
    pred = Predictions(xy, np.array([0.5, 0.3, 0.2]))
    paths = tuple(
        np.stack(
            [40.0 + 7.0 * np.linspace(0, 8, 81), 1.8 + d * np.linspace(0, 1, 81) ** 2],
            axis=1,
        )
        for d in np.linspace(-3.0, 3.0, 9)
    )
    cands = Candidates(paths, np.abs(np.linspace(-3.0, 3.0, 9)), selected=4)
    traj = VizPath(paths[4], "lattice")
    safe = VizPath(paths[4][:40], "fallback")
    horizon = VizPath(paths[4][:21:2], "mpc", dot_every=5)
    rollout = VizPath(paths[6], "sim")
    trail = np.stack(
        [np.linspace(20.0, 40.0, 40), 1.8 + 0.3 * np.sin(np.linspace(0, 3, 40))], axis=1
    )
    loc = LocalizationTrails(
        trail, trail + np.array([0.0, 0.2]), (40.0, 1.8), (41.0, 2.4)
    )
    crops = TlCrops(
        crops=(
            np.full((12, 6, 3), 200, dtype=np.uint8),
            np.zeros((16, 8, 3), dtype=np.uint8),
        ),
        labels=("red 0.9", "green 0.7"),
    )
    return Scene(
        tick=1234,
        sim_time_s=61.7,
        episode_id=1,
        ego=ego,
        behavior="follow:7",
        source="lattice",
        degraded=("samples",),
        accel_mps2=0.42,
        steer_rad=-0.013,
        infractions=("red_light",),
        diag_cycle_ms={"planner_node": 6.3, "mpc_node": 0.4, "safety_layer_node": 0.9},
        occupancy=occ,
        lanes=lanes,
        reference_line=ref,
        agents=agents,
        gt_agents=gt,
        localization=loc,
        predictions=pred,
        candidates=cands,
        trajectory=traj,
        safe_trajectory=safe,
        mpc_horizon=horizon,
        sim_rollout=rollout,
        tl_crops=crops,
    )


def test_every_viz_layer_has_a_draw_function() -> None:
    assert set(bev_draw.LAYERS) == set(style.LAYER_NAMES)
    for name in style.LAYER_NAMES:
        assert callable(getattr(bev_draw, f"draw_{name}"))
    # The later milestones' layers draw from the fixture already (M1 task 13).
    for later in ("gt_agents", "tl_crops", "localization", "sim_rollout"):
        assert later in style.LAYER_NAMES


def test_fixture_scene_renders_byte_identically_twice(tmp_path: Path) -> None:
    scene = fixture_scene()
    a = render_frame(scene, tmp_path / "a.png")
    b = render_frame(scene, tmp_path / "b.png")
    assert a.shape == (800, 1000, 3)
    assert np.array_equal(a, b)
    assert (tmp_path / "a.png").read_bytes() == (tmp_path / "b.png").read_bytes()
    assert (tmp_path / "a.png").stat().st_size > 10_000
    # Something was drawn: the frame is not the flat background.
    assert len(np.unique(a.reshape(-1, 3), axis=0)) > 50
    # A subset of layers renders too, and differs from the full frame.
    subset = render_frame(scene, layers=["lanes", "trajectory"])
    assert not np.array_equal(subset, a)
    assert "INFRACTION: red_light" in header_lines(scene)[-1]
    assert "degraded: samples" in header_lines(scene)[1]


def test_empty_scene_renders() -> None:
    frame = render_frame(Scene(tick=0, sim_time_s=0.0))
    assert frame.shape == (800, 1000, 3)


def test_contact_sheet_tiles_twenty_frames(tmp_path: Path) -> None:
    frame = render_frame(fixture_scene())
    frames = [frame] * FRAMES_PER_SHEET
    sheet = tile(frames)
    assert sheet.shape == (4 * 400, 5 * 500, 3)
    partial = tile(frames[:7])
    assert partial.shape == (2 * 400, 5 * 500, 3)
    write_png(sheet, tmp_path / "sheets" / "000000_000019.png")
    assert (tmp_path / "sheets" / "000000_000019.png").stat().st_size > 1000
    with pytest.raises(ValueError, match="no frames"):
        tile([])
