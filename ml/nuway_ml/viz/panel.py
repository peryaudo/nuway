"""The standard frame (docs/02 §8.1): BEV + header + traffic-light strip + diag strip.

Fixed geometry whatever the scene holds, so frames of different ticks and
different runs line up pixel for pixel; PNGs carry no timestamp or software
tag, so two renders of one scene are byte-identical.
"""

from __future__ import annotations

import io
from typing import TYPE_CHECKING

import numpy as np
from matplotlib.figure import Figure
from PIL import Image

from nuway_ml.viz import style
from nuway_ml.viz.bev_draw import draw_ego, draw_layer

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

    from numpy.typing import NDArray

    from nuway_ml.viz.scene import Scene

# Axes rectangles in figure fractions: [left, bottom, width, height].
_HEADER = (0.01, 0.90, 0.98, 0.09)
_BEV = (0.01, 0.09, 0.69, 0.80)
_STRIP = (0.72, 0.50, 0.27, 0.38)
_INFO = (0.72, 0.09, 0.27, 0.38)
_DIAG = (0.01, 0.005, 0.98, 0.075)
PNG_METADATA = {"Software": None}  # drop matplotlib's version tag


def header_lines(scene: Scene) -> list[str]:
    """Compose the header text: tick, time, episode, behavior, source, speed, command, incidents."""
    speed = f"{scene.ego.speed_mps:.1f} m/s" if scene.ego is not None else "-"
    accel = f"{scene.accel_mps2:+.2f}" if scene.accel_mps2 is not None else "-"
    steer = f"{scene.steer_rad:+.3f}" if scene.steer_rad is not None else "-"
    first = (
        f"tick {scene.tick}   t = {scene.sim_time_s:.2f} s   episode {scene.episode_id}   "
        f"behavior: {scene.behavior or '-'}   source: {scene.source or '-'}   "
        f"v = {speed}   a = {accel} m/s^2   steer = {steer} rad"
    )
    lines = [first]
    if scene.degraded:
        lines.append("degraded: " + ", ".join(scene.degraded))
    if scene.infractions:
        lines.append("INFRACTION: " + ", ".join(scene.infractions))
    return lines


def diag_line(scene: Scene) -> str:
    """Compose the diag strip: per-node cycle times of the tick, in a fixed order."""
    if not scene.diag_cycle_ms:
        return "diag: -"
    parts = [
        f"{node.removesuffix('_node')} {ms:.1f}"
        for node, ms in sorted(scene.diag_cycle_ms.items())
    ]
    # Two lines past six nodes, so a full M1 stack fits the strip.
    half = (len(parts) + 1) // 2 if len(parts) > 6 else len(parts)
    lines = ["   ".join(parts[:half])]
    if parts[half:]:
        lines.append("   ".join(parts[half:]))
    return "diag [ms]: " + "\n           ".join(lines)


def render_frame(
    scene: Scene,
    path: Path | None = None,
    layers: Sequence[str] | None = None,
) -> NDArray[np.uint8]:
    """Render one scene to an RGB array (and to ``path`` as PNG when given).

    ``layers`` restricts the drawn layers to the given names (drawing order
    stays that of ``style.LAYER_NAMES``); the ego footprint is always drawn.
    """
    wanted = set(style.LAYER_NAMES if layers is None else layers)
    fig = Figure(
        figsize=style.FIGURE_SIZE_IN, dpi=style.DPI, facecolor=style.BACKGROUND
    )
    fig.set_layout_engine("none")

    ax = fig.add_axes(_BEV)
    ax.set_facecolor(style.BACKGROUND)
    ax.set_aspect("equal")
    ax.tick_params(colors=style.TEXT, labelsize=style.FONT_SIZE_LABEL)
    for spine in ax.spines.values():
        spine.set_color(style.GRID)
    ax.grid(True, color=style.GRID, linewidth=0.5)
    for name in style.LAYER_NAMES:
        if name == "tl_crops" or name not in wanted:
            continue
        draw_layer(ax, scene, name)
    if scene.ego is not None:
        draw_ego(ax, scene.ego)
        ax.set_xlim(
            scene.ego.x - style.VIEW_RADIUS_M, scene.ego.x + style.VIEW_RADIUS_M
        )
        ax.set_ylim(
            scene.ego.y - style.VIEW_RADIUS_M, scene.ego.y + style.VIEW_RADIUS_M
        )
    else:
        ax.set_xlim(-style.VIEW_RADIUS_M, style.VIEW_RADIUS_M)
        ax.set_ylim(-style.VIEW_RADIUS_M, style.VIEW_RADIUS_M)

    strip = fig.add_axes(_STRIP)
    strip.set_facecolor(style.BACKGROUND)
    strip.set_axis_off()
    if "tl_crops" in wanted and scene.tl_crops is not None and scene.tl_crops.crops:
        draw_layer(strip, scene, "tl_crops")
        strip.set_title("traffic lights", color=style.TEXT, fontsize=style.FONT_SIZE)

    info = fig.add_axes(_INFO)
    info.set_axis_off()
    info.text(
        0.0,
        1.0,
        "\n".join(_legend_lines(scene)),
        color=style.TEXT,
        fontsize=style.FONT_SIZE_LABEL,
        va="top",
        ha="left",
        family="monospace",
    )

    header = fig.add_axes(_HEADER)
    header.set_axis_off()
    header.text(
        0.0,
        0.5,
        "\n".join(header_lines(scene)),
        color=style.TEXT,
        fontsize=style.FONT_SIZE,
        va="center",
        family="monospace",
    )
    diag = fig.add_axes(_DIAG)
    diag.set_axis_off()
    diag.text(
        0.0,
        0.5,
        diag_line(scene),
        color=style.TEXT,
        fontsize=style.FONT_SIZE_LABEL,
        va="center",
        family="monospace",
    )

    # One PNG encoding serves both outputs, so the array and the file agree.
    buf = io.BytesIO()
    fig.savefig(
        buf,
        format="png",
        dpi=style.DPI,
        facecolor=style.BACKGROUND,
        metadata=PNG_METADATA,
    )
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(buf.getvalue())
    buf.seek(0)
    rgb: NDArray[np.uint8] = np.asarray(Image.open(buf).convert("RGB"), dtype=np.uint8)
    return rgb


def _legend_lines(scene: Scene) -> list[str]:
    lines = ["layers:"]
    for name in style.LAYER_NAMES:
        data = getattr(scene, name)
        present = data is not None and not (isinstance(data, tuple) and len(data) == 0)
        lines.append(f"  [{'x' if present else ' '}] {name}")
    if scene.candidates is not None:
        n = len(scene.candidates.paths)
        lines.append(f"candidates: {n}, selected {scene.candidates.selected}")
    if scene.agents:
        lines.append(f"agents: {len(scene.agents)}")
    return lines
