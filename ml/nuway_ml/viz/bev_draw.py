"""One ``draw_<layer>()`` per ``/nuway/viz/<layer>`` (docs/02 §3.9, §8.1).

Each function draws one layer of a :class:`~nuway_ml.viz.scene.Scene` onto a
matplotlib ``Axes`` in the map frame and returns nothing; ``LAYERS`` maps the
layer name to its function, and :func:`draw_layer` dispatches by name so a
renderer can take ``--layers`` as a list of names. The same vocabulary as the
marker node: adding a Foxglove layer without a ``draw_`` here is a defect.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import matplotlib.transforms as mtransforms
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

from nuway_ml.viz import style
from nuway_ml.viz.scene import (
    AgentBox,
    Candidates,
    EgoPose,
    LaneMap,
    LocalizationTrails,
    OccupancyRaster,
    Path,
    Predictions,
    RefLine,
    Scene,
    TlCrops,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

    from matplotlib.axes import Axes
    from numpy.typing import NDArray

    Array = NDArray[np.float64]


def _box_corners(x: float, y: float, yaw: float, length: float, width: float) -> Array:
    """Return the four corners of a centred box, counter-clockwise from the front-left."""
    c, s = np.cos(yaw), np.sin(yaw)
    hl, hw = 0.5 * length, 0.5 * width
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    rot = np.array([[c, -s], [s, c]])
    out: Array = local @ rot.T + np.array([x, y])
    return out


def _draw_boxes(ax: Axes, agents: Sequence[AgentBox], *, hollow: bool) -> None:
    for a in agents:
        color = style.CLASS_COLORS.get(a.class_id, style.CLASS_COLORS[0])
        alpha = 0.75 if a.visible else 0.35
        corners = _box_corners(a.x, a.y, a.yaw, a.length, a.width)
        ax.add_patch(
            Polygon(
                corners,
                closed=True,
                fill=not hollow,
                facecolor=color if not hollow else "none",
                edgecolor=color,
                linewidth=1.2 if hollow else 0.6,
                alpha=alpha,
                zorder=5,
            )
        )
        # A nose tick from the centre to the front edge shows the heading.
        nose = 0.5 * a.length * np.array([np.cos(a.yaw), np.sin(a.yaw)])
        ax.plot(
            [a.x, a.x + nose[0]],
            [a.y, a.y + nose[1]],
            color=style.EGO_COLOR if not hollow else color,
            linewidth=0.8,
            zorder=6,
        )
        if not hollow:
            ax.text(
                a.x,
                a.y + 0.5 * a.width + 0.4,
                f"{a.agent_id} {a.speed_mps:.1f}",
                color=style.TEXT,
                fontsize=style.FONT_SIZE_LABEL,
                ha="center",
                va="bottom",
                zorder=7,
            )


def draw_occupancy(ax: Axes, occ: OccupancyRaster) -> None:
    """Draw the colorized OccupancyGridMC, placed in the map with its base_link anchor."""
    ch = occ.channels
    height, width = ch.shape[1], ch.shape[2]
    # RGBA: free cells stay transparent so the dark background shows through;
    # the others are painted in priority order occupied > dynamic > unknown > drivable.
    rgba = np.zeros((height, width, 4), dtype=np.float64)
    thr = style.OCC_THRESHOLD
    for index, color in (
        (3, style.OCC_DRIVABLE),
        (2, style.OCC_UNKNOWN),
        (5, style.OCC_DYNAMIC),
        (0, style.OCC_OCCUPIED),
    ):
        mask = ch[index] > thr
        rgba[mask, :3] = color
        rgba[mask, 3] = style.OCC_ALPHA
    # Array rows are x (forward) and columns y (left); imshow wants rows
    # along the vertical data axis, so the image is transposed to [y, x].
    extent = (
        occ.x_min,
        occ.x_min + height * occ.resolution,
        occ.y_min,
        occ.y_min + width * occ.resolution,
    )
    image = ax.imshow(
        np.transpose(rgba, (1, 0, 2)),
        extent=extent,
        origin="lower",
        interpolation="nearest",
        zorder=0,
    )
    placement = (
        mtransforms.Affine2D()
        .rotate(occ.anchor.yaw)
        .translate(occ.anchor.x, occ.anchor.y)
    )
    image.set_transform(placement + ax.transData)


def draw_lanes(ax: Axes, lanes: LaneMap) -> None:
    """Lane centerlines by type, stop lines and crosswalk footprints."""
    driving = [c for c, d in zip(lanes.centerlines, lanes.drivable, strict=True) if d]
    other = [c for c, d in zip(lanes.centerlines, lanes.drivable, strict=True) if not d]
    if driving:
        ax.add_collection(
            LineCollection(
                driving,
                colors=style.LANE_DRIVING,
                linewidths=style.LINE_WIDTH_LANE,
                zorder=1,
            )
        )
    if other:
        ax.add_collection(
            LineCollection(
                other,
                colors=style.LANE_OTHER,
                linewidths=style.LINE_WIDTH_LANE,
                alpha=0.6,
                zorder=1,
            )
        )
    for line in lanes.stop_lines:
        # A 3.5 m bar across the driving direction (heading faces the traffic).
        nx, ny = -np.sin(line.heading), np.cos(line.heading)
        color = style.STOP_LINE_LIGHT if line.kind == "light" else style.STOP_LINE_SIGN
        ax.plot(
            [line.x + 1.75 * nx, line.x - 1.75 * nx],
            [line.y + 1.75 * ny, line.y - 1.75 * ny],
            color=color,
            linewidth=1.8,
            zorder=2,
        )
    for footprint in lanes.crosswalks:
        ax.add_patch(
            Polygon(
                footprint,
                closed=True,
                fill=False,
                edgecolor=style.CROSSWALK,
                linewidth=0.8,
                alpha=0.7,
                zorder=2,
            )
        )


def draw_reference_line(ax: Axes, line: RefLine) -> None:
    """Draw the reference line and its drivable bounds along the normals."""
    ax.plot(
        line.xy[:, 0],
        line.xy[:, 1],
        color=style.REFERENCE_LINE,
        linewidth=style.LINE_WIDTH_REFERENCE,
        zorder=3,
    )
    normal = np.stack([-np.sin(line.heading), np.cos(line.heading)], axis=1)
    left = line.xy + normal * line.left_bound[:, None]
    right = line.xy - normal * line.right_bound[:, None]
    for edge in (left, right):
        ax.plot(
            edge[:, 0],
            edge[:, 1],
            color=style.REFERENCE_BOUNDS,
            linewidth=0.7,
            alpha=0.7,
            zorder=3,
        )


def draw_agents(ax: Axes, agents: Sequence[AgentBox]) -> None:
    """Perceived agents: filled boxes by class with an "id speed" label."""
    _draw_boxes(ax, agents, hollow=False)


def draw_gt_agents(ax: Axes, agents: Sequence[AgentBox]) -> None:
    """GT agents drawn hollow, so they can be told from the perceived ones (M3)."""
    _draw_boxes(ax, agents, hollow=True)


def draw_localization(ax: Axes, loc: LocalizationTrails) -> None:
    """Estimated trail against the GT trail and the map->odom correction arrow (M5)."""
    ax.plot(
        loc.ground_truth[:, 0],
        loc.ground_truth[:, 1],
        color=style.LOCALIZATION_GT,
        linewidth=1.0,
        alpha=0.6,
        zorder=4,
    )
    ax.plot(
        loc.estimated[:, 0],
        loc.estimated[:, 1],
        color=style.LOCALIZATION_EST,
        linewidth=1.2,
        zorder=4,
    )
    (x0, y0), (x1, y1) = loc.correction_from, loc.correction_to
    ax.annotate(
        "",
        xy=(x1, y1),
        xytext=(x0, y0),
        arrowprops={
            "arrowstyle": "->",
            "color": style.LOCALIZATION_CORRECTION,
            "linewidth": 1.2,
        },
        zorder=8,
    )


def draw_predictions(ax: Axes, pred: Predictions) -> None:
    """One polyline per (sample, agent), alpha fading with the step."""
    s_n, a_n, t_n, _ = pred.xy.shape
    if t_n < 2:
        return
    segments = []
    colors = []
    base = np.array([0.9, 0.3, 0.9])
    fade = np.linspace(0.9, 0.1, t_n - 1)
    for s in range(s_n):
        for a in range(a_n):
            pts = pred.xy[s, a]
            for t in range(t_n - 1):
                segments.append(pts[t : t + 2])
                colors.append((*base, fade[t]))
    widths = np.repeat(
        style.LINE_WIDTH_PREDICTION * (0.5 + pred.weights), a_n * (t_n - 1)
    )
    ax.add_collection(
        LineCollection(segments, colors=colors, linewidths=widths, zorder=4)
    )


def draw_candidates(ax: Axes, cands: Candidates) -> None:
    """Lattice candidates, thin, colored by cost quantile (rank over the set)."""
    n = len(cands.paths)
    if n == 0:
        return
    order = np.argsort(cands.costs[:n], kind="stable")
    quantile = np.empty(n)
    quantile[order] = np.arange(n) / max(1, n - 1)
    colors = [(*style.cost_ramp(float(q)), 0.55) for q in quantile]
    ax.add_collection(
        LineCollection(
            list(cands.paths),
            colors=colors,
            linewidths=style.LINE_WIDTH_CANDIDATE,
            zorder=4,
        )
    )


def _draw_path(
    ax: Axes, path: Path, color: str, *, dashed: bool = False, zorder: int = 6
) -> None:
    ax.plot(
        path.xy[:, 0],
        path.xy[:, 1],
        color=color,
        linewidth=style.LINE_WIDTH_TRAJECTORY,
        linestyle="--" if dashed else "-",
        zorder=zorder,
    )
    if path.dot_every > 0:
        dots = path.xy[:: path.dot_every]
        ax.scatter(dots[:, 0], dots[:, 1], s=14, color=color, zorder=zorder + 1)


def draw_trajectory(ax: Axes, path: Path) -> None:
    """Draw the selected (refined) trajectory, thick, a dot every second."""
    _draw_path(ax, path, style.TRAJECTORY)


def draw_safe_trajectory(ax: Axes, path: Path) -> None:
    """Draw the safety layer's output where it differs from the plan."""
    _draw_path(ax, path, style.SAFE_TRAJECTORY, zorder=7)


def draw_mpc_horizon(ax: Axes, path: Path) -> None:
    """Draw the MPC's predicted horizon."""
    _draw_path(ax, path, style.MPC_HORIZON, zorder=8)


def draw_sim_rollout(ax: Axes, path: Path) -> None:
    """Draw the world-model rollout of M9, dashed."""
    _draw_path(ax, path, style.SIM_ROLLOUT, dashed=True, zorder=5)


def draw_tl_crops(ax: Axes, crops: TlCrops) -> None:
    """Draw the traffic-light crops strip (M4): crops stacked vertically with their labels."""
    ax.set_axis_off()
    n = len(crops.crops)
    if n == 0:
        return
    for i, (crop, label) in enumerate(zip(crops.crops, crops.labels, strict=True)):
        height, width = crop.shape[0], crop.shape[1]
        # Each crop occupies one row of the strip, scaled to a unit-width slot.
        scale = 1.0 / max(1, width)
        top = n - i
        ax.imshow(crop, extent=(0.0, 1.0, top - height * scale, top), zorder=1)
        ax.text(
            1.02,
            top - 0.5 * height * scale,
            label,
            color=style.TEXT,
            fontsize=style.FONT_SIZE_LABEL,
            va="center",
        )
    ax.set_xlim(0.0, 1.6)
    ax.set_ylim(0.0, float(n))
    ax.set_aspect("equal")


def draw_ego(ax: Axes, ego: EgoPose) -> None:
    """Draw the ego footprint (a 4.7 x 1.9 m box around the rear axle + 1.4 m)."""
    cx = ego.x + 1.4 * np.cos(ego.yaw)
    cy = ego.y + 1.4 * np.sin(ego.yaw)
    corners = _box_corners(cx, cy, ego.yaw, 4.7, 1.9)
    ax.add_patch(
        Polygon(
            corners,
            closed=True,
            fill=False,
            edgecolor=style.EGO_COLOR,
            linewidth=1.5,
            zorder=9,
        )
    )


LAYERS: dict[str, Callable[..., None]] = {
    "occupancy": draw_occupancy,
    "lanes": draw_lanes,
    "reference_line": draw_reference_line,
    "agents": draw_agents,
    "gt_agents": draw_gt_agents,
    "localization": draw_localization,
    "predictions": draw_predictions,
    "candidates": draw_candidates,
    "trajectory": draw_trajectory,
    "safe_trajectory": draw_safe_trajectory,
    "mpc_horizon": draw_mpc_horizon,
    "sim_rollout": draw_sim_rollout,
    "tl_crops": draw_tl_crops,
}


def draw_layer(ax: Axes, scene: Scene, name: str) -> bool:
    """Draw ``scene``'s layer ``name`` if present; returns whether anything was drawn."""
    data = getattr(scene, name)
    if data is None or (isinstance(data, tuple) and len(data) == 0):
        return False
    LAYERS[name](ax, data)
    return True
