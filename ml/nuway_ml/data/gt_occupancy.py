"""GT occupancy generator: the one implementation (docs/01 Rules, M2 §3.3).

M0 ships the static part: a town-wide *drivable* raster built once from the
lane graph and sampled into the ego-centred ``OccupancyGridMC`` every planning
tick by ``gt_perception_node.py`` (``M0_bringup.md`` §2.9). The M0 grid is
``free = drivable``, ``unknown = 1 - drivable`` and zero elsewhere, which keeps
the channel invariant ``unknown = 1 - occupied - free``. M2 adds
``generate_gt_occupancy`` (semantic LiDAR) and M6 ``generate_from_geometry``.

numpy only: this module must import without torch (``import_light`` tests).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from nuway_ml.common.geometry import SE2, apply
from nuway_ml.common.occupancy import (
    NUM_OCCUPANCY_CHANNELS,
    GridSpec,
    OccupancyChannel,
    grid_to_world,
)

if TYPE_CHECKING:
    from collections.abc import Iterable

Array = NDArray[np.float64]
BoolGrid = NDArray[np.bool_]


@dataclass(frozen=True, slots=True)
class LanePolyline:
    """One lane as the rasterizer needs it: centerline ``(N, 2)`` and width ``(N,)``, map frame."""

    centerline: Array
    width: Array


@dataclass(frozen=True, slots=True)
class StaticRaster:
    """Town-wide boolean raster in the map frame; rows index x, cols index y."""

    resolution: float
    x_min: float
    y_min: float
    drivable: BoolGrid

    @property
    def height(self) -> int:
        """Number of rows (x cells)."""
        return int(self.drivable.shape[0])

    @property
    def width(self) -> int:
        """Number of cols (y cells)."""
        return int(self.drivable.shape[1])


def _lane_samples(lane: LanePolyline, step_m: float) -> Array:
    """Dense ``(M, 2)`` points covering the lane surface at ``step_m`` spacing."""
    pts = np.asarray(lane.centerline, dtype=np.float64)
    widths = np.asarray(lane.width, dtype=np.float64)
    if pts.shape[0] < 2:
        return np.zeros((0, 2), dtype=np.float64)
    p0 = pts[:-1]
    p1 = pts[1:]
    seg = p1 - p0
    seg_len = np.linalg.norm(seg, axis=1)
    keep = seg_len > 1e-9
    if not np.any(keep):
        return np.zeros((0, 2), dtype=np.float64)
    p0, p1, seg, seg_len = p0[keep], p1[keep], seg[keep], seg_len[keep]
    w0 = widths[:-1][keep]
    w1 = widths[1:][keep]
    n_along = int(np.ceil(seg_len.max() / step_m)) + 1
    n_across = int(np.ceil(max(w0.max(), w1.max()) / step_m)) + 1
    t = np.linspace(0.0, 1.0, n_along)  # (A,)
    u = np.linspace(-0.5, 0.5, n_across)  # (C,)
    # (S, A, 2) points along every segment, their normals and widths.
    along = p0[:, None, :] + t[None, :, None] * seg[:, None, :]
    normal = np.stack([-seg[:, 1], seg[:, 0]], axis=1) / seg_len[:, None]  # (S, 2)
    width = w0[:, None] + t[None, :] * (w1 - w0)[:, None]  # (S, A)
    offsets = width[:, :, None] * u[None, None, :]  # (S, A, C)
    out = along[:, :, None, :] + offsets[..., None] * normal[:, None, None, :]
    return np.asarray(out.reshape(-1, 2), dtype=np.float64)


def rasterize_drivable(
    lanes: Iterable[LanePolyline], resolution: float = 0.5, margin_m: float = 5.0
) -> StaticRaster:
    """Rasterize the lane surfaces (centerline +- width / 2) into a map-frame raster.

    Cells are marked by dense sampling at half the resolution along and across
    every centerline segment, so no cell inside a lane is skipped. The raster
    bounds are the lane extent plus ``margin_m``.
    """
    lane_list = list(lanes)
    samples = [_lane_samples(lane, 0.5 * resolution) for lane in lane_list]
    all_pts = (
        np.concatenate(samples, axis=0)
        if samples
        else np.zeros((0, 2), dtype=np.float64)
    )
    if all_pts.shape[0] == 0:
        return StaticRaster(resolution, 0.0, 0.0, np.zeros((1, 1), dtype=np.bool_))
    lo = np.floor((all_pts.min(axis=0) - margin_m) / resolution) * resolution
    hi = all_pts.max(axis=0) + margin_m
    rows = int(np.ceil((hi[0] - lo[0]) / resolution)) + 1
    cols = int(np.ceil((hi[1] - lo[1]) / resolution)) + 1
    drivable = np.zeros((rows, cols), dtype=np.bool_)
    r = np.floor((all_pts[:, 0] - lo[0]) / resolution).astype(np.int64)
    c = np.floor((all_pts[:, 1] - lo[1]) / resolution).astype(np.int64)
    inside = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
    drivable[r[inside], c[inside]] = True
    return StaticRaster(resolution, float(lo[0]), float(lo[1]), drivable)


def sample_raster(raster: StaticRaster, spec: GridSpec, ego: SE2) -> BoolGrid:
    """Look the raster up at every cell center of the ego grid ``spec`` (base_link at ``ego``).

    Nearest-cell lookup; cells outside the raster read False.
    """
    rows, cols = np.meshgrid(
        np.arange(spec.height), np.arange(spec.width), indexing="ij"
    )
    centers = grid_to_world(spec, rows, cols).reshape(-1, 2)
    world = apply(ego, centers)
    r = np.floor((world[:, 0] - raster.x_min) / raster.resolution).astype(np.int64)
    c = np.floor((world[:, 1] - raster.y_min) / raster.resolution).astype(np.int64)
    inside = (r >= 0) & (r < raster.height) & (c >= 0) & (c < raster.width)
    out = np.zeros(centers.shape[0], dtype=np.bool_)
    out[inside] = raster.drivable[r[inside], c[inside]]
    return np.asarray(out.reshape(spec.height, spec.width), dtype=np.bool_)


def static_grid(drivable: BoolGrid) -> NDArray[np.float32]:
    """Build the M0 channel stack ``(6, H, W)`` from a drivable mask (free = drivable, unknown = the rest)."""
    grid = np.zeros((NUM_OCCUPANCY_CHANNELS, *drivable.shape), dtype=np.float32)
    d = drivable.astype(np.float32)
    grid[OccupancyChannel.FREE] = d
    grid[OccupancyChannel.UNKNOWN] = 1.0 - d
    grid[OccupancyChannel.DRIVABLE] = d
    return grid


def no_input_grid(spec: GridSpec) -> NDArray[np.float32]:
    """Build the no-input grid of docs/02 §2: ``unknown = 1`` everywhere, every other channel zero."""
    grid = np.zeros((NUM_OCCUPANCY_CHANNELS, spec.height, spec.width), dtype=np.float32)
    grid[OccupancyChannel.UNKNOWN] = 1.0
    return grid
