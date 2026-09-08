"""Occupancy grid conventions of ``docs/02_interfaces.md`` §1/§4 (M0).

Mirrors ``nuway_common/occupancy.h``: ``GridSpec``, world <-> grid index
conversion (numpy, no torch: ``gt_perception_node`` imports this) and
:func:`bilinear_sample`, the one torch function here, which imports torch lazily
inside its body (``docs/03_style_and_conventions.md`` §6.1). Grids are in
``base_link``; rows index x (forward), columns index y (left); cell ``(i, j)``
center is ``(x_min + (i + 0.5) res, y_min + (j + 0.5) res)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    import torch

Array = NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class GridSpec:
    """Grid geometry. Defaults are the stack-wide values of docs/02 §1."""

    resolution: float = 0.5
    x_min: float = -50.0
    y_min: float = -50.0
    height: int = 200
    width: int = 200


class OccupancyChannel(IntEnum):
    """Fixed channel order of ``OccupancyGridMC``."""

    OCCUPIED = 0
    FREE = 1
    UNKNOWN = 2
    DRIVABLE = 3
    HEIGHT_MAX = 4
    DYNAMIC = 5


OCCUPANCY_CHANNEL_NAMES: tuple[str, ...] = (
    "occupied",
    "free",
    "unknown",
    "drivable",
    "height_max",
    "dynamic",
)
NUM_OCCUPANCY_CHANNELS = len(OCCUPANCY_CHANNEL_NAMES)


def world_to_grid_coord(spec: GridSpec, x: Array | float, y: Array | float) -> Array:
    """World (base_link) -> continuous ``(..., 2)`` grid coordinates (row, col).

    Integer values fall on cell centers; nothing is clamped.
    """
    row = (np.asarray(x, dtype=np.float64) - spec.x_min) / spec.resolution - 0.5
    col = (np.asarray(y, dtype=np.float64) - spec.y_min) / spec.resolution - 0.5
    return np.stack([row, col], axis=-1)


def world_to_grid(spec: GridSpec, x: float, y: float) -> tuple[int, int] | None:
    """World -> integer ``(row, col)`` of the cell containing (x, y); None outside."""
    row = int(np.floor((x - spec.x_min) / spec.resolution))
    col = int(np.floor((y - spec.y_min) / spec.resolution))
    if row < 0 or row >= spec.height or col < 0 or col >= spec.width:
        return None
    return row, col


def world_to_grid_array(
    spec: GridSpec, x: Array, y: Array
) -> tuple[NDArray[np.int64], NDArray[np.int64], NDArray[np.bool_]]:
    """Vectorised :func:`world_to_grid`: ``(rows, cols, inside)`` arrays."""
    rows = np.floor((np.asarray(x) - spec.x_min) / spec.resolution).astype(np.int64)
    cols = np.floor((np.asarray(y) - spec.y_min) / spec.resolution).astype(np.int64)
    inside = (rows >= 0) & (rows < spec.height) & (cols >= 0) & (cols < spec.width)
    return rows, cols, inside


def grid_to_world(spec: GridSpec, row: Array | int, col: Array | int) -> Array:
    """Cell center(s) of ``(row, col)`` in world (base_link), shape ``(..., 2)``."""
    x = spec.x_min + (np.asarray(row, dtype=np.float64) + 0.5) * spec.resolution
    y = spec.y_min + (np.asarray(col, dtype=np.float64) + 0.5) * spec.resolution
    return np.stack([x, y], axis=-1)


def bilinear_sample(
    spec: GridSpec,
    channel: torch.Tensor,
    x: torch.Tensor,
    y: torch.Tensor,
    outside: float = 0.0,
) -> torch.Tensor:
    """Bilinearly sample one ``(H, W)`` channel at world points ``x``, ``y`` ``(N,)``.

    Points beyond the outermost cell centers return ``outside``. Differentiable
    with respect to ``channel``.
    """
    import torch  # noqa: PLC0415  -- lazy: this module must import without torch (03 §6.1)

    grid = torch.as_tensor(channel, dtype=torch.float64)
    xs = torch.as_tensor(x, dtype=torch.float64)
    ys = torch.as_tensor(y, dtype=torch.float64)
    if spec.height < 2 or spec.width < 2:
        return torch.full_like(xs, outside)  # no cell to interpolate in
    row = (xs - spec.x_min) / spec.resolution - 0.5
    col = (ys - spec.y_min) / spec.resolution - 0.5
    inside = (
        (row >= 0) & (col >= 0) & (row <= spec.height - 1) & (col <= spec.width - 1)
    )
    r0 = torch.clamp(torch.floor(row).long(), 0, spec.height - 2)
    c0 = torch.clamp(torch.floor(col).long(), 0, spec.width - 2)
    fr = torch.clamp(row - r0.to(row.dtype), 0.0, 1.0)
    fc = torch.clamp(col - c0.to(col.dtype), 0.0, 1.0)
    top = (1.0 - fc) * grid[r0, c0] + fc * grid[r0, c0 + 1]
    bottom = (1.0 - fc) * grid[r0 + 1, c0] + fc * grid[r0 + 1, c0 + 1]
    value = (1.0 - fr) * top + fr * bottom
    return torch.where(inside, value, torch.full_like(value, outside))
