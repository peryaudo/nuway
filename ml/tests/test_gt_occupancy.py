"""Tests of the M0 part of nuway_ml.data.gt_occupancy (drivable raster). Torch-free."""

from __future__ import annotations

import math

import numpy as np
import pytest

from nuway_ml.common.geometry import SE2
from nuway_ml.common.occupancy import GridSpec, OccupancyChannel
from nuway_ml.data import gt_occupancy

pytestmark = pytest.mark.import_light


def _straight(
    x0: float, x1: float, y: float, width: float
) -> gt_occupancy.LanePolyline:
    xs = np.arange(x0, x1 + 1e-9, 1.0)
    pts = np.stack([xs, np.full_like(xs, y)], axis=1)
    return gt_occupancy.LanePolyline(pts, np.full(xs.shape[0], width))


def _cell(raster: gt_occupancy.StaticRaster, x: float, y: float) -> bool:
    r = math.floor((x - raster.x_min) / raster.resolution)
    c = math.floor((y - raster.y_min) / raster.resolution)
    return bool(raster.drivable[r, c])


def test_rasterize_marks_the_lane_surface_only() -> None:
    raster = gt_occupancy.rasterize_drivable([_straight(0.0, 20.0, 0.0, 4.0)])
    assert raster.resolution == 0.5
    assert raster.x_min <= -5.0
    for x in (0.1, 10.3, 19.9):
        assert _cell(raster, x, 0.0)
        assert _cell(raster, x, 1.8)
        assert _cell(raster, x, -1.8)
        assert not _cell(raster, x, 2.6)
        assert not _cell(raster, x, -2.6)
    assert not _cell(raster, -1.0, 0.0)
    assert not _cell(raster, 21.5, 0.0)
    # Nothing marked outside the lane bounding box.
    marked = raster.drivable.sum()
    assert 20 * 4 / 0.25 * 0.9 < marked < 22 * 5 / 0.25


def test_rasterize_handles_empty_and_degenerate_input() -> None:
    empty = gt_occupancy.rasterize_drivable([])
    assert empty.drivable.shape == (1, 1)
    single = gt_occupancy.LanePolyline(np.array([[0.0, 0.0]]), np.array([3.0]))
    assert not gt_occupancy.rasterize_drivable([single]).drivable.any()


def test_sample_raster_follows_the_ego_pose() -> None:
    raster = gt_occupancy.rasterize_drivable([_straight(0.0, 40.0, 0.0, 4.0)])
    spec = GridSpec(0.5, -10.0, -10.0, 40, 40)
    # Ego on the lane, heading +x: the row through the ego is drivable, the
    # cells 5 m to the side are not.
    mask = gt_occupancy.sample_raster(raster, spec, SE2(20.0, 0.0, 0.0))
    assert mask.shape == (40, 40)
    assert mask[20, 20]
    assert mask[30, 20]
    assert not mask[20, 30]
    assert not mask[20, 10]
    # Ego rotated by 90 deg: the lane now runs along the grid's y (cols).
    mask = gt_occupancy.sample_raster(raster, spec, SE2(20.0, 0.0, math.pi / 2))
    assert mask[20, 20]
    assert mask[20, 30]
    assert not mask[30, 20]
    # Far away: nothing drivable.
    assert not gt_occupancy.sample_raster(raster, spec, SE2(500.0, 500.0, 0.0)).any()


def test_channel_invariants() -> None:
    spec = GridSpec(0.5, -10.0, -10.0, 40, 40)
    drivable = np.zeros((40, 40), dtype=np.bool_)
    drivable[10:30, 15:25] = True
    grid = gt_occupancy.static_grid(drivable)
    assert grid.shape == (6, 40, 40)
    assert grid.dtype == np.float32
    np.testing.assert_allclose(
        grid[OccupancyChannel.UNKNOWN],
        1.0 - grid[OccupancyChannel.OCCUPIED] - grid[OccupancyChannel.FREE],
    )
    np.testing.assert_array_equal(grid[OccupancyChannel.DRIVABLE], drivable)
    assert not grid[OccupancyChannel.OCCUPIED].any()
    assert not grid[OccupancyChannel.DYNAMIC].any()
    blank = gt_occupancy.no_input_grid(spec)
    assert blank.shape == (6, 40, 40)
    assert (blank[OccupancyChannel.UNKNOWN] == 1.0).all()
    assert blank.sum() == 40 * 40
