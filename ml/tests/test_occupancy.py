import numpy as np
import pytest

from nuway_ml.common.occupancy import (
    NUM_OCCUPANCY_CHANNELS,
    OCCUPANCY_CHANNEL_NAMES,
    GridSpec,
    OccupancyChannel,
    bilinear_sample,
    grid_to_world,
    world_to_grid,
    world_to_grid_array,
    world_to_grid_coord,
)


@pytest.mark.import_light
def test_world_to_grid_round_trips_through_cell_centers():
    spec = GridSpec()
    assert world_to_grid(spec, 0.1, -0.1) == (100, 99)
    np.testing.assert_allclose(grid_to_world(spec, 100, 99), [0.25, -0.25])
    assert world_to_grid(spec, 50.0, 0.0) is None
    assert world_to_grid(spec, -50.1, 0.0) is None
    assert world_to_grid(spec, -50.0, 49.99) is not None
    np.testing.assert_allclose(world_to_grid_coord(spec, 0.25, -0.25), [100.0, 99.0])
    rows, cols, inside = world_to_grid_array(
        spec, np.array([0.1, 50.0]), np.array([-0.1, 0.0])
    )
    assert rows[0] == 100
    assert cols[0] == 99
    assert inside.tolist() == [True, False]


@pytest.mark.import_light
def test_channel_names_follow_the_fixed_order():
    assert OCCUPANCY_CHANNEL_NAMES[OccupancyChannel.UNKNOWN] == "unknown"
    assert OCCUPANCY_CHANNEL_NAMES[OccupancyChannel.DYNAMIC] == "dynamic"
    assert NUM_OCCUPANCY_CHANNELS == 6


def test_bilinear_sample_interpolates_between_centers():
    torch = pytest.importorskip("torch")
    spec = GridSpec(resolution=1.0, x_min=0.0, y_min=0.0, height=2, width=2)
    channel = torch.tensor([[0.0, 1.0], [2.0, 3.0]], dtype=torch.float64)
    xs = torch.tensor([1.0, 0.5, 1.5, 0.5, 1.0, 0.25])
    ys = torch.tensor([1.0, 0.5, 0.5, 1.5, 0.5, 0.5])
    out = bilinear_sample(spec, channel, xs, ys, outside=-1.0)
    np.testing.assert_allclose(out.numpy(), [1.5, 0.0, 2.0, 1.0, 1.0, -1.0])
