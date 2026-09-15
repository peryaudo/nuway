"""The plain-data view of one tick that the layers are drawn from.

Everything is map frame and SI unless a field says otherwise; arrays are
``numpy`` ``float64`` unless stated. ``render_bag.py`` fills a ``Scene``
from a decoded MCAP, training code from tensors, tests from fixtures; the
draw functions never see a ROS message, which is what keeps them
importable without ``rclpy`` (docs/02 §8.1).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Mapping

    import numpy as np
    from numpy.typing import NDArray

    Array = NDArray[np.float64]
    Image = NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class EgoPose:
    """Ego rear-axle pose and speed."""

    x: float
    y: float
    yaw: float
    speed_mps: float = 0.0


@dataclass(frozen=True, slots=True)
class AgentBox:
    """One agent box (centre pose, footprint)."""

    agent_id: int
    class_id: int
    x: float
    y: float
    yaw: float
    length: float
    width: float
    speed_mps: float = 0.0
    visible: bool = True


@dataclass(frozen=True, slots=True)
class StopLine:
    """A stop line: centre, heading of the traffic it faces, ``light`` or ``sign``."""

    x: float
    y: float
    heading: float
    kind: str
    state: str = ""  # a light's observed state on this tick: red | yellow | green | ""


@dataclass(frozen=True, slots=True)
class LaneMap:
    """Lane centerlines ([N, 2] each), their drivability, stop lines, sign volumes, crosswalks."""

    centerlines: tuple[Array, ...]
    drivable: tuple[bool, ...]
    stop_lines: tuple[StopLine, ...] = ()
    sign_volumes: tuple[Array, ...] = ()  # stop-sign trigger volumes, [M, 2]
    crosswalks: tuple[Array, ...] = ()  # closed footprints, [M, 2]


@dataclass(frozen=True, slots=True)
class RefLine:
    """The reference line samples and their drivable bounds (positive distances)."""

    xy: Array  # [N, 2]
    heading: Array  # [N]
    left_bound: Array  # [N]
    right_bound: Array  # [N]


@dataclass(frozen=True, slots=True)
class OccupancyRaster:
    """The OccupancyGridMC of a tick in base_link: rows along x, columns along y."""

    resolution: float
    x_min: float
    y_min: float
    channels: Array  # [6, H, W]: occupied, free, unknown, drivable, height_max, dynamic
    anchor: EgoPose  # the base_link pose the grid is attached to


@dataclass(frozen=True, slots=True)
class Predictions:
    """Joint prediction samples: ``xy`` is [S, A, T, 2], ``weights`` [S]."""

    xy: Array
    weights: Array


@dataclass(frozen=True, slots=True)
class Candidates:
    """Lattice candidates ([P, 2] paths), their costs and the selected index."""

    paths: tuple[Array, ...]
    costs: Array
    selected: int = -1


@dataclass(frozen=True, slots=True)
class Path:
    """A trajectory-like polyline with its ``source`` and the sample spacing."""

    xy: Array  # [N, 2]
    source: str = ""
    dot_every: int = 10  # a dot every N samples (1 s at the 0.1 s spacing)


@dataclass(frozen=True, slots=True)
class LocalizationTrails:
    """Estimated and GT pose trails ([N, 2]) and the current map->odom correction."""

    estimated: Array
    ground_truth: Array
    correction_from: tuple[float, float]
    correction_to: tuple[float, float]


@dataclass(frozen=True, slots=True)
class TlCrops:
    """Traffic-light crops (RGB uint8 images) with a label each (M4)."""

    crops: tuple[Image, ...]
    labels: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class Scene:
    """One tick: header facts, the diag strip and every layer (``None`` = absent)."""

    tick: int
    sim_time_s: float
    episode_id: int = 0
    ego: EgoPose | None = None
    behavior: str = ""
    source: str = ""
    degraded: tuple[str, ...] = ()
    accel_mps2: float | None = None
    steer_rad: float | None = None
    infractions: tuple[str, ...] = ()
    diag_cycle_ms: Mapping[str, float] = field(default_factory=dict)
    occupancy: OccupancyRaster | None = None
    lanes: LaneMap | None = None
    reference_line: RefLine | None = None
    agents: tuple[AgentBox, ...] = ()
    gt_agents: tuple[AgentBox, ...] = ()
    localization: LocalizationTrails | None = None
    predictions: Predictions | None = None
    candidates: Candidates | None = None
    trajectory: Path | None = None
    safe_trajectory: Path | None = None
    mpc_horizon: Path | None = None
    sim_rollout: Path | None = None
    tl_crops: TlCrops | None = None
