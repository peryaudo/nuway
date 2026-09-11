"""Colors, widths and figure geometry of every render (docs/02 §8.1).

One style for the eval renders and the training renders, so two frames of
the same scene are directly comparable; the hues mirror the Foxglove
layers of ``nuway_viz/marker_node.cc``.
"""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")  # before pyplot is imported anywhere; no display ever

# The /nuway/viz/<layer> vocabulary of docs/02 §3.9, in drawing order (the
# raster first, the ego-relative trajectories last so they stay on top).
# `tl_crops` is a strip beside the BEV rather than a layer on it.
LAYER_NAMES: tuple[str, ...] = (
    "occupancy",
    "lanes",
    "reference_line",
    "agents",
    "gt_agents",
    "localization",
    "predictions",
    "candidates",
    "trajectory",
    "safe_trajectory",
    "mpc_horizon",
    "sim_rollout",
    "tl_crops",
)

# Figure geometry: fixed size and DPI, so a render diff is meaningful.
FIGURE_SIZE_IN: tuple[float, float] = (10.0, 8.0)
DPI = 100
VIEW_RADIUS_M = 45.0  # the BEV shows the ego +- this many metres
SHEET_COLUMNS = 5  # contact sheet: 5 x 4 = 20 frames
SHEET_ROWS = 4
SHEET_SCALE = 0.5  # each frame is halved on a sheet

BACKGROUND = "#101418"
TEXT = "#e6e6e6"
GRID = "#2a2f36"

# Agent classes (nuway_msgs/Agent CLASS_*), the marker node's hues.
CLASS_COLORS: dict[int, str] = {
    0: "#f2f2f2",  # unknown
    1: "#4090ff",  # car
    2: "#9a59e6",  # truck
    3: "#1ad9d9",  # bicycle
    4: "#1ad9d9",  # motorcycle
    5: "#ffd933",  # pedestrian
    6: "#999999",  # static obstacle
}
EGO_COLOR = "#ffffff"
LANE_DRIVING = "#8c8c99"
LANE_OTHER = "#667359"
STOP_LINE_LIGHT = "#ff3333"
STOP_LINE_SIGN = "#ff991a"
# A light's stop line takes its observed state's color when the tick has one.
LIGHT_STATE_COLORS: dict[str, str] = {
    "red": "#ff3333",
    "yellow": "#ffcc00",
    "green": "#33cc55",
}
CROSSWALK = "#e6e6e6"
REFERENCE_LINE = "#ff8000"
REFERENCE_BOUNDS = "#ffbf4d"
PREDICTION = "#e64de6"
TRAJECTORY = "#1a99ff"
SAFE_TRAJECTORY = "#ff2626"
MPC_HORIZON = "#33ff66"
SIM_ROLLOUT = "#ffa64d"
LOCALIZATION_EST = "#66ffcc"
LOCALIZATION_GT = "#ffffff"
LOCALIZATION_CORRECTION = "#ff66a3"

# Occupancy raster palette (RGB in [0, 1]); priority occupied > dynamic >
# unknown > drivable > free, as in the marker node.
OCC_FREE = (1.0, 1.0, 1.0)
OCC_DRIVABLE = (0.88, 0.94, 0.88)
OCC_UNKNOWN = (0.59, 0.59, 0.59)
OCC_DYNAMIC = (0.24, 0.47, 1.0)
OCC_OCCUPIED = (0.86, 0.16, 0.16)
OCC_THRESHOLD = 0.5
OCC_ALPHA = 0.55

LINE_WIDTH_LANE = 0.8
LINE_WIDTH_REFERENCE = 2.0
LINE_WIDTH_CANDIDATE = 0.6
LINE_WIDTH_TRAJECTORY = 2.5
LINE_WIDTH_PREDICTION = 1.2
FONT_SIZE = 8
FONT_SIZE_LABEL = 6


def cost_ramp(quantile: float) -> tuple[float, float, float]:
    """Green -> yellow -> red over the cost quantile in [0, 1] (cheapest green)."""
    q = min(1.0, max(0.0, quantile))
    return (min(1.0, 2.0 * q), min(1.0, 2.0 * (1.0 - q)), 0.1)
