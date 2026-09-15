# Foxglove layouts

Import a layout in Foxglove (Layouts → Import from file) and connect to
`ws://<devbox>:8765` (`foxglove_bridge`, launched by `viz.launch.py`).

- `bev_m0.json` (M0 task 12): BEV 3D panel following `base_link` with the
  `lanes` and `reference_line` layers (the reference line plus the pure
  pursuit lookahead point), control and speed plots, the raw
  `ControlCommand` and the `world_manager` diag. The LiDAR topic is listed
  but hidden.
- `bev_planning.json` (M1 task 12): the M1 §3.11 live view. The 3D panel
  draws every `/nuway/viz/<layer>` of `docs/02_interfaces.md` §3.9 that M1
  produces — `lanes`, `reference_line` (line, drivable bounds), `agents`
  (perception, filled by class), `gt_agents` (hollow, off by default),
  `predictions` (polylines faded with time), `candidates` (thin, green →
  red by cost quantile), `trajectory` (thick blue), `safe_trajectory` (red,
  only where it differs from the plan), `mpc_horizon` (green) — with the
  `occupancy` raster in an Image panel below it; then control, speed and
  cycle-time plots, the `BehaviorDecision`, the candidate costs
  (`cost` table, `cost_breakdown_names`, `selected_index`) and the safety
  layer / planner / MPC diag messages.

Layouts are for a human at the devbox; every drawn layer has a headless
matplotlib twin under `ml/nuway_ml/viz/` (`draw_<layer>()` in `bev_draw.py`,
M1 task 13), which is what `tools/viz/render_bag.py` and the eval report use.
