# Foxglove layouts

Import a layout in Foxglove (Layouts → Import from file) and connect to
`ws://<devbox>:8765` (`foxglove_bridge`, launched by `viz.launch.py`).

- `bev_m0.json` (M0 task 12): BEV 3D panel following `base_link` with the
  `lanes` and `reference_line` layers (the reference line plus the pure
  pursuit lookahead point), control and speed plots, the raw
  `ControlCommand` and the `world_manager` diag. The LiDAR topic is listed
  but hidden.

Layouts are for a human at the devbox; every drawn layer has a headless
matplotlib twin under `ml/nuway_ml/viz/` (M1 task 13). This M0 layout is the
one known exception until then (`M0_bringup.md` task 12).
