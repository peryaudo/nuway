# nuway

A toy modular autonomous-driving stack for CARLA 0.9.16 on plain ROS 2 Jazzy: C++17
runtime nodes, Python 3.12 for ML, ground-truth cheat twins and training. Built milestone
by milestone (M0 → M10) against the design docs in `docs/`.

- Start with `docs/00_overview.md`, then `docs/01_directory_structure.md`,
  `docs/02_interfaces.md`, `docs/03_style_and_conventions.md`, and the milestone you work on.
- Machine setup: `docs/04_setup.md`. Then, in every shell: `source setup_env.sh`.
- Build: `colcon build` (from `ros2_ws/`, defaults in `ros2_ws/colcon_defaults.yaml`).
- Python: `uv sync`, `uv run pytest`, `tools/lint/lint_py.sh --fix`.
- Working procedure for contributors and Claude Code: `CLAUDE.md`.
