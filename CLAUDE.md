# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. It is the single home for working procedure; `docs/` is the spec and is authoritative for everything it describes.

## What this is

A toy modular autonomous-driving stack for CARLA 0.9.16 on plain ROS 2 Jazzy (no Autoware), C++17 runtime nodes, Python 3.12 for ML/GT nodes/training. Built milestone by milestone (M0 → M10) against the design docs.

## How to work

- Read, in order: `docs/00_overview.md` → `docs/01_directory_structure.md` → `docs/02_interfaces.md` → `docs/03_style_and_conventions.md` → the current milestone doc in full. `docs/04_setup.md` is the machine-setup procedure.
- The current milestone is the lowest one in `docs/milestones/` whose **Completion criteria** are not all ticked. Work its **Task list** in order; a task that depends on an earlier unchecked task waits. Tick `[ ]` → `[x]` in place when a task is done and verified, and tick any completion criterion it fully satisfies.
- Outline the plan for multi-file or multi-package work before writing code.
- Small compilable increments: build and test the touched package after every change (`colcon build --packages-select <pkg>` + its tests for C++; `tools/lint/lint_py.sh --fix` + the relevant `pytest` subset for Python).
- When a spec is ambiguous, choose the simplest option that satisfies the completion criterion and append a dated entry to the milestone's **Decisions log**.
- New messages, topics, services, or profile-level config keys go into `docs/02_interfaces.md` first. Rename a node, topic, param, CLI flag, or file → update `docs/` in the same commit.
- **Debug from the renders.** After an eval run read `data/eval_runs/<run_id>/report.md` and the incident PNGs it links; `tools/viz/render_bag.py` renders any bag; training renders are under `data/checkpoints/<exp>/<ts>/viz/`. Foxglove and the W&B UI are for humans; you cannot see them (`docs/02_interfaces.md` §8).
- Run `/verify` before every commit; every step must be clean. A commit that skipped the lint/format/tidy/test sequence in `docs/03_style_and_conventions.md` §7.4 is treated as a bug.
- Work on a feature branch and open a PR with `gh`; do not commit to `main` directly. Commit subject `<type>: <lowercase imperative summary>` (`docs:`, `feat:`, `fix:`, ...), body as `- ` bullets explaining decisions.
- Commit in small, meaningful units on the PR branch: one logical change per commit (a task, a fix, a rename with its doc update), each of which builds and passes `/verify` on its own. Do not batch a whole milestone into one commit, and do not split one logical change across commits that only pass together.
- Never commit anything under `data/` or `external/`. Never leave `main` unable to complete a route with some combination of `use_gt.*` toggles.

## Environment and commands

```bash
source setup_env.sh                  # once per shell: sources ROS, creates/syncs .venv, sources ros2_ws/install
colcon build --packages-select <pkg> # defaults from ros2_ws/colcon_defaults.yaml
colcon test --packages-select <pkg> && colcon test-result --verbose
tools/lint/format_cpp.sh --fix       # uv run clang-format + gersemi
tools/lint/tidy_cpp.sh --packages <pkg>
tools/lint/lint_py.sh --fix          # ruff format, ruff check --fix, mypy
uv run pytest ml tools               # markers: slow, carla, gpu, import_light
uv sync                              # NO torch: the GT-only env for M0–M2 and the import_light CI job
uv sync --group carla --group train --group viz --group leaderboard   # full workstation
tools/carla/start_carla.sh           # wraps: ~/carla/CarlaUE4.sh -RenderOffScreen --ros2 -carla-rpc-port=2000
ros2 launch nuway_bringup stack.launch.py profile:=m0_gt_all
```

- Python deps enter only via `uv add` (lock in the same commit). No `pip`, `conda`, `poetry`, `venv`, `requirements.txt`, no `make`. Tools (`ruff`, `mypy`, `clang-format`, `clang-tidy`, `gersemi`, `pre-commit`) are `uv run` binaries pinned in `uv.lock`, not apt.
- The venv must come from `/usr/bin/python3.12` with `--system-site-packages` (`rclpy` ABI). Never create it by hand; `setup_env.sh` does it.
- `torch` is the `nuway-ml[torch]` extra on the **cu126** index (not cu130), pulled in only by the `train`/`infer` groups. Modules that must import without torch (`nuway_ml/common/*`, `data/gt_occupancy.py`) import it lazily inside functions.
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. The per-topic `Failed to parse type hash ... USER_DATA '(null)'` warning from CARLA topics is benign.
- `CYCLONEDDS_URI` points at `configs/cyclonedds.xml` (exported by `setup_env.sh`): CycloneDDS allows only 10 participants per host by default and the M0 stack alone has 9 nodes, so without it `ros2 topic`, the eval harness and Foxglove fail with `Failed to find a free participant index`.
- Sanitizer build: `colcon build --cmake-args -DNUWAY_SANITIZE=address,undefined --build-base build_asan --install-base install_asan`. Tidy-in-build: `-DNUWAY_CLANG_TIDY=ON`.

## CARLA gotchas (verified on the dev box; details in `docs/00_overview.md` §4, `docs/04_setup.md`)

- **Never pass `-quality-level=Low`**: the server segfaults on the first `load_world()`. Run at default Epic quality and accept ~3× slower rendering.
- Sensor blueprints need the `ros_name` attribute (not `role_name`); otherwise topics come out as `/carla/actorNNN/...`.
- CARLA's native `camera_info` is broken (`fx ≈ -22973`). No node subscribes to it; `sensor_rig.py` publishes ours from the rig JSON.
- Native `/carla/**` topics are already ROS-convention. `carla_conv` applies only to data read through the CARLA Python API, and only `nuway_common/carla_conv.hpp` + `nuway_ml/common/carla_conv.py` may contain conversion arithmetic.
- `enable_for_ros()` exists on sensors only, not vehicles.

## Architecture reminders (full statements in the docs)

- Lockstep (`docs/00_overview.md` §2.5, `docs/02_interfaces.md` §2): only `world_manager` ticks and owns `/clock`; no wall-clock timers in nodes; current-tick barrier and no-input convention on every node; fallbacks only after `/nuway/sim/tick_timeout`.
- `nuway_msgs` is the only package defining msgs/srvs. `ml/` never imports `rclpy`. `tools/` imports from `ros2_ws` only via the `nuway_py` pybind package (`docs/01_directory_structure.md` Rules).
- `nuway_ml/common/*` mirrors `nuway_common/*`; keep `tests/integration/test_geometry_parity.py` green.
- Every visualization has a headless matplotlib/Agg twin writing PNGs under `data/`; a Foxglove-only layer is incomplete.

## Style deltas from Google C++ / PEP 8 (full tables: `docs/03_style_and_conventions.md` §2–§3, §9)

C++: `.hpp`/`.cpp`; include guards `NUWAY_<PKG>_<FILE>_HPP_`, `#pragma once` forbidden; braces on every single-statement `if`/`for`/`while`; no exceptions in project code (return `std::optional`/`bool`/result struct, catch third-party throws at the node boundary); functions `CamelCase`, cheap accessors `lower_snake_case`, constants `kCamelCase`, members trailing `_`; units in names (`speed_mps`, `dt_s`, `yaw_rad`); no `using namespace`, no RTTI, no `std::bind`, `enum class` always; gtest files `<unit>_test.cpp` under `test/` must not link `rclcpp`.

Python: full annotations, `mypy --strict` for `ml/`, `tools/`, `tests/`; 3.12 syntax only (`X | None`, `dict[str, int]`); every `# noqa` / `# type: ignore` carries a code and a justification; `frozen=True, slots=True` dataclasses or `TypedDict` at module boundaries; rclpy nodes end in `Node` with `self._pub_<what>`, `self._sub_<what>`, `_on_<message>()`; training entry points are Hydra apps (`argparse` forbidden there; `tools/` and `ml/scripts/` keep `argparse`); `wandb` imported only in `run_logger.py` and entry points.

Precedence on conflict: formatter output > linter > `03_style_and_conventions.md` > upstream style guide > ROS 2 conventions.
