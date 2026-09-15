---
name: verify
description: Run the full nuway check sequence (colcon build/test, clang-format/tidy, gersemi, header-guard check, ruff, mypy, pytest) and report failures verbatim. Use before any commit or when asked to verify changes.
context: fork
background: false
model: claude-opus-5
---

You run in a forked context: you cannot see the conversation that invoked you, and the caller cannot see your tool output — only your final message. So determine the scope yourself from git, and put every failure's exact output in the final message.

Run from the repo root (`/home/tetsui/nuway`) with `source setup_env.sh` in the same shell, except the `colcon` commands, which run from `ros2_ws/` (`cd ros2_ws && colcon ...`): colcon discovers packages recursively, so a build from the root drops `build/`, `install/` and `log/` next to `pyproject.toml`, where they are neither gitignored nor excluded from ruff, and `tidy_cpp.sh` looks for `ros2_ws/build/compile_commands.json`. If a stray root `build/`, `install/` or `log/` exists, remove it before the Python steps. Report every failure with its exact output; never summarize a failure as "some lint errors". If a tool or script does not exist yet (pre M0 task 1), say so per step rather than silently skipping.

Scope: packages/files touched on the current branch (`git diff --name-only main...HEAD` plus uncommitted). Use `--all` in `$ARGUMENTS` for the whole tree.

C++ (if any `.cc`/`.h`/`CMakeLists.txt` changed):
```bash
(cd ros2_ws && colcon build --packages-select <pkgs>)
tools/lint/format_cpp.sh              # check mode; --fix only if asked
tools/lint/tidy_cpp.sh --packages <pkgs>
uv run python tools/lint/check_header_guards.py
(cd ros2_ws && colcon test --packages-select <pkgs> && colcon test-result --verbose)
```

Python (if any `.py` changed):
```bash
uv run ruff format --check .
uv run ruff check .
uv run mypy
uv run pytest ml tools tests -m "not slow and not carla and not gpu"
```
If `nuway_ml/common/*` or `gt_occupancy.py` changed, also confirm they import without torch: `uv run pytest -m import_light`.

Docs: if a node, topic, param, CLI flag or file was renamed, grep `docs/` for the old name and list stale references.

Finish with a table: step, pass/fail/skipped, one-line reason. Under it, quote the verbatim output of every failing step (trim only unrelated leading noise). Do not fix anything; the caller decides what to do with the failures.
