#!/usr/bin/env bash
# Clone the CARLA Leaderboard and scenario_runner into external/ at the pinned commits
# (docs/03_style_and_conventions.md §6.4; M1 §3.12). Idempotent: an existing checkout is
# fetched and reset to the pin. The Python dependencies are the `leaderboard` uv group
# (`uv sync --group leaderboard`), never installed here.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EXTERNAL="$ROOT/external"

# Pins (verified 2026-09-11 against CARLA 0.9.16; see the M1 Decisions log, task 19):
#   leaderboard master: the current Leaderboard 2.x evaluator (MIN_CARLA_VERSION 0.9.14),
#     SENSORS/MAP tracks; no ROS track exists, leaderboard_agent.py runs on SENSORS.
#   scenario_runner master: the "0.9.16 release" commit.
LEADERBOARD_REPO="https://github.com/carla-simulator/leaderboard.git"
LEADERBOARD_COMMIT="aec81319de6bec57372f25aa94083c9ab0bd4223"
SCENARIO_RUNNER_REPO="https://github.com/carla-simulator/scenario_runner.git"
SCENARIO_RUNNER_COMMIT="94ff3b8af752bad2b9d464ad5105868906aa34c0"

checkout() {
  local name=$1 repo=$2 commit=$3
  local dir="$EXTERNAL/$name"
  if [ ! -d "$dir/.git" ]; then
    git clone -q "$repo" "$dir"
  fi
  git -C "$dir" fetch -q origin
  git -C "$dir" checkout -q --detach "$commit"
  echo "$name @ $(git -C "$dir" rev-parse --short HEAD)"
}

mkdir -p "$EXTERNAL"
checkout leaderboard "$LEADERBOARD_REPO" "$LEADERBOARD_COMMIT"
checkout scenario_runner "$SCENARIO_RUNNER_REPO" "$SCENARIO_RUNNER_COMMIT"
# Upstream fixes we carry as patches on the pinned commits (tools/eval/patches/, one per fix,
# each idempotent): scenario_runner's BackgroundBehavior reads `.id` on a *list* of junctions
# when a route enters its second hard-coded "complex junction" (Town03's roundabout), which
# crashes every Town03 route at its first tick.
for patch in "$ROOT"/tools/eval/patches/scenario_runner_*.patch; do
  if git -C "$EXTERNAL/scenario_runner" apply --check --reverse "$patch" 2>/dev/null; then
    echo "patch already applied: $(basename "$patch")"
  else
    git -C "$EXTERNAL/scenario_runner" apply "$patch"
    echo "patch applied: $(basename "$patch")"
  fi
done
# The pinned runner's route parser reads only id/town/positions/weather (M1 §3.10): our
# `protocol` attribute is ignored. Checked here so a re-pin that starts validating attributes
# fails loudly.
grep -q "route.attrib\['id'\]" "$EXTERNAL/leaderboard/leaderboard/utils/route_parser.py"
echo "PYTHONPATH additions: $EXTERNAL/leaderboard:$EXTERNAL/scenario_runner:\$CARLA_ROOT/PythonAPI/carla (run_leaderboard.sh sets them)"
