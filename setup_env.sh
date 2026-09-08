#!/usr/bin/env bash
# setup_env.sh — the single definition of the nuway development environment.
# `source` it in every shell (docs/03_style_and_conventions.md §6.3). Idempotent.
#
#   1. verify prerequisites (Ubuntu 24.04, ROS Jazzy, uv, ninja, ccache, mold, cmake >= 3.28, gcc-13)
#   2. source ROS, export COLCON_DEFAULTS_FILE and RMW_IMPLEMENTATION
#   3. create .venv from /usr/bin/python3.12 with system site packages (rclpy ABI), uv sync
#   4. activate .venv, source ros2_ws/install/setup.bash if built
#   5. install the pre-commit hook if missing
#
# Environment knobs: NUWAY_FULL=1 syncs every dependency group (carla, train, viz,
# leaderboard); NUWAY_INFER=1 syncs carla + infer only.

if [ -n "${ZSH_VERSION:-}" ]; then
  _nuway_root="$(cd "$(dirname "${(%):-%N}")" && pwd)"
  _nuway_shell=zsh
else
  _nuway_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _nuway_shell=bash
fi
export NUWAY_ROOT="${_nuway_root}"

_nuway_fail() {
  echo "setup_env.sh: $1" >&2
  return 1
}

_nuway_check_prereqs() {
  local missing=()
  if ! grep -q 'VERSION_ID="24.04"' /etc/os-release 2>/dev/null; then
    echo "setup_env.sh: warning: expected Ubuntu 24.04 (docs/00_overview.md §4)" >&2
  fi
  [ -f /opt/ros/jazzy/setup.bash ] || missing+=("ros-jazzy-desktop (see docs/04_setup.md §1)")
  command -v uv >/dev/null 2>&1 || missing+=("uv  (curl -LsSf https://astral.sh/uv/install.sh | sh)")
  command -v ninja >/dev/null 2>&1 || missing+=("ninja-build")
  command -v ccache >/dev/null 2>&1 || missing+=("ccache")
  command -v mold >/dev/null 2>&1 || missing+=("mold")
  command -v gcc-13 >/dev/null 2>&1 || missing+=("gcc-13 g++-13")
  [ -x /usr/bin/python3.12 ] || missing+=("python3.12 (system)")
  if command -v cmake >/dev/null 2>&1; then
    local cmake_ver
    cmake_ver="$(cmake --version | head -1 | awk '{print $3}')"
    if [ "$(printf '%s\n' "3.28" "${cmake_ver}" | sort -V | head -1)" != "3.28" ]; then
      missing+=("cmake >= 3.28 (have ${cmake_ver})")
    fi
  else
    missing+=("cmake")
  fi
  if [ "${#missing[@]}" -ne 0 ]; then
    echo "setup_env.sh: missing prerequisites:" >&2
    printf '  - %s\n' "${missing[@]}" >&2
    echo "  sudo apt-get install -y build-essential gcc-13 g++-13 cmake ninja-build ccache mold" >&2
    return 1
  fi
}

_nuway_check_prereqs || return 1

# 2. ROS + colcon defaults + middleware.
# shellcheck disable=SC1091
source "/opt/ros/jazzy/setup.${_nuway_shell}"
export COLCON_DEFAULTS_FILE="${NUWAY_ROOT}/ros2_ws/colcon_defaults.yaml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Raise CycloneDDS's participant cap (10 per host by default; the stack alone has 9 nodes).
export CYCLONEDDS_URI="file://${NUWAY_ROOT}/configs/cyclonedds.xml"

# 3. uv venv from the system interpreter (rclpy ABI), then sync.
if [ ! -f "${NUWAY_ROOT}/.venv/pyvenv.cfg" ]; then
  echo "setup_env.sh: creating .venv from /usr/bin/python3.12 (system site packages)"
  (cd "${NUWAY_ROOT}" && uv venv --python /usr/bin/python3.12 --system-site-packages .venv) \
    || _nuway_fail "uv venv failed" || return 1
fi
# Without a flag the sync is --inexact: it installs what the lock requires but
# keeps groups a previous NUWAY_FULL=1 / NUWAY_INFER=1 sync already installed,
# so re-sourcing in a new shell never strips torch or carla from a workstation.
_nuway_sync_args=(--inexact)
if [ "${NUWAY_FULL:-0}" = "1" ]; then
  _nuway_sync_args=(--group carla --group train --group viz --group leaderboard)
elif [ "${NUWAY_INFER:-0}" = "1" ]; then
  _nuway_sync_args=(--group carla --group infer)
fi
(cd "${NUWAY_ROOT}" && uv sync "${_nuway_sync_args[@]}") || _nuway_fail "uv sync failed" || return 1
unset _nuway_sync_args

# 4. activate the venv, then the colcon workspace if it has been built.
# shellcheck disable=SC1091
source "${NUWAY_ROOT}/.venv/bin/activate"
# ament_python entry points are generated with a /usr/bin/python3 shebang, so
# `ros2 run` / `ros2 launch` would not see the venv even when it is activated.
# The venv is that same interpreter with system site packages, so exposing its
# site-packages on PYTHONPATH is ABI-safe (docs/03 §6.1).
# nuway_ml is an editable install (a .pth file, which only site directories
# honour), so ml/ goes on PYTHONPATH as well.
for _nuway_dir in "${NUWAY_ROOT}/ml" "${NUWAY_ROOT}/.venv/lib/python3.12/site-packages"; do
  case ":${PYTHONPATH:-}:" in
    *":${_nuway_dir}:"*) ;;
    *) export PYTHONPATH="${_nuway_dir}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
done
unset _nuway_dir
if [ -f "${NUWAY_ROOT}/ros2_ws/install/setup.${_nuway_shell}" ]; then
  # shellcheck disable=SC1091
  source "${NUWAY_ROOT}/ros2_ws/install/setup.${_nuway_shell}"
fi
if [ -d "${NUWAY_ROOT}/ros2_ws/build" ]; then
  (cd "${NUWAY_ROOT}" && uv run --frozen python tools/lint/merge_compile_commands.py --quiet) || true
fi

# 5. pre-commit hook.
if [ -d "${NUWAY_ROOT}/.git" ] && [ ! -f "${NUWAY_ROOT}/.git/hooks/pre-commit" ]; then
  (cd "${NUWAY_ROOT}" && uv run --frozen pre-commit install >/dev/null) || true
fi

unset _nuway_root _nuway_shell
unset -f _nuway_fail _nuway_check_prereqs
