#!/usr/bin/env bash
# tools/lint/tidy_cpp.sh [--fix] [--packages pkg ...] [files...]
# Merges colcon's compile databases, then runs the uv-pinned clang-tidy over the
# .cpp files of the selected nuway_* packages (default: all that have been built)
# (docs/03_style_and_conventions.md §7.4). Warnings are errors (.clang-tidy).
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"

fix=0
packages=()
files=()
mode=files
for arg in "$@"; do
  case "$arg" in
    --fix) fix=1 ;;
    --packages) mode=packages ;;
    --*) echo "unknown flag $arg" >&2; exit 2 ;;
    *) if [ "$mode" = packages ]; then packages+=("$arg"); else files+=("$arg"); fi ;;
  esac
done

uv run --frozen python tools/lint/merge_compile_commands.py --quiet
db="ros2_ws/build/compile_commands.json"
[ -f "$db" ] || { echo "tidy_cpp.sh: no $db; colcon build first" >&2; exit 1; }

if [ "${#files[@]}" -eq 0 ]; then
  if [ "${#packages[@]}" -eq 0 ]; then
    while IFS= read -r d; do packages+=("$(basename "$d")"); done < <(
      find ros2_ws/build -mindepth 1 -maxdepth 1 -type d -name 'nuway_*' | sort
    )
  fi
  for pkg in "${packages[@]}"; do
    while IFS= read -r f; do files+=("$f"); done < <(
      find "ros2_ws/src/$pkg" -name '*.cpp' | sort
    )
  done
fi
if [ "${#files[@]}" -eq 0 ]; then
  echo "tidy_cpp.sh: nothing to check"
  exit 0
fi

# Only files present in the compile database can be analysed (generated
# packages and headers-only targets are not).
checked=()
for f in "${files[@]}"; do
  abs="$(realpath "$f")"
  if grep -Fq "\"$abs\"" "$db"; then checked+=("$abs"); fi
done
if [ "${#checked[@]}" -eq 0 ]; then
  echo "tidy_cpp.sh: none of the requested files are in $db"
  exit 0
fi

extra=()
if [ "$fix" -eq 1 ]; then extra+=(--fix); fi
uv run --frozen clang-tidy -p ros2_ws/build --quiet "${extra[@]}" "${checked[@]}"
