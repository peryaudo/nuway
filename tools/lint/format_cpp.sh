#!/usr/bin/env bash
# tools/lint/format_cpp.sh [--fix] [files...]
# clang-format (C++) and gersemi (CMake) over every nuway_* package and tests/,
# through the uv-pinned binaries (docs/03_style_and_conventions.md §7.4).
# Without --fix it is a check and exits non-zero on any diff.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"

fix=0
files=()
for arg in "$@"; do
  case "$arg" in
    --fix) fix=1 ;;
    *) files+=("$arg") ;;
  esac
done

cpp_files=()
cmake_files=()
if [ "${#files[@]}" -eq 0 ]; then
  while IFS= read -r f; do cpp_files+=("$f"); done < <(
    find ros2_ws/src -path 'ros2_ws/src/nuway_*' \( -name '*.h' -o -name '*.cc' \) | sort
    find tests -type f \( -name '*.h' -o -name '*.cc' \) 2>/dev/null | sort
  )
  while IFS= read -r f; do cmake_files+=("$f"); done < <(
    find ros2_ws/src -path 'ros2_ws/src/nuway_*' \( -name 'CMakeLists.txt' -o -name '*.cmake' \) | sort
  )
else
  for f in "${files[@]}"; do
    case "$f" in
      *.h|*.cc) cpp_files+=("$f") ;;
      CMakeLists.txt|*/CMakeLists.txt|*.cmake) cmake_files+=("$f") ;;
    esac
  done
fi

status=0
if [ "${#cpp_files[@]}" -gt 0 ]; then
  if [ "$fix" -eq 1 ]; then
    uv run --frozen clang-format -i "${cpp_files[@]}"
  else
    uv run --frozen clang-format --dry-run --Werror "${cpp_files[@]}" || status=1
  fi
fi
if [ "${#cmake_files[@]}" -gt 0 ]; then
  if [ "$fix" -eq 1 ]; then
    uv run --frozen gersemi -i "${cmake_files[@]}"
  else
    uv run --frozen gersemi --check "${cmake_files[@]}" || status=1
  fi
fi
if [ "$status" -ne 0 ]; then
  echo "format_cpp.sh: formatting differences found (run with --fix)" >&2
fi
exit "$status"
