#!/usr/bin/env bash
# tools/lint/lint_py.sh [--fix] [files...]
# ruff format, ruff check and mypy from the repo root with the root pyproject.toml
# (docs/03_style_and_conventions.md §7.4). Without --fix, format is a check.
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
if [ "${#files[@]}" -eq 0 ]; then files=("."); fi

status=0
if [ "$fix" -eq 1 ]; then
  uv run --frozen ruff format "${files[@]}" || status=1
  uv run --frozen ruff check --fix "${files[@]}" || status=1
else
  uv run --frozen ruff format --check "${files[@]}" || status=1
  uv run --frozen ruff check "${files[@]}" || status=1
fi
# mypy always checks the configured `files` list, so the whole tree stays consistent.
uv run --frozen mypy || status=1
exit "$status"
