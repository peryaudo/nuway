#!/usr/bin/env bash
# PostToolUse (Write|Edit) hook: format the edited file with the uv-pinned tools.
# No-op until M0 task 1 lands uv.lock / .clang-format (docs/03_style_and_conventions.md §7).
set -u
root="${CLAUDE_PROJECT_DIR:-$(pwd)}"
f="$(jq -r '.tool_response.filePath // .tool_input.file_path // empty')"
[ -n "$f" ] && [ -f "$f" ] || exit 0
[ -f "$root/uv.lock" ] || exit 0
cd "$root" || exit 0
case "$f" in
  *.py)  uv run --frozen ruff format "$f" ;;
  *.cpp|*.hpp)
    [ -f "$root/.clang-format" ] || exit 0
    uv run --frozen clang-format -i "$f" ;;
esac
