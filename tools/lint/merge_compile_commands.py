"""Merge colcon's per-package compile_commands.json into one database (M0).

colcon writes ``ros2_ws/build/<pkg>/compile_commands.json`` per package; clangd
and ``run-clang-tidy`` want a single ``ros2_ws/build/compile_commands.json``
(``docs/03_style_and_conventions.md`` §6.2). Vendored and generated packages are
skipped so clang-tidy never sees code we do not own.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = REPO_ROOT / "ros2_ws" / "build"
SKIPPED_PACKAGES = {"carla_msgs", "nuway_msgs"}


def merge(build_dir: Path) -> list[dict[str, str]]:
    """Return the union of every ``<pkg>/compile_commands.json`` under build_dir."""
    entries: list[dict[str, str]] = []
    for db in sorted(build_dir.glob("*/compile_commands.json")):
        if db.parent.name in SKIPPED_PACKAGES:
            continue
        with db.open() as f:
            entries.extend(json.load(f))
    return entries


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=BUILD_DIR)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)
    if not args.build_dir.is_dir():
        if not args.quiet:
            print(f"no build directory at {args.build_dir}; run colcon build first")
        return 1
    entries = merge(args.build_dir)
    out = args.build_dir / "compile_commands.json"
    with out.open("w") as f:
        json.dump(entries, f, indent=1)
    if not args.quiet:
        print(f"wrote {len(entries)} entries to {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
