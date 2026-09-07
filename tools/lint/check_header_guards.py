"""Check include guards of every nuway C++ header (M0).

Expected macro: ``NUWAY_<PACKAGE>_<PATH>_<FILE>_HPP_`` derived from the header's
path below its package's ``include/`` directory (``docs/03_style_and_conventions.md``
§2.2), e.g. ``include/nuway_common/frenet.hpp`` -> ``NUWAY_COMMON_FRENET_HPP_``.
Headers outside ``include/`` use their path relative to the package directory,
prefixed with the package name (``nuway_map/src/foo.hpp`` -> ``NUWAY_MAP_SRC_FOO_HPP_``).
``#pragma once`` is rejected. Exit status 1 lists every offending file.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = REPO_ROOT / "ros2_ws" / "src"


def expected_guard(header: Path, package_dir: Path) -> str:
    """Return the guard macro a header must use."""
    include_dir = package_dir / "include"
    if include_dir in header.parents:
        rel = header.relative_to(include_dir)
    else:
        rel = Path(package_dir.name) / header.relative_to(package_dir)
    parts = [p.upper() for p in rel.with_suffix("").parts] + ["HPP"]
    return "_".join(parts).replace("-", "_") + "_"


def check_header(header: Path, package_dir: Path) -> str | None:
    """Return an error message for the header, or None if it is compliant."""
    text = header.read_text()
    if re.search(r"^\s*#\s*pragma\s+once", text, flags=re.MULTILINE):
        return "uses #pragma once"
    guard = expected_guard(header, package_dir)
    ifndef = re.search(r"^#ifndef\s+(\S+)", text, flags=re.MULTILINE)
    define = re.search(r"^#define\s+(\S+)", text, flags=re.MULTILINE)
    endif = re.search(r"^#endif\s*//\s*(\S+)\s*$", text, flags=re.MULTILINE)
    if ifndef is None or define is None:
        return f"missing include guard {guard}"
    if ifndef.group(1) != guard or define.group(1) != guard:
        return f"guard is {ifndef.group(1)}, expected {guard}"
    if endif is None or endif.group(1) != guard:
        return f"closing line must be '#endif  // {guard}'"
    return None


def iter_headers(src_dir: Path) -> list[tuple[Path, Path]]:
    """Return (header, package_dir) pairs for every nuway_* package."""
    pairs: list[tuple[Path, Path]] = []
    for package_dir in sorted(src_dir.glob("nuway_*")):
        if not (package_dir / "package.xml").is_file():
            continue
        for header in sorted(package_dir.rglob("*.hpp")):
            pairs.append((header, package_dir))
    return pairs


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src-dir", type=Path, default=SRC_DIR)
    args = parser.parse_args(argv)
    errors = 0
    for header, package_dir in iter_headers(args.src_dir):
        problem = check_header(header, package_dir)
        if problem is not None:
            print(f"{header.relative_to(REPO_ROOT)}: {problem}")
            errors += 1
    if errors:
        print(f"{errors} header(s) with bad include guards")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
