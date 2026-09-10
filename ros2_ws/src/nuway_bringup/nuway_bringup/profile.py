"""Profile YAML (docs/02 §5) -> per-node parameter dicts for the launch files (M0 §2.10).

A profile is one YAML per stack configuration. Every node gets its package's
``config/defaults.yaml`` block, then the profile keys that map onto its
parameters (``PROFILE_PARAMS`` below), then any ``<node_name>:`` block of the
profile verbatim (nested keys flattened to dotted names), then
``use_sim_time: true``. Relative paths in the profile are resolved against the
repo root (``NUWAY_ROOT`` from ``setup_env.sh``, else the working directory),
so ``ros2 launch`` works from any directory.

No rclpy import: ``tests/integration/test_bringup_profile.py`` exercises this
module without a ROS graph.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

PROFILE_DIR = Path("configs/profiles")
# Profile keys that carry a path relative to the repo root.
PATH_KEYS = ("sensors", "vehicle", "carla.town", "map_dir")
# (profile key, node parameter) per node. Nodes absent here read defaults only.
PROFILE_PARAMS: dict[str, tuple[tuple[str, str], ...]] = {
    "world_manager": (
        ("carla.host", "carla.host"),
        ("carla.port", "carla.port"),
        ("carla.town", "carla.town"),
        ("carla.fixed_delta_seconds", "carla.fixed_delta_seconds"),
        ("carla.sync", "carla.sync"),
        ("carla.no_rendering", "carla.no_rendering"),
        ("carla.lockstep_timeout_s", "carla.lockstep_timeout_s"),
        ("carla.lockstep_startup_timeout_s", "carla.lockstep_startup_timeout_s"),
        ("carla.realtime_factor", "carla.realtime_factor"),
        ("carla.traffic.tm_port", "carla.traffic.tm_port"),
        ("carla.traffic.seed", "carla.traffic.seed"),
        ("sensors", "sensors"),
        ("vehicle", "vehicle"),
    ),
    "control_adapter": (("vehicle", "vehicle"),),
    "map_server_node": (("carla.town", "town"),),
    "pure_pursuit_pid_node": (("vehicle", "vehicle"),),
}


def repo_root() -> Path:
    """Repo root: ``NUWAY_ROOT`` (exported by setup_env.sh) or the working directory."""
    return Path(os.environ.get("NUWAY_ROOT", os.getcwd())).resolve()


def resolve_profile_path(profile: str) -> Path:
    """``m0_gt_all`` -> ``<root>/configs/profiles/m0_gt_all.yaml``; a path is used as is."""
    candidate = Path(profile)
    if candidate.suffix in (".yaml", ".yml") or candidate.exists():
        return candidate if candidate.is_absolute() else repo_root() / candidate
    return repo_root() / PROFILE_DIR / f"{profile}.yaml"


def flatten(mapping: Mapping[str, Any], prefix: str = "") -> dict[str, Any]:
    """Nested mapping -> dotted keys (lists stay lists)."""
    out: dict[str, Any] = {}
    for key, value in mapping.items():
        name = f"{prefix}{key}"
        if isinstance(value, Mapping):
            out.update(flatten(value, f"{name}."))
        else:
            out[name] = value
    return out


def load_profile(path: Path) -> dict[str, Any]:
    """Read a profile YAML into a nested dict (empty blocks become {})."""
    with Path(path).open() as f:
        loaded = yaml.safe_load(f) or {}
    if not isinstance(loaded, dict):
        msg = f"{path}: profile must be a mapping"
        raise ValueError(msg)
    return loaded


def get(profile: Mapping[str, Any], dotted: str, default: object = None) -> object:
    """Look ``a.b.c`` up in a nested mapping."""
    node: Any = profile
    for part in dotted.split("."):
        if not isinstance(node, Mapping) or part not in node:
            return default
        node = node[part]
    return node


def resolve_paths(flat: dict[str, Any], root: Path) -> dict[str, Any]:
    """Make the path-valued keys absolute (``carla.town`` only when it is a .xodr file)."""
    out = dict(flat)
    for key in PATH_KEYS:
        value = out.get(key)
        if not isinstance(value, str) or not value:
            continue
        if key == "carla.town" and not value.endswith(".xodr"):
            continue
        if not Path(value).is_absolute():
            out[key] = str(root / value)
    return out


def load_defaults(defaults_file: Path | None, node: str) -> dict[str, Any]:
    """Read the ``<node>: ros__parameters:`` block of a package defaults file, flattened."""
    if defaults_file is None or not Path(defaults_file).exists():
        return {}
    with Path(defaults_file).open() as f:
        loaded = yaml.safe_load(f) or {}
    block = (
        loaded.get(node, {}).get("ros__parameters", {})
        if isinstance(loaded, dict)
        else {}
    )
    return flatten(block) if isinstance(block, Mapping) else {}


def node_params(
    profile: Mapping[str, Any],
    node: str,
    defaults_file: Path | None = None,
    root: Path | None = None,
) -> dict[str, Any]:
    """Parameters of ``node``: defaults < mapped profile keys < ``<node>:`` block < use_sim_time."""
    base = root if root is not None else repo_root()
    params = load_defaults(defaults_file, node)
    for profile_key, param in PROFILE_PARAMS.get(node, ()):
        value = get(profile, profile_key)
        if value is not None:
            params[param] = value
    block = profile.get(node)
    if isinstance(block, Mapping):
        params.update(flatten(block))
    params = resolve_paths(params, base)
    if node == "map_server_node":
        town = params.get("town")
        if isinstance(town, str) and town.endswith(".xodr"):
            params["town"] = Path(town).stem
    params["use_sim_time"] = True
    return params


def use_gt(profile: Mapping[str, Any], module: str) -> bool:
    """Read the ``use_gt.<module>`` toggle (docs/02 §6); missing means GT."""
    return bool(get(profile, f"use_gt.{module}", True))


def prediction_source(profile: Mapping[str, Any]) -> str:
    """``prediction.source``: const_vel | learned (docs/02 §5); missing means const_vel."""
    return str(get(profile, "prediction.source", "const_vel"))


def controller(profile: Mapping[str, Any]) -> str:
    """``control.controller``: pure_pursuit | mpc | none (docs/02 §5)."""
    return str(get(profile, "control.controller", "pure_pursuit"))
