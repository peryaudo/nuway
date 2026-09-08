"""nuway_bringup.launch_util: launch arguments are usable inside the profile setup (no ROS graph)."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any

import pytest

launch = pytest.importorskip("launch")

from launch import LaunchContext  # noqa: E402 -- after the importorskip
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402

from nuway_bringup.launch_util import profile_launch  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
LAUNCH_DIR = ROOT / "ros2_ws/src/nuway_bringup/launch"


def _visit_all(description: Any, context: LaunchContext) -> None:
    """Visit the entities in order, as the launch service does."""
    for entity in description.entities:
        entity.visit(context)


def test_extra_arguments_are_declared_before_the_setup_runs() -> None:
    seen: list[str] = []

    def setup(context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
        assert profile["profile"] == "m0_gt_all"
        seen.append(LaunchConfiguration("flag").perform(context))
        return []

    description = profile_launch(
        setup, extra_args=[DeclareLaunchArgument("flag", default_value="x")]
    )
    context = LaunchContext()
    # The profile is given as the launch service would; `flag` is left to its default.
    context.launch_configurations["profile"] = str(
        ROOT / "configs/profiles/m0_gt_all.yaml"
    )
    _visit_all(description, context)
    assert seen == ["x"]


def test_every_launch_file_declares_its_arguments_before_the_opaque_function() -> None:
    for path in sorted(LAUNCH_DIR.glob("*.launch.py")):
        spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
        assert spec is not None
        assert spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
            entities = list(module.generate_launch_description().entities)
        except (ModuleNotFoundError, LookupError) as err:
            # launch_ros missing, or the package is not in the ament index
            # (pytest without a colcon build, as in the python CI job).
            pytest.skip(f"{path.name}: {err}")
        opaque = [i for i, e in enumerate(entities) if isinstance(e, OpaqueFunction)]
        declared = [
            i for i, e in enumerate(entities) if isinstance(e, DeclareLaunchArgument)
        ]
        if opaque:
            assert max(declared) < min(opaque), path.name
