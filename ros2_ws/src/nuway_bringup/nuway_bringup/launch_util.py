"""Helpers shared by the launch files: profile-driven Node actions (M0 §2.10)."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nuway_bringup.profile import load_profile, node_params, resolve_profile_path

PROFILE_ARG = "profile"


def profile_from_context(context: LaunchContext) -> dict[str, Any]:
    """Load the profile selected by the ``profile`` launch argument."""
    name = LaunchConfiguration(PROFILE_ARG).perform(context)
    return load_profile(resolve_profile_path(name))


def defaults_file(package: str) -> Path:
    """``<share>/<package>/config/defaults.yaml`` (may not exist)."""
    return Path(get_package_share_directory(package)) / "config" / "defaults.yaml"


def stack_node(
    profile: dict[str, Any], package: str, executable: str, node: str
) -> Node:
    """Build the Node action for ``node`` with its merged parameters."""
    return Node(
        package=package,
        executable=executable,
        name=node,
        output="screen",
        parameters=[node_params(profile, node, defaults_file(package))],
    )


SetupFn = Callable[[LaunchContext, dict[str, Any]], Iterable[Any]]


def profile_launch(setup: SetupFn) -> LaunchDescription:
    """LaunchDescription with the ``profile`` argument and ``setup(context, profile)``."""

    def _launch(context: LaunchContext) -> list[Any]:
        return list(setup(context, profile_from_context(context)))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                PROFILE_ARG,
                default_value="m0_gt_all",
                description="profile name under configs/profiles/ or a YAML path",
            ),
            OpaqueFunction(function=_launch),
        ]
    )
