"""Visualization: marker_node and foxglove_bridge (docs/02 §3.9).

`foxglove:=false` skips the bridge (CI, headless eval). The layouts are in
nuway_viz/foxglove/; they serve a human at the devbox (docs/02 §8).
"""

from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from nuway_bringup.launch_util import profile_launch, stack_node


def setup(context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    actions: list[Any] = [
        stack_node(profile, "nuway_viz", "marker_node", "marker_node")
    ]
    if LaunchConfiguration("foxglove").perform(context).lower() in ("true", "1", "yes"):
        bridge = Path(get_package_share_directory("foxglove_bridge")) / "launch"
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(bridge / "foxglove_bridge_launch.xml")),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return profile_launch(
        setup,
        extra_args=[
            DeclareLaunchArgument(
                "foxglove", default_value="true", description="also run foxglove_bridge"
            )
        ],
    )
