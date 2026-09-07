"""world_manager (hosts gt_publisher and sensor_rig) + control_adapter (M0 §2.10).

control_adapter is skipped when control.controller is `none` (sysid: the
sweep script writes the raw CARLA control itself, M0 §2.6).
"""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch, stack_node
from nuway_bringup.profile import controller


def setup(_context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    nodes = [
        stack_node(profile, "nuway_carla_bridge", "world_manager", "world_manager")
    ]
    if controller(profile) != "none":
        nodes.append(
            stack_node(
                profile, "nuway_carla_bridge", "control_adapter", "control_adapter"
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
