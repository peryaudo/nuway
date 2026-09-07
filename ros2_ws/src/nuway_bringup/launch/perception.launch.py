"""Perception: use_gt.perception and use_gt.traffic_lights select their node pair independently (02 §6)."""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch, stack_node
from nuway_bringup.profile import use_gt


def setup(_context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    if not use_gt(profile, "perception"):
        msg = "use_gt.perception: false needs the M3 perception_node"
        raise NotImplementedError(msg)
    if not use_gt(profile, "traffic_lights"):
        msg = "use_gt.traffic_lights: false needs the M4 traffic_light_node"
        raise NotImplementedError(msg)
    return [
        stack_node(
            profile, "nuway_perception", "gt_perception_node", "gt_perception_node"
        ),
        stack_node(
            profile,
            "nuway_perception",
            "gt_traffic_light_node",
            "gt_traffic_light_node",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
