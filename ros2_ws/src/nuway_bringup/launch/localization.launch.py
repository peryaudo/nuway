"""Localization: gt_pose_node when use_gt.localization, the M5 chain otherwise."""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch, stack_node
from nuway_bringup.profile import use_gt


def setup(_context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    if not use_gt(profile, "localization"):
        msg = "use_gt.localization: false needs the M5 localization nodes"
        raise NotImplementedError(msg)
    return [stack_node(profile, "nuway_localization", "gt_pose_node", "gt_pose_node")]


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
