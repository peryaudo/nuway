"""map_server_node and route_planner_node (M0 §2.10; §3.3 map & route)."""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch, stack_node


def setup(_context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    return [
        stack_node(profile, "nuway_map", "map_server_node", "map_server_node"),
        stack_node(profile, "nuway_route", "route_planner_node", "route_planner_node"),
    ]


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
