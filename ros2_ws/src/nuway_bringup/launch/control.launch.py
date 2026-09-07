"""Control: control.controller selects pure_pursuit (M0), mpc (M1) or none (sysid)."""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch, stack_node
from nuway_bringup.profile import controller


def setup(_context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    choice = controller(profile)
    if choice == "none":
        return []
    if choice == "pure_pursuit":
        return [
            stack_node(
                profile,
                "nuway_control",
                "pure_pursuit_pid_node",
                "pure_pursuit_pid_node",
            )
        ]
    if choice == "mpc":
        msg = "control.controller: mpc needs the M1 mpc_node"
        raise NotImplementedError(msg)
    msg = f"control.controller: unknown value {choice!r}"
    raise ValueError(msg)


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
