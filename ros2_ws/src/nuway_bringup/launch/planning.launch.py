"""Planning: behavior_fsm_node + planner_node + safety_layer_node (M1), gt_planning_node (M6).

``use_gt.planning`` selects the M6 expert; every profile launches the
safety layer (M1 §3.7).
"""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch, stack_node
from nuway_bringup.profile import get


def setup(_context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    # use_gt.planning defaults to false: the GT twin is the M6 expert.
    if bool(get(profile, "use_gt.planning", False)):
        msg = "use_gt.planning: true needs the M6 gt_planning_node"
        raise NotImplementedError(msg)
    return [
        stack_node(profile, "nuway_planning", "behavior_fsm_node", "behavior_fsm_node"),
    ]


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
