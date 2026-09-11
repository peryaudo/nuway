"""Prediction: const_vel_node in every profile (M1 §3.1), the learned/GT primary later.

const_vel_node always publishes ``/nuway/prediction/fallback_samples``; it
publishes ``/nuway/prediction/samples`` too when it is the primary producer,
i.e. ``use_gt.prediction`` is off and ``prediction.source`` is ``const_vel``
(docs/02 §3.6, §6).
"""

from typing import Any

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from nuway_bringup.launch_util import profile_launch, stack_node
from nuway_bringup.profile import get, prediction_source


def setup(context: LaunchContext, profile: dict[str, Any]) -> list[Any]:
    # Unlike the other toggles, use_gt.prediction defaults to false: the GT
    # twin needs a recorded log (docs/02 §5).
    source = prediction_source(profile)
    if bool(get(profile, "use_gt.prediction", False)):
        msg = "use_gt.prediction: true needs the M6 gt_prediction_node"
        raise NotImplementedError(msg)
    if source == "learned":
        msg = "prediction.source: learned needs the M7 prediction_node"
        raise NotImplementedError(msg)
    if source != "const_vel":
        msg = f"prediction.source: unknown value {source!r}"
        raise ValueError(msg)
    # The M1 §5 delay injection: a wall-clock sleep in the node's tick that
    # must not change any result under the current-tick barrier.
    delay_ms = float(LaunchConfiguration("callback_delay_ms").perform(context))
    return [
        stack_node(
            profile,
            "nuway_prediction",
            "const_vel_node",
            "const_vel_node",
            extra_params={"publish_primary": True, "callback_delay_ms": delay_ms},
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return profile_launch(
        setup,
        extra_args=[
            DeclareLaunchArgument(
                "callback_delay_ms",
                default_value="0",
                description="wall-clock sleep injected into const_vel_node's tick",
            )
        ],
    )
