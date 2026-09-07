"""gt_traffic_light_node (M0 §2.9): republishes GT traffic lights as the perception topic.

``/nuway/gt/traffic_lights`` -> ``/nuway/perception/traffic_lights`` unchanged,
on planning ticks, once the pose of the tick has arrived (an invalid pose
yields an empty array, docs/02 §2). Separate from gt_perception_node so that
``use_gt.perception`` and ``use_gt.traffic_lights`` are independent launch
choices (docs/02 §6). Cross-tick state: only the barrier of GtTwinNode.
"""

from __future__ import annotations

import sys
from typing import Any

import rclpy
from builtin_interfaces.msg import Time
from nuway_msgs.msg import EgoState, TrafficLightArray

from nuway_ml.common.frames import (
    FRAME_MAP,
    TOPIC_GT_TRAFFIC_LIGHTS,
    TOPIC_PERCEPTION_TRAFFIC_LIGHTS,
)
from nuway_perception.gt_twin_node import GtTwinNode
from nuway_rclpy.ros_qos import qos

NODE_NAME = "gt_traffic_light_node"
LIGHTS_INPUT = "gt_traffic_lights"


class GtTrafficLightNode(GtTwinNode):
    """Cheat twin of traffic_light_node."""

    def __init__(self) -> None:
        """Wire the topics."""
        super().__init__(
            NODE_NAME, {LIGHTS_INPUT: (TrafficLightArray, TOPIC_GT_TRAFFIC_LIGHTS)}
        )
        self._pub_traffic_lights = self.create_publisher(
            TrafficLightArray, TOPIC_PERCEPTION_TRAFFIC_LIGHTS, qos("stream")
        )

    def _publish_no_input(self, stamp: Time) -> None:
        msg = TrafficLightArray()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_MAP
        self._pub_traffic_lights.publish(msg)

    def _process(self, stamp: Time, _pose: EgoState, inputs: dict[str, Any]) -> str:
        gt: TrafficLightArray | None = inputs.get(LIGHTS_INPUT)
        if gt is None:
            self._publish_no_input(stamp)
            return "no gt traffic lights"
        msg = TrafficLightArray()
        msg.header.stamp = stamp
        msg.header.frame_id = gt.header.frame_id or FRAME_MAP
        msg.lights = list(gt.lights)
        self._pub_traffic_lights.publish(msg)
        return ""


def main(args: list[str] | None = None) -> int:
    """Entry point."""
    rclpy.init(args=args)
    node = GtTrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
