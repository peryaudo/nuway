"""gt_perception_node (M0 §2.9): GT agents in base_link + the static-drivable occupancy grid.

On every planning tick it republishes ``/nuway/gt/agents`` transformed into
``base_link`` as ``/nuway/perception/agents`` and publishes
``/nuway/perception/occupancy`` with ``drivable`` sampled from a town-wide
raster of the lane graph (``nuway_ml.data.gt_occupancy``, the one occupancy
implementation), ``free = drivable``, ``unknown = 1 - drivable`` and the
other channels zero. M2 replaces the grid with ``generate_gt_occupancy``.

Cross-tick state is the tick barrier of :class:`GtTwinNode` (reset there) and
the static raster, which depends only on the town. numpy only; never torch.
"""

from __future__ import annotations

import sys
from typing import Any

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from numpy.typing import NDArray
from nuway_msgs.msg import (
    Agent,
    AgentArray,
    EgoState,
    Lane,
    LaneGraph,
    OccupancyGridMC,
)

from nuway_ml.common.frames import (
    FRAME_BASE_LINK,
    TOPIC_GT_AGENTS,
    TOPIC_LANE_GRAPH,
    TOPIC_PERCEPTION_AGENTS,
    TOPIC_PERCEPTION_OCCUPANCY,
)
from nuway_ml.common.geometry import (
    SE2,
    SE3,
    apply,
    compose,
    inverse,
    rotate,
    to_se2,
    wrap_angles,
)
from nuway_ml.common.occupancy import (
    NUM_OCCUPANCY_CHANNELS,
    OCCUPANCY_CHANNEL_NAMES,
    GridSpec,
)
from nuway_ml.data import gt_occupancy
from nuway_perception.gt_twin_node import GtTwinNode
from nuway_rclpy.ros_conv import pose_from_se3, se3_from_pose
from nuway_rclpy.ros_qos import qos

NODE_NAME = "gt_perception_node"
AGENTS_INPUT = "gt_agents"
HISTORY_LEN = int(Agent.HISTORY_LEN)


def agent_in_base_link(agent: Agent, ego: SE3, ego_2d: SE2) -> Agent:
    """Return a copy of ``agent`` (map frame) expressed in ``base_link``."""
    out = Agent()
    out.id = agent.id
    out.class_id = agent.class_id
    out.score = agent.score
    local = compose(inverse(ego), se3_from_pose(agent.pose))
    assert isinstance(local, SE3)
    out.pose = pose_from_se3(local)
    out.length = agent.length
    out.width = agent.width
    out.height = agent.height
    body_inv = inverse(ego_2d)
    assert isinstance(body_inv, SE2)
    v = rotate(body_inv, np.array([agent.vx, agent.vy], dtype=np.float64))
    out.vx = float(v[0])
    out.vy = float(v[1])
    out.yaw_rate = agent.yaw_rate
    out.visible = agent.visible
    out.history_len = agent.history_len
    history = np.array(agent.history, dtype=np.float64).reshape(HISTORY_LEN, 3)
    n = int(agent.history_len)
    if n > 0:
        xy = apply(body_inv, history[:n, :2])
        yaw = wrap_angles(history[:n, 2] - ego_2d.yaw)
        history[:n, :2] = xy
        history[:n, 2] = yaw
    out.history = [float(v) for v in history.reshape(-1)]
    return out


class GtPerceptionNode(GtTwinNode):
    """Cheat twin of perception_node."""

    def __init__(self) -> None:
        """Declare the grid parameters and wire the topics."""
        super().__init__(NODE_NAME, {AGENTS_INPUT: (AgentArray, TOPIC_GT_AGENTS)})
        self.declare_parameter("grid.resolution", 0.5)
        self.declare_parameter("grid.x_min", -50.0)
        self.declare_parameter("grid.y_min", -50.0)
        self.declare_parameter("grid.height", 200)
        self.declare_parameter("grid.width", 200)
        self._spec = GridSpec(
            resolution=float(self.get_parameter("grid.resolution").value),
            x_min=float(self.get_parameter("grid.x_min").value),
            y_min=float(self.get_parameter("grid.y_min").value),
            height=int(self.get_parameter("grid.height").value),
            width=int(self.get_parameter("grid.width").value),
        )
        self._raster: gt_occupancy.StaticRaster | None = None
        self._pub_agents = self.create_publisher(
            AgentArray, TOPIC_PERCEPTION_AGENTS, qos("stream")
        )
        self._pub_occupancy = self.create_publisher(
            OccupancyGridMC, TOPIC_PERCEPTION_OCCUPANCY, qos("stream")
        )
        self._sub_lane_graph = self.create_subscription(
            LaneGraph, TOPIC_LANE_GRAPH, self._on_lane_graph, qos("latched")
        )

    def _on_lane_graph(self, msg: LaneGraph) -> None:
        lanes = [
            gt_occupancy.LanePolyline(
                np.array([[p.x, p.y] for p in lane.centerline], dtype=np.float64),
                np.array(lane.width, dtype=np.float64),
            )
            for lane in msg.lanes
            if lane.type == Lane.TYPE_DRIVING
        ]
        self._raster = gt_occupancy.rasterize_drivable(lanes, self._spec.resolution)
        self.get_logger().info(
            f"lane graph: {len(lanes)} driving lanes rasterized into "
            f"{self._raster.height}x{self._raster.width} cells"
        )

    def _grid_msg(self, stamp: Time, data: NDArray[np.float32]) -> OccupancyGridMC:
        msg = OccupancyGridMC()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_BASE_LINK
        msg.resolution = float(self._spec.resolution)
        msg.x_min = float(self._spec.x_min)
        msg.y_min = float(self._spec.y_min)
        msg.height = int(self._spec.height)
        msg.width = int(self._spec.width)
        msg.num_channels = NUM_OCCUPANCY_CHANNELS
        msg.channel_names = list(OCCUPANCY_CHANNEL_NAMES)
        msg.data = data.reshape(-1).tolist()
        return msg

    def _publish_no_input(self, stamp: Time) -> None:
        agents = AgentArray()
        agents.header.stamp = stamp
        agents.header.frame_id = FRAME_BASE_LINK
        self._pub_agents.publish(agents)
        self._pub_occupancy.publish(
            self._grid_msg(stamp, gt_occupancy.no_input_grid(self._spec))
        )

    def _process(self, stamp: Time, pose: EgoState, inputs: dict[str, Any]) -> str:
        ego = se3_from_pose(pose.pose)
        ego_2d = to_se2(ego)
        agents = AgentArray()
        agents.header.stamp = stamp
        agents.header.frame_id = FRAME_BASE_LINK
        gt: AgentArray | None = inputs.get(AGENTS_INPUT)
        message = ""
        if gt is None:
            message = "no gt agents"
        else:
            agents.agents = [agent_in_base_link(a, ego, ego_2d) for a in gt.agents]
        self._pub_agents.publish(agents)
        if self._raster is None:
            self._pub_occupancy.publish(
                self._grid_msg(stamp, gt_occupancy.no_input_grid(self._spec))
            )
            message = (message + "; " if message else "") + "no lane graph"
        else:
            drivable = gt_occupancy.sample_raster(self._raster, self._spec, ego_2d)
            self._pub_occupancy.publish(
                self._grid_msg(stamp, gt_occupancy.static_grid(drivable))
            )
        return message


def main(args: list[str] | None = None) -> int:
    """Entry point."""
    rclpy.init(args=args)
    node = GtPerceptionNode()
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
