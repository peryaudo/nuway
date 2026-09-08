"""nuway_perception: map-frame agents expressed in base_link (no ROS graph)."""

# ruff: noqa: PT009 -- colcon runs these with unittest discovery (no pytest in the system interpreter)
from __future__ import annotations

import math
import unittest

import numpy as np
from nuway_msgs.msg import Agent

from nuway_ml.common.geometry import SE2, SE3, rpy_to_quaternion
from nuway_perception.gt_perception_node import HISTORY_LEN, agent_in_base_link
from nuway_rclpy.ros_conv import pose_from_xyz_yaw


class AgentInBaseLinkTest(unittest.TestCase):
    """An agent 10 m ahead of an ego facing +y ends up on base_link's +x axis (float32 fields)."""

    def test_transform(self) -> None:
        ego_2d = SE2(5.0, 5.0, math.pi / 2)
        ego = SE3(np.array([5.0, 5.0, 0.0]), rpy_to_quaternion(0.0, 0.0, math.pi / 2))
        agent = Agent()
        agent.id = 7
        agent.pose = pose_from_xyz_yaw(5.0, 15.0, 0.0, math.pi / 2)
        agent.vx = 0.0
        agent.vy = 3.0  # moving along map +y, i.e. straight ahead of the ego
        agent.history_len = 1
        history = np.zeros((HISTORY_LEN, 3))
        history[0] = [5.0, 15.0, math.pi / 2]
        agent.history = [float(v) for v in history.reshape(-1)]
        out = agent_in_base_link(agent, ego, ego_2d)
        self.assertEqual(out.id, 7)
        self.assertAlmostEqual(out.pose.position.x, 10.0, places=5)
        self.assertAlmostEqual(out.pose.position.y, 0.0, places=5)
        self.assertAlmostEqual(out.vx, 3.0, places=5)
        self.assertAlmostEqual(out.vy, 0.0, places=5)
        self.assertAlmostEqual(out.history[0], 10.0, places=5)
        self.assertAlmostEqual(out.history[1], 0.0, places=5)
        self.assertAlmostEqual(out.history[2], 0.0, places=5)


if __name__ == "__main__":
    unittest.main()
