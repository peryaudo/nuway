"""nuway_perception: map-frame agents expressed in base_link (no ROS graph)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from nuway_msgs.msg import Agent

from nuway_ml.common.geometry import SE2, SE3, rpy_to_quaternion
from nuway_perception.gt_perception_node import HISTORY_LEN, agent_in_base_link
from nuway_rclpy.ros_conv import pose_from_xyz_yaw


def test_agent_ahead_of_an_ego_facing_plus_y_lands_on_base_link_x() -> None:
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
    assert out.id == 7
    # float32 message fields: 1e-5 absolute.
    assert out.pose.position.x == pytest.approx(10.0, abs=1e-5)
    assert out.pose.position.y == pytest.approx(0.0, abs=1e-5)
    assert out.vx == pytest.approx(3.0, abs=1e-5)
    assert out.vy == pytest.approx(0.0, abs=1e-5)
    assert out.history[0] == pytest.approx(10.0, abs=1e-5)
    assert out.history[1] == pytest.approx(0.0, abs=1e-5)
    assert out.history[2] == pytest.approx(0.0, abs=1e-5)
