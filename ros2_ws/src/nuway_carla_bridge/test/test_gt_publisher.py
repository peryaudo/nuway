"""Walker velocity from the pose ring buffer (CARLA's physics velocity is zero)."""

from __future__ import annotations

import math

from nuway_carla_bridge.gt_publisher import kinematic_velocity


def test_walker_velocity_is_the_one_tick_pose_difference() -> None:
    history = [(0.0, 0.0, 0.0), (0.05, 0.1, 0.1)]
    vx, vy, yaw_rate = kinematic_velocity(history, (0.1, 0.2, 0.15), 0.05)
    assert (vx, vy) == (1.0, 2.0)
    assert math.isclose(yaw_rate, 1.0)


def test_walker_yaw_rate_wraps_across_pi() -> None:
    _, _, yaw_rate = kinematic_velocity([(0.0, 0.0, 3.1)], (0.0, 0.0, -3.1), 0.05)
    assert math.isclose(yaw_rate, (2 * math.pi - 6.2) / 0.05)


def test_a_walker_that_just_appeared_stands() -> None:
    assert kinematic_velocity([], (1.0, 2.0, 0.3), 0.05) == (0.0, 0.0, 0.0)
