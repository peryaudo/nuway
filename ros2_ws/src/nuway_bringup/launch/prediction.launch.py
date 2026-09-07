"""Prediction: nothing in M0 (const_vel_node arrives with M1)."""

from typing import Any

from launch import LaunchContext, LaunchDescription

from nuway_bringup.launch_util import profile_launch


def setup(_context: LaunchContext, _profile: dict[str, Any]) -> list[Any]:
    return []


def generate_launch_description() -> LaunchDescription:
    return profile_launch(setup)
