"""THE launch file (M0 §2.10): `ros2 launch nuway_bringup stack.launch.py profile:=m0_gt_all`.

Includes the per-subsystem launches in stack order; each of them reads the
same profile through nuway_bringup.profile.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SUBSYSTEMS = (
    "sim",
    "map",
    "localization",
    "perception",
    "prediction",
    "planning",
    "control",
    "viz",
)


def generate_launch_description() -> LaunchDescription:
    launch_dir = Path(get_package_share_directory("nuway_bringup")) / "launch"
    profile = LaunchConfiguration("profile")
    delay = LaunchConfiguration("callback_delay_ms")
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / f"{name}.launch.py")),
            launch_arguments=(
                {"profile": profile, "callback_delay_ms": delay}
                if name == "prediction"
                else {"profile": profile}
            ).items(),
        )
        for name in SUBSYSTEMS
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="m0_gt_all",
                description="profile name under configs/profiles/ or a YAML path",
            ),
            DeclareLaunchArgument(
                "callback_delay_ms",
                default_value="0",
                description="wall-clock sleep injected into const_vel_node's tick "
                "(M1 §5 determinism check; 0 = off)",
            ),
            *includes,
        ]
    )
