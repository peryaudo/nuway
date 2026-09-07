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
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / f"{name}.launch.py")),
            launch_arguments={"profile": profile}.items(),
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
            *includes,
        ]
    )
