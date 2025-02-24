import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("so_arm100_description"),
                        "launch",
                        "controllers_bringup.launch.py",
                    ),
                ),
                launch_arguments=[
                    ("hardware_type", LaunchConfiguration("hardware_type")),
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("so_arm100_moveit_config"),
                        "launch",
                        "moveit_rviz.launch.py",
                    ),
                ),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("so_arm100_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ),
                ),
            ),
        ],
    )
