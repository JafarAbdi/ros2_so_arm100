from launch import LaunchDescription
from launch_ros.actions import Node

from so_arm100_description.launch_utils import MoveItConfigs


def generate_launch_description():
    moveit_configs = MoveItConfigs()

    return LaunchDescription(
        [
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[
                    moveit_configs.to_dict(),
                ],
            ),
        ],
    )
