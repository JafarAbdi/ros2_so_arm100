from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from so_arm100_description.launch_utils import MoveItConfigs


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Whether to use simulation time to launch the MoveIt node.",
    )

    moveit_configs = MoveItConfigs()
    moveit_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_configs.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            moveit_node,
        ],
    )
