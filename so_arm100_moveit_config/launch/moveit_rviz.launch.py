from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from so_arm100_description.launch_utils import MoveItConfigs


def generate_launch_description():
    moveit_configs = MoveItConfigs()

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value="move_group.rviz"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("so_arm100_moveit_config"),
                            "rviz",
                            LaunchConfiguration("rviz_config"),
                        ],
                    ),
                ],
                parameters=[
                    moveit_configs.joint_limits,
                    moveit_configs.robot_description,
                    moveit_configs.robot_description_semantic,
                    moveit_configs.robot_description_kinematics,
                    moveit_configs.planning_pipelines,
                ],
            ),
        ],
    )
