from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveitpy_simple.moveit_configs_utils.moveit_configs_builder import (
    MoveItConfigsBuilder,
)


def generate_launch_description():
    moveit_configs = (
        MoveItConfigsBuilder("so_arm100_moveit_config")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_pipelines()
        .to_moveit_configs()
    )

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
