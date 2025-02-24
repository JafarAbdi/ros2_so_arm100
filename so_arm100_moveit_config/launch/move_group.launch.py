from launch import LaunchDescription
from launch_ros.actions import Node
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
    ).to_moveit_configs()

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
