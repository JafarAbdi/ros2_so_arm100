"""Launch file to visualize the SO-ARM101 robot model.

Launches the robot_state_publisher with the SO-ARM101 URDF (processed via xacro),
a joint_state_publisher_gui for interactive joint control, and optionally RViz
for 3D visualization.

Usage:
    ros2 launch so_arm101_description view_description.launch.py
    ros2 launch so_arm101_description view_description.launch.py rviz:=true
    ros2 launch so_arm101_description view_description.launch.py joint_state_publisher:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Open RViz."
    )
    jsp_arg = DeclareLaunchArgument(
        "joint_state_publisher",
        default_value="true",
        description="Run joint state publisher gui node.",
    )

    pkg_so_arm101_description = get_package_share_directory("so_arm101_description")

    # Robot description via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("so_arm101_description"),
                    "urdf",
                    "so_arm101.urdf.xacro",
                ]
            ),
        ]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description_content, value_type=str
                )
            }
        ],
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("joint_state_publisher")),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_so_arm101_description, "rviz", "config.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            rviz_arg,
            jsp_arg,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )
