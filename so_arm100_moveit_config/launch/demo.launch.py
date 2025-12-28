import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    hardware_type_arg = DeclareLaunchArgument("hardware_type")
    hardware_type = LaunchConfiguration("hardware_type")

    controllers_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so_arm100_description"),
                "launch",
                "controllers_bringup.launch.py",
            ),
        ),
        launch_arguments=[
            ("hardware_type", hardware_type),
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so_arm100_moveit_config"),
                "launch",
                "moveit_rviz.launch.py",
            ),
        ),
    )

    use_sim_time = PythonExpression(["'", hardware_type, "' == 'mujoco'"])
    moveit_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("so_arm100_moveit_config"),
                "launch",
                "move_group.launch.py",
            ),
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            hardware_type_arg,
            controllers_launch,
            rviz_launch,
            moveit_launch,
        ],
    )
