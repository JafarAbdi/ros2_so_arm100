from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from so_arm100_description.launch_utils import launch_configurations, load_xacro

startup_controllers = [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "gripper_controller",
]


@launch_configurations
def make_robot_state_publisher_node(args):
    robot_description = load_xacro(
        get_package_share_path("so_arm100_description")
        / "urdf"
        / "so_arm100.urdf.xacro",
        mappings={"ros2_control_hardware_type": args.hardware_type},
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description},
            ],
        ),
    ]


def generate_launch_description():
    ros2_controllers_file = (
        get_package_share_path("so_arm100_description")
        / "control"
        / "ros2_controllers.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="mock_components"),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[ros2_controllers_file],
                remappings=[("~/robot_description", "/robot_description")],
                # To get logs from spdlog
                output="screen",
                # Colorful output
                emulate_tty=True,
            ),
            *make_robot_state_publisher_node(),
        ]
        + [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
            for controller in startup_controllers
        ],
    )
