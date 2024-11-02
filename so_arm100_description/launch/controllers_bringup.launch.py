from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from moveitpy_simple.moveit_configs_utils.file_loaders import load_xacro

startup_controllers = [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "gripper_controller",
]


def generate_launch_description():
    robot_description = load_xacro(
        get_package_share_path("so_arm100_description")
        / "urdf"
        / "so_arm100.urdf.xacro",
        mappings={"ros2_control_hardware_type": "mock_components"},
    )
    ros2_controllers_file = (
        get_package_share_path("so_arm100_description")
        / "control"
        / "ros2_controllers.yaml"
    )

    return LaunchDescription(
        [
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
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                ],
            ),
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
