import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

from so_arm100_description.launch_utils import launch_configurations, load_xacro

startup_controllers = [
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "gripper_controller",
]


@launch_configurations
def make_robot_state_publisher_node(args):
    namespace: str = args.namespace
    if len(namespace) > 0:
        namespace = namespace + "/"
    robot_description = load_xacro(
        get_package_share_path("so_arm100_description")
        / "urdf"
        / "so_arm100.urdf.xacro",
        mappings={
            "prefix": namespace,
            "ros2_control_file": args.ros2_control_xacro_file,
            "ros2_control_hardware_type": args.hardware_type,
            "usb_port": args.usb_port,
        },
    )
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[
                {"robot_description": robot_description},
            ],
        ),
    ]


def generate_launch_description():
    bringup_dir = get_package_share_path("so_arm100_description")

    ros2_control_xacro_file_arg = DeclareLaunchArgument(
        "ros2_control_xacro_file",
        default_value=os.path.join(
            bringup_dir, "control", "so_arm100.ros2_control.xacro"
        ),
        description="Full path to the ros2_control xacro file",
    )

    # Get controller configuration from launch arg
    controller_config_file_arg = DeclareLaunchArgument(
        "controller_config_file",
        default_value=os.path.join(bringup_dir, "control", "ros2_controllers.yaml"),
        description="Full path to the controller configuration file to use",
    )

    ros2_controllers_file = LaunchConfiguration("controller_config_file")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'ros2_controllers.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    ros2_controllers_file = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={"<robot_namespace>": (namespace, "/")},
        condition=IfCondition(use_namespace),
    )
    ros2_controllers_file = ReplaceString(
        source_file=ros2_controllers_file,
        replacements={"<robot_namespace>": ("")},
        condition=UnlessCondition(use_namespace),
    )

    namespaced_ros2_controllers_file = ParameterFile(
        RewrittenYaml(
            source_file=ros2_controllers_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="mock_components",
        description="Hardware type for the robot. Supported types [mock_components, real]",
    )

    usb_port_arg = DeclareLaunchArgument(
        "usb_port",
        default_value="/dev/LeRobotFollower",
        description="USB port for the robot. Only used when hardware_type is real",
    )

    use_namespace_arg = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the controller stack",
    )

    # Define namespace as a launch argument
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the controller_manager and related nodes",
    )

    return LaunchDescription(
        [
            hardware_type_arg,
            usb_port_arg,
            use_namespace_arg,
            namespace_arg,
            ros2_control_xacro_file_arg,
            controller_config_file_arg,
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[namespaced_ros2_controllers_file],
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
                namespace=LaunchConfiguration("namespace"),
                arguments=[controller],
            )
            for controller in startup_controllers
        ],
    )
