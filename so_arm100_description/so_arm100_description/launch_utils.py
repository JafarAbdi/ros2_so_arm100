import pathlib
from copy import deepcopy
from dataclasses import InitVar, dataclass, field, make_dataclass
from functools import wraps
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext

from so_arm100_description.constants import (
    DEFAULT_PLANNING_PIPELINE,
    JOINT_LIMITS_FILENAME,
    KINEMATICS_FILENAME,
    MOVEIT_CONFIG_PACKAGE_NAME,
    MOVEIT_CPP_FILENAME,
    PLANNING_PIPELINES,
    ROBOT_DESCRIPTION_FILENAME,
    ROBOT_DESCRIPTION_PACKAGE_PATH,
    ROBOT_DESCRIPTION_SEMANTIC_FILENAME,
    TRAJECTORY_EXECUTION_FILENAME,
)


def load_yaml(file_path: Path) -> dict | None:
    """Load a yaml file and render it with the given mappings."""
    if not file_path.exists():
        msg = f"File {file_path} doesn't exist"
        raise FileNotFoundError(msg)

    try:
        with file_path.open("r") as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_xacro(file_path: Path, mappings: dict | None = None) -> str:
    """Load a xacro file and render it with the given mappings."""
    if not file_path.exists():
        msg = f"File {file_path} doesn't exist"
        raise FileNotFoundError(msg)

    # We need to deepcopy the mappings because xacro.process_file modifies them
    file = xacro.process_file(
        file_path,
        mappings=deepcopy(mappings) if mappings else {},
    )
    return file.toxml()


def launch_configurations(func):
    """Decorator to pass launch configurations to a function.

    The function must take a single argument, which is a dataclass with the launch configurations as attributes.
    """

    def args_to_dataclass(context: LaunchContext, *args, **kwargs):
        launch_configurations = make_dataclass(
            "LAUNCH_CONFIGURATIONS",
            context.launch_configurations.keys(),
            slots=True,
            frozen=True,
        )
        return func(launch_configurations(**context.launch_configurations))

    @wraps(func)
    def wrapper(*args, **kwargs):
        return [OpaqueFunction(function=args_to_dataclass)]

    return wrapper


@dataclass(slots=True)
class MoveItConfigs:
    """Class containing MoveIt related parameters."""

    # A dictionary that contains the mappings to the URDF file.
    mappings: InitVar[dict | None] = None
    # A dictionary that has the contents of the URDF file.
    robot_description: dict = field(default_factory=dict)
    # A dictionary that has the contents of the SRDF file.
    robot_description_semantic: dict = field(default_factory=dict)
    # A dictionary IK solver specific parameters.
    robot_description_kinematics: dict = field(default_factory=dict)
    # A dictionary that contains the planning pipelines parameters.
    planning_pipelines: dict = field(default_factory=dict)
    # A dictionary contains parameters for trajectory execution & moveit controller managers.
    trajectory_execution: dict = field(default_factory=dict)
    # A dictionary that has the planning scene monitor's parameters.
    planning_scene_monitor: dict = field(default_factory=dict)
    # A dictionary containing move_group's non-default capabilities.
    move_group_capabilities: dict = field(default_factory=dict)
    # A dictionary containing the overridden position/velocity/acceleration limits.
    joint_limits: dict = field(default_factory=dict)
    # A dictionary containing MoveItCpp related parameters.
    moveit_cpp: dict = field(default_factory=dict)

    def __post_init__(self, mappings: dict):
        """Load all the MoveIt related parameters.

        Args:
            mappings: A dictionary containing the mappings to the URDF file.
        """
        if mappings is None:
            mappings = {}

        self.robot_description = {
            "robot_description": load_xacro(
                ROBOT_DESCRIPTION_PACKAGE_PATH / "urdf" / ROBOT_DESCRIPTION_FILENAME,
                mappings,
            ),
        }
        moveit_config_package_path = pathlib.Path(
            get_package_share_directory(MOVEIT_CONFIG_PACKAGE_NAME),
        )
        self.robot_description_semantic = {
            "robot_description_semantic": load_xacro(
                moveit_config_package_path
                / "config"
                / ROBOT_DESCRIPTION_SEMANTIC_FILENAME,
                mappings,
            ),
        }

        self.robot_description_kinematics = {
            "robot_description_kinematics": load_yaml(
                moveit_config_package_path / "config" / KINEMATICS_FILENAME,
            ),
        }

        self.planning_pipelines = {
            "planning_pipelines.pipeline_names": PLANNING_PIPELINES,
            "default_planning_pipeline": DEFAULT_PLANNING_PIPELINE,
        }
        for planning_pipeline in PLANNING_PIPELINES:
            self.planning_pipelines[planning_pipeline] = load_yaml(
                moveit_config_package_path
                / "config"
                / f"{planning_pipeline}_planning.yaml",
            )

        self.trajectory_execution = load_yaml(
            moveit_config_package_path / "config" / TRAJECTORY_EXECUTION_FILENAME,
        )

        self.planning_scene_monitor = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }

        self.joint_limits = {
            "robot_description_planning": load_yaml(
                moveit_config_package_path / "config" / JOINT_LIMITS_FILENAME,
            ),
        }

        self.moveit_cpp = load_yaml(
            moveit_config_package_path / "config" / MOVEIT_CPP_FILENAME,
        )

        # TODO: Uncomment once MTC get released
        # > self.move_group_capabilities = {
        # >     "capabilities": "move_group/ExecuteTaskSolutionCapability",
        # > }

    def to_dict(self) -> dict:
        """Merge all the parameters in a dict."""
        return (
            self.robot_description
            | self.robot_description_semantic
            | self.robot_description_kinematics
            | self.planning_pipelines
            | self.trajectory_execution
            | self.planning_scene_monitor
            | self.joint_limits
            | self.moveit_cpp
            | self.move_group_capabilities
        )
