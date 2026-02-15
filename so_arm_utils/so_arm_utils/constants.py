"""Constants for the robot configuration package."""
import pathlib
from dataclasses import dataclass
from typing import Final, Literal
from ament_index_python.packages import get_package_share_directory

# Shared constants (same for all robots)
KINEMATICS_FILENAME: Final = "kinematics.yaml"
JOINT_LIMITS_FILENAME: Final = "joint_limits.yaml"
TRAJECTORY_EXECUTION_FILENAME: Final = "trajectory_execution.yaml"
PLANNING_PIPELINES: Final = ["ompl"]
DEFAULT_PLANNING_PIPELINE: Final = PLANNING_PIPELINES[0]
MOVEIT_CPP_FILENAME: Final = "moveit_cpp.yaml"

RobotName = Literal["so_arm100", "so_arm101"]


@dataclass(frozen=True, slots=True)
class RobotConstants:
    """Robot-specific constants."""
    robot_name: str
    robot_description_package_name: str
    moveit_config_package_name: str
    robot_description_filename: str
    robot_description_semantic_filename: str
    robot_description_package_path: pathlib.Path


def get_robot_constants(robot_name: RobotName) -> RobotConstants:
    """Factory function to get robot-specific constants.
    
    Args:
        robot_name: Name of the robot ("so_arm100" or "so_arm101")
    
    Returns:
        RobotConstants dataclass with robot-specific paths and filenames
    """
    robot_description_package_name = f"{robot_name}_description"
    
    return RobotConstants(
        robot_name=robot_name,
        robot_description_package_name=robot_description_package_name,
        moveit_config_package_name=f"{robot_name}_moveit_config",
        robot_description_filename=f"{robot_name}.urdf.xacro",
        robot_description_semantic_filename=f"{robot_name}.srdf",
        robot_description_package_path=pathlib.Path(
            get_package_share_directory(robot_description_package_name)
        ),
    )