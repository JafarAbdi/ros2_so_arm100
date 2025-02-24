"""Constants for the robot configuration package."""

import pathlib
from typing import Final

from ament_index_python.packages import get_package_share_directory

ROBOT_NAME: Final = "so_arm100"
ROBOT_DESCRIPTION_PACKAGE_NAME: Final = f"{ROBOT_NAME}_description"
MOVEIT_CONFIG_PACKAGE_NAME: Final = f"{ROBOT_NAME}_moveit_config"
ROBOT_DESCRIPTION_FILENAME: Final = f"{ROBOT_NAME}.urdf.xacro"
ROBOT_DESCRIPTION_SEMANTIC_FILENAME: Final = f"{ROBOT_NAME}.srdf"
KINEMATICS_FILENAME: Final = "kinematics.yaml"
JOINT_LIMITS_FILENAME: Final = "joint_limits.yaml"
TRAJECTORY_EXECUTION_FILENAME: Final = "trajectory_execution.yaml"
PLANNING_PIPELINES: Final = ["ompl"]
DEFAULT_PLANNING_PIPELINE: Final = PLANNING_PIPELINES[0]  # Default to OMPL
MOVEIT_CPP_FILENAME: Final = "moveit_cpp.yaml"
ROBOT_DESCRIPTION_PACKAGE_PATH: Final = pathlib.Path(
    get_package_share_directory(ROBOT_DESCRIPTION_PACKAGE_NAME),
)
