"""Shared MoveIt configuration for the JONNY5 launch files."""

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def jonny5_moveit_config():
    # Planning only needs the kinematic model: the ros2_control block is served
    # to the controller_manager by control.launch.py (robot_state_publisher).
    urdf = os.path.join(
        get_package_share_directory("jonny5_description"), "urdf", "jonny5.urdf.xacro"
    )
    return (
        MoveItConfigsBuilder("jonny5", package_name="jonny5_moveit_config")
        .robot_description(file_path=urdf, mappings={"ros2_control": "false"})
        .robot_description_semantic(file_path="config/jonny5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
