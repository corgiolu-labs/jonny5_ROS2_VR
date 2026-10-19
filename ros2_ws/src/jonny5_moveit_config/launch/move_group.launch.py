"""JONNY5 MoveIt 2: ros2_control (control.launch.py) + move_group (+ optional RViz).

Mock:        ros2 launch jonny5_moveit_config move_group.launch.py
Real robot:  ros2 launch jonny5_moveit_config move_group.launch.py mock_hardware:=false
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import jonny5_moveit_config  # noqa: E402


def generate_launch_description():
    moveit_config = jonny5_moveit_config()
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("jonny5_bringup"), "launch", "control.launch.py")
        ),
        launch_arguments={
            "mock_hardware": LaunchConfiguration("mock_hardware"),
            "controller": "joint_trajectory_controller",
        }.items(),
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(
            get_package_share_directory("jonny5_description"), "rviz", "jonny5.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("mock_hardware", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        control,
        move_group,
        rviz,
    ])
