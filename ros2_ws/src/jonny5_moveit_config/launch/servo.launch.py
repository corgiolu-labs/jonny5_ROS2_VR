"""JONNY5 VR teleoperation through MoveIt Servo on ros2_control.

WebXR -> jonny5_vr_bridge -> TeleopIntent -> jonny5_intent_to_servo -> JointJog (or TwistStamped)
      -> servo_node -> JointTrajectory -> joint_trajectory_controller -> Jonny5System (SPI v2)

Mock:        ros2 launch jonny5_moveit_config servo.launch.py
Real robot:  ros2 launch jonny5_moveit_config servo.launch.py mock_hardware:=false

teleop_mode:=joint (default) maps each stick axis to one joint; teleop_mode:=twist
drives the tool in Cartesian space (experimental: ill-conditioned on JONNY5).
vr_port:=8557 puts the bridge behind the HTTPS /ws proxy used by the WebXR page
(the legacy ws-teleop service must be stopped).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import jonny5_moveit_config  # noqa: E402


def generate_launch_description():
    moveit_config = jonny5_moveit_config()
    servo_params = {
        "moveit_servo": ParameterBuilder("jonny5_moveit_config").yaml("config/servo.yaml").to_dict()
    }
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("jonny5_bringup"), "launch", "control.launch.py")
        ),
        launch_arguments={
            "mock_hardware": LaunchConfiguration("mock_hardware"),
            "controller": "joint_trajectory_controller",
        }.items(),
    )
    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            {"update_period": 0.01, "planning_group_name": "arm"},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    intent_to_servo = Node(
        package="jonny5_teleop_vr",
        executable="intent_to_servo_node",
        name="jonny5_intent_to_servo",
        output="screen",
        parameters=[{"command_mode": LaunchConfiguration("teleop_mode")}],
    )
    vr_bridge = Node(
        package="jonny5_teleop_vr",
        executable="ws_teleop_bridge_node",
        name="jonny5_vr_bridge",
        output="screen",
        parameters=[{"bind_port": ParameterValue(LaunchConfiguration("vr_port"), value_type=int)}],
        condition=IfCondition(LaunchConfiguration("vr_bridge")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("mock_hardware", default_value="true"),
        DeclareLaunchArgument("vr_bridge", default_value="true",
                              description="Start the WebXR/WebSocket -> TeleopIntent bridge."),
        DeclareLaunchArgument("vr_port", default_value="8567",
                              description="Bridge port (8557 = behind the HTTPS /ws proxy)."),
        DeclareLaunchArgument("teleop_mode", default_value="joint",
                              description="joint (stick -> joint velocity) or twist (experimental)."),
        control,
        servo,
        intent_to_servo,
        vr_bridge,
    ])
