from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    params_file = PathJoinSubstitution([
        FindPackageShare("jonny5_bringup"), "config", "jonny5.params.yaml"
    ])
    robot_description_file = PathJoinSubstitution([
        FindPackageShare("jonny5_description"), "urdf", "jonny5.urdf.xacro"
    ])

    robot_description = {
        "robot_description": Command(["xacro ", robot_description_file])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_enabled",
            default_value="false",
            description="Enable real hardware intent writes. Keep false for graph validation.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="jonny5_hardware",
            executable="spi_bridge_node",
            name="jonny5_spi_bridge",
            output="screen",
            parameters=[params_file, {"hardware_enabled": hardware_enabled}],
        ),
        Node(
            package="jonny5_teleop_vr",
            executable="ws_teleop_bridge_node",
            name="jonny5_vr_bridge",
            output="screen",
            parameters=[params_file],
        ),
    ])
