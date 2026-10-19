"""JONNY5 ros2_control bringup (see ros2_ws/docs/ROS2_CONTROL.md).

Owns the SPI link through jonny5_control/Jonny5System (protocol v2 joint
streaming). Do NOT run it together with bringup.launch.py: only one process
may drive /dev/spidev0.0.

Mock (no hardware):
  ros2 launch jonny5_bringup control.launch.py
Real robot (v2 firmware flashed, arm clear, E-STOP in hand):
  ros2 launch jonny5_bringup control.launch.py mock_hardware:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock_hardware = LaunchConfiguration("mock_hardware")
    spi_device = LaunchConfiguration("spi_device")
    transfer_len = LaunchConfiguration("transfer_len")
    controller = LaunchConfiguration("controller")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([FindPackageShare("jonny5_description"), "urdf", "jonny5.urdf.xacro"]),
            " ros2_control:=true",
            " mock_hardware:=", mock_hardware,
            " spi_device:=", spi_device,
            " transfer_len:=", transfer_len,
        ]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("jonny5_control"), "config", "jonny5_controllers.yaml"]
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", *extra],
            output="screen",
        )

    return LaunchDescription([
        DeclareLaunchArgument("mock_hardware", default_value="true",
                              description="Simulated v2 firmware instead of /dev/spidev."),
        DeclareLaunchArgument("spi_device", default_value="/dev/spidev0.0"),
        DeclareLaunchArgument("transfer_len", default_value="128",
                              description="SPI transfer length: 128 (padded firmware build) or 64."),
        DeclareLaunchArgument("controller", default_value="joint_trajectory_controller",
                              description="joint_trajectory_controller or forward_position_controller."),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_yaml],
            output="screen",
        ),
        spawner("joint_state_broadcaster"),
        spawner("imu_sensor_broadcaster"),
        spawner(controller),
    ])
