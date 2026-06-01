"""JONNY5 native ROS2 bringup (see ADR-001).

Launches the native control plane:
  - robot_state_publisher  (URDF/Xacro)
  - jonny5_spi_driver      (native SPI data plane; mock by default, real on the Pi)
  - jonny5_vr_bridge       (WebXR/WebSocket -> TeleopIntent)
  - jonny5_teleop_intent_sim  (optional, dry-run intent without a headset)

Dry-run (no hardware):
  ros2 launch jonny5_bringup bringup.launch.py
Real Pi:
  ros2 launch jonny5_bringup bringup.launch.py use_mock_spi:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_spi = LaunchConfiguration("use_mock_spi")
    spi_device = LaunchConfiguration("spi_device")
    legacy_root = LaunchConfiguration("legacy_root")
    sim_intent = LaunchConfiguration("sim_intent")

    params_file = PathJoinSubstitution([
        FindPackageShare("jonny5_bringup"), "config", "jonny5.params.yaml"
    ])
    robot_description_file = PathJoinSubstitution([
        FindPackageShare("jonny5_description"), "urdf", "jonny5.urdf.xacro"
    ])
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", robot_description_file]), value_type=str
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_spi",
            default_value="true",
            description="Use the synthetic SPI worker (no hardware). Set false on the real Pi.",
        ),
        DeclareLaunchArgument(
            "spi_device",
            default_value="/dev/spidev0.0",
            description="SPI device for the real hardware path.",
        ),
        DeclareLaunchArgument(
            "legacy_root",
            default_value="",
            description="Dir containing the legacy 'controller' package. Empty = auto-resolve.",
        ),
        DeclareLaunchArgument(
            "sim_intent",
            default_value="false",
            description="Publish simulated VR intents for dry-run without a headset.",
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
            executable="spi_driver_node",
            name="jonny5_spi_driver",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_mock_spi": ParameterValue(use_mock_spi, value_type=bool),
                    "spi_device": ParameterValue(spi_device, value_type=str),
                    "legacy_root": ParameterValue(legacy_root, value_type=str),
                },
            ],
        ),
        Node(
            package="jonny5_teleop_vr",
            executable="ws_teleop_bridge_node",
            name="jonny5_vr_bridge",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="jonny5_sim",
            executable="teleop_intent_sim_node",
            name="jonny5_teleop_intent_sim",
            output="screen",
            condition=IfCondition(sim_intent),
        ),
    ])
