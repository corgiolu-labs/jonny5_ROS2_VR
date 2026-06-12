# JONNY5 ROS2 VR

ROS2 migration workspace for JONNY5, a VR-teleoperated 6-DoF robot arm.

This repository keeps the working legacy subsystems in place while adding a ROS2 layer around them:

- `firmware/`: STM32/Zephyr real-time firmware, kept unchanged during the first migration phase.
- `raspberry/`: existing Raspberry Pi controller, SPI protocol, WebSocket services, kinematics and tools.
- `web/`: existing dashboard and WebXR/WebRTC VR frontend.
- `ros2_ws/`: ROS2 workspace with packages for messages, robot description, bringup, hardware bridge and VR teleop bridge.

## First Migration Target

The first ROS2 layer does not replace the firmware or the low-latency WebRTC video path. It standardizes the control and observation surfaces:

- ROS2 messages for VR intent, STM32 status and SPI telemetry.
- URDF/Xacro robot model with joint limits and camera/IMU frames.
- Bridge nodes that adapt the existing JSON/WebSocket/SPI-oriented code into ROS2 topics.
- Launch files and parameters for repeatable bringup.

## Packages

```text
ros2_ws/src/
  jonny5_msgs/          Custom interfaces for non-standard JONNY5 data.
  jonny5_description/   URDF/Xacro, joint limits and RViz config.
  jonny5_hardware/      Raspberry/STM32 bridge nodes.
  jonny5_teleop_vr/     WebXR/WebSocket to ROS2 teleoperation bridge.
  jonny5_bringup/       Launch and runtime parameter composition.
```

## Build Sketch

```bash
cd ~/jonny5_ROS2_VR/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch jonny5_bringup bringup.launch.py use_mock_spi:=true
```

The initial bridge defaults to dry-run mode so the ROS2 graph can be inspected before hardware actuation is enabled.
