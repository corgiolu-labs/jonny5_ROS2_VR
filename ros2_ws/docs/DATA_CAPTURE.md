# ROS 2 data capture

The recording helper captures the ROS-facing state of a JONNY5 session into a timestamped rosbag2 directory.

## Record

Start either mock or hardware bring-up, then source the workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash tools/record_dataset.sh
```

An optional first argument selects the output root:

```bash
bash tools/record_dataset.sh /data/jonny5
```

Recorded topics:

| Topic | Meaning |
|---|---|
| `/joint_states` | Named joint position observations |
| `/imu/data` | End-effector orientation and IMU metadata exposed by the driver |
| `/jonny5/status` | Robot and bridge health/state |
| `/jonny5/spi/telemetry` | JONNY5-specific STM32 telemetry |
| `/jonny5/teleop/intent` | Operator or simulated control intent |

## Inspect and replay

```bash
ros2 bag info bags/<bag-directory>
ros2 bag play bags/<bag-directory>
```

## Dataset boundaries

- Mock-mode bags validate schemas, timing flow and tooling; they are not physical-robot datasets.
- Hardware-mode recordings require an explicit experiment log containing robot configuration, firmware revision, calibration, operator, test conditions and safety observer.
- The native WebRTC video stream is intentionally outside ROS 2 and is not recorded by this helper. Any externally recorded MP4 must be timestamped and aligned separately before claiming synchronized multimodal capture.
- Recording availability does not by itself establish calibration quality, clock synchronization accuracy or dataset fitness for robot learning.

