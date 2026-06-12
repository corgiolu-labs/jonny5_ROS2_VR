# JONNY5 ROS2 Dry-Run Smoke Test

This smoke test validates the local ROS2 migration layer without robot hardware.

It verifies:

- the ROS2 workspace builds with `colcon`
- the dry-run bringup launch starts
- the mock SPI worker (default) feeds `/joint_states`, `/imu/data`, `/jonny5/status`, and `/jonny5/spi/telemetry`
- simulated VR intent publishes `/jonny5/teleop/intent`
- a JSON message sent to `ws://127.0.0.1:8567` is bridged into `/jonny5/teleop/intent`

## Prerequisites

Run this from WSL Ubuntu with ROS2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
```

The WebSocket bridge requires:

```bash
sudo apt update
sudo apt install -y python3-websockets
```

## Run

From PowerShell:

```powershell
wsl -d Ubuntu -- bash -lc "cd <repo-root>/ros2_ws && bash tools/smoke_ros2_dryrun.sh"
```

Or directly from WSL:

```bash
cd <repo-root>/ros2_ws
bash tools/smoke_ros2_dryrun.sh
```

Expected final line:

```text
[PASS] JONNY5 ROS2 dry-run smoke test completed
```

## Manual Launch

For interactive inspection:

```bash
cd <repo-root>/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jonny5_bringup bringup.launch.py use_mock_spi:=true sim_intent:=true
```

Then, in another WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source <repo-root>/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /jonny5/status
ros2 topic echo /jonny5/teleop/intent
```

## Notes

The test intentionally uses `use_mock_spi:=true` (the default); it never writes to real robot hardware.