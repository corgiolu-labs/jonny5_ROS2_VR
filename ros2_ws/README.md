# ROS2 Workspace

This workspace is intentionally incremental. The legacy JONNY5 runtime remains available while ROS2 packages are introduced around it.

Recommended build command:

```bash
colcon build --symlink-install
```

Recommended first launch:

```bash
ros2 launch jonny5_bringup bringup.launch.py hardware_enabled:=false
```
