# JONNY5 with MoveIt 2

`jonny5_moveit_config` runs MoveIt 2 on top of the ros2_control stack described in [ROS2_CONTROL.md](ROS2_CONTROL.md):
- `Jonny5System` talks to the STM32 over SPI protocol v2;
- `joint_trajectory_controller` executes motions from both `move_group` and MoveIt Servo, so no controller switch is needed.

```text
move_group (OMPL / Pilz) ──FollowJointTrajectory──┐
                                                   ├─> joint_trajectory_controller -> Jonny5System -> SPI v2 -> STM32
MoveIt Servo ──────JointTrajectory stream──────────┘
      ^
TwistStamped  <─ jonny5_intent_to_servo <─ TeleopIntent <─ jonny5_vr_bridge <─ WebXR (headset)
```

## Contents

| File | Purpose |
|---|---|
| `config/jonny5.srdf` | Group `arm` (chain `base_link → ee_link`); poses `home` and `ready`; collision matrix |
| `config/kinematics.yaml` | KDL IK solver |
| `config/joint_limits.yaml` | Velocity limits (as in the URDF) and conservative accelerations; default scaling 0.3 |
| `config/moveit_controllers.yaml` | `joint_trajectory_controller` (FollowJointTrajectory) |
| `config/pilz_cartesian_limits.yaml` | Pilz LIN/CIRC limits (0.2 m/s) |
| `config/servo.yaml` | MoveIt Servo: output to `/joint_trajectory_controller/joint_trajectory`, 100 Hz |
| `launch/move_group.launch.py` | `control.launch.py` + `move_group` (+ RViz with `rviz:=true`) |
| `launch/servo.launch.py` | `control.launch.py` + `servo_node` + `jonny5_intent_to_servo` + VR bridge |

Planning pipelines: OMPL (default) and `pilz_industrial_motion_planner` (PTP / LIN / CIRC).

## Planning

```bash
ros2 launch jonny5_moveit_config move_group.launch.py            # mock
ros2 launch jonny5_moveit_config move_group.launch.py mock_hardware:=false rviz:=true
```

Any MoveIt client works: the `/move_action` action, the RViz MotionPlanning panel, or `moveit_py`. The group is `arm`, and `ready` is a good starting pose.

## VR teleoperation through MoveIt Servo

```bash
ros2 launch jonny5_moveit_config servo.launch.py                  # mock
ros2 launch jonny5_moveit_config servo.launch.py mock_hardware:=false
# behind the HTTPS /ws proxy of the WebXR page (stop jonny5-ws-teleop first):
ros2 launch jonny5_moveit_config servo.launch.py mock_hardware:=false vr_port:=8557
```

`jonny5_intent_to_servo` has two command modes (`teleop_mode` launch argument):

**`joint` (default)**: each stick axis drives one joint through Servo JOINT_JOG
(`control_msgs/JointJog`), 0.35 rad/s at full stick:

| VR input | Joint | Motion |
|---|---|---|
| left stick right/left (`joy_x`) | `base_joint` | base turns right / left |
| left stick forward/back (`joy_y`) | `shoulder_joint` | shoulder forward / back |
| right stick up/down (`pitch`) | `elbow_joint` | forearm up (tool rises) / down |
| right stick right/left (`yaw`) | `wrist_yaw_joint` | wrist turns right / left |

Mapping and signs are the `joint_axes` / `joint_signs` parameters. Verified on the
robot (hardware test, phase 8, with a scripted headset): all four directions, deadman
release and stream loss.

**`twist` (experimental)**: `TwistStamped` in `base_link`, sticks drive the tool in
Cartesian space. On JONNY5 this is ill-conditioned almost everywhere: the shoulder
and elbow pitch axes are only 60 mm apart, so small tool motions need large, opposite
shoulder/elbow motions, and near the singularity thresholds Servo scales and bends
the commanded direction ("forward" came out as "up" on the robot). A Cartesian mode
for this arm needs its own IK (e.g. the legacy damped least squares of mode 5).

| VR input (`twist`) | Motion |
|---|---|
| left stick forward/back (`joy_y`) | tool +x / −x |
| left stick left/right (`joy_x`) | tool +y / −y |
| right stick up/down (`pitch`) | tool +z / −z |
| right stick left/right (`yaw`) | rotation about z |

The `joint_trajectory_controller` must accept trajectories that end with a non-zero
velocity (`allow_nonzero_velocity_at_trajectory_end: true`). With the Jazzy default
(false) it rejected every Servo command while a stick was held; the arm then moved
only after the deadman was released, catching up with Servo's integrated pose.

The arm moves only when all of these hold; otherwise the node publishes nothing and Servo stops after 0.15 s:
- the deadman is held (both grips);
- the mode is not IDLE;
- the last intent is less than 0.2 s old.

Other safety layers still apply:
- Servo's collision and singularity checks;
- the URDF limits in `Jonny5System`;
- all of the firmware protections (IDLE only, E-STOP, 100 ms hold, 500 ms watchdog).

Parameters of `jonny5_intent_to_servo`:

| Parameter | Default |
|---|---|
| `command_mode` | `joint` (launch `teleop_mode`) |
| `joint_axes` / `joint_signs` | base, shoulder, elbow, wrist yaw / −1, 1, −1, −1 |
| `max_joint_vel` | 0.35 rad/s |
| `max_linear` | 0.05 m/s |
| `max_angular` | 0.5 rad/s |
| `deadzone` | 0.08 |
| `intent_timeout_s` | 0.2 |
| `frame_id` | `base_link` |

### Start from `ready`, not `home`

In `home` the arm points straight up. That pose is a kinematic singularity: the base and wrist-yaw axes are collinear, and Servo refuses to move. Move to `ready` first, with move_group or a single `FollowJointTrajectory` goal.

### Singularity thresholds

Servo judges singularities from the Jacobian condition number, which mixes metres and radians. On a ~0.35 m arm that number is large everywhere. Over 20k random poses within the joint limits:

| Minimum | Median | 95th percentile |
|---|---|---|
| ≈ 56 | ≈ 107 | ≈ 1170 |

The Servo defaults (10 / 30) would stop the arm everywhere. `servo.yaml` therefore uses 150 (start slowing down) and 400 (hard stop). Retune these on the real arm if Servo stops too early or too late.

## Verified in simulation (mock firmware)

- `move_group`:
  - a joint-space goal to `ready` is planned and executed;
  - the result is `SUCCESS`;
  - the final `/joint_states` is within 0.01 rad of the target.
- Servo:
  - from `ready`, with the deadman held and the right stick up, the shoulder, elbow and wrist move;
  - after the deadman is released, nothing moves.
- Unit tests for the intent-to-twist mapping (deadman, IDLE, deadzone, scaling).

## Still open

- Head tracking (the headset pose) is not mapped yet. Candidates are Servo pose tracking or wrist-only joint jog.
- `ready` and the Cartesian speeds need tuning on the real arm.
