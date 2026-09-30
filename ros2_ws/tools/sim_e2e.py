#!/usr/bin/env python3
"""
sim_e2e.py -- end-to-end checks of the JONNY5 ROS 2 stack on mock hardware.

Run after `colcon build` and `source install/setup.bash`:

    python3 tools/sim_e2e.py                    # all scenarios
    python3 tools/sim_e2e.py control moveit     # a subset

Scenarios (each starts its own launch file, then stops it):
  spi_v2   bringup.launch.py protocol_version:=2: status/telemetry v2, joint streaming
  control  control.launch.py: controllers active, FollowJointTrajectory goal reached
  moveit   move_group.launch.py: plan + execute to the SRDF "ready" pose
  servo    servo.launch.py: VR intent with deadman moves the arm, without deadman it stops

Exit code 0 only if every selected scenario passes.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time
from typing import Callable, Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["base_joint", "shoulder_joint", "elbow_joint",
          "wrist_yaw_joint", "wrist_pitch_joint", "wrist_roll_joint"]
READY = [0.0, 0.35, 0.7, 0.0, -0.5, 0.0]


class Launch:
    def __init__(self, *args: str) -> None:
        self.log = tempfile.NamedTemporaryFile(prefix="j5_e2e_", suffix=".log", delete=False)
        self.proc = subprocess.Popen(["ros2", "launch", *args], stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self) -> None:
        for sig, wait in ((signal.SIGINT, 8), (signal.SIGKILL, 3)):
            if self.proc.poll() is not None:
                break
            os.killpg(self.proc.pid, sig)
            try:
                self.proc.wait(wait)
            except subprocess.TimeoutExpired:
                pass


class Probe(Node):
    def __init__(self, name: str = "j5_sim_e2e") -> None:
        super().__init__(name)
        self.js: Dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.js.update(zip(m.name, m.position)), 10)

    def spin_for(self, seconds: float, every: Callable[[], None] | None = None) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if every:
                every()
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, cond: Callable[[], bool], timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def wait_controllers(self, timeout: float = 60.0) -> bool:
        """joint_states flowing and the JTC action server up (controllers active)."""
        from control_msgs.action import FollowJointTrajectory

        if not self.wait_for(lambda: len(self.js) == 6, timeout):
            return False
        ac = ActionClient(self, FollowJointTrajectory,
                          "/joint_trajectory_controller/follow_joint_trajectory")
        return ac.wait_for_server(timeout_sec=timeout)

    def joints(self) -> List[float]:
        return [self.js.get(j, float("nan")) for j in JOINTS]

    def send_trajectory(self, positions: List[float], sec: int = 2) -> int:
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint

        ac = ActionClient(self, FollowJointTrajectory,
                          "/joint_trajectory_controller/follow_joint_trajectory")
        if not ac.wait_for_server(timeout_sec=30):
            return -999
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.sec = sec
        goal.trajectory.points = [pt]
        f = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30)
        handle = f.result()
        if handle is None or not handle.accepted:
            return -998
        r = handle.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=sec + 30)
        return r.result().result.error_code if r.result() else -997


# Nodes of the stacks under test. A fixed pause was not enough on the Pi: the
# previous scenario's controller_manager / move_group were still in the graph
# when the next one started, and its JTC goal failed.
STACK_NODES = {"controller_manager", "move_group", "servo_node", "jonny5_spi_driver",
               "robot_state_publisher", "joint_trajectory_controller"}


def wait_graph_clear(timeout: float = 30.0) -> bool:
    """Wait until the previous scenario's nodes have left the ROS graph."""
    probe = rclpy.create_node("j5_sim_e2e_gap")
    try:
        time.sleep(2.0)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if not STACK_NODES & set(probe.get_node_names()):
                return True
        return False
    finally:
        probe.destroy_node()


def check(ok: bool, what: str, results: List[str]) -> bool:
    results.append(("PASS " if ok else "FAIL ") + what)
    return ok


def scenario_spi_v2(node: Probe, res: List[str]) -> bool:
    from jonny5_msgs.msg import RobotStatus, SpiTelemetry
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import SetBool

    status: Dict[str, object] = {}
    tel: Dict[str, object] = {}
    node.create_subscription(RobotStatus, "/jonny5/status", lambda m: status.update(msg=m), 10)
    node.create_subscription(SpiTelemetry, "/jonny5/spi/telemetry", lambda m: tel.update(msg=m), 10)
    ok = check(node.wait_for(lambda: "msg" in status and "msg" in tel, 30), "status + telemetry published", res)
    if not ok:
        return False
    ok &= check(tel["msg"].protocol_version == 2 and status["msg"].state == "IDLE"
                and status["msg"].stm32_online, "telemetry v2, FSM IDLE, stm32 online", res)
    pub = node.create_publisher(Float64MultiArray, "/jonny5/joint_commands", 10)
    cli = node.create_client(SetBool, "/jonny5/joint_stream/enable")
    cli.wait_for_service(timeout_sec=10)

    def enable() -> bool:
        f = cli.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, f, timeout_sec=10)
        return bool(f.result() and f.result().success)

    far = Float64MultiArray(data=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    node.spin_for(0.5, lambda: pub.publish(far))
    ok &= check(not enable(), "enable refused while the command is far from the current pose", res)
    here = Float64MultiArray(data=node.joints())
    node.spin_for(0.5, lambda: pub.publish(here))
    ok &= check(enable(), "enable accepted with the command at the current pose", res)
    cmd = Float64MultiArray(data=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    node.spin_for(4.0, lambda: pub.publish(cmd))
    ok &= check(abs(node.joints()[0] - 0.2) < 0.02, f"streaming reaches base 0.2 rad (got {node.joints()[0]:.3f})", res)
    t = tel["msg"]
    ok &= check(t.joint_stream_active and t.crc_errors_pi == 0 and t.rx_seq_gaps == 0,
                "stream live, 0 CRC errors, 0 sequence gaps", res)
    return ok


def scenario_control(node: Probe, res: List[str]) -> bool:
    ok = check(node.wait_controllers(), "controllers active (joint_states + JTC)", res)
    target = [0.3, -0.2, 0.2, 0.1, -0.1, 0.0]
    code = node.send_trajectory(target)
    ok &= check(code == 0, f"FollowJointTrajectory SUCCESSFUL (error_code {code})", res)
    node.spin_for(0.5)
    err = max(abs(a - b) for a, b in zip(node.joints(), target))
    ok &= check(err < 0.02, f"joints at target (max error {err:.4f} rad)", res)
    return ok


def scenario_moveit(node: Probe, res: List[str]) -> bool:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint

    if not check(node.wait_controllers(), "controllers active (joint_states + JTC)", res):
        return False
    ac = ActionClient(node, MoveGroup, "/move_action")
    if not check(ac.wait_for_server(timeout_sec=60), "move_group /move_action available", res):
        return False
    node.spin_for(2.0)  # let move_group's current state monitor see the joint states
    goal = MoveGroup.Goal()
    goal.request.group_name = "arm"
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 5.0
    goal.request.max_velocity_scaling_factor = 0.5
    goal.request.max_acceleration_scaling_factor = 0.5
    c = Constraints()
    for j, v in zip(JOINTS, READY):
        c.joint_constraints.append(JointConstraint(joint_name=j, position=v, tolerance_above=0.01,
                                                   tolerance_below=0.01, weight=1.0))
    goal.request.goal_constraints.append(c)
    f = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f, timeout_sec=30)
    r = f.result().get_result_async()
    rclpy.spin_until_future_complete(node, r, timeout_sec=60)
    code = r.result().result.error_code.val
    ok = check(code == 1, f"plan + execute to 'ready' (MoveItErrorCodes {code})", res)
    node.spin_for(0.5)
    err = max(abs(a - b) for a, b in zip(node.joints(), READY))
    ok &= check(err < 0.02, f"joints at 'ready' (max error {err:.4f} rad)", res)
    return ok


def scenario_servo(node: Probe, res: List[str]) -> bool:
    from jonny5_msgs.msg import TeleopIntent

    ok = check(node.wait_controllers(), "controllers active (joint_states + JTC)", res)
    ok &= check(node.send_trajectory(READY) == 0, "moved to 'ready' (non-singular start)", res)
    node.spin_for(3.0)  # let Servo switch to TWIST
    pub = node.create_publisher(TeleopIntent, "/jonny5/teleop/intent", 10)
    msg = TeleopIntent(mode=TeleopIntent.MODE_MANUAL, buttons_left=2, buttons_right=2, pitch=32767)
    start = node.joints()
    node.spin_for(3.0, lambda: pub.publish(msg))
    moved = max(abs(a - b) for a, b in zip(node.joints(), start))
    ok &= check(moved > 0.05, f"deadman held + stick up moves the arm ({moved:.3f} rad)", res)
    msg.buttons_right = 0
    node.spin_for(0.5, lambda: pub.publish(msg))
    mid = node.joints()
    node.spin_for(2.0, lambda: pub.publish(msg))
    still = max(abs(a - b) for a, b in zip(node.joints(), mid))
    ok &= check(still < 0.005, f"deadman released: no motion ({still:.4f} rad)", res)
    return ok


SCENARIOS = {
    "spi_v2": (["jonny5_bringup", "bringup.launch.py", "protocol_version:=2"], scenario_spi_v2),
    "control": (["jonny5_bringup", "control.launch.py"], scenario_control),
    "moveit": (["jonny5_moveit_config", "move_group.launch.py"], scenario_moveit),
    "servo": (["jonny5_moveit_config", "servo.launch.py", "vr_bridge:=false"], scenario_servo),
}


def main() -> int:
    names = sys.argv[1:] or list(SCENARIOS)
    unknown = [n for n in names if n not in SCENARIOS]
    if unknown:
        print("unknown scenario(s):", unknown, "choose from", list(SCENARIOS))
        return 2
    rclpy.init()
    summary = []
    for name in names:
        args, fn = SCENARIOS[name]
        launch = Launch(*args)
        node = Probe(f"j5_sim_e2e_{name}")
        results: List[str] = []
        try:
            time.sleep(3.0)
            ok = fn(node, results)
        except Exception as exc:  # report and continue with the next scenario
            ok = check(False, f"exception: {exc!r}", results)
        finally:
            node.destroy_node()
            launch.stop()
        print(f"[{'PASS' if ok else 'FAIL'}] {name}  (launch log: {launch.log.name})")
        for line in results:
            print("   ", line)
        if not ok:
            with open(launch.log.name, errors="replace") as fh:
                lines = [ln for ln in fh if "WARN" in ln or "ERROR" in ln or "rror" in ln]
            print("    --- launch log warnings/errors (last 40) ---")
            for ln in lines[-40:]:
                print("    |", ln.rstrip())
        summary.append(ok)
        if not wait_graph_clear():
            print("    (previous stack still in the ROS graph after 30 s)")
    rclpy.shutdown()
    print("ALL PASS" if all(summary) else "SOME SCENARIOS FAILED")
    return 0 if all(summary) else 1


if __name__ == "__main__":
    sys.exit(main())
