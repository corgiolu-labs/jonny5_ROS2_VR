#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_FILE="$(mktemp /tmp/jonny5_ros2_dryrun.XXXXXX.log)"
ECHO_DIR="$(mktemp -d /tmp/jonny5_ros2_echo.XXXXXX)"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

fail() {
  echo "[FAIL] $*" >&2
  echo "--- launch log tail (${LOG_FILE}) ---" >&2
  tail -n 80 "${LOG_FILE}" >&2 || true
  exit 1
}

pass() {
  echo "[ OK ] $*"
}

source_ros() {
  set +u
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # JONNY5 currently targets ROS2 Jazzy in WSL/Ubuntu Noble.
    source /opt/ros/jazzy/setup.bash
  elif [[ -n "${ROS_DISTRO:-}" ]] && [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    set -u
    fail "ROS2 setup.bash not found. Install/source ROS2 before running this smoke test."
  fi
  set -u
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if ros2 topic list | grep -Fxq "${topic}"; then
      pass "topic available: ${topic}"
      return 0
    fi
    sleep 1
  done
  fail "topic not available: ${topic}"
}

wait_for_node() {
  local node="$1"
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if ros2 node list | grep -Fxq "${node}"; then
      pass "node available: ${node}"
      return 0
    fi
    sleep 1
  done
  fail "node not available: ${node}"
}

echo "[JONNY5] ROS2 dry-run smoke test"
echo "[JONNY5] workspace: ${WS_DIR}"

cd "${WS_DIR}"
source_ros
pass "sourced ROS_DISTRO=${ROS_DISTRO:-unknown}"

colcon build --symlink-install
pass "colcon build completed"

set +u
source install/setup.bash
set -u
pass "sourced workspace overlay"

ros2 launch jonny5_bringup bringup.launch.py \
  hardware_enabled:=false \
  sim_telemetry:=true \
  sim_intent:=true \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

sleep 8

wait_for_node "/robot_state_publisher"
wait_for_node "/jonny5_legacy_telemetry_sim"
wait_for_node "/jonny5_teleop_intent_sim"
wait_for_node "/jonny5_spi_bridge"
wait_for_node "/jonny5_vr_bridge"

wait_for_topic "/joint_states"
wait_for_topic "/imu/data"
wait_for_topic "/jonny5/status"
wait_for_topic "/jonny5/spi/telemetry"
wait_for_topic "/jonny5/teleop/intent"

timeout 8s ros2 topic echo --once /joint_states >"${ECHO_DIR}/joint_states.txt" \
  || fail "no message received on /joint_states"
grep -q "base_joint" "${ECHO_DIR}/joint_states.txt" \
  || fail "/joint_states did not include base_joint"
pass "/joint_states publishes simulated joints"

timeout 8s ros2 topic echo --once /jonny5/status >"${ECHO_DIR}/status.txt" \
  || fail "no message received on /jonny5/status"
grep -q "state: IDLE" "${ECHO_DIR}/status.txt" \
  || fail "/jonny5/status did not report IDLE"
grep -q "imu_online: true" "${ECHO_DIR}/status.txt" \
  || fail "/jonny5/status did not report imu_online=true"
pass "/jonny5/status publishes healthy simulated state"

timeout 8s ros2 topic echo --once /jonny5/teleop/intent >"${ECHO_DIR}/sim_intent.txt" \
  || fail "no message received on /jonny5/teleop/intent from simulator"
grep -q "mode: 2" "${ECHO_DIR}/sim_intent.txt" \
  || fail "simulated intent did not publish MODE_MANUAL"
pass "simulated teleop intent publishes"

python3 - <<'PY'
import asyncio
import json
import websockets

async def main():
    payload = {
        "mode": 2,
        "joy_x": 0.42,
        "joy_y": -0.25,
        "pitch": 0.12,
        "yaw": -0.18,
        "intensity": 0.75,
        "grip": 1,
        "heartbeat": 4242,
        "quat_w": 1.0,
        "quat_x": 0.0,
        "quat_y": 0.0,
        "quat_z": 0.0,
        "buttons_left": 2,
        "buttons_right": 2,
    }
    async with websockets.connect("ws://127.0.0.1:8567") as ws:
        for _ in range(5):
            await ws.send(json.dumps(payload))
            await asyncio.sleep(0.2)

asyncio.run(main())
PY

timeout 12s bash -lc "ros2 topic echo /jonny5/teleop/intent | tee '${ECHO_DIR}/ws_intent.txt' | grep -m1 'heartbeat: 4242'" \
  >/dev/null || fail "WebSocket intent heartbeat was not observed on ROS2 topic"
grep -q "grip: true" "${ECHO_DIR}/ws_intent.txt" \
  || fail "WebSocket intent grip=true was not observed on ROS2 topic"
pass "WebSocket JSON is bridged to /jonny5/teleop/intent"

echo "[PASS] JONNY5 ROS2 dry-run smoke test completed"
echo "[INFO] launch log: ${LOG_FILE}"