#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_ROOT="${1:-${WS_DIR}/bags}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${OUTPUT_ROOT}/jonny5_${STAMP}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 is not available; source the ROS 2 and workspace setup files first." >&2
  exit 1
fi

mkdir -p "${OUTPUT_ROOT}"

topics=(
  /joint_states
  /imu/data
  /jonny5/status
  /jonny5/spi/telemetry
  /jonny5/teleop/intent
)

echo "[JONNY5] Recording rosbag2 dataset to ${OUTPUT_DIR}"
echo "[JONNY5] Topics: ${topics[*]}"
echo "[JONNY5] Stop cleanly with Ctrl-C so rosbag2 can finalize its metadata."

ros2 bag record --output "${OUTPUT_DIR}" "${topics[@]}"

