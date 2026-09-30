// J5 SPI protocol v2 codec (C++). Mirrors firmware src/spi/j5_protocol.c and
// raspberry/controller/spi_dataplane/j5_protocol_v2.py.
// Specification: ros2_ws/docs/SPI_PROTOCOL_V2.md
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace jonny5_control::j5v2
{

constexpr std::size_t kFrameSize = 64;
constexpr std::size_t kPayloadOffset = 8;
constexpr std::size_t kPayloadLen = 54;

constexpr uint8_t kVersionV2 = 2;
constexpr uint8_t kFlagCrc16 = 0x01;

constexpr uint8_t kTypeTelemetry = 0x01;
constexpr uint8_t kTypeStatus = 0x03;
constexpr uint8_t kTypeJ5vr = 0x04;
constexpr uint8_t kTypeJ5ik = 0x05;
constexpr uint8_t kTypeTelemetryV2 = 0x08;

constexpr uint8_t kModeJointStream = 6;
constexpr uint8_t kJ5ikFlagStreamEnable = 1u << 7;

// TELEMETRY_V2 status_flags
constexpr uint8_t kStEstop = 1u << 0;
constexpr uint8_t kStMoveAllowed = 1u << 1;
constexpr uint8_t kStDeadman = 1u << 2;
constexpr uint8_t kStInput = 1u << 3;
constexpr uint8_t kStArmed = 1u << 4;
constexpr uint8_t kStFreeze = 1u << 5;
constexpr uint8_t kStSetpose = 1u << 6;
constexpr uint8_t kStImuValid = 1u << 7;

// TELEMETRY_V2 diag_flags
constexpr uint8_t kDgGuardSeen = 1u << 0;
constexpr uint8_t kDgImuPresent = 1u << 1;
constexpr uint8_t kDgImuEnabled = 1u << 2;
constexpr uint8_t kDgStreamLive = 1u << 3;
// Joint angles are the real arm pose (a SETPOSE completed since the STM32
// booted). Without it joint_cdeg are init defaults, not the arm.
constexpr uint8_t kDgPoseKnown = 1u << 4;

enum class FsmState : uint8_t { kSafe = 0, kIdle = 1, kStopped = 2 };

using Frame = std::array<uint8_t, kFrameSize>;

struct Telemetry
{
  uint16_t sequence = 0;         // echo of the Pi request sequence
  uint32_t stm_time_ms = 0;
  uint16_t stm_tx_seq = 0;
  uint16_t rx_seq_gaps = 0;
  uint8_t fsm_state = 0;
  uint8_t status_flags = 0;
  uint8_t mode = 0;
  uint8_t diag_flags = 0;
  std::array<int16_t, 6> joint_cdeg{};   // physical, B S G Y P R
  std::array<double, 4> quat{1.0, 0.0, 0.0, 0.0};  // w x y z
  std::array<double, 3> gyro{};    // rad/s
  std::array<double, 3> accel{};   // m/s^2
  uint16_t imu_sample_counter = 0;
  uint16_t rt_loop_period_us = 0;
  uint16_t rt_step_us = 0;
  uint16_t rt_overruns = 0;
  uint16_t crc_errors = 0;

  bool estop() const {return status_flags & kStEstop;}
  bool movement_allowed() const {return status_flags & kStMoveAllowed;}
  bool imu_valid() const {return status_flags & kStImuValid;}
  bool stream_live() const {return diag_flags & kDgStreamLive;}
  bool setpose_active() const {return status_flags & kStSetpose;}
  bool pose_known() const {return diag_flags & kDgPoseKnown;}
};

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
uint16_t crc16_ccitt(const uint8_t * data, std::size_t len);

/// Turn a 64-byte frame into v2: version=2, flags=CRC16, CRC in bytes 62-63.
void seal(Frame & frame);

/// True if the 64 bytes at ``frame`` are a v2 frame with a valid CRC.
bool crc_ok(const uint8_t * frame);

/// Empty-payload v2 request (e.g. TELEMETRY poll or STATUS).
Frame build_request(uint8_t frame_type, uint16_t sequence);

/// J5IK joint-stream frame: 6 physical targets in centi-degrees (B S G Y P R).
Frame build_j5ik(
  const std::array<int16_t, 6> & targets_cdeg, uint16_t sequence, uint16_t heartbeat,
  bool enable, uint16_t target_id = 0);

/// Decode a TELEMETRY_V2 frame; std::nullopt if not a CRC-valid 0x08 frame.
std::optional<Telemetry> parse_telemetry(const uint8_t * frame);

/// Encode a TELEMETRY_V2 frame (firmware side; used by the mock transport and tests).
Frame build_telemetry(const Telemetry & t, uint16_t sequence);

/// Locate the canonical 64-byte v2 frame inside a transfer (64 or 128 B padded).
/// Returns nullptr if no CRC-valid v2 frame is present.
const uint8_t * find_v2_frame(const uint8_t * rx, std::size_t len);

}  // namespace jonny5_control::j5v2
