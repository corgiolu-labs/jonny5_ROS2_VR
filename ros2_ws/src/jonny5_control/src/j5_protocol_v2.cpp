#include "jonny5_control/j5_protocol_v2.hpp"

#include <algorithm>
#include <cmath>

namespace jonny5_control::j5v2
{
namespace
{

void put_u16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}

void put_u32(uint8_t * p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v >> 24);
  p[1] = static_cast<uint8_t>(v >> 16);
  p[2] = static_cast<uint8_t>(v >> 8);
  p[3] = static_cast<uint8_t>(v & 0xFF);
}

uint16_t get_u16(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

int16_t get_i16(const uint8_t * p)
{
  return static_cast<int16_t>(get_u16(p));
}

uint32_t get_u32(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

int16_t to_i16(double v, double scale)
{
  const double x = std::round(v * scale);
  if (std::isnan(x)) {return 0;}
  return static_cast<int16_t>(std::clamp(x, -32768.0, 32767.0));
}

Frame header(uint8_t frame_type, uint16_t sequence)
{
  Frame f{};
  f[0] = 'J';
  f[1] = '5';
  f[2] = kVersionV2;
  f[3] = frame_type;
  put_u16(&f[4], sequence);
  f[6] = static_cast<uint8_t>(kFrameSize);
  f[7] = kFlagCrc16;
  return f;
}

}  // namespace

uint16_t crc16_ccitt(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void seal(Frame & frame)
{
  frame[2] = kVersionV2;
  frame[7] = kFlagCrc16;
  put_u16(&frame[62], crc16_ccitt(frame.data(), kFrameSize - 2));
}

bool crc_ok(const uint8_t * frame)
{
  if (frame[0] != 'J' || frame[1] != '5' || frame[2] != kVersionV2 ||
    frame[6] != kFrameSize || frame[7] != kFlagCrc16)
  {
    return false;
  }
  return crc16_ccitt(frame, kFrameSize - 2) == get_u16(&frame[62]);
}

Frame build_request(uint8_t frame_type, uint16_t sequence)
{
  Frame f = header(frame_type, sequence);
  seal(f);
  return f;
}

Frame build_j5ik(
  const std::array<int16_t, 6> & targets_cdeg, uint16_t sequence, uint16_t heartbeat,
  bool enable, uint16_t target_id)
{
  Frame f = header(kTypeJ5ik, sequence);
  uint8_t * p = &f[kPayloadOffset];
  p[0] = 1;  // valid
  p[1] = enable ? kJ5ikFlagStreamEnable : 0;
  put_u16(p + 2, target_id);
  put_u16(p + 4, heartbeat);
  p[6] = kModeJointStream;
  for (std::size_t i = 0; i < 6; ++i) {
    put_u16(p + 8 + 2 * i, static_cast<uint16_t>(targets_cdeg[i]));
  }
  seal(f);
  return f;
}

std::optional<Telemetry> parse_telemetry(const uint8_t * frame)
{
  if (!crc_ok(frame) || frame[3] != kTypeTelemetryV2) {
    return std::nullopt;
  }
  const uint8_t * p = frame + kPayloadOffset;
  Telemetry t;
  t.sequence = get_u16(frame + 4);
  t.stm_time_ms = get_u32(p + 0);
  t.stm_tx_seq = get_u16(p + 4);
  t.rx_seq_gaps = get_u16(p + 6);
  t.fsm_state = p[8];
  t.status_flags = p[9];
  t.mode = p[10];
  t.diag_flags = p[11];
  for (std::size_t i = 0; i < 6; ++i) {
    t.joint_cdeg[i] = get_i16(p + 12 + 2 * i);
  }
  for (std::size_t i = 0; i < 4; ++i) {
    t.quat[i] = get_i16(p + 24 + 2 * i) / 32767.0;
  }
  for (std::size_t i = 0; i < 3; ++i) {
    t.gyro[i] = get_i16(p + 32 + 2 * i) / 1000.0;
    t.accel[i] = get_i16(p + 38 + 2 * i) / 100.0;
  }
  t.imu_sample_counter = get_u16(p + 44);
  t.rt_loop_period_us = get_u16(p + 46);
  t.rt_step_us = get_u16(p + 48);
  t.rt_overruns = get_u16(p + 50);
  t.crc_errors = get_u16(p + 52);
  return t;
}

Frame build_telemetry(const Telemetry & t, uint16_t sequence)
{
  Frame f = header(kTypeTelemetryV2, sequence);
  uint8_t * p = &f[kPayloadOffset];
  put_u32(p + 0, t.stm_time_ms);
  put_u16(p + 4, t.stm_tx_seq);
  put_u16(p + 6, t.rx_seq_gaps);
  p[8] = t.fsm_state;
  p[9] = t.status_flags;
  p[10] = t.mode;
  p[11] = t.diag_flags;
  for (std::size_t i = 0; i < 6; ++i) {
    put_u16(p + 12 + 2 * i, static_cast<uint16_t>(t.joint_cdeg[i]));
  }
  for (std::size_t i = 0; i < 4; ++i) {
    put_u16(p + 24 + 2 * i, static_cast<uint16_t>(to_i16(t.quat[i], 32767.0)));
  }
  for (std::size_t i = 0; i < 3; ++i) {
    put_u16(p + 32 + 2 * i, static_cast<uint16_t>(to_i16(t.gyro[i], 1000.0)));
    put_u16(p + 38 + 2 * i, static_cast<uint16_t>(to_i16(t.accel[i], 100.0)));
  }
  put_u16(p + 44, t.imu_sample_counter);
  put_u16(p + 46, t.rt_loop_period_us);
  put_u16(p + 48, t.rt_step_us);
  put_u16(p + 50, t.rt_overruns);
  put_u16(p + 52, t.crc_errors);
  seal(f);
  return f;
}

const uint8_t * find_v2_frame(const uint8_t * rx, std::size_t len)
{
  if (len < kFrameSize) {return nullptr;}
  // Canonical offsets first (0, then 64 in the 128-byte padded transport).
  for (std::size_t off : {std::size_t{0}, kFrameSize}) {
    if (off + kFrameSize <= len && crc_ok(rx + off)) {return rx + off;}
  }
  for (std::size_t off = 1; off + kFrameSize <= len; ++off) {
    if (off != kFrameSize && crc_ok(rx + off)) {return rx + off;}
  }
  return nullptr;
}

}  // namespace jonny5_control::j5v2
