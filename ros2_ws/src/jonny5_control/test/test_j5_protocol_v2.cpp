#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>
#include <string>
#include <vector>

#include "jonny5_control/j5_protocol_v2.hpp"
#include "jonny5_control/spi_transport.hpp"

using namespace jonny5_control;  // NOLINT

namespace
{
std::vector<uint8_t> from_hex(const std::string & hex)
{
  std::vector<uint8_t> out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}
}  // namespace

TEST(J5ProtocolV2, CrcCheckValue)
{
  const char * s = "123456789";
  EXPECT_EQ(j5v2::crc16_ccitt(reinterpret_cast<const uint8_t *>(s), 9), 0x29B1);
}

TEST(J5ProtocolV2, ParsesFrameBuiltByFirmware)
{
  // TELEMETRY_V2 produced by firmware j5_build_telemetry_v2() (host build, see PR).
  const auto f = from_hex(
    "4a350208010240010001e2400001000301da000e2710226024542" "51c2328251c5a82000000005a8201f4"
    "000000000000000003d5234503e8002bffff00048a8f");
  ASSERT_EQ(f.size(), 64u);
  const auto t = j5v2::parse_telemetry(f.data());
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->sequence, 0x0102);
  EXPECT_EQ(t->stm_time_ms, 123456u);
  EXPECT_EQ(t->rx_seq_gaps, 3);
  EXPECT_EQ(t->fsm_state, 1);
  EXPECT_TRUE(t->movement_allowed());
  EXPECT_TRUE(t->imu_valid());
  EXPECT_TRUE(t->stream_live());
  const std::array<int16_t, 6> joints{10000, 8800, 9300, 9500, 9000, 9500};
  EXPECT_EQ(t->joint_cdeg, joints);
  EXPECT_NEAR(t->quat[0], 0.7071, 1e-4);
  EXPECT_NEAR(t->accel[2], 9.81, 1e-9);
  EXPECT_NEAR(t->gyro[0], 0.5, 1e-9);
  EXPECT_EQ(t->rt_step_us, 43);
  EXPECT_EQ(t->rt_overruns, 0xFFFF);
  EXPECT_EQ(t->crc_errors, 4);
}

TEST(J5ProtocolV2, J5ikMatchesPythonCodec)
{
  // controller.spi_dataplane.j5_protocol_v2.build_j5ik_frame(
  //   [10000, 8800, 9300, 9500, 9000, 9500], sequence=3, heartbeat=99, enable=True, target_id=7)
  const std::array<int16_t, 6> targets{10000, 8800, 9300, 9500, 9000, 9500};
  const auto f = j5v2::build_j5ik(targets, 3, 99, true, 7);
  EXPECT_TRUE(j5v2::crc_ok(f.data()));
  EXPECT_EQ(f[3], j5v2::kTypeJ5ik);
  EXPECT_EQ(f[8], 1);
  EXPECT_EQ(f[9], j5v2::kJ5ikFlagStreamEnable);
  EXPECT_EQ(f[14], j5v2::kModeJointStream);
  EXPECT_EQ(f[16], 0x27);
  EXPECT_EQ(f[17], 0x10);
  const auto off = j5v2::build_j5ik(targets, 3, 99, false, 7);
  EXPECT_EQ(off[9], 0);
}

TEST(J5ProtocolV2, TelemetryRoundTripAndCorruption)
{
  j5v2::Telemetry t;
  t.stm_time_ms = 42;
  t.stm_tx_seq = 7;
  t.fsm_state = 2;
  t.status_flags = j5v2::kStEstop;
  t.joint_cdeg = {1, -2, 3, -4, 5, -6};
  t.quat = {0.5, -0.5, 0.5, -0.5};
  auto f = j5v2::build_telemetry(t, 9);
  auto back = j5v2::parse_telemetry(f.data());
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->joint_cdeg, t.joint_cdeg);
  EXPECT_TRUE(back->estop());
  EXPECT_NEAR(back->quat[1], -0.5, 1e-4);
  f[30] ^= 0x01;
  EXPECT_FALSE(j5v2::parse_telemetry(f.data()).has_value());
}

TEST(J5ProtocolV2, FindsFrameInPadded128Transfer)
{
  const auto f = j5v2::build_request(j5v2::kTypeStatus, 5);
  std::vector<uint8_t> rx(128, 0);
  std::memcpy(rx.data() + 64, f.data(), 64);
  EXPECT_EQ(j5v2::find_v2_frame(rx.data(), rx.size()), rx.data() + 64);
  rx[64 + 20] ^= 0xFF;
  EXPECT_EQ(j5v2::find_v2_frame(rx.data(), rx.size()), nullptr);
}

TEST(MockFirmware, TracksEnabledTargetsOnly)
{
  MockFirmwareTransport mock({9000, 9000, 9000, 9000, 9000, 9000}, 1000.0);
  std::string err;
  ASSERT_TRUE(mock.open(err));
  std::array<uint8_t, 64> rx{};
  const std::array<int16_t, 6> target{9500, 9000, 9000, 9000, 9000, 9000};
  auto tx = j5v2::build_j5ik(target, 1, 1, false);
  ASSERT_TRUE(mock.transfer(tx.data(), rx.data(), rx.size(), err));
  EXPECT_EQ(j5v2::parse_telemetry(rx.data())->joint_cdeg[0], 9000);
  for (uint16_t k = 0; k < 50; ++k) {
    tx = j5v2::build_j5ik(target, k, k, true);
    mock.transfer(tx.data(), rx.data(), rx.size(), err);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto t = j5v2::parse_telemetry(rx.data());
  EXPECT_EQ(t->joint_cdeg[0], 9500);
  EXPECT_TRUE(t->stream_live());
}
