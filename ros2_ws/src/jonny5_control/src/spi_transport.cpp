#include "jonny5_control/spi_transport.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <utility>

namespace jonny5_control
{

SpidevTransport::SpidevTransport(std::string device, uint32_t speed_hz)
: device_(std::move(device)), speed_hz_(speed_hz) {}

SpidevTransport::~SpidevTransport() {close();}

bool SpidevTransport::open(std::string & error)
{
  close();
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    error = "open(" + device_ + "): " + std::strerror(errno);
    return false;
  }
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
    ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
    ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
  {
    error = "spidev configuration failed: " + std::string(std::strerror(errno));
    close();
    return false;
  }
  return true;
}

void SpidevTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SpidevTransport::transfer(
  const uint8_t * tx, uint8_t * rx, std::size_t len, std::string & error)
{
  if (fd_ < 0) {
    error = "spidev not open";
    return false;
  }
  spi_ioc_transfer xfer{};
  xfer.tx_buf = reinterpret_cast<uintptr_t>(tx);
  xfer.rx_buf = reinterpret_cast<uintptr_t>(rx);
  xfer.len = static_cast<uint32_t>(len);
  xfer.speed_hz = speed_hz_;
  xfer.bits_per_word = 8;
  if (ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer) < 0) {
    error = "SPI transfer failed: " + std::string(std::strerror(errno));
    return false;
  }
  return true;
}

MockFirmwareTransport::MockFirmwareTransport(
  std::array<int16_t, 6> initial_cdeg, double max_speed_deg_s)
: max_speed_cdeg_s_(max_speed_deg_s * 100.0)
{
  for (std::size_t i = 0; i < 6; ++i) {
    joint_cdeg_[i] = initial_cdeg[i];
  }
  telemetry_.fsm_state = static_cast<uint8_t>(j5v2::FsmState::kIdle);
  telemetry_.status_flags = j5v2::kStMoveAllowed | j5v2::kStImuValid;
  // The mock starts at a known pose (HOME), like the real arm after HOME.
  telemetry_.diag_flags = j5v2::kDgImuPresent | j5v2::kDgImuEnabled | j5v2::kDgPoseKnown;
  telemetry_.rt_loop_period_us = 1000;
  telemetry_.rt_step_us = 45;
}

bool MockFirmwareTransport::open(std::string &)
{
  start_ = last_ = std::chrono::steady_clock::now();
  return true;
}

void MockFirmwareTransport::close() {}

bool MockFirmwareTransport::transfer(
  const uint8_t * tx, uint8_t * rx, std::size_t len, std::string & error)
{
  if (len < j5v2::kFrameSize) {
    error = "mock transfer shorter than one frame";
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - last_).count();
  last_ = now;

  const uint16_t seq = static_cast<uint16_t>((tx[4] << 8) | tx[5]);
  if (j5v2::crc_ok(tx) && tx[3] == j5v2::kTypeJ5ik) {
    const uint8_t * p = tx + j5v2::kPayloadOffset;
    const bool enable = p[0] != 0 && p[6] == j5v2::kModeJointStream &&
      (p[1] & j5v2::kJ5ikFlagStreamEnable);
    telemetry_.mode = p[6];
    if (enable) {
      telemetry_.diag_flags |= j5v2::kDgStreamLive;
      const double max_step = max_speed_cdeg_s_ * std::clamp(dt, 0.0, 0.1);
      for (std::size_t i = 0; i < 6; ++i) {
        const auto target = static_cast<int16_t>((p[8 + 2 * i] << 8) | p[9 + 2 * i]);
        const double diff = std::clamp(target - joint_cdeg_[i], -max_step, max_step);
        joint_cdeg_[i] += diff;
      }
    } else {
      telemetry_.diag_flags &= static_cast<uint8_t>(~j5v2::kDgStreamLive);
    }
  }

  telemetry_.stm_time_ms = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now - start_).count());
  telemetry_.stm_tx_seq = static_cast<uint16_t>(telemetry_.stm_tx_seq + 1);
  telemetry_.imu_sample_counter = static_cast<uint16_t>(telemetry_.imu_sample_counter + 4);
  for (std::size_t i = 0; i < 6; ++i) {
    telemetry_.joint_cdeg[i] = static_cast<int16_t>(std::lround(joint_cdeg_[i]));
  }
  const j5v2::Frame reply = j5v2::build_telemetry(telemetry_, seq);
  std::memset(rx, 0, len);
  std::memcpy(rx, reply.data(), reply.size());
  return true;
}

}  // namespace jonny5_control
