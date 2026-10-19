// SPI transports for the JONNY5 ros2_control hardware interface.
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "jonny5_control/j5_protocol_v2.hpp"

namespace jonny5_control
{

/// Full-duplex, fixed-length transfer to the STM32 (64 B, or 128 B padded).
class Transport
{
public:
  virtual ~Transport() = default;
  virtual bool open(std::string & error) = 0;
  virtual void close() = 0;
  virtual bool transfer(const uint8_t * tx, uint8_t * rx, std::size_t len, std::string & error) = 0;
};

/// Linux spidev (/dev/spidevB.C), SPI mode 0, 8 bits per word.
class SpidevTransport : public Transport
{
public:
  SpidevTransport(std::string device, uint32_t speed_hz);
  ~SpidevTransport() override;
  bool open(std::string & error) override;
  void close() override;
  bool transfer(const uint8_t * tx, uint8_t * rx, std::size_t len, std::string & error) override;

private:
  std::string device_;
  uint32_t speed_hz_;
  int fd_ = -1;
};

/// Hardware-free stand-in for the v2 firmware: answers every request with a
/// TELEMETRY_V2 frame (FSM IDLE) and, while a J5IK frame carries the enable
/// flag, moves its joints toward the targets at ``max_speed_deg_s``.
class MockFirmwareTransport : public Transport
{
public:
  explicit MockFirmwareTransport(
    std::array<int16_t, 6> initial_cdeg, double max_speed_deg_s = 60.0);
  bool open(std::string & error) override;
  void close() override;
  bool transfer(const uint8_t * tx, uint8_t * rx, std::size_t len, std::string & error) override;

private:
  std::array<double, 6> joint_cdeg_;
  double max_speed_cdeg_s_;
  j5v2::Telemetry telemetry_;
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point last_;
};

}  // namespace jonny5_control
