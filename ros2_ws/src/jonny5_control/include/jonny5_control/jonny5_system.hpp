// ros2_control SystemInterface for JONNY5: streams joint position targets to the
// STM32 over SPI protocol v2 (J5IK, mode JOINT_STREAM) and reads TELEMETRY_V2.
#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "jonny5_control/j5_protocol_v2.hpp"
#include "jonny5_control/spi_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jonny5_control
{

/// Pure conversions (exposed for tests). Joint angle 0 rad = mechanical HOME.
double physical_cdeg_to_joint_rad(int16_t cdeg, double offset_deg, int dir);
int16_t joint_rad_to_physical_cdeg(double rad, double offset_deg, int dir);

class Jonny5System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Jonny5System)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  static constexpr std::size_t kJoints = 6;
  using Clock = std::chrono::steady_clock;

  bool exchange(const j5v2::Frame & tx);
  bool poll_telemetry(int attempts);
  void apply_telemetry();

  // Parameters
  bool mock_ = false;
  std::string spi_device_ = "/dev/spidev0.0";
  uint32_t spi_speed_hz_ = 1000000;
  std::size_t transfer_len_ = 128;
  std::array<double, kJoints> offsets_deg_{100.0, 88.0, 93.0, 95.0, 90.0, 95.0};
  std::array<int, kJoints> dirs_{1, -1, 1, 1, -1, 1};
  std::array<double, kJoints> cmd_min_{};
  std::array<double, kJoints> cmd_max_{};
  std::chrono::milliseconds link_timeout_{500};
  double arm_gate_rad_ = 0.05;

  std::unique_ptr<Transport> transport_;
  std::vector<uint8_t> tx_buf_;
  std::vector<uint8_t> rx_buf_;
  uint16_t sequence_ = 0;
  uint16_t heartbeat_ = 0;
  bool streaming_ = false;
  // Stream consent actually sent to the STM32 (STREAM_ENABLE). Granted only
  // while the firmware is IDLE without E-STOP and the command is at the
  // current pose; withdrawn whenever the firmware leaves IDLE.
  bool stream_armed_ = false;

  // Latest telemetry
  bool have_telemetry_ = false;
  j5v2::Telemetry telemetry_;
  Clock::time_point last_rx_{};
  uint16_t last_applied_tx_seq_ = 0;
  uint32_t last_applied_time_ms_ = 0;
  bool have_applied_ = false;
  double crc_errors_pi_ = 0.0;

  // ros2_control storage
  std::array<double, kJoints> pos_{};
  std::array<double, kJoints> vel_{};
  std::array<double, kJoints> cmd_{};
  std::array<double, 10> imu_{};   // orientation xyzw, angular_velocity xyz, linear_acceleration xyz
  std::vector<double> status_;     // gpio state interfaces, see kStatusNames
};

}  // namespace jonny5_control
