#include "jonny5_control/jonny5_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace jonny5_control
{
namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("Jonny5System");

// gpio "jonny5_status" state interfaces, in this order.
const std::array<const char *, 10> kStatusNames = {
  "fsm_state", "estop_active", "movement_allowed", "joint_stream_live", "link_ok",
  "rt_loop_period_us", "rt_overruns", "rx_seq_gaps", "crc_errors_stm", "crc_errors_pi"};

const std::array<const char *, 10> kImuNames = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

template<std::size_t N, typename T>
bool parse_list(const std::string & text, std::array<T, N> & out)
{
  std::stringstream ss(text);
  std::string item;
  std::size_t i = 0;
  while (std::getline(ss, item, ',')) {
    if (i >= N) {return false;}
    try {
      out[i++] = static_cast<T>(std::stod(item));
    } catch (const std::exception &) {
      return false;
    }
  }
  return i == N;
}

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}

}  // namespace

double physical_cdeg_to_joint_rad(int16_t cdeg, double offset_deg, int dir)
{
  const double deg = (cdeg / 100.0 - offset_deg) * (dir < 0 ? -1.0 : 1.0);
  return deg * M_PI / 180.0;
}

int16_t joint_rad_to_physical_cdeg(double rad, double offset_deg, int dir)
{
  double deg = offset_deg + (dir < 0 ? -1.0 : 1.0) * rad * 180.0 / M_PI;
  deg = std::clamp(deg, 0.0, 180.0);
  return static_cast<int16_t>(std::lround(deg * 100.0));
}

hardware_interface::CallbackReturn Jonny5System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    // xacro renders booleans as "True"/"False".
    std::string mock = param(info_, "mock_hardware", "false");
    std::transform(mock.begin(), mock.end(), mock.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    mock_ = (mock == "true" || mock == "1");
  }
  spi_device_ = param(info_, "spi_device", spi_device_);
  try {
    spi_speed_hz_ = static_cast<uint32_t>(std::stoul(param(info_, "spi_speed_hz", "1000000")));
    transfer_len_ = std::stoul(param(info_, "transfer_len", "128"));
    link_timeout_ = std::chrono::milliseconds(std::stoul(param(info_, "link_timeout_ms", "500")));
    arm_gate_rad_ = std::stod(param(info_, "arm_gate_rad", "0.05"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "Invalid numeric hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (transfer_len_ != 64 && transfer_len_ != 128) {
    RCLCPP_ERROR(kLogger, "transfer_len must be 64 or 128 (firmware padded mode), got %zu",
      transfer_len_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_list(param(info_, "servo_offsets_deg", "100,88,93,95,90,95"), offsets_deg_) ||
    !parse_list(param(info_, "servo_dirs", "1,-1,-1,1,-1,1"), dirs_))
  {
    RCLCPP_ERROR(kLogger, "servo_offsets_deg / servo_dirs need 6 comma-separated values");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJoints) {
    RCLCPP_ERROR(kLogger, "Expected %zu joints (B S G Y P R), got %zu", kJoints,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kJoints; ++i) {
    const auto & joint = info_.joints[i];
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(kLogger, "Joint '%s' needs exactly one 'position' command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto & ci = joint.command_interfaces[0];
    cmd_min_[i] = ci.min.empty() ? -std::numeric_limits<double>::infinity() : std::stod(ci.min);
    cmd_max_[i] = ci.max.empty() ? std::numeric_limits<double>::infinity() : std::stod(ci.max);
    for (const auto & si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION &&
        si.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_ERROR(kLogger, "Joint '%s': unsupported state interface '%s'",
          joint.name.c_str(), si.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  status_.assign(kStatusNames.size(), 0.0);
  pos_.fill(0.0);
  vel_.fill(0.0);
  cmd_.fill(std::numeric_limits<double>::quiet_NaN());
  imu_.fill(0.0);
  imu_[3] = 1.0;
  tx_buf_.assign(transfer_len_, 0);
  rx_buf_.assign(transfer_len_, 0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Jonny5System::on_configure(const rclcpp_lifecycle::State &)
{
  if (mock_) {
    std::array<int16_t, kJoints> home{};
    for (std::size_t i = 0; i < kJoints; ++i) {
      home[i] = static_cast<int16_t>(std::lround(offsets_deg_[i] * 100.0));
    }
    transport_ = std::make_unique<MockFirmwareTransport>(home);
  } else {
    transport_ = std::make_unique<SpidevTransport>(spi_device_, spi_speed_hz_);
  }
  std::string error;
  if (!transport_->open(error)) {
    RCLCPP_ERROR(kLogger, "Cannot open SPI transport: %s", error.c_str());
    transport_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Read the current pose so the joint_state_broadcaster has real data before activation.
  if (!poll_telemetry(20)) {
    RCLCPP_ERROR(kLogger, "No TELEMETRY_V2 from the STM32 on %s: is the v2 firmware flashed?",
      mock_ ? "mock" : spi_device_.c_str());
    transport_->close();
    transport_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  apply_telemetry();
  RCLCPP_INFO(kLogger, "JONNY5 hardware configured (%s, transfer %zu B)",
    mock_ ? "mock firmware" : spi_device_.c_str(), transfer_len_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Jonny5System::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (transport_) {
    transport_->close();
    transport_.reset();
  }
  have_telemetry_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Jonny5System::on_activate(const rclcpp_lifecycle::State &)
{
  if (!transport_ || !poll_telemetry(20)) {
    RCLCPP_ERROR(kLogger, "Activation failed: no telemetry from the STM32");
    return hardware_interface::CallbackReturn::ERROR;
  }
  apply_telemetry();
  // Start from where the arm is: no jump on the first write().
  cmd_ = pos_;
  stream_armed_ = false;
  if (telemetry_.estop()) {
    RCLCPP_WARN(kLogger, "E-STOP is latched: the arm will not move until it is released and "
      "re-enabled (SAFE -> ENABLE)");
  } else if (telemetry_.fsm_state != static_cast<uint8_t>(j5v2::FsmState::kIdle)) {
    RCLCPP_WARN(kLogger, "STM32 FSM state is %u (not IDLE): the arm stays still until an "
      "explicit UART ENABLE (joint streaming never re-arms the firmware by itself)",
      telemetry_.fsm_state);
  }
  streaming_ = true;
  RCLCPP_INFO(kLogger, "JONNY5 joint streaming enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Jonny5System::on_deactivate(const rclcpp_lifecycle::State &)
{
  streaming_ = false;
  stream_armed_ = false;
  if (transport_) {
    // Withdraw the stream consent: the firmware disables the JOINT_STREAM servos.
    std::array<int16_t, kJoints> hold{};
    for (std::size_t i = 0; i < kJoints; ++i) {
      hold[i] = joint_rad_to_physical_cdeg(pos_[i], offsets_deg_[i], dirs_[i]);
    }
    for (int k = 0; k < 3; ++k) {
      heartbeat_ = static_cast<uint16_t>(heartbeat_ + 1);
      if (heartbeat_ == 0) {heartbeat_ = 1;}
      exchange(j5v2::build_j5ik(hold, sequence_, heartbeat_, false));
    }
  }
  RCLCPP_INFO(kLogger, "JONNY5 joint streaming disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Jonny5System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t i = 0; i < kJoints; ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      double * ptr = si.name == hardware_interface::HW_IF_POSITION ? &pos_[i] : &vel_[i];
      out.emplace_back(info_.joints[i].name, si.name, ptr);
    }
  }
  for (const auto & sensor : info_.sensors) {
    for (const auto & si : sensor.state_interfaces) {
      const auto it = std::find(kImuNames.begin(), kImuNames.end(), si.name);
      if (it == kImuNames.end()) {
        RCLCPP_WARN(kLogger, "Sensor '%s': unknown state interface '%s' (ignored)",
          sensor.name.c_str(), si.name.c_str());
        continue;
      }
      out.emplace_back(sensor.name, si.name, &imu_[std::distance(kImuNames.begin(), it)]);
    }
  }
  for (const auto & gpio : info_.gpios) {
    for (const auto & si : gpio.state_interfaces) {
      const auto it = std::find(kStatusNames.begin(), kStatusNames.end(), si.name);
      if (it == kStatusNames.end()) {
        RCLCPP_WARN(kLogger, "GPIO '%s': unknown state interface '%s' (ignored)",
          gpio.name.c_str(), si.name.c_str());
        continue;
      }
      out.emplace_back(gpio.name, si.name, &status_[std::distance(kStatusNames.begin(), it)]);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> Jonny5System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (std::size_t i = 0; i < kJoints; ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_[i]);
  }
  return out;
}

hardware_interface::return_type Jonny5System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  apply_telemetry();
  const bool link_ok = have_telemetry_ && (Clock::now() - last_rx_) <= link_timeout_;
  status_[4] = link_ok ? 1.0 : 0.0;
  if (streaming_ && !link_ok) {
    RCLCPP_ERROR(kLogger, "No valid TELEMETRY_V2 for more than %lld ms: SPI link lost",
      static_cast<long long>(link_timeout_.count()));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Jonny5System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!streaming_ || !transport_) {
    return hardware_interface::return_type::OK;
  }
  std::array<double, kJoints> cmd_rad{};
  double max_jump = 0.0;
  for (std::size_t i = 0; i < kJoints; ++i) {
    double rad = std::isfinite(cmd_[i]) ? cmd_[i] : pos_[i];  // no command yet: hold
    rad = std::clamp(rad, cmd_min_[i], cmd_max_[i]);
    cmd_rad[i] = rad;
    max_jump = std::max(max_jump, std::abs(rad - pos_[i]));
  }

  // Stream consent gate (see stream_armed_).
  const bool fw_ready = have_telemetry_ && !telemetry_.estop() &&
    telemetry_.fsm_state == static_cast<uint8_t>(j5v2::FsmState::kIdle);
  auto clock = rclcpp::Clock(RCL_STEADY_TIME);
  if (stream_armed_ && !fw_ready) {
    stream_armed_ = false;
    RCLCPP_WARN(kLogger, "STM32 left IDLE (state %u, E-STOP %d): joint streaming paused. "
      "After UART ENABLE it resumes only once the command is at the current pose.",
      telemetry_.fsm_state, telemetry_.estop() ? 1 : 0);
  } else if (!stream_armed_ && fw_ready) {
    if (max_jump <= arm_gate_rad_) {
      stream_armed_ = true;
      RCLCPP_INFO(kLogger, "STM32 IDLE and command at the current pose: joint streaming armed");
    } else {
      RCLCPP_WARN_THROTTLE(kLogger, clock, 2000,
        "Command is %.3f rad away from the current pose: not streaming. Command the current "
        "pose (or restart the controller) to arm.", max_jump);
    }
  }

  std::array<int16_t, kJoints> targets{};
  for (std::size_t i = 0; i < kJoints; ++i) {
    const double rad = stream_armed_ ? cmd_rad[i] : pos_[i];
    targets[i] = joint_rad_to_physical_cdeg(rad, offsets_deg_[i], dirs_[i]);
  }
  heartbeat_ = static_cast<uint16_t>(heartbeat_ + 1);
  if (heartbeat_ == 0) {heartbeat_ = 1;}
  if (!exchange(j5v2::build_j5ik(targets, sequence_, heartbeat_, stream_armed_))) {
    // A single failed transfer is tolerated; read() reports a lost link after link_timeout.
    RCLCPP_WARN_THROTTLE(kLogger, clock, 1000, "SPI exchange failed");
  }
  return hardware_interface::return_type::OK;
}

bool Jonny5System::exchange(const j5v2::Frame & tx)
{
  std::fill(tx_buf_.begin(), tx_buf_.end(), 0);
  std::copy(tx.begin(), tx.end(), tx_buf_.begin());
  std::string error;
  const bool ok = transport_->transfer(tx_buf_.data(), rx_buf_.data(), transfer_len_, error);
  sequence_ = static_cast<uint16_t>(sequence_ + 1);
  if (!ok) {
    return false;
  }
  const uint8_t * frame = j5v2::find_v2_frame(rx_buf_.data(), transfer_len_);
  if (frame == nullptr) {
    // Count replies that look like v2 but fail the CRC (torn/corrupted frames).
    for (std::size_t off = 0; off + 3 <= transfer_len_; ++off) {
      if (rx_buf_[off] == 'J' && rx_buf_[off + 1] == '5' && rx_buf_[off + 2] == j5v2::kVersionV2) {
        crc_errors_pi_ += 1.0;
        break;
      }
    }
    return false;
  }
  if (auto t = j5v2::parse_telemetry(frame)) {
    telemetry_ = *t;
    have_telemetry_ = true;
    last_rx_ = Clock::now();
  }
  return true;
}

bool Jonny5System::poll_telemetry(int attempts)
{
  // The SPI slave answers one transfer late: poll until a TELEMETRY_V2 arrives.
  const auto before = last_rx_;
  for (int k = 0; k < attempts; ++k) {
    exchange(j5v2::build_request(j5v2::kTypeTelemetry, sequence_));
    if (have_telemetry_ && last_rx_ != before) {
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

void Jonny5System::apply_telemetry()
{
  if (!have_telemetry_) {
    return;
  }
  const j5v2::Telemetry & t = telemetry_;
  const bool new_sample = !have_applied_ || t.stm_tx_seq != last_applied_tx_seq_;
  if (new_sample) {
    const double dt = have_applied_ ? (t.stm_time_ms - last_applied_time_ms_) / 1000.0 : 0.0;
    for (std::size_t i = 0; i < kJoints; ++i) {
      const double p = physical_cdeg_to_joint_rad(t.joint_cdeg[i], offsets_deg_[i], dirs_[i]);
      vel_[i] = dt > 1e-4 ? (p - pos_[i]) / dt : 0.0;
      pos_[i] = p;
    }
    last_applied_tx_seq_ = t.stm_tx_seq;
    last_applied_time_ms_ = t.stm_time_ms;
    have_applied_ = true;
  }
  imu_[0] = t.quat[1];
  imu_[1] = t.quat[2];
  imu_[2] = t.quat[3];
  imu_[3] = t.quat[0];
  for (std::size_t i = 0; i < 3; ++i) {
    imu_[4 + i] = t.gyro[i];
    imu_[7 + i] = t.accel[i];
  }
  status_[0] = t.fsm_state;
  status_[1] = t.estop() ? 1.0 : 0.0;
  status_[2] = t.movement_allowed() ? 1.0 : 0.0;
  status_[3] = t.stream_live() ? 1.0 : 0.0;
  status_[5] = t.rt_loop_period_us;
  status_[6] = t.rt_overruns;
  status_[7] = t.rx_seq_gaps;
  status_[8] = t.crc_errors;
  status_[9] = crc_errors_pi_;
}

}  // namespace jonny5_control

PLUGINLIB_EXPORT_CLASS(jonny5_control::Jonny5System, hardware_interface::SystemInterface)
