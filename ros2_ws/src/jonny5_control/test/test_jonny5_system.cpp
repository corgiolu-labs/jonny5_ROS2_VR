#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "jonny5_control/jonny5_system.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using jonny5_control::Jonny5System;

namespace
{
const char * kUrdf = R"(<?xml version="1.0"?>
<robot name="jonny5">
  <link name="world"/>
  <ros2_control name="JONNY5" type="system">
    <hardware>
      <plugin>jonny5_control/Jonny5System</plugin>
      <param name="mock_hardware">true</param>
      <param name="transfer_len">128</param>
    </hardware>
    JOINTS
    <sensor name="imu_sensor">
      <state_interface name="orientation.x"/><state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/><state_interface name="orientation.w"/>
    </sensor>
    <gpio name="jonny5_status">
      <state_interface name="fsm_state"/><state_interface name="link_ok"/>
    </gpio>
  </ros2_control>
</robot>)";

std::string urdf()
{
  std::string joints;
  std::string kinematics;
  std::string parent = "world";
  for (const char * j : {"base_joint", "shoulder_joint", "elbow_joint", "wrist_yaw_joint",
      "wrist_pitch_joint", "wrist_roll_joint"})
  {
    const std::string child = std::string(j) + "_link";
    kinematics += "<link name=\"" + child + "\"/><joint name=\"" + j + "\" type=\"revolute\">"
      "<parent link=\"" + parent + "\"/><child link=\"" + child + "\"/><axis xyz=\"0 0 1\"/>"
      "<limit lower=\"-1.0\" upper=\"1.0\" effort=\"1\" velocity=\"2\"/></joint>";
    parent = child;
    joints += std::string("<joint name=\"") + j + "\">"
      "<command_interface name=\"position\"><param name=\"min\">-0.5</param>"
      "<param name=\"max\">0.5</param></command_interface>"
      "<state_interface name=\"position\"/><state_interface name=\"velocity\"/></joint>";
  }
  std::string text = kUrdf;
  text.replace(text.find("JOINTS"), 6, joints);
  text.replace(text.find("<link name=\"world\"/>"), 20, "<link name=\"world\"/>" + kinematics);
  return text;
}

double find(const std::vector<hardware_interface::StateInterface> & v, const std::string & name)
{
  for (const auto & si : v) {
    if (si.get_name() == name) {return si.get_value();}
  }
  return std::nan("");
}
}  // namespace

TEST(Conversions, RoundTrip)
{
  for (double rad : {-0.5, -0.1, 0.0, 0.2, 0.7}) {
    for (int dir : {1, -1}) {
      const auto cdeg = jonny5_control::joint_rad_to_physical_cdeg(rad, 93.0, dir);
      EXPECT_NEAR(jonny5_control::physical_cdeg_to_joint_rad(cdeg, 93.0, dir), rad, 2e-4);
    }
  }
  EXPECT_EQ(jonny5_control::joint_rad_to_physical_cdeg(0.0, 100.0, 1), 10000);
  EXPECT_EQ(jonny5_control::joint_rad_to_physical_cdeg(10.0, 100.0, 1), 18000);  // clamp
}

TEST(Jonny5System, MockLifecycleTracksCommand)
{
  const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf());
  ASSERT_EQ(infos.size(), 1u);
  Jonny5System hw;
  ASSERT_EQ(hw.on_init(infos[0]), hardware_interface::CallbackReturn::SUCCESS);
  rclcpp_lifecycle::State state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "x");
  ASSERT_EQ(hw.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);

  auto states = hw.export_state_interfaces();
  auto commands = hw.export_command_interfaces();
  ASSERT_EQ(states.size(), 6u * 2u + 4u + 2u);
  ASSERT_EQ(commands.size(), 6u);
  EXPECT_NEAR(find(states, "base_joint/position"), 0.0, 1e-6);  // mock starts at HOME
  EXPECT_NEAR(find(states, "imu_sensor/orientation.w"), 1.0, 1e-3);

  ASSERT_EQ(hw.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  const rclcpp::Time t0(0, 0, RCL_STEADY_TIME);
  const rclcpp::Duration dt(0, 10000000);

  // A command far from the current pose must not arm the stream (no jump).
  static_cast<void>(commands[0].set_value(0.3));
  for (int k = 0; k < 30; ++k) {
    hw.read(t0, dt);
    hw.write(t0, dt);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  hw.read(t0, dt);
  EXPECT_NEAR(find(states, "base_joint/position"), 0.0, 1e-3);
  EXPECT_EQ(find(states, "jonny5_status/fsm_state"), 1.0);

  // Back at the current pose the stream arms; then the controller moves the target:
  // 0.3 rad on the base, 0.9 (clamped to 0.5) on the elbow.
  static_cast<void>(commands[0].set_value(0.0));
  hw.read(t0, dt);
  hw.write(t0, dt);
  static_cast<void>(commands[0].set_value(0.3));
  static_cast<void>(commands[2].set_value(0.9));
  for (int k = 0; k < 150; ++k) {
    ASSERT_EQ(hw.read(t0, dt), hardware_interface::return_type::OK);
    ASSERT_EQ(hw.write(t0, dt), hardware_interface::return_type::OK);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  hw.read(t0, dt);
  EXPECT_NEAR(find(states, "base_joint/position"), 0.3, 2e-3);
  EXPECT_NEAR(find(states, "elbow_joint/position"), 0.5, 2e-3);
  EXPECT_NEAR(find(states, "shoulder_joint/position"), 0.0, 2e-3);
  EXPECT_EQ(find(states, "jonny5_status/fsm_state"), 1.0);
  EXPECT_EQ(find(states, "jonny5_status/link_ok"), 1.0);
  EXPECT_EQ(hw.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_cleanup(state), hardware_interface::CallbackReturn::SUCCESS);
}
