#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "so101_hardware/feetech_bus.hpp"
#include "so101_hardware/torque_limit.hpp"

using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using so101_hardware::resolve_torque_limit;
using so101_hardware::torque_limit_bytes;
using so101_hardware::torque_limit_from_command;

namespace
{
ComponentInfo joint(const std::string & name)
{
  ComponentInfo j;
  j.name = name;
  return j;
}
}  // namespace

TEST(TorqueLimit, RegisterAddress)
{
  // STS3215 RAM "Torque Limit", u16 little-endian, 0.1 % units (same as tools/feetech_bus.py)
  EXPECT_EQ(so101_hardware::TORQUE_LIMIT, 48);
}

TEST(TorqueLimit, DefaultIsFullTorque)
{
  EXPECT_EQ(resolve_torque_limit(HardwareInfo{}, joint("gripper")), 1000);
}

TEST(TorqueLimit, JointBeatsHardwareBeatsDefault)
{
  HardwareInfo info;
  info.hardware_parameters["torque_limit"] = "900";
  auto gripper = joint("gripper");
  gripper.parameters["torque_limit"] = "250";
  EXPECT_EQ(resolve_torque_limit(info, gripper), 250);
  EXPECT_EQ(resolve_torque_limit(info, joint("elbow_flex")), 900);
  auto empty = joint("wrist_roll");
  empty.parameters["torque_limit"] = "";
  EXPECT_EQ(resolve_torque_limit(info, empty), 900);
}

TEST(TorqueLimit, BoundsAccepted)
{
  for (const std::string ok : {"0", "1", "1000"}) {
    auto j = joint("gripper");
    j.parameters["torque_limit"] = ok;
    EXPECT_EQ(resolve_torque_limit(HardwareInfo{}, j), std::stoi(ok));
  }
}

TEST(TorqueLimit, InvalidValuesRejected)
{
  for (const std::string bad : {"-1", "1001", "65535", "50%", "30.5", "abc", " "}) {
    auto j = joint("gripper");
    j.parameters["torque_limit"] = bad;
    try {
      resolve_torque_limit(HardwareInfo{}, j);
      ADD_FAILURE() << "accepted '" << bad << "'";
    } catch (const std::invalid_argument & e) {
      const std::string msg = e.what();
      EXPECT_NE(msg.find("gripper"), std::string::npos) << msg;
      EXPECT_NE(msg.find("torque_limit"), std::string::npos) << msg;
      EXPECT_NE(msg.find("0..1000"), std::string::npos) << msg;
    }
  }
  HardwareInfo info;
  info.hardware_parameters["torque_limit"] = "2000";
  EXPECT_THROW(resolve_torque_limit(info, joint("gripper")), std::invalid_argument);
}

TEST(TorqueLimit, CommandClampedToCeiling)
{
  EXPECT_EQ(torque_limit_from_command(200.0, 1000), 200);
  EXPECT_EQ(torque_limit_from_command(200.4, 1000), 200);
  EXPECT_EQ(torque_limit_from_command(200.6, 1000), 201);
  EXPECT_EQ(torque_limit_from_command(-5.0, 1000), 0);
  EXPECT_EQ(torque_limit_from_command(5000.0, 1000), 1000);
  // the parameter is the ceiling: a runtime command can only lower the limit
  EXPECT_EQ(torque_limit_from_command(800.0, 300), 300);
  EXPECT_EQ(torque_limit_from_command(100.0, 300), 100);
  // a bad ceiling never lets the register go past 1000
  EXPECT_EQ(torque_limit_from_command(1500.0, 4000), 1000);
}

TEST(TorqueLimit, NonFiniteCommandKeepsTheApplied)
{
  EXPECT_FALSE(torque_limit_from_command(std::numeric_limits<double>::quiet_NaN(), 1000).has_value());
  EXPECT_FALSE(torque_limit_from_command(std::numeric_limits<double>::infinity(), 1000).has_value());
  EXPECT_FALSE(torque_limit_from_command(-std::numeric_limits<double>::infinity(), 1000).has_value());
}

TEST(TorqueLimit, RegisterEncoding)
{
  EXPECT_EQ(torque_limit_bytes(0), (std::vector<uint8_t>{0x00, 0x00}));
  EXPECT_EQ(torque_limit_bytes(250), (std::vector<uint8_t>{0xFA, 0x00}));
  EXPECT_EQ(torque_limit_bytes(300), (std::vector<uint8_t>{0x2C, 0x01}));
  EXPECT_EQ(torque_limit_bytes(1000), (std::vector<uint8_t>{0xE8, 0x03}));
  // clamped, never wraps into a large u16
  EXPECT_EQ(torque_limit_bytes(-1), (std::vector<uint8_t>{0x00, 0x00}));
  EXPECT_EQ(torque_limit_bytes(70000), (std::vector<uint8_t>{0xE8, 0x03}));
}

// The joint <param> and the torque_limit command/state interfaces survive URDF parsing.
TEST(TorqueLimit, FromParsedUrdf)
{
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="r">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="wrist_roll" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="gripper" type="revolute">
    <parent link="l1"/><child link="l2"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <ros2_control name="arm" type="system">
    <hardware><plugin>so101_hardware/SO101Hardware</plugin><param name="torque_limit">950</param></hardware>
    <joint name="wrist_roll">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="gripper">
      <param name="torque_limit">400</param>
      <command_interface name="position"/>
      <command_interface name="torque_limit"/>
      <state_interface name="position"/>
      <state_interface name="torque_limit"/>
    </joint>
  </ros2_control>
</robot>)";
  const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(infos.size(), 1u);
  ASSERT_EQ(infos[0].joints.size(), 2u);
  EXPECT_EQ(resolve_torque_limit(infos[0], infos[0].joints[0]), 950);
  EXPECT_EQ(resolve_torque_limit(infos[0], infos[0].joints[1]), 400);
  ASSERT_EQ(infos[0].joints[1].command_interfaces.size(), 2u);
  EXPECT_EQ(infos[0].joints[1].command_interfaces[1].name, "torque_limit");
}
