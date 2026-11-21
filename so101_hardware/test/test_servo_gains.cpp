#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/component_parser.hpp"
#include "so101_hardware/servo_gains.hpp"

using hardware_interface::ComponentInfo;
using hardware_interface::HardwareInfo;
using so101_hardware::resolve_gains;

namespace
{
ComponentInfo joint(const std::string & name)
{
  ComponentInfo j;
  j.name = name;
  return j;
}
}  // namespace

TEST(ServoGains, Defaults)
{
  const auto g = resolve_gains(HardwareInfo{}, joint("shoulder_pan"));
  EXPECT_EQ(g.p, 16);
  EXPECT_EQ(g.d, 32);
  EXPECT_EQ(g.i, 0);
}

TEST(ServoGains, HardwareLevelOverride)
{
  HardwareInfo info;
  info.hardware_parameters["p_coefficient"] = "24";
  info.hardware_parameters["i_coefficient"] = "254";
  const auto g = resolve_gains(info, joint("elbow_flex"));
  EXPECT_EQ(g.p, 24);
  EXPECT_EQ(g.d, 32);
  EXPECT_EQ(g.i, 254);
}

TEST(ServoGains, JointLevelBeatsHardwareLevel)
{
  HardwareInfo info;
  info.hardware_parameters["p_coefficient"] = "24";
  info.hardware_parameters["d_coefficient"] = "40";
  auto pan = joint("shoulder_pan");
  pan.parameters["p_coefficient"] = "0";
  const auto g = resolve_gains(info, pan);
  EXPECT_EQ(g.p, 0);
  EXPECT_EQ(g.d, 40);
  EXPECT_EQ(g.i, 0);
  // another joint without its own value still gets the hardware-level one
  EXPECT_EQ(resolve_gains(info, joint("wrist_flex")).p, 24);
}

TEST(ServoGains, EmptyValueIsUnset)
{
  HardwareInfo info;
  info.hardware_parameters["p_coefficient"] = "24";
  auto j = joint("gripper");
  j.parameters["p_coefficient"] = "";
  EXPECT_EQ(resolve_gains(info, j).p, 24);
}

TEST(ServoGains, InvalidValuesRejected)
{
  for (const std::string bad : {"abc", "-1", "255", "1000", "16.5", "16x", " "}) {
    auto j = joint("shoulder_lift");
    j.parameters["d_coefficient"] = bad;
    try {
      resolve_gains(HardwareInfo{}, j);
      ADD_FAILURE() << "accepted '" << bad << "'";
    } catch (const std::invalid_argument & e) {
      const std::string msg = e.what();
      EXPECT_NE(msg.find("shoulder_lift"), std::string::npos) << msg;
      EXPECT_NE(msg.find("d_coefficient"), std::string::npos) << msg;
    }
  }
  HardwareInfo info;
  info.hardware_parameters["i_coefficient"] = "300";
  EXPECT_THROW(resolve_gains(info, joint("gripper")), std::invalid_argument);
}

// The <param> a joint carries in the ros2_control block reaches ComponentInfo::parameters.
TEST(ServoGains, FromParsedUrdf)
{
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="r">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <joint name="shoulder_pan" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="wrist_flex" type="revolute">
    <parent link="l1"/><child link="l2"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <ros2_control name="arm" type="system">
    <hardware><plugin>so101_hardware/SO101Hardware</plugin><param name="p_coefficient">20</param></hardware>
    <joint name="shoulder_pan">
      <param name="p_coefficient">32</param>
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
    <joint name="wrist_flex">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>)";
  const auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  ASSERT_EQ(infos.size(), 1u);
  ASSERT_EQ(infos[0].joints.size(), 2u);
  EXPECT_EQ(resolve_gains(infos[0], infos[0].joints[0]).p, 32);
  EXPECT_EQ(resolve_gains(infos[0], infos[0].joints[1]).p, 20);
  EXPECT_EQ(resolve_gains(infos[0], infos[0].joints[1]).d, 32);
}
