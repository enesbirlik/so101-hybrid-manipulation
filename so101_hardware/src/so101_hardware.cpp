#include "so101_hardware/so101_hardware.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware
{

using CallbackReturn = hardware_interface::SystemInterface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("SO101Hardware");

std::string param(const hardware_interface::HardwareInfo & info, const std::string & key, const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return it == info.hardware_parameters.end() || it->second.empty() ? fallback : it->second;
}

int homing_offset_from_register(uint16_t raw)
{
  return decode_sm(raw, 11);
}
}  // namespace

CallbackReturn SO101Hardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & info = get_hardware_info();
  port_ = param(info, "port", "");
  torque_limit_ = std::stoi(param(info, "torque_limit", "1000"));
  acceleration_ = std::stoi(param(info, "acceleration", "0"));
  goal_speed_ = std::stoi(param(info, "goal_speed", "0"));
  // LeRobot's SO-100/101 gains: the factory P of 32 makes the arm shake.
  p_coefficient_ = std::stoi(param(info, "p_coefficient", "16"));
  d_coefficient_ = std::stoi(param(info, "d_coefficient", "32"));
  i_coefficient_ = std::stoi(param(info, "i_coefficient", "0"));
  max_failures_ = std::stoi(param(info, "max_bus_failures", "5"));
  const std::string calibration_file = param(
    info, "calibration_file",
    ament_index_cpp::get_package_share_directory("so101_bringup") + "/config/so101_calibration.yaml");

  YAML::Node cal;
  try {
    cal = YAML::LoadFile(calibration_file)["joints"];
  } catch (const YAML::Exception & e) {
    RCLCPP_FATAL(LOGGER, "cannot read %s: %s", calibration_file.c_str(), e.what());
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : info.joints) {
    const auto c = cal[joint.name];
    if (!c) {
      RCLCPP_FATAL(LOGGER, "joint %s is not calibrated in %s", joint.name.c_str(), calibration_file.c_str());
      return CallbackReturn::ERROR;
    }
    JointCalibration j;
    j.name = joint.name;
    j.id = static_cast<uint8_t>(c["id"].as<int>());
    j.homing_offset = c["homing_offset"].as<int>();
    j.range_min = c["range_min"].as<int>();
    j.range_max = c["range_max"].as<int>();
    j.zero = c["zero"].as<int>();
    j.direction = c["direction"].as<int>();
    joints_.push_back(j);
    ids_.push_back(j.id);
  }
  RCLCPP_INFO(LOGGER, "%zu joints from %s", joints_.size(), calibration_file.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101Hardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    const std::string port = port_.empty() ? FeetechBus::find_port() : port_;
    bus_ = std::make_unique<FeetechBus>(port);
    RCLCPP_INFO(LOGGER, "bus open: %s", port.c_str());
    // Same startup check as `so101_calibrate.py check`: every servo answers and still has the
    // homing offset the calibration was made with (a swapped servo or lost EEPROM would not).
    for (const auto & j : joints_) {
      if (!bus_->ping(j.id)) {
        RCLCPP_FATAL(LOGGER, "%s (id %d) does not answer", j.name.c_str(), j.id);
        bus_.reset();
        return CallbackReturn::ERROR;
      }
      const int offset = homing_offset_from_register(bus_->read_u16(j.id, HOMING_OFFSET));
      if (offset != j.homing_offset) {
        RCLCPP_FATAL(LOGGER, "%s (id %d): homing offset %d, calibration says %d: recalibrate",
                     j.name.c_str(), j.id, offset, j.homing_offset);
        bus_.reset();
        return CallbackReturn::ERROR;
      }
    }
  } catch (const FeetechError & e) {
    RCLCPP_FATAL(LOGGER, "%s", e.what());
    bus_.reset();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101Hardware::on_activate(const rclcpp_lifecycle::State &)
{
  std::vector<int> ticks, speed;
  if (!read_bus(ticks, speed)) {
    RCLCPP_FATAL(LOGGER, "cannot read the joints");
    return CallbackReturn::ERROR;
  }
  try {
    // Hold where the arm is: goal = present position before the torque comes on, and the
    // commands start at the present position, so nothing jumps.
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto & j = joints_[i];
      const double q = to_rad(j, ticks[i]);
      set_state(j.name + "/position", q);
      set_state(j.name + "/velocity", 0.0);
      set_command(j.name + "/position", q);
      // RAM only (the EEPROM lock stays on), so the factory gains return after a power cycle.
      bus_->write_u8(j.id, P_COEFFICIENT, static_cast<uint8_t>(p_coefficient_));
      bus_->write_u8(j.id, D_COEFFICIENT, static_cast<uint8_t>(d_coefficient_));
      bus_->write_u8(j.id, I_COEFFICIENT, static_cast<uint8_t>(i_coefficient_));
      bus_->write_u8(j.id, ACCELERATION, static_cast<uint8_t>(acceleration_));
      bus_->write_u16(j.id, GOAL_SPEED, static_cast<uint16_t>(goal_speed_));
      bus_->write_u16(j.id, GOAL_POSITION, static_cast<uint16_t>(ticks[i]));
      bus_->write_u8(j.id, TORQUE_ENABLE, 1);
      bus_->write_u16(j.id, TORQUE_LIMIT, static_cast<uint16_t>(torque_limit_));
    }
  } catch (const FeetechError & e) {
    RCLCPP_FATAL(LOGGER, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  failures_ = 0;
  RCLCPP_INFO(LOGGER, "torque on, holding the present pose (P %d, D %d, I %d)",
              p_coefficient_, d_coefficient_, i_coefficient_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101Hardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Torque stays on: switching it off here would drop the arm. Release it by hand with
  // `python3 tools/so101_calibrate.py off` while supporting the arm.
  RCLCPP_WARN(LOGGER, "deactivated, torque stays on (release: tools/so101_calibrate.py off, support the arm)");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101Hardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  bus_.reset();
  return CallbackReturn::SUCCESS;
}

double SO101Hardware::to_rad(const JointCalibration & c, int ticks) const
{
  return c.direction * (ticks - c.zero) * 2.0 * M_PI / TICKS_PER_TURN;
}

int SO101Hardware::to_ticks(const JointCalibration & c, double rad) const
{
  return c.zero + static_cast<int>(std::lround(c.direction * rad * TICKS_PER_TURN / (2.0 * M_PI)));
}

bool SO101Hardware::read_bus(std::vector<int> & ticks, std::vector<int> & speed)
{
  try {
    const auto data = bus_->sync_read(ids_, PRESENT_POSITION, 4);
    ticks.clear();
    speed.clear();
    for (const auto & d : data) {
      ticks.push_back(decode_sm(static_cast<uint16_t>(d[0] | (d[1] << 8)), 15));
      speed.push_back(decode_sm(static_cast<uint16_t>(d[2] | (d[3] << 8)), 15));
    }
    return true;
  } catch (const FeetechError & e) {
    RCLCPP_WARN_THROTTLE(LOGGER, *rclcpp::Clock::make_shared(), 1000, "bus read: %s", e.what());
    return false;
  }
}

return_type SO101Hardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<int> ticks, speed;
  if (!read_bus(ticks, speed)) {
    // A missed reply now and then is normal on this bus; keep the last state.
    if (++failures_ >= max_failures_) {
      RCLCPP_FATAL(LOGGER, "%d bus reads failed in a row", failures_);
      return return_type::ERROR;
    }
    return return_type::OK;
  }
  failures_ = 0;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    set_state(j.name + "/position", to_rad(j, ticks[i]));
    set_state(j.name + "/velocity", j.direction * speed[i] * 2.0 * M_PI / TICKS_PER_TURN);
  }
  return return_type::OK;
}

return_type SO101Hardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::vector<std::vector<uint8_t>> goals;
  for (const auto & j : joints_) {
    const double cmd = get_command<double>(j.name + "/position");
    if (!std::isfinite(cmd)) {
      return return_type::OK;  // no command yet
    }
    // Never past the calibrated range (the servos' own position limits as well).
    const int t = std::clamp(to_ticks(j, cmd), j.range_min, j.range_max);
    goals.push_back({static_cast<uint8_t>(t & 0xFF), static_cast<uint8_t>(t >> 8)});
  }
  try {
    bus_->sync_write(ids_, GOAL_POSITION, goals);
  } catch (const FeetechError & e) {
    RCLCPP_WARN_THROTTLE(LOGGER, *rclcpp::Clock::make_shared(), 1000, "bus write: %s", e.what());
  }
  return return_type::OK;
}

}  // namespace so101_hardware

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101Hardware, hardware_interface::SystemInterface)
