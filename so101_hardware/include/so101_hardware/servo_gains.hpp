// Position loop gains of one servo (registers 21..23, u8) from the ros2_control parameters.
#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/hardware_info.hpp"

namespace so101_hardware
{

struct ServoGains
{
  int p = 16;   // LeRobot's SO-100/101 values: the factory P of 32 makes the arm shake
  int d = 32;
  int i = 0;
};

namespace detail
{
inline const std::string * find_param(const std::unordered_map<std::string, std::string> & params, const std::string & key)
{
  const auto it = params.find(key);
  return it == params.end() || it->second.empty() ? nullptr : &it->second;
}

inline int resolve_gain(
  const hardware_interface::HardwareInfo & info, const hardware_interface::ComponentInfo & joint,
  const std::string & key, int fallback)
{
  const std::string * text = find_param(joint.parameters, key);
  if (text == nullptr) {
    text = find_param(info.hardware_parameters, key);
  }
  if (text == nullptr) {
    return fallback;
  }
  int value = -1;
  try {
    size_t used = 0;
    value = std::stoi(*text, &used);
    if (used != text->size()) {
      throw std::invalid_argument(*text);
    }
  } catch (const std::logic_error &) {
    throw std::invalid_argument("joint " + joint.name + ": " + key + " '" + *text + "' is not an integer 0..254");
  }
  if (value < 0 || value > 254) {
    throw std::invalid_argument("joint " + joint.name + ": " + key + " " + *text + " is outside 0..254");
  }
  return value;
}
}  // namespace detail

// Joint <param>, else hardware <param>, else the default. Throws std::invalid_argument naming the
// joint and the parameter if a value is not an integer in 0..254.
inline ServoGains resolve_gains(const hardware_interface::HardwareInfo & info, const hardware_interface::ComponentInfo & joint)
{
  const ServoGains defaults;
  ServoGains gains;
  gains.p = detail::resolve_gain(info, joint, "p_coefficient", defaults.p);
  gains.d = detail::resolve_gain(info, joint, "d_coefficient", defaults.d);
  gains.i = detail::resolve_gain(info, joint, "i_coefficient", defaults.i);
  return gains;
}

}  // namespace so101_hardware
