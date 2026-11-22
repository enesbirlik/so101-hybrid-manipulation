// Torque limit of one servo (RAM register 48, u16, 0.1 % units: 0..1000 = 0..100 % of full output).
//
// Two sources:
//   - the `torque_limit` parameter (joint, else hardware, else 1000), written on activation;
//   - the optional `torque_limit` joint command interface, which can only lower it at runtime
//     (e.g. while the gripper squeezes a paper cube). The parameter is the ceiling.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "so101_hardware/servo_gains.hpp"

namespace so101_hardware
{

constexpr int TORQUE_LIMIT_MAX = 1000;   // 100 %

// Joint <param name="torque_limit">, else the hardware one, else 1000 (full output, what the
// servo also starts with at power up). Throws std::invalid_argument outside 0..1000.
inline int resolve_torque_limit(
  const hardware_interface::HardwareInfo & info, const hardware_interface::ComponentInfo & joint)
{
  return detail::resolve_int(info, joint, "torque_limit", TORQUE_LIMIT_MAX, 0, TORQUE_LIMIT_MAX);
}

// The register value for a runtime command: rounded and clamped to 0..ceiling. No value (keep
// what is applied) for NaN / inf, which is also what the command interface holds before any
// controller writes it.
inline std::optional<int> torque_limit_from_command(double command, int ceiling)
{
  if (!std::isfinite(command)) {
    return std::nullopt;
  }
  const int hi = std::clamp(ceiling, 0, TORQUE_LIMIT_MAX);
  return static_cast<int>(std::lround(std::clamp(command, 0.0, static_cast<double>(hi))));
}

// Little-endian u16, as written to address 48 (no sign bit: the value is never negative).
inline std::vector<uint8_t> torque_limit_bytes(int value)
{
  const auto v = static_cast<uint16_t>(std::clamp(value, 0, TORQUE_LIMIT_MAX));
  return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>(v >> 8)};
}

}  // namespace so101_hardware
