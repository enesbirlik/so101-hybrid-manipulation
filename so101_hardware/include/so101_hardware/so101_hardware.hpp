// ros2_control SystemInterface for the six ST3215 servos of the SO-101.
//
// The single owner of the serial bus while it runs. One SYNC READ (position, speed, load,
// voltage, temperature and current of all servos) and one SYNC WRITE (goal positions) per
// control period, plus a SYNC WRITE of the torque limits in the cycles where a runtime
// torque_limit command changes them. Ticks <-> radians use
// so101_bringup/config/so101_calibration.yaml:
//   joint_rad = direction * (ticks - zero) * 2*pi / 4096
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "so101_hardware/feetech_bus.hpp"
#include "so101_hardware/servo_gains.hpp"
#include "so101_hardware/torque_limit.hpp"

namespace so101_hardware
{

struct JointCalibration
{
  std::string name;
  uint8_t id = 0;
  int homing_offset = 0;
  int range_min = 0;
  int range_max = 0;
  int zero = 0;
  int direction = 1;
};

// One servo's present state, as read from the bus.
struct ServoReading
{
  int ticks = 0;
  int speed = 0;             // step/s
  double load = 0.0;         // %, register 60 in 0.1 % steps, signed like the joint
  double voltage = 0.0;      // V
  double temperature = 0.0;  // deg C
  double current = 0.0;      // A
};

class SO101Hardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double to_rad(const JointCalibration & c, int ticks) const;
  int to_ticks(const JointCalibration & c, double rad) const;
  bool read_bus(std::vector<ServoReading> & servos);
  void set_torque(bool on);
  void write_torque_limits();

  std::vector<JointCalibration> joints_;
  std::vector<uint8_t> ids_;
  std::unique_ptr<FeetechBus> bus_;
  std::string port_;
  // Torque limit (0.1 %) per joint: the parameter, written on activation, is also the ceiling
  // for the optional runtime `torque_limit` command interface.
  std::vector<int> torque_ceiling_;
  std::vector<int> torque_applied_;   // last value written to the servo
  std::vector<bool> torque_command_;  // the joint declares a torque_limit command interface
  std::vector<bool> torque_state_;    // ... and a torque_limit state interface
  int acceleration_ = 0;      // 100 step/s^2, 0 = servo default (no limit)
  int goal_speed_ = 0;        // step/s, 0 = no limit
  std::vector<ServoGains> gains_;   // per joint, written to RAM on activation
  int max_failures_ = 5;      // consecutive bus failures before read() reports an error
  int failures_ = 0;
  bool telemetry_ = false;    // the URDF declares load/current/temperature/voltage state interfaces
  bool torque_gpio_ = false;  // the URDF declares the torque/enable GPIO
  bool torque_on_ = false;
  // After the torque comes back on, hold the pose it came on at until the position commands are
  // back near it (the controllers restart and command the present pose): a stale command from
  // before the torque went off would make the arm jump.
  bool resync_ = false;
  std::vector<int> hold_ticks_;
  std::vector<int> present_ticks_;   // from the last read
};

}  // namespace so101_hardware
