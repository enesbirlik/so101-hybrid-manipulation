// ros2_control SystemInterface for the six ST3215 servos of the SO-101.
//
// The single owner of the serial bus while it runs. One SYNC READ (position, speed, load,
// voltage, temperature and current of all servos) and one SYNC WRITE (goal positions) per
// control period. Ticks <-> radians use
// so101_bringup/config/so101_calibration.yaml:
//   joint_rad = direction * (ticks - zero) * 2*pi / 4096
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "so101_hardware/feetech_bus.hpp"

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
  double load = 0.0;         // % of the torque limit, signed like the joint
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

  std::vector<JointCalibration> joints_;
  std::vector<uint8_t> ids_;
  std::unique_ptr<FeetechBus> bus_;
  std::string port_;
  int torque_limit_ = 1000;   // 0.1 %
  int acceleration_ = 0;      // 100 step/s^2, 0 = servo default (no limit)
  int goal_speed_ = 0;        // step/s, 0 = no limit
  int p_coefficient_ = 16;    // position loop gains, written to RAM on activation
  int d_coefficient_ = 32;
  int i_coefficient_ = 0;
  int max_failures_ = 5;      // consecutive bus failures before read() reports an error
  int failures_ = 0;
  bool telemetry_ = false;    // the URDF declares load/current/temperature/voltage state interfaces
};

}  // namespace so101_hardware
