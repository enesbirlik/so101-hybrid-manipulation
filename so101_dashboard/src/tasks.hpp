// Tasks: start named MoveIt goals or helper scripts, follow them, stop everything.
// One task at a time. The ROS callbacks run on the executor thread; start/stop are called from the GUI.
#pragma once

#include <sys/types.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ros_link.hpp"

namespace so101_dashboard
{

struct MotionOptions
{
  std::string pipeline = "pilz_industrial_motion_planner";
  std::string planner = "PTP";
  double velocity = 0.8;       // scaling factors, as in the saved RViz layout
  double acceleration = 0.5;
};

class TaskRunner
{
public:
  TaskRunner(rclcpp::Node::SharedPtr node, State & state);
  ~TaskRunner();

  // A named state of the SRDF ("arm"/"zero", "gripper"/"open", ...), planned and executed by move_group.
  bool start_named(const std::string & label, const std::string & group, const std::string & name,
    const MotionOptions & options);
  // A helper script (argv[0] = absolute path); its exit code is the result.
  bool start_process(const std::string & label, const std::vector<std::string> & argv);
  // Cancels the MoveIt goal, every trajectory controller goal and the script.
  void stop();
  // All servos' torque. Off stops any motion first. On restarts the trajectory controllers, so
  // they command the present pose (the hardware holds that pose until they do).
  void set_torque(bool on);

  bool move_group_online() const;
  std::vector<std::string> named_states(const std::string & group) const;

private:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  void load_srdf();
  void begin(const std::string & label);                       // under mutex_
  void finish(bool ok, const std::string & result);            // takes state_.mutex
  void poll_process();
  void restart_controllers();
  void log(const std::string & label, const std::string & result, bool ok);   // takes state_.mutex

  rclcpp::Node::SharedPtr node_;
  State & state_;
  rclcpp_action::Client<MoveGroup>::SharedPtr move_group_;
  std::vector<rclcpp_action::Client<FollowJointTrajectory>::SharedPtr> controllers_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr planned_path_sub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_;
  rclcpp::TimerBase::SharedPtr restart_timer_;

  mutable std::mutex mutex_;
  std::map<std::string, std::map<std::string, std::map<std::string, double>>> named_;  // group -> name -> joints
  rclcpp_action::ClientGoalHandle<MoveGroup>::SharedPtr goal_;
  pid_t child_ = -1;
};

}  // namespace so101_dashboard
