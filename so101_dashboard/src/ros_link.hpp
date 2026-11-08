// ROS side of the dashboard: subscriptions run on a background executor and write into State,
// which the render loop reads under the mutex.
#pragma once

#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace so101_dashboard
{

double now_s();  // steady clock, seconds since start

// Fixed-size ring of (t, value), for the rolling plots.
class History
{
public:
  History() : History(1200) {}
  explicit History(size_t capacity) : t_(capacity), v_(capacity) {}
  void push(double t, double v);
  // Copies the samples in time order.
  void ordered(std::vector<double> & t, std::vector<double> & v) const;
  size_t size() const { return full_ ? t_.size() : head_; }

private:
  std::vector<double> t_, v_;
  size_t head_ = 0;
  bool full_ = false;
};

// Messages per second over the last second.
class RateMeter
{
public:
  void tick(double t);
  double hz(double t) const;
  double last() const { return stamps_.empty() ? -1.0 : stamps_.back(); }

private:
  mutable std::deque<double> stamps_;
};

struct Joint
{
  double position = NAN;  // rad
  double velocity = NAN;  // rad/s
  History position_hist;
};

struct State
{
  std::mutex mutex;
  std::vector<std::string> joint_order;  // as first seen on /joint_states
  std::map<std::string, Joint> joints;
  RateMeter joint_states_rate;
};

class RosLink : public rclcpp::Node
{
public:
  explicit RosLink(State & state);

private:
  void on_joint_states(const sensor_msgs::msg::JointState & msg);

  State & state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
};

}  // namespace so101_dashboard
