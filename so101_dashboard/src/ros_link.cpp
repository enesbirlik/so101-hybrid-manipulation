#include "ros_link.hpp"

#include <algorithm>
#include <chrono>

namespace so101_dashboard
{

double now_s()
{
  static const auto start = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

void History::push(double t, double v)
{
  t_[head_] = t;
  v_[head_] = v;
  if (++head_ == t_.size()) {
    head_ = 0;
    full_ = true;
  }
}

void History::ordered(std::vector<double> & t, std::vector<double> & v) const
{
  t.clear();
  v.clear();
  if (full_) {
    t.insert(t.end(), t_.begin() + head_, t_.end());
    v.insert(v.end(), v_.begin() + head_, v_.end());
  }
  t.insert(t.end(), t_.begin(), t_.begin() + head_);
  v.insert(v.end(), v_.begin(), v_.begin() + head_);
}

void RateMeter::tick(double t)
{
  stamps_.push_back(t);
  while (!stamps_.empty() && stamps_.front() < t - 1.0) {
    stamps_.pop_front();
  }
}

double RateMeter::hz(double t) const
{
  while (!stamps_.empty() && stamps_.front() < t - 1.0) {
    stamps_.pop_front();
  }
  return static_cast<double>(stamps_.size());
}

RosLink::RosLink(State & state)
: Node("so101_dashboard"), state_(state)
{
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState & msg) {on_joint_states(msg);});
}

void RosLink::on_joint_states(const sensor_msgs::msg::JointState & msg)
{
  const double t = now_s();
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.joint_states_rate.tick(t);
  for (size_t i = 0; i < msg.name.size(); ++i) {
    const auto & name = msg.name[i];
    auto it = state_.joints.find(name);
    if (it == state_.joints.end()) {
      it = state_.joints.emplace(name, Joint{}).first;
      state_.joint_order.push_back(name);
      // /joint_states lists them alphabetically; show them base to gripper
      static const std::vector<std::string> arm_order{
        "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
      auto rank = [](const std::string & n) {
          return std::find(arm_order.begin(), arm_order.end(), n) - arm_order.begin();
        };
      std::stable_sort(state_.joint_order.begin(), state_.joint_order.end(),
        [&](const std::string & a, const std::string & b) {return rank(a) < rank(b);});
    }
    Joint & j = it->second;
    if (i < msg.position.size()) {
      j.position = msg.position[i];
      j.position_hist.push(t, msg.position[i] * 180.0 / M_PI);
    }
    if (i < msg.velocity.size()) {
      j.velocity = msg.velocity[i];
    }
  }
}

}  // namespace so101_dashboard
