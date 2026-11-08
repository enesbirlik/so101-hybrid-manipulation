// ROS side of the dashboard: subscriptions run on a background executor and write into State,
// which the render loop reads under the mutex.
#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
  // servo telemetry from /dynamic_joint_states (NAN without it, e.g. on mock hardware)
  double load = NAN;         // % of the torque limit
  double current = NAN;      // A
  double temperature = NAN;  // deg C
  double voltage = NAN;      // V
  History position_hist;     // deg
  History load_hist;         // %
  History current_hist;      // mA
  History temperature_hist;  // deg C
};

// Latest decoded camera frame, RGB8.
struct CameraFrame
{
  std::vector<uint8_t> rgb;
  int width = 0, height = 0;
  uint64_t version = 0;
};

// A LINE_LIST marker (the camera field of view): point pairs in `frame`.
struct LineMarker
{
  std::string frame;
  std::vector<std::array<float, 3>> points;
  double received = -1.0;
};

// The plan move_group published last (/display_planned_path), for the ghost robot.
struct Ghost
{
  std::vector<std::string> joints;
  std::vector<double> times;                   // s from the start
  std::vector<std::vector<double>> positions;  // per point, in `joints` order
  double received = -1.0;
};

struct TaskStatus
{
  bool running = false;
  std::string label;
  std::string phase;     // move_group feedback ("PLANNING", "MONITOR", ...) or "running"
  double started = 0.0;
};

struct TaskLogEntry
{
  std::string clock;     // wall time it started, HH:MM:SS
  std::string label;
  std::string result;
  double duration = 0.0;
  bool ok = false;
};

struct State
{
  std::mutex mutex;
  std::vector<std::string> joint_order;  // base to gripper
  std::map<std::string, Joint> joints;
  RateMeter joint_states_rate;
  std::string robot_description;
  uint64_t description_version = 0;   // bumped on every new description
  tf2_ros::Buffer * tf = nullptr;      // thread-safe on its own, no mutex needed

  std::vector<std::string> image_topics;   // every sensor_msgs/CompressedImage topic
  std::string image_topic;                 // the one to show, chosen in the camera panel
  CameraFrame frame;
  RateMeter image_rate;
  LineMarker fov;

  TaskStatus task;
  std::deque<TaskLogEntry> task_log;   // newest first
  Ghost ghost;
};

class RosLink : public rclcpp::Node
{
public:
  explicit RosLink(State & state);

private:
  void on_joint_states(const sensor_msgs::msg::JointState & msg);
  void on_dynamic_joint_states(const control_msgs::msg::DynamicJointState & msg);
  void on_image(const sensor_msgs::msg::CompressedImage & msg);
  // Lists image topics and follows the camera panel's choice.
  void refresh_image_topics();

  State & state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  std::string subscribed_image_topic_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr fov_sub_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::CallbackGroup::SharedPtr image_group_;   // decoding must not hold up the joint states
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace so101_dashboard
