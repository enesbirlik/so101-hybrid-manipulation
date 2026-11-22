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
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace so101_dashboard
{

double now_s();  // steady clock, seconds since start

// "cube_3", "3", "K3" -> 3; -1 for anything else.
int cube_index(const std::string & id);

// Fixed-size ring of (t, value), for the rolling plots.
class History
{
public:
  History() : History(6000) {}   // 60 s at 100 Hz
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

// A detected cube (so101_perception's cube detector, /cubes/markers): a box in `frame`.
struct CubeMarker
{
  std::string frame;
  std::array<float, 3> position{};
  float yaw = 0.0f;                 // rad about z; the detector only reports yaw
  float edge = 0.03f;               // m
  std::array<float, 4> rgba{};      // alpha < 0.5: remembered, out of view
  std::string label;
};

// The plan move_group published last (/display_planned_path), for the ghost robot.
struct Ghost
{
  std::vector<std::string> joints;
  std::vector<double> times;                   // s from the start
  std::vector<std::vector<double>> positions;  // per point, in `joints` order
  double received = -1.0;
};

// A cube from /cubes/tracked (the detector's filter, or the FAKE cube on mock), in base_link.
struct TrackedCube
{
  std::array<float, 3> position{};
  float yaw = 0.0f;       // rad, folded into [-45, 45) deg
  float edge = 0.03f;     // m
  float score = 0.0f;     // >= 0.5: stable
  double received = -1.0;
};

// The pick node's own state topic (/cube_pick_place/state, transient-local JSON), when it has one.
struct PickNodeState
{
  double received = -1.0;   // -1: never heard (older pick node): the dashboard keeps its own record
  bool busy = false;
  std::string held;         // "cube_<k>", empty when nothing is held
};

// What the dashboard believes is between the jaws: from the PickCube / PlaceCube results, or from
// the pick node's state topic when there is one.
struct Holding
{
  std::string cube;         // "cube_<k>", empty: nothing held
  bool uncertain = false;   // e.g. a pick stopped during the lift
  std::string source;       // "pick result", "pick node", ...
};

// Cube selection and place target of the 3D view. GUI thread only.
struct CubeSelection
{
  int cube = -1;            // selected cube index, -1: none
  int stack_on = -1;        // while holding: the cube clicked as the place target, -1: none
  bool has_target = false;  // while holding: a table point was clicked
  float x = 0.0f, y = 0.0f; // that point, base_link [m]
  float yaw_deg = 0.0f;     // yaw of the placed cube
  bool any_yaw = false;     // NaN to the action: the smallest wrist roll change
  std::string message;      // why the last click was refused
  double message_at = -1.0;
};

struct TaskStatus
{
  bool running = false;
  std::string label;
  std::string phase;     // move_group feedback ("PLANNING", "MONITOR", ...), pick/place phase or "running"
  std::string detail;    // pick/place feedback detail
  double started = 0.0;
  std::vector<std::pair<std::string, double>> phases;   // pick/place: each phase and when it began
};

struct TaskLogEntry
{
  std::string clock;     // wall time it started, HH:MM:SS
  std::string label;
  std::string result;
  double duration = 0.0;
  bool ok = false;
  std::string details;   // tooltip: the phases a pick/place went through
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
  bool camera_shown = true;                // panel closed: no subscription, no decoding
  CameraFrame frame;
  RateMeter image_rate;
  LineMarker fov;
  std::map<int, CubeMarker> cubes;     // by cube index
  std::map<int, TrackedCube> tracked;  // by cube index, the last /cubes/tracked
  PickNodeState pick_node;
  Holding holding;
  CubeSelection selection;
  std::deque<std::string> test_commands;   // SO101_DASHBOARD_TEST=1: from /so101_dashboard/test
  std::string screenshot;                  // file name of a requested screenshot (test hook)

  TaskStatus task;
  std::deque<TaskLogEntry> task_log;   // newest first
  double torque = NAN;                 // torque/enable state: 1 on, 0 off, NAN unknown
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
  void on_tracked(const vision_msgs::msg::Detection3DArray & msg);
  void on_pick_state(const std_msgs::msg::String & msg);
  // Lists image topics and follows the camera panel's choice.
  void refresh_image_topics();

  State & state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  std::string subscribed_image_topic_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr fov_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cubes_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr tracked_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pick_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_sub_;
  rclcpp::TimerBase::SharedPtr topics_timer_;
  rclcpp::CallbackGroup::SharedPtr image_group_;   // decoding must not hold up the joint states
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace so101_dashboard
