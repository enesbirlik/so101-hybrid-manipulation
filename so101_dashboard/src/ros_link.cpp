#include "ros_link.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace so101_dashboard
{

double now_s()
{
  static const auto start = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

int cube_index(const std::string & id)
{
  std::string s = id;
  if (s.rfind("cube_", 0) == 0) {
    s = s.substr(5);
  } else if (!s.empty() && (s[0] == 'K' || s[0] == 'k')) {
    s = s.substr(1);
  }
  if (s.empty() || s.size() > 3 || !std::all_of(s.begin(), s.end(), ::isdigit)) {
    return -1;
  }
  return std::stoi(s);
}

namespace
{

// The raw value of `"key":` in a flat or nested JSON text: a string without its quotes, an object
// with its braces, or a literal (true, null, 3). Empty when the key is missing. Enough for the pick
// node's small state message; not a JSON parser.
std::string json_value(const std::string & text, const std::string & key, bool * is_string = nullptr)
{
  const size_t k = text.find("\"" + key + "\"");
  if (k == std::string::npos) {
    return "";
  }
  size_t i = text.find(':', k + key.size() + 2);
  if (i == std::string::npos) {
    return "";
  }
  i = text.find_first_not_of(" \t\r\n", i + 1);
  if (i == std::string::npos) {
    return "";
  }
  if (is_string) {
    *is_string = text[i] == '"';
  }
  if (text[i] == '"') {
    const size_t end = text.find('"', i + 1);
    return end == std::string::npos ? "" : text.substr(i + 1, end - i - 1);
  }
  if (text[i] == '{') {
    int depth = 0;
    for (size_t j = i; j < text.size(); ++j) {
      depth += text[j] == '{' ? 1 : text[j] == '}' ? -1 : 0;
      if (depth == 0) {
        return text.substr(i, j - i + 1);
      }
    }
    return "";
  }
  const size_t end = text.find_first_of(",}] \t\r\n", i);
  return text.substr(i, end == std::string::npos ? std::string::npos : end - i);
}

}  // namespace

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
  dynamic_joint_states_sub_ = create_subscription<control_msgs::msg::DynamicJointState>(
    "/dynamic_joint_states", rclcpp::SensorDataQoS(),
    [this](const control_msgs::msg::DynamicJointState & msg) {on_dynamic_joint_states(msg);});
  description_sub_ = create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String & msg) {
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.robot_description = msg.data;
      ++state_.description_version;
    });
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  state_.tf = tf_buffer_.get();

  fov_sub_ = create_subscription<visualization_msgs::msg::Marker>(
    "/wrist_camera/fov", 1, [this](const visualization_msgs::msg::Marker & msg) {
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.fov.frame = msg.header.frame_id;
      state_.fov.points.clear();
      for (const auto & p : msg.points) {
        state_.fov.points.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
      }
      state_.fov.received = now_s();
    });
  cubes_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/cubes/markers", 10, [this](const visualization_msgs::msg::MarkerArray & msg) {
      std::lock_guard<std::mutex> lock(state_.mutex);
      for (const auto & m : msg.markers) {
        if (m.action == visualization_msgs::msg::Marker::DELETE) {
          if (m.ns == "cubes") {
            state_.cubes.erase(m.id);
          }
          continue;
        }
        if (m.ns == "cube_labels") {
          state_.cubes[m.id].label = m.text;
          continue;
        }
        if (m.ns != "cubes") {
          continue;
        }
        auto & cube = state_.cubes[m.id];
        cube.frame = m.header.frame_id;
        cube.position = {static_cast<float>(m.pose.position.x), static_cast<float>(m.pose.position.y),
          static_cast<float>(m.pose.position.z)};
        const auto & q = m.pose.orientation;
        cube.yaw = static_cast<float>(std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        cube.edge = static_cast<float>(m.scale.x);
        cube.rgba = {m.color.r, m.color.g, m.color.b, m.color.a};
      }
    });
  tracked_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/cubes/tracked", 10, [this](const vision_msgs::msg::Detection3DArray & msg) {on_tracked(msg);});
  // the pick node's state, if it publishes one (latched): idle/busy and the held cube
  pick_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/cube_pick_place/state", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String & msg) {on_pick_state(msg);});
  // hidden test hook: GUI actions (clicks, buttons, screenshots) from a topic, for mock tests only
  const char * test = std::getenv("SO101_DASHBOARD_TEST");
  if (test && std::string(test) == "1") {
    RCLCPP_WARN(get_logger(), "test hook on: /so101_dashboard/test drives the GUI");
    test_sub_ = create_subscription<std_msgs::msg::String>(
      "/so101_dashboard/test", 10, [this](const std_msgs::msg::String & msg) {
        std::lock_guard<std::mutex> lock(state_.mutex);
        state_.test_commands.push_back(msg.data);
      });
  }
  image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  topics_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {refresh_image_topics();});
}

void RosLink::on_tracked(const vision_msgs::msg::Detection3DArray & msg)
{
  // merged per cube (several publishers, e.g. two FAKE cubes, each send their own), dropped when
  // no message mentioned it for TRACKED_TIMEOUT_S; the detector sends all of them in every message
  constexpr double TRACKED_TIMEOUT_S = 2.0;
  const double now = now_s();
  std::lock_guard<std::mutex> lock(state_.mutex);
  auto & cubes = state_.tracked;
  for (auto it = cubes.begin(); it != cubes.end(); ) {
    it = now - it->second.received > TRACKED_TIMEOUT_S ? cubes.erase(it) : std::next(it);
  }
  for (const auto & d : msg.detections) {
    const int k = cube_index(d.id);
    if (k < 0 || d.results.empty() || (!msg.header.frame_id.empty() && msg.header.frame_id != "base_link")) {
      continue;
    }
    const auto & pose = d.results[0].pose.pose;
    TrackedCube c;
    c.position = {static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.z)};
    c.yaw = static_cast<float>(2.0 * std::atan2(pose.orientation.z, pose.orientation.w));
    c.edge = d.bbox.size.x > 0 ? static_cast<float>(d.bbox.size.x) : 0.03f;
    c.score = static_cast<float>(d.results[0].hypothesis.score);
    c.received = now;
    cubes[k] = c;
  }
}

void RosLink::on_pick_state(const std_msgs::msg::String & msg)
{
  const std::string & text = msg.data;
  PickNodeState s;
  s.received = now_s();
  const std::string busy = json_value(text, "busy");
  const std::string status = json_value(text, "state").empty() ? json_value(text, "status") : json_value(text, "state");
  s.busy = busy == "true" || status == "busy";
  for (const char * key : {"held", "holding", "held_cube"}) {
    bool is_string = false;
    std::string v = json_value(text, key, &is_string);
    if (!v.empty() && v.front() == '{') {
      v = json_value(v, "cube", &is_string);
    }
    if (is_string && !v.empty()) {
      const int k = cube_index(v);
      s.held = k >= 0 ? "cube_" + std::to_string(k) : v;
      break;
    }
  }
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.pick_node = s;
  state_.holding = Holding{s.held, false, "pick node"};
}

void RosLink::refresh_image_topics()
{
  static const char * preferred = "/wrist_camera/image_raw/compressed";
  std::vector<std::string> topics;
  for (const auto & [name, types] : get_topic_names_and_types()) {
    if (std::find(types.begin(), types.end(), "sensor_msgs/msg/CompressedImage") != types.end()) {
      topics.push_back(name);
    }
  }
  std::string wanted;
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.image_topics = topics;
    if (state_.image_topic.empty() && std::find(topics.begin(), topics.end(), preferred) != topics.end()) {
      state_.image_topic = preferred;
    }
    wanted = state_.camera_shown ? state_.image_topic : "";
  }
  if (wanted == subscribed_image_topic_) {
    return;
  }
  image_sub_.reset();
  subscribed_image_topic_ = wanted;
  if (!wanted.empty()) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = image_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      wanted, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::CompressedImage & msg) {on_image(msg);}, options);
  }
}

void RosLink::on_image(const sensor_msgs::msg::CompressedImage & msg)
{
  cv::Mat bgr = cv::imdecode(msg.data, cv::IMREAD_COLOR);
  if (bgr.empty()) {
    return;
  }
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.image_rate.tick(now_s());
  state_.frame.width = rgb.cols;
  state_.frame.height = rgb.rows;
  state_.frame.rgb.assign(rgb.data, rgb.data + rgb.total() * rgb.elemSize());
  ++state_.frame.version;
}

void RosLink::on_dynamic_joint_states(const control_msgs::msg::DynamicJointState & msg)
{
  const double t = now_s();
  std::lock_guard<std::mutex> lock(state_.mutex);
  for (size_t i = 0; i < msg.joint_names.size() && i < msg.interface_values.size(); ++i) {
    if (msg.joint_names[i] == "torque") {   // the torque GPIO
      const auto & iv = msg.interface_values[i];
      for (size_t k = 0; k < iv.interface_names.size() && k < iv.values.size(); ++k) {
        if (iv.interface_names[k] == "enable") {
          state_.torque = iv.values[k];
        }
      }
      continue;
    }
    const auto it = state_.joints.find(msg.joint_names[i]);
    if (it == state_.joints.end()) {
      continue;   // known from /joint_states first
    }
    Joint & j = it->second;
    const auto & iv = msg.interface_values[i];
    for (size_t k = 0; k < iv.interface_names.size() && k < iv.values.size(); ++k) {
      const std::string & name = iv.interface_names[k];
      const double v = iv.values[k];
      if (name == "load") {
        j.load = v;
        j.load_hist.push(t, v);
      } else if (name == "current") {
        j.current = v;
        j.current_hist.push(t, v * 1000.0);
      } else if (name == "temperature") {
        j.temperature = v;
        j.temperature_hist.push(t, v);
      } else if (name == "voltage") {
        j.voltage = v;
      }
    }
  }
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
