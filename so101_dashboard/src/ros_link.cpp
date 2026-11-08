#include "ros_link.hpp"

#include <algorithm>
#include <chrono>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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
  image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  topics_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {refresh_image_topics();});
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
    wanted = state_.image_topic;
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
