#include "tasks.hpp"

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <ctime>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tinyxml2.h>

namespace so101_dashboard
{

namespace
{

constexpr size_t LOG_LENGTH = 50;

std::string error_name(int code)
{
  static const std::map<int, const char *> names{
    {-1, "FAILURE"}, {-2, "PLANNING_FAILED"}, {-3, "INVALID_MOTION_PLAN"}, {-4, "PLAN_INVALIDATED"},
    {-5, "CONTROL_FAILED"}, {-7, "TIMED_OUT"}, {-8, "PREEMPTED"}, {-10, "START_STATE_IN_COLLISION"},
    {-12, "GOAL_IN_COLLISION"}, {-15, "INVALID_GROUP_NAME"}, {-16, "INVALID_GOAL_CONSTRAINTS"},
    {-26, "START_STATE_INVALID"}, {-27, "GOAL_STATE_INVALID"}, {-31, "NO_IK_SOLUTION"}};
  const auto it = names.find(code);
  return it != names.end() ? it->second : "error " + std::to_string(code);
}

}  // namespace

TaskRunner::TaskRunner(rclcpp::Node::SharedPtr node, State & state)
: node_(std::move(node)), state_(state)
{
  move_group_ = rclcpp_action::create_client<MoveGroup>(node_, "move_action");
  for (const char * name : {"/arm_controller/follow_joint_trajectory", "/gripper_controller/follow_joint_trajectory"}) {
    controllers_.push_back(rclcpp_action::create_client<FollowJointTrajectory>(node_, name));
  }
  planned_path_sub_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
    "/display_planned_path", 5, [this](const moveit_msgs::msg::DisplayTrajectory & msg) {
      if (msg.trajectory.empty()) {
        return;
      }
      const auto & jt = msg.trajectory.back().joint_trajectory;
      Ghost g;
      g.joints = jt.joint_names;
      for (const auto & p : jt.points) {
        g.times.push_back(rclcpp::Duration(p.time_from_start).seconds());
        g.positions.push_back(p.positions);
      }
      g.received = now_s();
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.ghost = std::move(g);
    });
  process_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this] {poll_process();});
  torque_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/torque_controller/commands", 1);
  switch_controller_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");
  pick_ = rclcpp_action::create_client<PickCube>(node_, "/pick_cube");
  place_ = rclcpp_action::create_client<PlaceCube>(node_, "/place_cube");
  overview_ = node_->create_client<Trigger>("/cube_pick_place/overview");
  load_srdf();
}

TaskRunner::~TaskRunner()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (child_ > 0) {
    kill(-child_, SIGINT);
  }
}

void TaskRunner::load_srdf()
{
  std::string path;
  try {
    path = ament_index_cpp::get_package_share_directory("so101_moveit_config") + "/config/so101.srdf";
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "no so101_moveit_config: %s", e.what());
    return;
  }
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS || !doc.RootElement()) {
    RCLCPP_WARN(node_->get_logger(), "cannot read %s", path.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto * gs = doc.RootElement()->FirstChildElement("group_state"); gs; gs = gs->NextSiblingElement("group_state")) {
    auto & joints = named_[gs->Attribute("group")][gs->Attribute("name")];
    for (auto * j = gs->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
      joints[j->Attribute("name")] = j->DoubleAttribute("value");
    }
  }
}

std::vector<std::string> TaskRunner::named_states(const std::string & group) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  const auto it = named_.find(group);
  if (it != named_.end()) {
    for (const auto & [name, joints] : it->second) {
      names.push_back(name);
    }
  }
  return names;
}

bool TaskRunner::move_group_online() const
{
  return move_group_->action_server_is_ready();
}

void TaskRunner::begin(const std::string & label)
{
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.task = TaskStatus{};
  state_.task.running = true;
  state_.task.label = label;
  state_.task.phase = "starting";
  state_.task.started = now_s();
}

void TaskRunner::log(const std::string & label, const std::string & result, bool ok)
{
  char clock[16];
  const std::time_t now = std::time(nullptr);
  std::strftime(clock, sizeof(clock), "%H:%M:%S", std::localtime(&now));
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.task_log.push_front(TaskLogEntry{clock, label, result, 0.0, ok, ""});
  if (state_.task_log.size() > LOG_LENGTH) {
    state_.task_log.pop_back();
  }
}

void TaskRunner::set_torque(bool on)
{
  if (!on) {
    stop();
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {on ? 1.0 : 0.0};
  torque_pub_->publish(msg);
  if (!on) {
    log("TORQUE OFF", "sent", true);
    return;
  }
  // give the hardware a few cycles to switch, then restart the controllers
  std::lock_guard<std::mutex> lock(mutex_);
  restart_timer_ = node_->create_wall_timer(std::chrono::milliseconds(300), [this] {
        restart_timer_->cancel();
        restart_controllers();
      });
}

void TaskRunner::restart_controllers()
{
  using Switch = controller_manager_msgs::srv::SwitchController;
  static const std::vector<std::string> controllers{"arm_controller", "gripper_controller"};
  if (!switch_controller_->service_is_ready()) {
    log("TORQUE ON", "no controller_manager", false);
    return;
  }
  auto request = std::make_shared<Switch::Request>();
  request->deactivate_controllers = controllers;
  request->strictness = Switch::Request::STRICT;
  request->timeout = rclcpp::Duration::from_seconds(2.0);
  switch_controller_->async_send_request(request, [this](rclcpp::Client<Switch>::SharedFuture done) {
      if (!done.get()->ok) {
        log("TORQUE ON", "controllers not stopped", false);
        return;
      }
      auto again = std::make_shared<Switch::Request>();
      again->activate_controllers = controllers;
      again->strictness = Switch::Request::STRICT;
      again->timeout = rclcpp::Duration::from_seconds(2.0);
      switch_controller_->async_send_request(again, [this](rclcpp::Client<Switch>::SharedFuture result) {
        log("TORQUE ON", result.get()->ok ? "on, controllers restarted" : "controllers not restarted",
        result.get()->ok);
      });
    });
}

void TaskRunner::finish(bool ok, const std::string & result)
{
  std::lock_guard<std::mutex> lock(state_.mutex);
  if (!state_.task.running) {
    return;
  }
  TaskLogEntry entry;
  entry.label = state_.task.label;
  entry.result = result;
  entry.ok = ok;
  entry.duration = now_s() - state_.task.started;
  // pick/place: every phase with its start, e.g. "search +0.0 s > approach +2.1 s > ..."
  for (const auto & [phase, at] : state_.task.phases) {
    char buf[96];
    snprintf(buf, sizeof(buf), "%s%s +%.1f s", entry.details.empty() ? "" : " > ", phase.c_str(),
      at - state_.task.started);
    entry.details += buf;
  }
  const std::time_t started = std::time(nullptr) - static_cast<std::time_t>(entry.duration);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&started));
  entry.clock = buf;
  state_.task_log.push_front(entry);
  if (state_.task_log.size() > LOG_LENGTH) {
    state_.task_log.pop_back();
  }
  state_.task.running = false;
}

bool TaskRunner::start_named(const std::string & label, const std::string & group, const std::string & name,
  const MotionOptions & options)
{
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    if (state_.task.running) {
      return false;
    }
  }
  std::map<std::string, double> joints;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto g = named_.find(group);
    if (g == named_.end() || !g->second.count(name)) {
      return false;
    }
    joints = g->second.at(name);
  }
  begin(label);
  if (!move_group_->action_server_is_ready()) {
    finish(false, "move_group offline");
    return false;
  }

  MoveGroup::Goal goal;
  auto & r = goal.request;
  r.group_name = group;
  r.pipeline_id = options.pipeline;
  r.planner_id = options.planner;
  // Pilz needs an IK solver for the group and the gripper has none: plan its joint goals with OMPL.
  if (group == "gripper" && r.pipeline_id == "pilz_industrial_motion_planner") {
    r.pipeline_id = "ompl";
    r.planner_id = "RRTConnect";
  }
  r.num_planning_attempts = 5;
  r.allowed_planning_time = 3.0;
  r.max_velocity_scaling_factor = options.velocity;
  r.max_acceleration_scaling_factor = options.acceleration;
  r.start_state.is_diff = true;
  moveit_msgs::msg::Constraints constraints;
  for (const auto & [joint, value] : joints) {
    moveit_msgs::msg::JointConstraint c;
    c.joint_name = joint;
    c.position = value;
    c.tolerance_above = c.tolerance_below = 0.001;
    c.weight = 1.0;
    constraints.joint_constraints.push_back(c);
  }
  r.goal_constraints.push_back(constraints);
  goal.planning_options.plan_only = false;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  rclcpp_action::Client<MoveGroup>::SendGoalOptions callbacks;
  callbacks.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<MoveGroup>::SharedPtr handle) {
      if (!handle) {
        finish(false, "rejected");
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      goal_ = handle;
    };
  callbacks.feedback_callback = [this](rclcpp_action::ClientGoalHandle<MoveGroup>::SharedPtr,
    const std::shared_ptr<const MoveGroup::Feedback> feedback) {
      std::lock_guard<std::mutex> lock(state_.mutex);
      state_.task.phase = feedback->state;
    };
  callbacks.result_callback = [this](const rclcpp_action::ClientGoalHandle<MoveGroup>::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_.reset();
      }
      const int code = result.result ? result.result->error_code.val : 0;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        finish(true, "done");
      } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
        finish(false, "stopped");
      } else {
        finish(false, error_name(code));
      }
    };
  move_group_->async_send_goal(goal, callbacks);
  return true;
}

bool TaskRunner::start_process(const std::string & label, const std::vector<std::string> & argv)
{
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    if (state_.task.running) {
      return false;
    }
  }
  std::vector<char *> args;
  for (const auto & a : argv) {
    args.push_back(const_cast<char *>(a.c_str()));
  }
  args.push_back(nullptr);
  begin(label);
  const pid_t pid = fork();
  if (pid == 0) {
    setsid();   // its own process group, so stop() reaches its children too
    execv(args[0], args.data());
    _exit(127);
  }
  if (pid < 0) {
    finish(false, "fork failed");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    child_ = pid;
  }
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.task.phase = "running";
  return true;
}

void TaskRunner::poll_process()
{
  int status = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (child_ <= 0 || waitpid(child_, &status, WNOHANG) != child_) {
      return;
    }
    child_ = -1;
  }
  if (WIFEXITED(status)) {
    const int code = WEXITSTATUS(status);
    finish(code == 0, code == 0 ? "done" : code == 127 ? "cannot start" : "exit " + std::to_string(code));
  } else {
    finish(false, "stopped");
  }
}

bool TaskRunner::claim(const std::string & label)
{
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    if (state_.task.running) {
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_on_accept_ = false;
  }
  begin(label);
  return true;
}

void TaskRunner::on_phase(const std::string & phase, const std::string & detail)
{
  std::lock_guard<std::mutex> lock(state_.mutex);
  if (!state_.task.running) {
    return;
  }
  if (state_.task.phases.empty() || state_.task.phases.back().first != phase) {
    state_.task.phases.emplace_back(phase, now_s());
  }
  state_.task.phase = phase;
  state_.task.detail = detail;
}

void TaskRunner::set_holding(const Holding & h)
{
  std::lock_guard<std::mutex> lock(state_.mutex);
  if (state_.pick_node.received < 0) {   // the node's own state topic, when there is one, is the truth
    state_.holding = h;
  }
}

void TaskRunner::forget_holding()
{
  std::lock_guard<std::mutex> lock(state_.mutex);
  state_.holding = Holding{};
}

bool TaskRunner::pick_online() const
{
  return pick_->action_server_is_ready();
}

bool TaskRunner::place_online() const
{
  return place_->action_server_is_ready();
}

bool TaskRunner::look_online() const
{
  return overview_->service_is_ready();
}

bool TaskRunner::start_pick(int cube)
{
  const std::string key = "cube_" + std::to_string(cube);
  if (!claim("PICK K" + std::to_string(cube))) {
    return false;
  }
  if (!pick_->action_server_is_ready()) {
    finish(false, "/pick_cube offline");
    return false;
  }
  PickCube::Goal goal;
  goal.cube_id = std::to_string(cube);
  goal.hover_only = false;
  rclcpp_action::Client<PickCube>::SendGoalOptions callbacks;
  callbacks.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<PickCube>::SharedPtr handle) {
      if (!handle) {
        finish(false, "rejected (pick node busy?)");
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      pick_goal_ = handle;
      if (cancel_on_accept_) {
        pick_->async_cancel_goal(handle);
      }
    };
  callbacks.feedback_callback = [this](rclcpp_action::ClientGoalHandle<PickCube>::SharedPtr,
    const std::shared_ptr<const PickCube::Feedback> feedback) {
      on_phase(feedback->phase, feedback->detail);
    };
  callbacks.result_callback = [this, key](const rclcpp_action::ClientGoalHandle<PickCube>::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pick_goal_.reset();
      }
      const auto & r = result.result;
      const std::string message = r ? r->message : "no result";
      const std::string phase = r ? r->failed_phase : "";
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && r && r->success) {
        set_holding(Holding{key, false, "pick result"});
        finish(true, message);
        return;
      }
      const bool canceled = result.code == rclcpp_action::ResultCode::CANCELED;
      // what the pick node now believes (pick_place_node.py): it records the cube after the verify,
      // and drops the record again only when the lift loses it
      if (phase == "lift" && message.find("lost the cube") == std::string::npos) {
        set_holding(Holding{key, true, canceled ? "pick stopped in the lift" : "pick failed in the lift"});
      } else if (phase == "setup" && message.find("already holding") != std::string::npos) {
        const size_t at = message.find("already holding ") + 16;
        set_holding(Holding{message.substr(at, message.find(':', at) - at), false, "pick node"});
      } else {
        set_holding(Holding{});
      }
      finish(false, canceled ? "stopped: " + message :
        "failed" + (phase.empty() ? "" : " in " + phase) + ": " + message);
    };
  pick_->async_send_goal(goal, callbacks);
  return true;
}

bool TaskRunner::start_place(double x, double y, double yaw_deg, int on_cube)
{
  char label[48];
  if (on_cube >= 0) {
    snprintf(label, sizeof(label), "STACK ON K%d", on_cube);
  } else {
    snprintf(label, sizeof(label), "PLACE %.1f,%.1f", x * 100.0, y * 100.0);
  }
  if (!claim(label)) {
    return false;
  }
  if (!place_->action_server_is_ready()) {
    finish(false, "/place_cube offline");
    return false;
  }
  PlaceCube::Goal goal;
  goal.x = x;
  goal.y = y;
  goal.yaw_deg = yaw_deg;
  goal.on_cube_id = on_cube >= 0 ? std::to_string(on_cube) : "";
  int held_cube;   // for the 3D view: which cube the result puts down
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    held_cube = cube_index(state_.holding.cube);
  }
  rclcpp_action::Client<PlaceCube>::SendGoalOptions callbacks;
  callbacks.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<PlaceCube>::SharedPtr handle) {
      if (!handle) {
        finish(false, "rejected (pick node busy?)");
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      place_goal_ = handle;
      if (cancel_on_accept_) {
        place_->async_cancel_goal(handle);
      }
    };
  callbacks.feedback_callback = [this](rclcpp_action::ClientGoalHandle<PlaceCube>::SharedPtr,
    const std::shared_ptr<const PlaceCube::Feedback> feedback) {
      on_phase(feedback->phase, feedback->detail);
    };
  callbacks.result_callback = [this, held_cube, yaw_deg](
    const rclcpp_action::ClientGoalHandle<PlaceCube>::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        place_goal_.reset();
      }
      const auto & r = result.result;
      const std::string message = r ? r->message : "no result";
      const std::string phase = r ? r->failed_phase : "";
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && r && r->success) {
        {
          std::lock_guard<std::mutex> lock(state_.mutex);
          PlacedResult & p = state_.placed;
          p.cube = held_cube;
          p.position = {static_cast<float>(r->placed_position.x), static_cast<float>(r->placed_position.y),
            static_cast<float>(r->placed_position.z)};
          p.yaw_deg = static_cast<float>(yaw_deg);
          p.at = now_s();
          ++p.seq;
        }
        set_holding(Holding{});
        finish(true, message);
        return;
      }
      const bool canceled = result.code == rclcpp_action::ResultCode::CANCELED;
      // the node drops its record once the jaws opened (release done), and refuses without a cube
      if (phase == "retreat" || (phase == "setup" && message.find("not holding") != std::string::npos)) {
        set_holding(Holding{});
      } else if (phase == "release") {
        std::lock_guard<std::mutex> lock(state_.mutex);
        if (state_.pick_node.received < 0) {
          state_.holding.uncertain = true;
          state_.holding.source = "place stopped in the release";
        }
      }
      finish(false, canceled ? "stopped: " + message :
        "failed" + (phase.empty() ? "" : " in " + phase) + ": " + message);
    };
  place_->async_send_goal(goal, callbacks);
  return true;
}

bool TaskRunner::start_look()
{
  if (!claim("LOOK")) {
    return false;
  }
  if (!overview_->service_is_ready()) {
    finish(false, "/cube_pick_place/overview offline");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_.mutex);
    state_.task.phase = "overview";
  }
  overview_->async_send_request(std::make_shared<Trigger::Request>(),
    [this](rclcpp::Client<Trigger>::SharedFuture done) {
      const auto response = done.get();
      finish(response->success, response->message.empty() ? (response->success ? "done" : "failed") :
      response->message);
    });
  return true;
}

void TaskRunner::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_) {
    move_group_->async_cancel_goal(goal_);
  }
  // pick/place: cancel the goal (the node stops its moves, the gripper is left as it is); a goal
  // still on its way is canceled as soon as it is accepted
  if (pick_goal_) {
    pick_->async_cancel_goal(pick_goal_);
  }
  if (place_goal_) {
    place_->async_cancel_goal(place_goal_);
  }
  cancel_on_accept_ = true;
  // Every goal on the trajectory controllers, whoever sent it: the arm holds where it is.
  for (const auto & c : controllers_) {
    if (c->action_server_is_ready()) {
      c->async_cancel_all_goals();
    }
  }
  if (child_ > 0) {
    kill(-child_, SIGINT);
  }
}

}  // namespace so101_dashboard
