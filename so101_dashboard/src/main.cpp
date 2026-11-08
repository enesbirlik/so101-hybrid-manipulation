// SO-101 dashboard: one raylib window with docked ImGui panels; ROS runs on a background thread.
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>

#include <imgui.h>
#include <implot.h>
#include <raylib.h>
#include <rlImGui.h>
#include <rclcpp/rclcpp.hpp>

#include "camera_view.hpp"
#include "layout.hpp"
#include "panels.hpp"
#include "robot_view.hpp"
#include "ros_link.hpp"
#include "tasks.hpp"
#include "theme.hpp"

namespace sd = so101_dashboard;

namespace
{

constexpr float STATUS_BAR_H = 30.0f;

// F12 or `kill -USR1 <pid>` saves so101_dashboard.png into the working directory.
std::atomic<bool> screenshot_requested{false};

// Saved panel layout: ~/.local/state/so101_dashboard/imgui.ini
std::string layout_file()
{
  const char * home = std::getenv("HOME");
  const std::filesystem::path dir = std::filesystem::path(home ? home : ".") / ".local/state/so101_dashboard";
  std::filesystem::create_directories(dir);
  return (dir / "imgui.ini").string();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGUSR1, [](int) {screenshot_requested = true;});
  sd::State state;
  auto node = std::make_shared<sd::RosLink>(state);
  sd::TaskRunner tasks(node, state);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor] {executor.spin();});

  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
  InitWindow(1600, 960, "SO-101 // MISSION CONTROL");
  SetWindowMinSize(900, 600);
  SetTargetFPS(60);
  SetExitKey(KEY_NULL);   // Esc must not close the window

  rlImGuiSetLoadFontsCallback(sd::theme::load_fonts);
  rlImGuiSetup(true);
  ImPlot::CreateContext();
  sd::theme::apply_style();
  ImGuiIO & io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  const std::string ini = layout_file();
  io.IniFilename = ini.c_str();
  sd::Layout layout;
  auto view = std::make_unique<sd::RobotView>();
  auto camera = std::make_unique<sd::CameraView>();

  while (!WindowShouldClose() && rclcpp::ok()) {
    BeginDrawing();
    ClearBackground(sd::theme::to_raylib(sd::theme::BG));
    rlImGuiBegin();
    layout.begin_frame(STATUS_BAR_H);
    // a closed panel is not drawn at all (and a closed camera panel stops the image subscription)
    if (bool * open = layout.open(sd::WIN_VIEW); *open) {
      view->draw(state, open);
    }
    {
      bool * open = layout.open(sd::WIN_CAMERA);
      {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.camera_shown = *open;
      }
      if (*open) {
        camera->draw(state, open);
      }
    }
    if (bool * open = layout.open(sd::WIN_TASKS); *open) {
      sd::draw_tasks(state, tasks, open);
    }
    if (bool * open = layout.open(sd::WIN_TASK_LOG); *open) {
      sd::draw_task_log(state, open);
    }
    if (bool * open = layout.open(sd::WIN_JOINTS); *open) {
      sd::draw_joints(state, open);
    }
    if (bool * open = layout.open(sd::WIN_PLOTS); *open) {
      sd::draw_plots(state, open);
    }
    sd::draw_status_bar(state, STATUS_BAR_H);
    rlImGuiEnd();
    EndDrawing();
    if (IsKeyPressed(KEY_F12) || screenshot_requested.exchange(false)) {
      TakeScreenshot("so101_dashboard.png");
    }
  }

  view.reset();   // GPU resources go before the window
  camera.reset();
  ImPlot::DestroyContext();
  rlImGuiShutdown();
  CloseWindow();
  executor.cancel();
  ros_thread.join();
  rclcpp::shutdown();
  return 0;
}
