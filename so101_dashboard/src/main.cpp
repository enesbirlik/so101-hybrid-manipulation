// SO-101 dashboard: one raylib window with docked ImGui panels; ROS runs on a background thread.
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>

#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>
#include <raylib.h>
#include <rlImGui.h>
#include <rclcpp/rclcpp.hpp>

#include "camera_view.hpp"
#include "panels.hpp"
#include "robot_view.hpp"
#include "ros_link.hpp"
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

// The dock space fills the window above the status bar. The first run (no saved layout) splits it:
// 3D view left, tasks and camera right, joints along the bottom.
void dock_space()
{
  const ImGuiViewport * vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, vp->WorkSize.y - STATUS_BAR_H));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
  ImGui::Begin("##dock_host", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings |
    ImGuiWindowFlags_NoNav);
  ImGui::PopStyleVar();
  const ImGuiID id = ImGui::GetID("main_dock");
  if (ImGui::DockBuilderGetNode(id) == nullptr) {
    ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(id, ImGui::GetContentRegionAvail());
    ImGuiID top, bottom, left, right, right_top, right_bottom;
    ImGui::DockBuilderSplitNode(id, ImGuiDir_Down, 0.36f, &bottom, &top);
    ImGui::DockBuilderSplitNode(top, ImGuiDir_Left, 0.58f, &left, &right);
    ImGui::DockBuilderSplitNode(right, ImGuiDir_Up, 0.6f, &right_top, &right_bottom);
    ImGui::DockBuilderDockWindow(sd::WIN_VIEW, left);
    ImGui::DockBuilderDockWindow(sd::WIN_CAMERA, right_top);
    ImGui::DockBuilderDockWindow(sd::WIN_TASKS, right_bottom);
    ImGui::DockBuilderDockWindow(sd::WIN_JOINTS, bottom);
    ImGui::DockBuilderFinish(id);
  }
  ImGui::DockSpace(id);
  ImGui::End();
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGUSR1, [](int) {screenshot_requested = true;});
  sd::State state;
  auto node = std::make_shared<sd::RosLink>(state);
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
  auto view = std::make_unique<sd::RobotView>();
  auto camera = std::make_unique<sd::CameraView>();

  while (!WindowShouldClose() && rclcpp::ok()) {
    BeginDrawing();
    ClearBackground(sd::theme::to_raylib(sd::theme::BG));
    rlImGuiBegin();
    dock_space();
    view->draw(state);
    camera->draw(state);
    sd::draw_placeholder(sd::WIN_TASKS, "tasks: next step");
    sd::draw_joints(state);
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
