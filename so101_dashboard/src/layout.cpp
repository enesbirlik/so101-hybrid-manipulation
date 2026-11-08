#include "layout.hpp"

#include <cstdio>
#include <cstring>

#include <imgui_internal.h>

#include "panels.hpp"
#include "theme.hpp"

namespace so101_dashboard
{

namespace
{

constexpr const char * SETTINGS_TYPE = "Dashboard";

const char * key_label(ImGuiKey key) {return ImGui::GetKeyName(key);}

}  // namespace

Layout::Layout()
: panels_{
    {WIN_VIEW, ImGuiKey_F1, true},
    {WIN_CAMERA, ImGuiKey_F2, true},
    {WIN_TASKS, ImGuiKey_F3, true},
    {WIN_TASK_LOG, ImGuiKey_F4, true},
    {WIN_JOINTS, ImGuiKey_F5, true},
    {WIN_PLOTS, ImGuiKey_F6, true}}
{
  // "[Dashboard][Panels]" section in imgui.ini: one "NAME=0|1" line per panel
  ImGuiSettingsHandler handler;
  handler.TypeName = SETTINGS_TYPE;
  handler.TypeHash = ImHashStr(SETTINGS_TYPE);
  handler.UserData = this;
  handler.ReadOpenFn = [](ImGuiContext *, ImGuiSettingsHandler *, const char *) -> void * {
      return reinterpret_cast<void *>(1);
    };
  handler.ReadLineFn = [](ImGuiContext *, ImGuiSettingsHandler * h, void *, const char * line) {
      char name[64];
      int value;
      if (std::sscanf(line, "%63[^=]=%d", name, &value) == 2) {
        if (bool * open = static_cast<Layout *>(h->UserData)->open(name)) {
          *open = value != 0;
        }
      }
    };
  handler.WriteAllFn = [](ImGuiContext *, ImGuiSettingsHandler * h, ImGuiTextBuffer * buf) {
      buf->appendf("[%s][Panels]\n", SETTINGS_TYPE);
      for (const auto & p : static_cast<Layout *>(h->UserData)->panels_) {
        buf->appendf("%s=%d\n", p.name, p.open ? 1 : 0);
      }
      buf->append("\n");
    };
  ImGui::AddSettingsHandler(&handler);
}

bool * Layout::open(const char * name)
{
  for (auto & p : panels_) {
    if (std::strcmp(p.name, name) == 0) {
      return &p.open;
    }
  }
  return nullptr;
}

void Layout::begin_frame(float status_bar_height)
{
  if (!ImGui::GetIO().WantTextInput) {
    for (auto & p : panels_) {
      if (ImGui::IsKeyPressed(p.key, false)) {
        p.open = !p.open;
      }
    }
  }
  menu_bar();
  dock_space(status_bar_height);
}

void Layout::menu_bar()
{
  if (!ImGui::BeginMainMenuBar()) {
    return;
  }
  ImGui::TextColored(theme::CYAN, "SO-101");
  ImGui::TextColored(theme::DIM, "//");
  if (ImGui::BeginMenu("View")) {
    for (auto & p : panels_) {
      ImGui::MenuItem(p.name, key_label(p.key), &p.open);
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Show all")) {
      for (auto & p : panels_) {
        p.open = true;
      }
    }
    if (ImGui::MenuItem("Reset layout")) {
      reset_ = true;
    }
    ImGui::EndMenu();
  }
  // the open panels, as quick toggles
  ImGui::Dummy(ImVec2(16, 0));
  for (auto & p : panels_) {
    ImGui::PushID(p.name);
    ImGui::PushStyleColor(ImGuiCol_Text, p.open ? theme::CYAN : theme::DIM);
    if (ImGui::Selectable(p.name, false, 0, ImVec2(ImGui::CalcTextSize(p.name).x, 0))) {
      p.open = !p.open;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("%s: %s", key_label(p.key), p.open ? "hide" : "show");
    ImGui::PopID();
  }
  ImGui::EndMainMenuBar();
}

// The dock space fills the window between the menu and the status bar. Without a saved layout
// (first run, or "Reset layout"): 3D view top left; camera, tasks and the task log on the right;
// joints and plots along the bottom.
void Layout::dock_space(float status_bar_height)
{
  const ImGuiViewport * vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(vp->WorkPos);
  ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, vp->WorkSize.y - status_bar_height));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(4, 4));
  ImGui::Begin("##dock_host", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings |
    ImGuiWindowFlags_NoNav);
  ImGui::PopStyleVar();
  const ImGuiID id = ImGui::GetID("main_dock");
  if (reset_) {
    ImGui::DockBuilderRemoveNode(id);
    for (auto & p : panels_) {
      p.open = true;
    }
    reset_ = false;
  }
  if (ImGui::DockBuilderGetNode(id) == nullptr) {
    ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(id, ImGui::GetContentRegionAvail());
    // The node on the far side of each split keeps the "central" flag, and a central node stays
    // even when empty: split so that the 3D view ends up central, and every other panel can be
    // closed without leaving a hole.
    ImGuiID top, bottom, left, right, camera, below_camera, tasks, log, joints, plots;
    ImGui::DockBuilderSplitNode(id, ImGuiDir_Down, 0.34f, &bottom, &top);
    ImGui::DockBuilderSplitNode(top, ImGuiDir_Right, 0.44f, &right, &left);
    ImGui::DockBuilderSplitNode(right, ImGuiDir_Up, 0.52f, &camera, &below_camera);
    ImGui::DockBuilderSplitNode(below_camera, ImGuiDir_Up, 0.6f, &tasks, &log);
    ImGui::DockBuilderSplitNode(bottom, ImGuiDir_Left, 0.56f, &joints, &plots);
    ImGui::DockBuilderDockWindow(WIN_VIEW, left);
    ImGui::DockBuilderDockWindow(WIN_CAMERA, camera);
    ImGui::DockBuilderDockWindow(WIN_TASKS, tasks);
    ImGui::DockBuilderDockWindow(WIN_TASK_LOG, log);
    ImGui::DockBuilderDockWindow(WIN_JOINTS, joints);
    ImGui::DockBuilderDockWindow(WIN_PLOTS, plots);
    ImGui::DockBuilderFinish(id);
  }
  ImGui::DockSpace(id);
  ImGui::End();
}

}  // namespace so101_dashboard
