#include "pick_place_ui.hpp"

#include <algorithm>
#include <cmath>

#include <imgui.h>

#include "tasks.hpp"
#include "theme.hpp"

namespace so101_dashboard
{

namespace
{

// A tooltip that also shows over a disabled button (why it is disabled).
void tooltip(const char * text)
{
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled | ImGuiHoveredFlags_DelayShort)) {
    ImGui::SetTooltip("%s", text);
  }
}

// Why a pick/place button is disabled, or nullptr.
const char * blocked(const CubeUi & ui, bool server_online, const char * server)
{
  static std::string why;
  if (ui.running) {
    return "a task is running";
  }
  if (ui.limp) {
    return "the torque is off";
  }
  if (!server_online) {
    why = std::string(server) + " is offline: start so101_manipulation's pick_place.launch.py";
    return why.c_str();
  }
  if (ui.node_busy) {
    return "the pick node is busy";
  }
  return nullptr;
}

void push_button_colour(const ImVec4 & c)
{
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(c.x, c.y, c.z, 0.22f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(c.x, c.y, c.z, 0.45f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(c.x, c.y, c.z, 0.7f));
  ImGui::PushStyleColor(ImGuiCol_Border, c);
}

}  // namespace

CubeUi cube_ui(State & state, TaskRunner & tasks)
{
  CubeUi ui;
  ui.pick_online = tasks.pick_online();
  ui.place_online = tasks.place_online();
  ui.look_online = tasks.look_online();
  std::lock_guard<std::mutex> lock(state.mutex);
  ui.running = state.task.running;
  ui.limp = state.torque == 0.0;
  ui.node_state = state.pick_node.received >= 0;
  ui.node_busy = ui.node_state && state.pick_node.busy && !state.task.running;
  ui.holding = state.holding;
  ui.held = cube_index(state.holding.cube);
  ui.sel = state.selection;
  ui.reach = state.reach;
  if (ui.sel.cube >= 0) {
    const auto m = state.cubes.find(ui.sel.cube);
    const auto t = state.tracked.find(ui.sel.cube);
    if (m != state.cubes.end() && m->second.frame == "base_link") {
      ui.selected_seen = true;
      ui.selected_x = m->second.position[0];
      ui.selected_y = m->second.position[1];
    } else if (t != state.tracked.end()) {
      ui.selected_seen = true;
      ui.selected_x = t->second.position[0];
      ui.selected_y = t->second.position[1];
    }
  }
  return ui;
}

bool holding(const CubeUi & ui)
{
  return !ui.holding.cube.empty();
}

std::string held_name(const CubeUi & ui)
{
  return ui.held >= 0 ? "K" + std::to_string(ui.held) : "the cube";
}

bool reach_band(const ReachTable & table, float pan_deg, bool every_yaw, float & near, float & far)
{
  if (table.rows.empty()) {
    near = REACH_MIN_M;
    far = REACH_MAX_M;
    return true;
  }
  const auto & rows = table.rows;
  if (pan_deg < rows.front().pan_deg - 1e-3f || pan_deg > rows.back().pan_deg + 1e-3f) {
    return false;
  }
  size_t i = 0;
  while (i + 2 < rows.size() && pan_deg > rows[i + 1].pan_deg) {
    ++i;
  }
  const ReachRow & a = rows[i];
  const ReachRow & b = rows.size() > 1 ? rows[i + 1] : rows[i];
  const float span = b.pan_deg - a.pan_deg;
  const float t = span > 1e-6f ? std::clamp((pan_deg - a.pan_deg) / span, 0.0f, 1.0f) : 0.0f;
  const float an = every_yaw ? a.near_all : a.near, af = every_yaw ? a.far_all : a.far;
  const float bn = every_yaw ? b.near_all : b.near, bf = every_yaw ? b.far_all : b.far;
  // NaN on one side: no band there (it does not fade in between pans)
  if (!std::isfinite(an) || !std::isfinite(af) || !std::isfinite(bn) || !std::isfinite(bf)) {
    return false;
  }
  near = an + (bn - an) * t;
  far = af + (bf - af) * t;
  return far > near;
}

std::string reach_problem(const ReachTable & table, float x, float y)
{
  const float r = std::hypot(x, y);
  const float pan = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
  float near = 0.0f, far = 0.0f;
  const bool band = reach_band(table, pan, false, near, far);
  if (band && r >= near && r <= far) {
    return "";
  }
  char buf[160];
  if (!band) {
    snprintf(buf, sizeof(buf), "out of reach: nothing reaches at %.0f deg pan (the reach table covers %.0f..%.0f deg)",
      pan, table.rows.front().pan_deg, table.rows.back().pan_deg);
  } else {
    snprintf(buf, sizeof(buf), "out of reach: %.1f cm from the base axis (%.0f-%.0f cm at this pan%s)", r * 100.0f,
      near * 100.0f, far * 100.0f, table.rows.empty() ? ", fixed ring" : "");
  }
  return buf;
}

bool pick_selected(State & state, TaskRunner & tasks)
{
  const CubeUi ui = cube_ui(state, tasks);
  if (ui.sel.cube < 0 || holding(ui) || blocked(ui, ui.pick_online, "/pick_cube")) {
    return false;
  }
  return tasks.start_pick(ui.sel.cube);
}

bool place_at_target(State & state, TaskRunner & tasks)
{
  const CubeUi ui = cube_ui(state, tasks);
  if (!holding(ui) || !ui.sel.has_target || !reach_problem(ui.reach, ui.sel.x, ui.sel.y).empty() ||
    blocked(ui, ui.place_online, "/place_cube"))
  {
    return false;
  }
  return tasks.start_place(ui.sel.x, ui.sel.y, ui.sel.any_yaw ? NAN : ui.sel.yaw_deg, -1);
}

bool stack_on_target(State & state, TaskRunner & tasks)
{
  const CubeUi ui = cube_ui(state, tasks);
  if (!holding(ui) || ui.sel.stack_on < 0 || ui.sel.stack_on == ui.held ||
    blocked(ui, ui.place_online, "/place_cube"))
  {
    return false;
  }
  return tasks.start_place(0.0, 0.0, NAN, ui.sel.stack_on);   // the lower cube sets the yaw
}

void draw_cube_actions(State & state, TaskRunner & tasks, const CubeUi & ui, const char * id)
{
  ImGui::PushID(id);
  char label[64];
  if (!holding(ui)) {
    if (ui.sel.cube < 0) {
      ImGui::TextColored(theme::DIM, "click a cube in the 3D view to select it");
      ImGui::PopID();
      return;
    }
    const char * why = blocked(ui, ui.pick_online, "/pick_cube");
    snprintf(label, sizeof(label), "PICK K%d", ui.sel.cube);
    push_button_colour(theme::CYAN);
    ImGui::BeginDisabled(why != nullptr);
    if (ImGui::Button(label, ImVec2(150, 34))) {
      pick_selected(state, tasks);
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor(4);
    tooltip(why ? why : "find the cube with the wrist camera, grasp it from above (/pick_cube); STOP cancels");
    if (ui.selected_seen && !reach_problem(ui.reach, ui.selected_x, ui.selected_y).empty()) {
      ImGui::TextColored(theme::AMBER, "outside the reach band: the pick may fail");
    }
    ImGui::PopID();
    return;
  }

  const char * why = blocked(ui, ui.place_online, "/place_cube");
  if (ui.sel.stack_on >= 0) {
    snprintf(label, sizeof(label), "STACK ON K%d (untested)", ui.sel.stack_on);
    push_button_colour(theme::AMBER);
    ImGui::BeginDisabled(why != nullptr);
    if (ImGui::Button(label, ImVec2(260, 34))) {
      stack_on_target(state, tasks);
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor(4);
    tooltip(why ? why : "put the held cube on top of this one (/place_cube on_cube_id).\n"
      "UNTESTED: stacking has not run on the real arm yet");
  } else if (ui.sel.has_target) {
    snprintf(label, sizeof(label), "PLACE HERE (%.1f, %.1f) cm", ui.sel.x * 100.0f, ui.sel.y * 100.0f);
    push_button_colour(theme::OK);
    ImGui::BeginDisabled(why != nullptr);
    if (ImGui::Button(label, ImVec2(260, 34))) {
      place_at_target(state, tasks);
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor(4);
    tooltip(why ? why : "put the held cube down here (/place_cube); STOP cancels");
  } else {
    ImGui::TextColored(theme::DIM, "click the table inside the green ring to put %s down,\n"
      "or another cube to stack on it", held_name(ui).c_str());
    ImGui::PopID();
    return;
  }
  if (ui.sel.stack_on >= 0) {   // a stacked cube takes the lower cube's yaw (pick node)
    ImGui::TextColored(theme::DIM, "yaw: that of K%d", ui.sel.stack_on);
    ImGui::PopID();
    return;
  }
  // the cube's yaw on the table
  CubeSelection sel = ui.sel;
  bool changed = ImGui::Checkbox("any yaw", &sel.any_yaw);
  tooltip("let the pick node choose the yaw with the smallest wrist roll change");
  if (!sel.any_yaw) {
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150);
    changed |= ImGui::SliderFloat("yaw", &sel.yaw_deg, -45.0f, 45.0f, "%.0f deg");
  }
  if (changed) {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.selection.any_yaw = sel.any_yaw;
    state.selection.yaw_deg = sel.yaw_deg;
  }
  ImGui::PopID();
}

void draw_cube_section(State & state, TaskRunner & tasks)
{
  const CubeUi ui = cube_ui(state, tasks);
  ImGui::SeparatorText("CUBES");

  // pick node availability
  ImGui::TextColored(theme::DIM, "PICK NODE");
  ImGui::SameLine();
  if (ui.pick_online && ui.place_online) {
    ImGui::TextColored(theme::OK, "online");
  } else if (ui.pick_online || ui.place_online) {
    ImGui::TextColored(theme::AMBER, "%s only", ui.pick_online ? "/pick_cube" : "/place_cube");
  } else {
    ImGui::TextColored(theme::DIM, "offline");
  }
  tooltip("so101_manipulation's action servers /pick_cube and /place_cube\n"
    "(ros2 launch so101_manipulation pick_place.launch.py)");
  if (ui.node_state) {
    ImGui::SameLine();
    ImGui::TextColored(ui.node_busy ? theme::AMBER : theme::DIM, ui.node_busy ? " busy" : " idle");
  }

  // held cube
  ImGui::TextColored(theme::DIM, "HOLDING  ");
  ImGui::SameLine();
  if (holding(ui)) {
    if (ui.held >= 0) {
      ImGui::TextColored(theme::AMBER, "K%d", ui.held);
    } else {
      ImGui::TextColored(theme::AMBER, "%s", ui.holding.cube.c_str());
    }
    ImGui::SameLine();
    ImGui::TextColored(theme::DIM, "%s(%s)", ui.holding.uncertain ? "maybe " : "", ui.holding.source.c_str());
    if (!ui.node_state) {
      ImGui::SameLine();
      if (ImGui::SmallButton("forget")) {
        tasks.forget_holding();
      }
      tooltip("the dashboard forgets the held cube (nothing moves): after the cube was taken out by hand");
    }
  } else {
    ImGui::TextColored(theme::DIM, "nothing");
  }

  draw_cube_actions(state, tasks, ui, "tasks");

  // look at the table
  const bool look_ok = ui.look_online && !ui.running && !ui.limp;
  ImGui::BeginDisabled(!look_ok);
  if (ImGui::Button("LOOK", ImVec2(118, 0))) {
    tasks.start_look();
  }
  ImGui::EndDisabled();
  tooltip(!ui.look_online ? "no /cube_pick_place/overview service (this pick node does not offer it)" :
    ui.running ? "a task is running" : ui.limp ? "the torque is off" :
    "the arm to a pose where the wrist camera sees the table (/cube_pick_place/overview)");
}

}  // namespace so101_dashboard
