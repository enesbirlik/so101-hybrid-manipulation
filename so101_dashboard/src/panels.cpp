#include "panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include <imgui.h>
#include <implot.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include "tasks.hpp"
#include "theme.hpp"

namespace so101_dashboard
{

namespace
{

constexpr double STALE_S = 0.5;       // no message for this long: the link is down
constexpr double TEMP_WARN_C = 50.0;
constexpr double TEMP_HOT_C = 60.0;
constexpr double LOAD_WARN = 60.0;    // % of the torque limit
constexpr double VOLT_LOW = 11.0;     // 12 V supply
constexpr double TREND_SECONDS = 10.0;

// What the plots (and the table's trend column) show.
enum Metric {POSITION, LOAD, CURRENT, TEMPERATURE};
const char * METRIC_NAMES[] = {"position deg", "load %", "current mA", "temperature C"};
int metric = POSITION;

double deg(double rad) {return rad * 180.0 / M_PI;}

const History & history(const Joint & j, int m)
{
  return m == LOAD ? j.load_hist : m == CURRENT ? j.current_hist :
         m == TEMPERATURE ? j.temperature_hist : j.position_hist;
}

// One colour per joint, base to gripper.
const ImVec4 & joint_colour(size_t i)
{
  static const ImVec4 colours[] = {theme::CYAN, theme::AMBER, theme::ROSE, theme::OK,
    ImVec4(0.62f, 0.55f, 1.0f, 1.0f), theme::TEXT};
  return colours[i % IM_ARRAYSIZE(colours)];
}

void label_value(const char * label, const char * value, const ImVec4 & colour)
{
  ImGui::TextColored(theme::DIM, "%s", label);
  ImGui::SameLine();
  ImGui::TextColored(colour, "%s", value);
}

// A number, or a dim dash without data.
void cell(double v, const char * fmt, const ImVec4 & colour)
{
  if (std::isfinite(v)) {
    ImGui::TextColored(colour, fmt, v);
  } else {
    ImGui::TextColored(theme::DIM, "     -");
  }
}

// Signed horizontal bar centred on zero, |v| up to 100.
void load_bar(double v)
{
  const ImVec2 p = ImGui::GetCursorScreenPos();
  const float w = ImGui::GetContentRegionAvail().x, h = 8.0f;
  const float y = p.y + ImGui::GetTextLineHeight() / 2 - h / 2 + 2;
  ImDrawList * dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + w, y + h), ImGui::GetColorU32(ImVec4(1, 1, 1, 0.04f)));
  dl->AddLine(ImVec2(p.x + w / 2, y - 2), ImVec2(p.x + w / 2, y + h + 2), ImGui::GetColorU32(theme::BORDER));
  if (std::isfinite(v)) {
    const float x = p.x + w / 2 + static_cast<float>(std::clamp(v, -100.0, 100.0) / 100.0) * w / 2;
    dl->AddRectFilled(ImVec2(std::min(x, p.x + w / 2), y), ImVec2(std::max(x, p.x + w / 2), y + h),
      ImGui::GetColorU32(std::abs(v) > LOAD_WARN ? theme::AMBER : theme::CYAN));
  }
  ImGui::Dummy(ImVec2(w, ImGui::GetTextLineHeight()));
}

const ImVec4 & temperature_colour(double c)
{
  return c > TEMP_HOT_C ? theme::ALERT : c > TEMP_WARN_C ? theme::AMBER : theme::TEXT;
}

}  // namespace

void draw_joints(State & state, bool * open)
{
  if (!ImGui::Begin(WIN_JOINTS, open)) {
    ImGui::End();
    return;
  }
  std::vector<double> t, v;
  const double now = now_s();
  std::lock_guard<std::mutex> lock(state.mutex);
  if (state.joint_order.empty()) {
    ImGui::TextColored(theme::DIM, "waiting for /joint_states ...");
    ImGui::End();
    return;
  }
  // right-click a header to hide or show columns; widths and order are saved with the layout
  const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_Hideable |
    ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_SizingFixedFit |
    ImGuiTableFlags_ScrollY;
  if (ImGui::BeginTable("joints", 9, flags)) {
    ImGui::TableSetupScrollFreeze(1, 1);
    ImGui::TableSetupColumn("JOINT", ImGuiTableColumnFlags_NoHide, 120);
    ImGui::TableSetupColumn("POS deg", 0, 70);
    ImGui::TableSetupColumn("VEL deg/s", 0, 80);
    ImGui::TableSetupColumn("LOAD %", 0, 64);
    ImGui::TableSetupColumn("LOAD", 0, 110);
    ImGui::TableSetupColumn("CURRENT mA", 0, 90);
    ImGui::TableSetupColumn("TEMP C", 0, 60);
    ImGui::TableSetupColumn("VOLT V", 0, 60);
    ImGui::TableSetupColumn("TREND", ImGuiTableColumnFlags_DefaultHide | ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();
    for (const auto & name : state.joint_order) {
      const Joint & j = state.joints.at(name);
      ImGui::TableNextRow(ImGuiTableRowFlags_None, 28);
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(name.c_str());
      ImGui::TableNextColumn();
      cell(deg(j.position), "%+7.1f", theme::CYAN);
      ImGui::TableNextColumn();
      const double vel = deg(j.velocity);
      cell(vel, "%+7.1f", std::abs(vel) > 1.0 ? theme::AMBER : theme::DIM);
      ImGui::TableNextColumn();
      cell(j.load, "%+6.1f", std::abs(j.load) > LOAD_WARN ? theme::AMBER : theme::TEXT);
      ImGui::TableNextColumn();
      load_bar(j.load);
      ImGui::TableNextColumn();
      cell(j.current * 1000.0, "%6.0f", theme::TEXT);
      ImGui::TableNextColumn();
      cell(j.temperature, "%5.0f", temperature_colour(j.temperature));
      ImGui::TableNextColumn();
      cell(j.voltage, "%5.1f", j.voltage < VOLT_LOW ? theme::AMBER : theme::TEXT);
      if (ImGui::TableNextColumn()) {   // only when the trend column is shown
        history(j, metric).ordered(t, v);
        ImGui::PushID(name.c_str());
        if (ImPlot::BeginPlot("##trend", ImVec2(-1, 24), ImPlotFlags_CanvasOnly | ImPlotFlags_NoInputs)) {
          ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations,
            ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit);
          ImPlot::SetupAxisLimits(ImAxis_X1, now - TREND_SECONDS, now, ImPlotCond_Always);
          ImPlotSpec spec;
          spec.LineColor = theme::CYAN;
          spec.LineWeight = 1.5f;
          ImPlot::PlotLine("##p", t.data(), v.data(), static_cast<int>(t.size()), spec);
          ImPlot::EndPlot();
        }
        ImGui::PopID();
      }
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

void draw_plots(State & state, bool * open)
{
  static int window = 1;   // index into SPANS
  static const double SPANS[] = {10.0, 30.0, 60.0};
  static bool paused = false;
  static double frozen_at = 0.0;
  if (!ImGui::Begin(WIN_PLOTS, open)) {
    ImGui::End();
    return;
  }
  ImGui::SetNextItemWidth(170);
  ImGui::Combo("##metric", &metric, METRIC_NAMES, IM_ARRAYSIZE(METRIC_NAMES));
  for (int i = 0; i < IM_ARRAYSIZE(SPANS); ++i) {
    ImGui::SameLine();
    char label[16];
    snprintf(label, sizeof(label), "%.0f s", SPANS[i]);
    ImGui::RadioButton(label, &window, i);
  }
  ImGui::SameLine(0, 20);
  if (ImGui::Checkbox("pause", &paused) && paused) {
    frozen_at = now_s();
  }
  const double now = paused ? frozen_at : now_s();
  const double span = SPANS[window];

  std::vector<std::string> names;
  std::vector<std::vector<double>> ts, vs;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    for (const auto & name : state.joint_order) {
      names.push_back(name);
      ts.emplace_back();
      vs.emplace_back();
      history(state.joints.at(name), metric).ordered(ts.back(), vs.back());
      for (double & t : ts.back()) {
        t -= now;   // seconds before now
      }
    }
  }
  // click a joint in the legend to hide it
  if (ImPlot::BeginPlot("##plots", ImVec2(-1, -1), ImPlotFlags_NoTitle)) {
    ImPlot::SetupAxes("s", METRIC_NAMES[metric], 0, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxisLimits(ImAxis_X1, -span, 0, ImPlotCond_Always);
    ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
    for (size_t i = 0; i < names.size(); ++i) {
      ImPlotSpec spec;
      spec.LineColor = joint_colour(i);
      spec.LineWeight = 1.5f;
      ImPlot::PlotLine(names[i].c_str(), ts[i].data(), vs[i].data(), static_cast<int>(ts[i].size()), spec);
    }
    ImPlot::EndPlot();
  }
  ImGui::End();
}

void draw_tasks(State & state, TaskRunner & tasks, bool * open)
{
  struct Planner {const char * label, * pipeline, * planner;};
  static const Planner planners[] = {
    {"Pilz PTP", "pilz_industrial_motion_planner", "PTP"}, {"OMPL RRTConnect", "ompl", "RRTConnect"}};
  static int planner = 0;
  static MotionOptions options;
  if (!ImGui::Begin(WIN_TASKS, open)) {
    ImGui::End();
    return;
  }
  TaskStatus task;
  double torque;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    task = state.task;
    torque = state.torque;
  }
  const double now = now_s();
  const bool limp = torque == 0.0;   // NAN (no torque switch) counts as on

  // status line + STOP
  ImGui::AlignTextToFramePadding();
  ImGui::TextColored(theme::DIM, "STATUS");
  ImGui::SameLine();
  if (task.running) {
    const bool blink = static_cast<int>(now * 2) % 2 == 0;
    ImGui::TextColored(blink ? theme::AMBER : theme::DIM, "*");
    ImGui::SameLine();
    ImGui::TextColored(theme::AMBER, "%s", task.label.c_str());
    ImGui::SameLine();
    ImGui::TextColored(theme::DIM, "%s  %.1f s", task.phase.c_str(), now - task.started);
  } else {
    ImGui::TextColored(theme::OK, "IDLE");
  }
  const float stop_w = 110;
  ImGui::SameLine(ImGui::GetWindowWidth() - stop_w - 12);
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(theme::ALERT.x, theme::ALERT.y, theme::ALERT.z, 0.25f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(theme::ALERT.x, theme::ALERT.y, theme::ALERT.z, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, theme::ALERT);
  ImGui::PushStyleColor(ImGuiCol_Border, theme::ALERT);
  if (ImGui::Button("STOP", ImVec2(stop_w, 0))) {
    tasks.stop();
  }
  ImGui::PopStyleColor(4);

  // torque: switching it off takes a second click within CONFIRM_S, because the arm drops
  static double confirm_until = 0.0;
  constexpr double CONFIRM_S = 3.0;
  ImGui::AlignTextToFramePadding();
  ImGui::TextColored(theme::DIM, "TORQUE");
  ImGui::SameLine();
  if (std::isnan(torque)) {
    ImGui::TextColored(theme::DIM, "?");
  } else {
    ImGui::TextColored(limp ? theme::AMBER : theme::OK, limp ? "OFF  (the arm moves freely)" : "ON");
  }
  ImGui::SameLine(ImGui::GetWindowWidth() - 260 - 12);
  ImGui::BeginDisabled(std::isnan(torque));
  if (limp) {
    if (ImGui::Button("TORQUE ON", ImVec2(260, 0))) {
      tasks.set_torque(true);
    }
    ImGui::SetItemTooltip("holds the present pose; the controllers restart");
  } else {
    const bool armed = now < confirm_until;
    if (armed) {
      ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(theme::AMBER.x, theme::AMBER.y, theme::AMBER.z, 0.35f));
      ImGui::PushStyleColor(ImGuiCol_Border, theme::AMBER);
    }
    if (ImGui::Button(armed ? "HOLD THE ARM - CLICK AGAIN" : "TORQUE OFF", ImVec2(260, 0))) {
      if (armed) {
        tasks.set_torque(false);
        confirm_until = 0.0;
      } else {
        confirm_until = now + CONFIRM_S;
      }
    }
    if (armed) {
      ImGui::PopStyleColor(2);
    }
    ImGui::SetItemTooltip("all servos go limp: support the arm first");
  }
  ImGui::EndDisabled();

  // task buttons
  struct TaskButton {const char * label, * group, * state, * tip;};
  static const TaskButton buttons[] = {
    {"ZERO", "arm", "zero", "all joints to 0"},
    {"REST", "arm", "rest", "fold into the rest pose"},
    {"OPEN", "gripper", "open", "open the gripper"},
    {"CLOSE", "gripper", "closed", "close the gripper"}};
  const bool online = tasks.move_group_online();
  const ImVec2 size(118, 40);
  ImGui::BeginDisabled(task.running || limp);
  if (ImGui::Button("UNFOLD", size)) {
    static const std::string script =
      ament_index_cpp::get_package_prefix("so101_bringup") + "/lib/so101_bringup/so101_ready.py";
    tasks.start_process("UNFOLD", {script});
  }
  ImGui::SetItemTooltip("out of the folded rest, elbow first (so101_ready.py)");
  for (const auto & b : buttons) {
    // wrap onto the next line when the panel is narrow
    if (ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x + size.x <
      ImGui::GetWindowPos().x + ImGui::GetContentRegionMax().x)
    {
      ImGui::SameLine();
    }
    ImGui::BeginDisabled(!online);
    if (ImGui::Button(b.label, size)) {
      tasks.start_named(b.label, b.group, b.state, options);
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("%s (%s/%s)", b.tip, b.group, b.state);
  }
  ImGui::EndDisabled();

  // motion settings, folded away by default
  ImGui::Spacing();
  if (ImGui::CollapsingHeader("motion settings")) {
    ImGui::SetNextItemWidth(170);
    if (ImGui::Combo("planner", &planner, [](void *, int i) {return planners[i].label;}, nullptr,
      IM_ARRAYSIZE(planners)))
    {
      options.pipeline = planners[planner].pipeline;
      options.planner = planners[planner].planner;
    }
    float vel = static_cast<float>(options.velocity), acc = static_cast<float>(options.acceleration);
    ImGui::SetNextItemWidth(170);
    if (ImGui::SliderFloat("velocity", &vel, 0.1f, 1.0f, "%.2f")) {
      options.velocity = vel;
    }
    ImGui::SetNextItemWidth(170);
    if (ImGui::SliderFloat("acceleration", &acc, 0.1f, 1.0f, "%.2f")) {
      options.acceleration = acc;
    }
  }
  ImGui::TextColored(online ? theme::OK : theme::DIM, online ? "move_group online" : "move_group offline");
  ImGui::End();
}

void draw_task_log(State & state, bool * open)
{
  if (!ImGui::Begin(WIN_TASK_LOG, open)) {
    ImGui::End();
    return;
  }
  std::deque<TaskLogEntry> log;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    log = state.task_log;
  }
  if (ImGui::SmallButton("clear")) {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.task_log.clear();
  }
  if (ImGui::BeginTable("log", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("TIME", ImGuiTableColumnFlags_WidthFixed, 80);
    ImGui::TableSetupColumn("TASK", ImGuiTableColumnFlags_WidthFixed, 90);
    ImGui::TableSetupColumn("RESULT");
    ImGui::TableSetupColumn("TIME s", ImGuiTableColumnFlags_WidthFixed, 60);
    ImGui::TableHeadersRow();
    for (const auto & e : log) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextColored(theme::DIM, "%s", e.clock.c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(e.label.c_str());
      ImGui::TableNextColumn();
      ImGui::TextColored(e.ok ? theme::OK : theme::ALERT, "%s", e.result.c_str());
      ImGui::TableNextColumn();
      ImGui::Text("%6.1f", e.duration);
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

void draw_status_bar(State & state, float height)
{
  const ImGuiViewport * vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - height));
  ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, height));
  ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::BG);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::Begin("##status", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoDocking |
    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
  const double now = now_s();
  double hz, last, camera_hz, torque;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    torque = state.torque;
    hz = state.joint_states_rate.hz(now);
    last = state.joint_states_rate.last();
    camera_hz = state.image_rate.hz(now);
  }
  const bool link = last >= 0 && now - last < STALE_S;
  const bool blink = static_cast<int>(now * 2) % 2 == 0;

  ImGui::TextColored(link ? theme::OK : (blink ? theme::ALERT : theme::DIM), link ? "* LINK" : "* NO LINK");
  ImGui::SameLine(0, 30);
  char buf[64];
  snprintf(buf, sizeof(buf), "%5.1f Hz", hz);
  label_value("JOINT_STATES", buf, hz >= 90 ? theme::OK : hz > 0 ? theme::AMBER : theme::DIM);
  ImGui::SameLine(0, 30);
  snprintf(buf, sizeof(buf), "%4.1f fps", camera_hz);
  label_value("CAMERA", buf, camera_hz > 10 ? theme::OK : camera_hz > 0 ? theme::AMBER : theme::DIM);
  ImGui::SameLine(0, 30);
  if (!std::isnan(torque)) {
    label_value("TORQUE", torque == 0.0 ? "OFF" : "ON", torque == 0.0 ? theme::AMBER : theme::OK);
    ImGui::SameLine(0, 30);
  }
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  label_value("DOMAIN", domain ? domain : "0", theme::TEXT);

  const std::time_t wall = std::time(nullptr);
  std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&wall));
  ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(buf).x - 14);
  ImGui::TextColored(theme::AMBER, "%s", buf);
  ImGui::End();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor();
}

}  // namespace so101_dashboard
