#include "panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>

#include <imgui.h>
#include <implot.h>

#include "theme.hpp"

namespace so101_dashboard
{

namespace
{

constexpr double PLOT_SECONDS = 10.0;
constexpr double STALE_S = 0.5;   // no message for this long: the link is down

double deg(double rad) {return rad * 180.0 / M_PI;}

constexpr double TEMP_WARN_C = 50.0;
constexpr double TEMP_HOT_C = 60.0;
constexpr double LOAD_WARN = 60.0;    // % of the torque limit
constexpr double VOLT_LOW = 11.0;     // 12 V supply

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

void draw_joints(State & state)
{
  enum Metric {POSITION, LOAD, CURRENT, TEMPERATURE};
  static int metric = POSITION;
  static const char * metric_names[] = {"position deg", "load %", "current mA", "temperature C"};
  if (!ImGui::Begin(WIN_JOINTS)) {
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
  const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
    ImGuiTableFlags_SizingStretchProp;
  if (ImGui::BeginTable("joints", 9, flags)) {
    ImGui::TableSetupColumn("JOINT", ImGuiTableColumnFlags_WidthFixed, 120);
    ImGui::TableSetupColumn("POS deg", ImGuiTableColumnFlags_WidthFixed, 70);
    ImGui::TableSetupColumn("VEL deg/s", ImGuiTableColumnFlags_WidthFixed, 80);
    ImGui::TableSetupColumn("LOAD %", ImGuiTableColumnFlags_WidthFixed, 64);
    ImGui::TableSetupColumn("##loadbar", ImGuiTableColumnFlags_WidthFixed, 110);
    ImGui::TableSetupColumn("CURRENT mA", ImGuiTableColumnFlags_WidthFixed, 90);
    ImGui::TableSetupColumn("TEMP C", ImGuiTableColumnFlags_WidthFixed, 60);
    ImGui::TableSetupColumn("VOLT V", ImGuiTableColumnFlags_WidthFixed, 60);
    ImGui::TableSetupColumn("##plot");
    // header row, with the plotted metric as a combo in the last column
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    for (int c = 0; c < 8; ++c) {
      ImGui::TableSetColumnIndex(c);
      ImGui::TableHeader(ImGui::TableGetColumnName(c));
    }
    ImGui::TableSetColumnIndex(8);
    ImGui::TextColored(theme::DIM, "LAST 10 s:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(170);
    ImGui::Combo("##metric", &metric, metric_names, IM_ARRAYSIZE(metric_names));

    for (const auto & name : state.joint_order) {
      const Joint & j = state.joints.at(name);
      ImGui::TableNextRow(ImGuiTableRowFlags_None, 34);
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
      ImGui::TableNextColumn();
      const History & hist = metric == LOAD ? j.load_hist : metric == CURRENT ? j.current_hist :
        metric == TEMPERATURE ? j.temperature_hist : j.position_hist;
      hist.ordered(t, v);
      ImGui::PushID(name.c_str());
      if (ImPlot::BeginPlot("##spark", ImVec2(-1, 30), ImPlotFlags_CanvasOnly | ImPlotFlags_NoInputs)) {
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations,
          ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, now - PLOT_SECONDS, now, ImPlotCond_Always);
        ImPlotSpec spec;
        spec.LineColor = metric == POSITION ? theme::CYAN : metric == TEMPERATURE ? theme::ROSE : theme::AMBER;
        spec.LineWeight = 1.5f;
        ImPlot::PlotLine("##p", t.data(), v.data(), static_cast<int>(t.size()), spec);
        ImPlot::EndPlot();
      }
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

void draw_placeholder(const char * window, const char * text)
{
  if (ImGui::Begin(window)) {
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 size = ImGui::CalcTextSize(text);
    ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + (avail.x - size.x) * 0.5f,
      ImGui::GetCursorPosY() + (avail.y - size.y) * 0.5f));
    ImGui::TextColored(theme::DIM, "%s", text);
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
  double hz, last;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    hz = state.joint_states_rate.hz(now);
    last = state.joint_states_rate.last();
  }
  const bool link = last >= 0 && now - last < STALE_S;
  const bool blink = static_cast<int>(now * 2) % 2 == 0;

  ImGui::TextColored(theme::CYAN, "SO-101");
  ImGui::SameLine();
  ImGui::TextColored(theme::DIM, "// MISSION CONTROL");
  ImGui::SameLine(0, 30);
  ImGui::TextColored(link ? theme::OK : (blink ? theme::ALERT : theme::DIM), link ? "* LINK" : "* NO LINK");
  ImGui::SameLine(0, 30);
  char buf[64];
  snprintf(buf, sizeof(buf), "%5.1f Hz", hz);
  label_value("JOINT_STATES", buf, hz >= 90 ? theme::OK : hz > 0 ? theme::AMBER : theme::DIM);
  ImGui::SameLine(0, 30);
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
