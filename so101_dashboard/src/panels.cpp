#include "panels.hpp"

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

void label_value(const char * label, const char * value, const ImVec4 & colour)
{
  ImGui::TextColored(theme::DIM, "%s", label);
  ImGui::SameLine();
  ImGui::TextColored(colour, "%s", value);
}

}  // namespace

void draw_joints(State & state)
{
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
  if (ImGui::BeginTable("joints", 4, flags)) {
    ImGui::TableSetupColumn("JOINT", ImGuiTableColumnFlags_WidthFixed, 130);
    ImGui::TableSetupColumn("POS deg", ImGuiTableColumnFlags_WidthFixed, 80);
    ImGui::TableSetupColumn("VEL deg/s", ImGuiTableColumnFlags_WidthFixed, 80);
    ImGui::TableSetupColumn("LAST 10 s");
    ImGui::TableHeadersRow();
    for (const auto & name : state.joint_order) {
      const Joint & j = state.joints.at(name);
      ImGui::TableNextRow(ImGuiTableRowFlags_None, 34);
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(name.c_str());
      ImGui::TableNextColumn();
      ImGui::TextColored(theme::CYAN, "%+7.1f", deg(j.position));
      ImGui::TableNextColumn();
      const double vel = deg(j.velocity);
      ImGui::TextColored(std::abs(vel) > 1.0 ? theme::AMBER : theme::DIM, "%+7.1f", vel);
      ImGui::TableNextColumn();
      j.position_hist.ordered(t, v);
      ImGui::PushID(name.c_str());
      if (ImPlot::BeginPlot("##spark", ImVec2(-1, 30), ImPlotFlags_CanvasOnly | ImPlotFlags_NoInputs)) {
        ImPlot::SetupAxes(nullptr, nullptr, ImPlotAxisFlags_NoDecorations,
          ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, now - PLOT_SECONDS, now, ImPlotCond_Always);
        ImPlotSpec spec;
        spec.LineColor = theme::CYAN;
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
