#include "theme.hpp"

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <implot.h>

namespace so101_dashboard::theme
{

void load_fonts()
{
  const std::string path = ament_index_cpp::get_package_share_directory("so101_dashboard") +
    "/fonts/ShareTechMono-Regular.ttf";
  ImFontConfig config;
  config.PixelSnapH = true;
  config.OversampleH = 2;
  ImGuiIO & io = ImGui::GetIO();
  if (!io.Fonts->AddFontFromFileTTF(path.c_str(), FONT_SIZE, &config)) {
    io.Fonts->AddFontDefault();
  }
}

void apply_style()
{
  ImGuiStyle & s = ImGui::GetStyle();
  s.WindowRounding = 2.0f;
  s.ChildRounding = 2.0f;
  s.FrameRounding = 2.0f;
  s.TabRounding = 1.0f;
  s.GrabRounding = 1.0f;
  s.ScrollbarRounding = 1.0f;
  s.WindowBorderSize = 1.0f;
  s.FrameBorderSize = 1.0f;
  s.TabBorderSize = 1.0f;
  s.WindowPadding = ImVec2(10, 8);
  s.FramePadding = ImVec2(8, 4);
  s.ItemSpacing = ImVec2(8, 6);
  s.CellPadding = ImVec2(6, 3);
  s.WindowTitleAlign = ImVec2(0.0f, 0.5f);

  ImVec4 * c = s.Colors;
  auto alpha = [](ImVec4 v, float a) {v.w = a; return v;};
  c[ImGuiCol_Text] = TEXT;
  c[ImGuiCol_TextDisabled] = DIM;
  c[ImGuiCol_WindowBg] = PANEL;
  c[ImGuiCol_ChildBg] = alpha(PANEL, 0.0f);
  c[ImGuiCol_PopupBg] = PANEL;
  c[ImGuiCol_Border] = BORDER;
  c[ImGuiCol_BorderShadow] = alpha(BG, 0.0f);
  c[ImGuiCol_FrameBg] = alpha(CYAN, 0.05f);
  c[ImGuiCol_FrameBgHovered] = alpha(CYAN, 0.12f);
  c[ImGuiCol_FrameBgActive] = alpha(CYAN, 0.2f);
  c[ImGuiCol_TitleBg] = BG;
  c[ImGuiCol_TitleBgActive] = BG;
  c[ImGuiCol_TitleBgCollapsed] = BG;
  c[ImGuiCol_MenuBarBg] = BG;
  c[ImGuiCol_ScrollbarBg] = alpha(BG, 0.5f);
  c[ImGuiCol_ScrollbarGrab] = BORDER;
  c[ImGuiCol_ScrollbarGrabHovered] = alpha(CYAN, 0.5f);
  c[ImGuiCol_ScrollbarGrabActive] = CYAN;
  c[ImGuiCol_CheckMark] = CYAN;
  c[ImGuiCol_SliderGrab] = alpha(CYAN, 0.7f);
  c[ImGuiCol_SliderGrabActive] = CYAN;
  c[ImGuiCol_Button] = alpha(CYAN, 0.08f);
  c[ImGuiCol_ButtonHovered] = alpha(CYAN, 0.22f);
  c[ImGuiCol_ButtonActive] = alpha(CYAN, 0.4f);
  c[ImGuiCol_Header] = alpha(CYAN, 0.1f);
  c[ImGuiCol_HeaderHovered] = alpha(CYAN, 0.2f);
  c[ImGuiCol_HeaderActive] = alpha(CYAN, 0.3f);
  c[ImGuiCol_Separator] = BORDER;
  c[ImGuiCol_ResizeGrip] = alpha(CYAN, 0.1f);
  c[ImGuiCol_ResizeGripHovered] = alpha(CYAN, 0.4f);
  c[ImGuiCol_ResizeGripActive] = CYAN;
  c[ImGuiCol_Tab] = BG;
  c[ImGuiCol_TabHovered] = alpha(CYAN, 0.25f);
  c[ImGuiCol_TabSelected] = alpha(CYAN, 0.14f);
  c[ImGuiCol_TabSelectedOverline] = CYAN;
  c[ImGuiCol_TabDimmed] = BG;
  c[ImGuiCol_TabDimmedSelected] = alpha(CYAN, 0.08f);
  c[ImGuiCol_TabDimmedSelectedOverline] = alpha(CYAN, 0.4f);
  c[ImGuiCol_DockingPreview] = alpha(CYAN, 0.3f);
  c[ImGuiCol_DockingEmptyBg] = BG;
  c[ImGuiCol_PlotLines] = CYAN;
  c[ImGuiCol_PlotHistogram] = AMBER;
  c[ImGuiCol_TableHeaderBg] = alpha(CYAN, 0.06f);
  c[ImGuiCol_TableBorderStrong] = BORDER;
  c[ImGuiCol_TableBorderLight] = alpha(BORDER, 0.5f);
  c[ImGuiCol_TableRowBg] = alpha(BG, 0.0f);
  c[ImGuiCol_TableRowBgAlt] = alpha(CYAN, 0.025f);
  c[ImGuiCol_TextSelectedBg] = alpha(CYAN, 0.3f);
  c[ImGuiCol_NavCursor] = CYAN;

  ImPlotStyle & p = ImPlot::GetStyle();
  p.PlotPadding = ImVec2(4, 4);
  ImVec4 * pc = p.Colors;
  pc[ImPlotCol_FrameBg] = alpha(BG, 0.0f);
  pc[ImPlotCol_PlotBg] = alpha(BG, 0.6f);
  pc[ImPlotCol_PlotBorder] = BORDER;
  pc[ImPlotCol_LegendBg] = PANEL;
  pc[ImPlotCol_LegendBorder] = BORDER;
  pc[ImPlotCol_AxisGrid] = alpha(BORDER, 0.6f);
  pc[ImPlotCol_AxisText] = DIM;
}

}  // namespace so101_dashboard::theme
