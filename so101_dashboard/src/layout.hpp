// Window layout: which panels are open (View menu, F1..F6, saved in imgui.ini), the dock space
// and its default arrangement.
#pragma once

#include <vector>

#include <imgui.h>

namespace so101_dashboard
{

class Layout
{
public:
  struct Panel
  {
    const char * name;
    ImGuiKey key;    // toggles it
    bool open;
  };

  Layout();   // after the ImGui context exists, before the first frame (registers the ini handler)

  // Menu bar, shortcuts and dock space; call once per frame before the panels.
  void begin_frame(float status_bar_height);
  // nullptr for an unknown name; otherwise pass it to ImGui::Begin as p_open.
  bool * open(const char * name);

private:
  void menu_bar();
  void dock_space(float status_bar_height);

  std::vector<Panel> panels_;
  bool reset_ = false;
};

}  // namespace so101_dashboard
