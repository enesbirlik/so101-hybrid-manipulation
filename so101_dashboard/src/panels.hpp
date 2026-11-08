// ImGui panels. Each reads State under its mutex and draws one window.
#pragma once

#include "ros_link.hpp"

namespace so101_dashboard
{

constexpr const char * WIN_VIEW = "3D VIEW";
constexpr const char * WIN_TASKS = "TASKS";
constexpr const char * WIN_CAMERA = "CAMERA";
constexpr const char * WIN_JOINTS = "JOINTS";

void draw_joints(State & state);
void draw_placeholder(const char * window, const char * text);
// Bottom strip across the whole window.
void draw_status_bar(State & state, float height);

}  // namespace so101_dashboard
