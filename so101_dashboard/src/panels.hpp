// ImGui panels. Each reads State under its mutex and draws one window; `open` is its close button.
#pragma once

#include "ros_link.hpp"

namespace so101_dashboard
{

constexpr const char * WIN_VIEW = "3D VIEW";
constexpr const char * WIN_CAMERA = "CAMERA";
constexpr const char * WIN_TASKS = "TASKS";
constexpr const char * WIN_TASK_LOG = "TASK LOG";
constexpr const char * WIN_JOINTS = "JOINTS";
constexpr const char * WIN_PLOTS = "PLOTS";

class TaskRunner;

void draw_joints(State & state, bool * open);
void draw_plots(State & state, bool * open);
void draw_tasks(State & state, TaskRunner & tasks, bool * open);
void draw_task_log(State & state, bool * open);
// Bottom strip across the whole window.
void draw_status_bar(State & state, float height);

}  // namespace so101_dashboard
