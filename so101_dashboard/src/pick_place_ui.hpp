// Cube pick and place from the GUI: the buttons shared by the 3D view's popup and the TASKS panel,
// and the checks behind them. The selection itself is made by clicking in the 3D view (robot_view).
#pragma once

#include <string>

#include "ros_link.hpp"

namespace so101_dashboard
{

class TaskRunner;

// Where picking and placing work, measured like the pick node does: the distance of the cube centre
// from the base_link z axis. From the pick node's reach table (/cube_pick_place/reach) when it
// publishes one; else this fixed ring (vertical jaws, older pick nodes).
constexpr float REACH_MIN_M = 0.15f;
constexpr float REACH_MAX_M = 0.29f;

// Which band of the reach table: where some / every cube yaw can be picked, or where a held cube
// can be put down (vertical jaws; a table without place columns gives the some-yaw pick band).
enum class Reach {SomeYaw, EveryYaw, Place};

// The reach band at `pan_deg` (atan2(y, x) in base_link), interpolated between the table's pans.
// False where nothing reaches (outside the table's pans, or NaN). Without a table: the fixed ring
// at every pan.
bool reach_band(const ReachTable & table, float pan_deg, Reach kind, float & near, float & far);
bool has_place_band(const ReachTable & table);

// Everything the buttons depend on, read once per frame.
struct CubeUi
{
  bool running = false;          // any task
  bool limp = false;             // torque off
  bool pick_online = false;      // /pick_cube
  bool place_online = false;     // /place_cube
  bool look_online = false;      // /cube_pick_place/overview
  bool node_state = false;       // the pick node publishes its own state topic
  bool node_busy = false;
  Holding holding;
  int held = -1;                 // held cube index, -1: none or unknown
  CubeSelection sel;
  ReachTable reach;              // the pick node's reach table, empty: the fixed ring
  bool selected_seen = false;    // the selected cube is in /cubes/markers or /cubes/tracked
  float selected_x = 0.0f, selected_y = 0.0f;
};

CubeUi cube_ui(State & state, TaskRunner & tasks);
bool holding(const CubeUi & ui);
std::string held_name(const CubeUi & ui);   // "K2", or "the cube" when the node did not say which
// Empty when (x, y) is within reach (the table's some-yaw pick band, the place band for a place,
// or the fixed ring); else why not.
std::string reach_problem(const ReachTable & table, float x, float y, bool place = false);

// The actions behind the buttons (and the test hook). Each returns false when it did not start.
bool pick_selected(State & state, TaskRunner & tasks);
bool place_at_target(State & state, TaskRunner & tasks);
bool stack_on_target(State & state, TaskRunner & tasks);

// The controls for the present selection: PICK K<n>, or PLACE HERE / STACK ON with the yaw.
// `id` keeps the widgets of the popup and of the TASKS panel apart.
void draw_cube_actions(State & state, TaskRunner & tasks, const CubeUi & ui, const char * id);
// The CUBES section of the TASKS panel: pick node, holding, selection, actions, LOOK.
void draw_cube_section(State & state, TaskRunner & tasks);

}  // namespace so101_dashboard
