// 3D view: the robot from its URDF, posed by forward kinematics from the joint states,
// drawn with a rim-light ("hologram") shader into a render texture shown in an ImGui window.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <imgui.h>
#include <raylib.h>
#include <urdf/model.h>

#include "ros_link.hpp"

namespace so101_dashboard
{

using JointValues = std::map<std::string, double>;
using LinkPoses = std::map<std::string, Matrix>;

class TaskRunner;

class RobotView
{
public:
  RobotView();   // after InitWindow: loads the shader
  ~RobotView();  // before CloseWindow

  // Draws the "3D VIEW" window. Reloads the model when a new robot description arrives.
  // A click (no drag) selects a cube, or, while a cube is held, the place target.
  void draw(State & state, TaskRunner & tasks, bool * open);

  // World transforms (frame = URDF root) of every link for the given joint values.
  LinkPoses forward_kinematics(const JointValues & q) const;

private:
  struct Visual
  {
    std::string link;
    Matrix origin;      // visual origin in the link frame, mesh scale included
    int mesh = -1;      // index into meshes_
    bool table = false; // drawn as a grid, not as a solid
  };

  void load_model(const std::string & xml);
  int load_mesh(const std::string & uri);
  int primitive(const std::string & key, Mesh mesh);
  void handle_input(bool hovered, bool active);
  void update_camera();
  void render(const LinkPoses & poses, const LinkPoses & ghost);
  // The last plan: link poses at its goal (empty when there is none to show); fills path_.
  LinkPoses ghost_poses(const Ghost & ghost, const JointValues & q, bool task_running);
  void draw_robot(const LinkPoses & poses, Color base, Color rim);
  void draw_table(const LinkPoses & poses);
  // Soft shadows on the table under the cubes and the gripper (depth writes off).
  void draw_shadows(const LinkPoses & poses) const;
  void draw_labels(const LinkPoses & poses, const JointValues & q, ImVec2 origin, float scale) const;
  void draw_table_labels(ImVec2 origin) const;   // the distance rings' "10 cm" ...
  void check_fk(State & state, const LinkPoses & poses);
  void update_fov(State & state);   // camera field of view, from its marker and TF
  // Cube states (live / remembered / held / placed) and their smoothed poses, from the copied inputs.
  void update_cubes(const LinkPoses & poses);
  // The cube between the jaws (centre and orientation) from the gripper links; false without them.
  bool jaw_cube_pose(const LinkPoses & poses, float edge, Matrix & out) const;
  void draw_cubes(bool translucent) const;
  void draw_cube_labels(ImVec2 origin, float scale) const;
  // Pick and place: the reach ring, the selection and the place target (3D) ...
  void draw_pick_place_3d() const;
  // ... and the popup next to it with the buttons.
  void draw_pick_place_popup(State & state, TaskRunner & tasks, ImVec2 origin, ImVec2 size);
  // A click at `px` (render texture pixels): selects a cube or a place target. Returns what it did.
  std::string click(State & state, Vector2 px);
  // SO101_DASHBOARD_TEST=1: GUI actions from /so101_dashboard/test.
  void run_test_commands(State & state, TaskRunner & tasks);
  bool screen_of(Vector3 p, ImVec2 origin, ImVec2 & out) const;   // false when behind the camera

  std::shared_ptr<urdf::Model> model_;
  uint64_t description_version_ = 0;
  std::vector<Visual> visuals_;
  std::vector<Mesh> meshes_;
  std::map<std::string, int> mesh_cache_;
  Vector3 table_size_{0, 0, 0};

  Shader shader_{};
  int loc_view_pos_ = -1, loc_base_ = -1, loc_rim_ = -1;
  Material material_{};
  RenderTexture2D target_{};
  int target_w_ = 0, target_h_ = 0;

  Camera3D camera_{};
  float yaw_ = -0.75f, pitch_ = 0.32f, distance_ = 0.8f;
  Vector3 focus_{0.12f, 0.0f, 0.1f};
  bool scanlines_ = true;
  bool labels_ = true;

  double fk_error_mm_ = -1.0;   // FK vs TF at the gripper frame, -1 = not checked yet
  double fk_checked_at_ = -10.0;
  // A cube as drawn. Its state decides the style:
  enum class CubeState {Live, Remembered, Held, Placed};
  struct ViewCube
  {
    int index = -1;
    CubeMarker marker;            // the displayed (smoothed) pose; for clicks and labels
    CubeState state = CubeState::Live;
    bool stable = true;
    double age = 0.0;             // s since the detector last saw it
    Matrix pose{};                // centre and orientation (held: tilted with the jaws)
    float alpha = 1.0f;           // smoothed opacity, 0..1
  };
  // Per cube, kept across frames: the smoothed pose and opacity.
  struct CubeTrack
  {
    Vector3 pos{};
    float yaw = 0.0f;
    float alpha = 0.0f;
    double held_since = -1.0;     // while held: when it went into the jaws (the slide there takes 150 ms)
  };
  // A cube put down by a place: shown there until the detector confirms (or contradicts) it.
  struct Placed
  {
    Vector3 pos{};
    float yaw = 0.0f;
    double at = 0.0;              // now_s() of the release
    double disagree_since = -1.0; // fresh detections elsewhere since then
  };
  std::vector<Vector3> fov_world_;   // line segment end points
  std::vector<ViewCube> cubes_;      // in the model root frame, rebuilt each frame by update_cubes()
  // inputs copied from the state under the mutex, for update_cubes()
  std::map<int, CubeMarker> in_markers_;
  std::map<int, TrackedCube> in_tracked_;
  PlacedResult in_placed_;
  PickNodeState in_pick_node_;
  std::string in_task_label_;
  bool in_task_running_ = false;
  std::map<int, CubeTrack> tracks_;
  std::map<int, Placed> placed_;
  uint64_t placed_seq_ = 0;
  int prev_held_ = -1;
  Matrix last_jaw_pose_{};           // the held cube's pose in the last frame
  bool last_jaw_valid_ = false;
  double last_update_ = -1.0;
  // the gripper's geometry, from the model
  Vector3 moving_tip_local_{};       // moving jaw tip in its link frame (farthest mesh vertex)
  bool have_moving_tip_ = false;
  float hold_gap_ = 0.03f;           // tip distance with the jaws closed on a cube
  // the base's footprint on the table (base_link xy box), from the root link's meshes
  Vector2 base_min_{0, 0}, base_max_{0, 0};
  std::vector<Vector3> path_;        // tool (gripper_frame_link) positions along the last plan
  float table_top_ = 0.0f;           // z of the table surface (the grid), base_link
  Vector2 table_centre_{0.2f, 0.0f};  // xy of the table top centre
  CubeSelection selection_;          // copied from the state each frame, for drawing
  int held_ = -1;                    // held cube index, -1: none
  bool holding_ = false;
  std::string held_cube_;            // the last held cube, to notice a change
};

}  // namespace so101_dashboard
