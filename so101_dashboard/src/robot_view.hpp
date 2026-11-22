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
  void draw_labels(const LinkPoses & poses, const JointValues & q, ImVec2 origin, float scale) const;
  void check_fk(State & state, const LinkPoses & poses);
  void update_fov(State & state);   // camera field of view, from its marker and TF
  void draw_cubes() const;
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
  struct ViewCube
  {
    int index = -1;
    CubeMarker marker;
  };
  std::vector<Vector3> fov_world_;   // line segment end points
  std::vector<ViewCube> cubes_;      // in the model root frame, copied from the state each frame
  std::vector<Vector3> path_;        // tool (gripper_frame_link) positions along the last plan
  float table_top_ = 0.0f;           // z of the table surface (the grid), base_link
  CubeSelection selection_;          // copied from the state each frame, for drawing
  int held_ = -1;                    // held cube index, -1: none
  bool holding_ = false;
  std::string held_cube_;            // the last held cube, to notice a change
};

}  // namespace so101_dashboard
