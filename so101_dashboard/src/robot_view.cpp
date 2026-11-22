#include "robot_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <raymath.h>
#include <rlgl.h>
#include <rlImGui.h>

#include "panels.hpp"
#include "pick_place_ui.hpp"
#include "tasks.hpp"
#include "theme.hpp"

namespace so101_dashboard
{

namespace
{

constexpr int SUPERSAMPLE = 2;   // the render texture has no MSAA: draw at 2x, show scaled down
constexpr float GRID_STEP = 0.05f;
constexpr double GHOST_LINGER_S = 3.0;   // the plan stays this long after the task ends

// Lambert key light plus a rim term that lights silhouettes: reads as a hologram.
const char * VERTEX_SHADER = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 fragPos;
out vec3 fragNormal;
void main()
{
  fragPos = vec3(matModel * vec4(vertexPosition, 1.0));
  fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
)";

const char * FRAGMENT_SHADER = R"(#version 330
in vec3 fragPos;
in vec3 fragNormal;
uniform vec3 viewPos;
uniform vec4 baseColor;
uniform vec4 rimColor;
out vec4 finalColor;
void main()
{
  vec3 n = normalize(fragNormal);
  vec3 v = normalize(viewPos - fragPos);
  float ndv = abs(dot(n, v));
  float rim = pow(1.0 - ndv, 2.2);
  float key = abs(dot(n, normalize(vec3(0.3, -0.4, 1.0))));
  vec3 c = baseColor.rgb * (0.35 + 0.65 * key) + rimColor.rgb * rim * 1.3;
  finalColor = vec4(c, mix(baseColor.a, 1.0, rim * rimColor.a));
}
)";

Matrix pose_matrix(const urdf::Pose & p)
{
  const Quaternion q{static_cast<float>(p.rotation.x), static_cast<float>(p.rotation.y),
    static_cast<float>(p.rotation.z), static_cast<float>(p.rotation.w)};
  // raymath: MatrixMultiply(a, b) applies a first, then b
  return MatrixMultiply(QuaternionToMatrix(q),
           MatrixTranslate(p.position.x, p.position.y, p.position.z));
}

Vector3 translation(const Matrix & m) {return Vector3{m.m12, m.m13, m.m14};}

void set_vec4(Shader shader, int loc, Color c)
{
  const float v[4] = {c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f};
  SetShaderValue(shader, loc, v, SHADER_UNIFORM_VEC4);
}

// Binary or ASCII STL -> triangle soup (x, y, z per vertex).
bool read_stl(const std::string & path, std::vector<float> & xyz)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }
  const std::string data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  xyz.clear();
  if (data.size() >= 84) {
    uint32_t count;
    std::memcpy(&count, data.data() + 80, 4);
    if (84 + 50ull * count == data.size()) {
      xyz.resize(count * 9ull);
      for (uint32_t i = 0; i < count; ++i) {
        std::memcpy(&xyz[i * 9ull], data.data() + 84 + 50ull * i + 12, 36);   // skip the facet normal
      }
      return count > 0;
    }
  }
  std::istringstream text(data);
  std::string token;
  while (text >> token) {
    if (token == "vertex") {
      float x, y, z;
      text >> x >> y >> z;
      xyz.insert(xyz.end(), {x, y, z});
    }
  }
  return !xyz.empty() && xyz.size() % 9 == 0;
}

// Flat-shaded mesh: one normal per face, computed (STL facet normals are not always right).
Mesh make_mesh(const std::vector<float> & xyz)
{
  Mesh mesh{};
  mesh.vertexCount = static_cast<int>(xyz.size() / 3);
  mesh.triangleCount = mesh.vertexCount / 3;
  mesh.vertices = static_cast<float *>(MemAlloc(xyz.size() * sizeof(float)));
  mesh.normals = static_cast<float *>(MemAlloc(xyz.size() * sizeof(float)));
  std::memcpy(mesh.vertices, xyz.data(), xyz.size() * sizeof(float));
  for (size_t t = 0; t < xyz.size(); t += 9) {
    const Vector3 a{xyz[t], xyz[t + 1], xyz[t + 2]};
    const Vector3 b{xyz[t + 3], xyz[t + 4], xyz[t + 5]};
    const Vector3 c{xyz[t + 6], xyz[t + 7], xyz[t + 8]};
    const Vector3 n = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
    for (int k = 0; k < 3; ++k) {
      mesh.normals[t + 3 * k] = n.x;
      mesh.normals[t + 3 * k + 1] = n.y;
      mesh.normals[t + 3 * k + 2] = n.z;
    }
  }
  UploadMesh(&mesh, false);
  return mesh;
}

std::string resolve_uri(const std::string & uri)
{
  const std::string package = "package://", file = "file://";
  if (uri.rfind(package, 0) == 0) {
    const std::string rest = uri.substr(package.size());
    const size_t slash = rest.find('/');
    return ament_index_cpp::get_package_share_directory(rest.substr(0, slash)) + rest.substr(slash);
  }
  if (uri.rfind(file, 0) == 0) {
    return uri.substr(file.size());
  }
  return uri;
}

}  // namespace

RobotView::RobotView()
{
  shader_ = LoadShaderFromMemory(VERTEX_SHADER, FRAGMENT_SHADER);
  shader_.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader_, "matModel");
  shader_.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(shader_, "matNormal");
  loc_view_pos_ = GetShaderLocation(shader_, "viewPos");
  loc_base_ = GetShaderLocation(shader_, "baseColor");
  loc_rim_ = GetShaderLocation(shader_, "rimColor");
  material_ = LoadMaterialDefault();
  material_.shader = shader_;
  camera_.up = Vector3{0, 0, 1};   // ROS frames: z up
  camera_.fovy = 40.0f;
  camera_.projection = CAMERA_PERSPECTIVE;
}

RobotView::~RobotView()
{
  for (Mesh & m : meshes_) {
    UnloadMesh(m);
  }
  if (target_w_ > 0) {
    UnloadRenderTexture(target_);
  }
  material_.shader = Shader{};   // the shader is unloaded on its own, not with the material
  UnloadMaterial(material_);
  UnloadShader(shader_);
}

int RobotView::primitive(const std::string & key, Mesh mesh)
{
  const auto it = mesh_cache_.find(key);
  if (it != mesh_cache_.end()) {
    UnloadMesh(mesh);
    return it->second;
  }
  meshes_.push_back(mesh);
  return mesh_cache_[key] = static_cast<int>(meshes_.size()) - 1;
}

int RobotView::load_mesh(const std::string & uri)
{
  const auto it = mesh_cache_.find(uri);
  if (it != mesh_cache_.end()) {
    return it->second;
  }
  std::vector<float> xyz;
  std::string path;
  try {
    path = resolve_uri(uri);
  } catch (const std::exception &) {
    path.clear();
  }
  if (path.empty() || !read_stl(path, xyz)) {
    TraceLog(LOG_WARNING, "3D view: cannot load mesh %s", uri.c_str());
    return mesh_cache_[uri] = -1;
  }
  meshes_.push_back(make_mesh(xyz));
  return mesh_cache_[uri] = static_cast<int>(meshes_.size()) - 1;
}

void RobotView::load_model(const std::string & xml)
{
  auto model = std::make_shared<urdf::Model>();
  if (!model->initString(xml)) {
    TraceLog(LOG_WARNING, "3D view: cannot parse the robot description");
    return;
  }
  model_ = model;
  visuals_.clear();
  for (const auto & [name, link] : model_->links_) {
    for (const auto & visual : link->visual_array) {
      if (!visual || !visual->geometry) {
        continue;
      }
      Visual v;
      v.link = name;
      v.origin = pose_matrix(visual->origin);
      const auto & g = *visual->geometry;
      if (g.type == urdf::Geometry::MESH) {
        const auto & mesh = static_cast<const urdf::Mesh &>(g);
        v.origin = MatrixMultiply(MatrixScale(mesh.scale.x, mesh.scale.y, mesh.scale.z), v.origin);
        v.mesh = load_mesh(mesh.filename);
      } else if (g.type == urdf::Geometry::BOX) {
        const auto & box = static_cast<const urdf::Box &>(g);
        if (name == "table") {
          v.table = true;
          table_size_ = Vector3{static_cast<float>(box.dim.x), static_cast<float>(box.dim.y),
            static_cast<float>(box.dim.z)};
        }
        std::ostringstream key;
        key << "box " << box.dim.x << ' ' << box.dim.y << ' ' << box.dim.z;
        v.mesh = primitive(key.str(), GenMeshCube(box.dim.x, box.dim.y, box.dim.z));
      } else if (g.type == urdf::Geometry::CYLINDER) {
        const auto & cyl = static_cast<const urdf::Cylinder &>(g);
        // raylib's cylinder stands on y = 0 along +y; URDF's is centred on z
        v.origin = MatrixMultiply(MatrixMultiply(MatrixTranslate(0, -cyl.length / 2, 0), MatrixRotateX(PI / 2)),
            v.origin);
        std::ostringstream key;
        key << "cylinder " << cyl.radius << ' ' << cyl.length;
        v.mesh = primitive(key.str(), GenMeshCylinder(cyl.radius, cyl.length, 24));
      } else if (g.type == urdf::Geometry::SPHERE) {
        const auto & sphere = static_cast<const urdf::Sphere &>(g);
        std::ostringstream key;
        key << "sphere " << sphere.radius;
        v.mesh = primitive(key.str(), GenMeshSphere(sphere.radius, 16, 16));
      }
      if (v.mesh >= 0) {
        visuals_.push_back(v);
      }
    }
  }
  TraceLog(LOG_INFO, "3D view: %s, %zu links, %zu visuals", model_->getName().c_str(),
    model_->links_.size(), visuals_.size());
}

LinkPoses RobotView::forward_kinematics(const JointValues & q) const
{
  LinkPoses poses;
  if (!model_ || !model_->getRoot()) {
    return poses;
  }
  std::function<void(const urdf::LinkConstSharedPtr &, const Matrix &)> walk =
    [&](const urdf::LinkConstSharedPtr & link, const Matrix & world) {
      poses[link->name] = world;
      for (const auto & joint : link->child_joints) {
        double value = 0.0;
        const std::string & source = joint->mimic ? joint->mimic->joint_name : joint->name;
        const auto it = q.find(source);
        if (it != q.end() && std::isfinite(it->second)) {
          value = joint->mimic ? it->second * joint->mimic->multiplier + joint->mimic->offset : it->second;
        }
        const Vector3 axis{static_cast<float>(joint->axis.x), static_cast<float>(joint->axis.y),
          static_cast<float>(joint->axis.z)};
        Matrix motion = MatrixIdentity();
        if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
          motion = MatrixRotate(axis, static_cast<float>(value));
        } else if (joint->type == urdf::Joint::PRISMATIC) {
          const Vector3 d = Vector3Scale(axis, static_cast<float>(value));
          motion = MatrixTranslate(d.x, d.y, d.z);
        }
        const Matrix local = MatrixMultiply(motion, pose_matrix(joint->parent_to_joint_origin_transform));
        const auto child = model_->getLink(joint->child_link_name);
        if (child) {
          walk(child, MatrixMultiply(local, world));
        }
      }
    };
  walk(model_->getRoot(), MatrixIdentity());
  return poses;
}

void RobotView::handle_input(bool hovered, bool active)
{
  const ImGuiIO & io = ImGui::GetIO();
  if (active && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)) {
    yaw_ -= io.MouseDelta.x * 0.008f;
    pitch_ = Clamp(pitch_ + io.MouseDelta.y * 0.008f, -1.45f, 1.45f);
  }
  if (active && (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f) ||
    ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f)))
  {
    // pan in the view plane
    const Vector3 forward = Vector3Normalize(Vector3Subtract(camera_.target, camera_.position));
    const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera_.up));
    const Vector3 up = Vector3CrossProduct(right, forward);
    const float k = distance_ * 0.0015f;
    focus_ = Vector3Add(focus_, Vector3Add(Vector3Scale(right, -io.MouseDelta.x * k),
        Vector3Scale(up, io.MouseDelta.y * k)));
  }
  if (hovered && io.MouseWheel != 0.0f) {
    distance_ = Clamp(distance_ * std::pow(0.88f, io.MouseWheel), 0.15f, 4.0f);
  }
  if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {   // back to the start view
    yaw_ = -0.75f;
    pitch_ = 0.32f;
    distance_ = 0.8f;
    focus_ = Vector3{0.12f, 0.0f, 0.1f};
  }
}

void RobotView::update_camera()
{
  camera_.target = focus_;
  camera_.position = Vector3Add(focus_, Vector3{
    distance_ * std::cos(pitch_) * std::cos(yaw_),
    distance_ * std::cos(pitch_) * std::sin(yaw_),
    distance_ * std::sin(pitch_)});
}

void RobotView::draw_table(const LinkPoses & poses)
{
  for (const Visual & v : visuals_) {
    if (!v.table || !poses.count(v.link)) {
      continue;
    }
    const Matrix m = MatrixMultiply(v.origin, poses.at(v.link));
    const Vector3 c = translation(m);
    const float top = c.z + table_size_.z / 2 + 0.0005f;
    table_top_ = top - 0.0005f;   // clicks on the table hit this plane
    const float hx = table_size_.x / 2, hy = table_size_.y / 2;
    const Color line = theme::to_raylib(ImVec4(theme::CYAN.x, theme::CYAN.y, theme::CYAN.z, 0.13f));
    const Color edge = theme::to_raylib(ImVec4(theme::CYAN.x, theme::CYAN.y, theme::CYAN.z, 0.45f));
    for (float x = -hx; x <= hx + 1e-4f; x += GRID_STEP) {
      DrawLine3D(Vector3{c.x + x, c.y - hy, top}, Vector3{c.x + x, c.y + hy, top}, line);
    }
    for (float y = -hy; y <= hy + 1e-4f; y += GRID_STEP) {
      DrawLine3D(Vector3{c.x - hx, c.y + y, top}, Vector3{c.x + hx, c.y + y, top}, line);
    }
    const Vector3 corners[4] = {{c.x - hx, c.y - hy, top}, {c.x + hx, c.y - hy, top},
      {c.x + hx, c.y + hy, top}, {c.x - hx, c.y + hy, top}};
    for (int i = 0; i < 4; ++i) {
      DrawLine3D(corners[i], corners[(i + 1) % 4], edge);
    }
  }
  // base frame axes, 5 cm
  DrawLine3D(Vector3{0, 0, 0}, Vector3{0.05f, 0, 0}, theme::to_raylib(theme::ALERT));
  DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 0.05f, 0}, theme::to_raylib(theme::OK));
  DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 0, 0.05f}, theme::to_raylib(theme::CYAN));
}

void RobotView::draw_robot(const LinkPoses & poses, Color base, Color rim)
{
  const float view[3] = {camera_.position.x, camera_.position.y, camera_.position.z};
  SetShaderValue(shader_, loc_view_pos_, view, SHADER_UNIFORM_VEC3);
  set_vec4(shader_, loc_base_, base);
  set_vec4(shader_, loc_rim_, rim);
  for (const Visual & v : visuals_) {
    const auto it = poses.find(v.link);
    if (v.table || it == poses.end()) {
      continue;
    }
    DrawMesh(meshes_[v.mesh], material_, MatrixMultiply(v.origin, it->second));
  }
}

void RobotView::render(const LinkPoses & poses, const LinkPoses & ghost)
{
  BeginTextureMode(target_);
  ClearBackground(Color{6, 10, 15, 255});
  BeginMode3D(camera_);
  draw_table(poses);
  draw_robot(poses, Color{14, 34, 42, 255}, theme::to_raylib(theme::CYAN));
  if (!ghost.empty()) {
    // translucent, and not hiding what is behind it
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    draw_robot(ghost, Color{60, 18, 40, 60}, theme::to_raylib(ImVec4(theme::ROSE.x, theme::ROSE.y, theme::ROSE.z, 0.9f)));
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    const Color trail = theme::to_raylib(theme::ROSE);
    for (size_t i = 1; i < path_.size(); ++i) {
      DrawLine3D(path_[i - 1], path_[i], trail);
    }
    if (!path_.empty()) {
      DrawSphere(path_.back(), 0.006f, trail);
    }
  }
  draw_cubes();
  draw_pick_place_3d();
  const Color fov = theme::to_raylib(ImVec4(theme::ROSE.x, theme::ROSE.y, theme::ROSE.z, 0.8f));
  for (size_t i = 0; i + 1 < fov_world_.size(); i += 2) {
    DrawLine3D(fov_world_[i], fov_world_[i + 1], fov);
  }
  EndMode3D();
  if (scanlines_) {
    for (int y = 0; y < target_h_; y += 3 * SUPERSAMPLE) {
      DrawRectangle(0, y, target_w_, SUPERSAMPLE, Color{0, 0, 0, 55});
    }
  }
  EndTextureMode();
}

void RobotView::draw_labels(const LinkPoses & poses, const JointValues & q, ImVec2 origin, float scale) const
{
  struct Label {const char * joint, * text; ImVec2 offset;};   // offset from the ring, px
  static const Label labels[] = {{"shoulder_pan", "J1", {-80, 4}}, {"shoulder_lift", "J2", {9, -18}},
    {"elbow_flex", "J3", {9, -18}}, {"wrist_flex", "J4", {9, -18}}, {"wrist_roll", "J5", {9, -18}},
    {"gripper", "G", {9, 2}}};
  ImDrawList * dl = ImGui::GetWindowDrawList();
  const ImU32 ring = ImGui::GetColorU32(theme::AMBER);
  const ImU32 text = ImGui::GetColorU32(theme::TEXT);
  for (const auto & [joint_name, label, offset] : labels) {
    const auto joint = model_->getJoint(joint_name);
    const auto qi = q.find(joint_name);
    if (!joint || qi == q.end() || !poses.count(joint->child_link_name)) {
      continue;
    }
    const Vector3 p = translation(poses.at(joint->child_link_name));
    // behind the camera?
    if (Vector3DotProduct(Vector3Subtract(p, camera_.position),
      Vector3Subtract(camera_.target, camera_.position)) <= 0)
    {
      continue;
    }
    const Vector2 s = GetWorldToScreenEx(p, camera_, target_w_, target_h_);
    const ImVec2 at(origin.x + s.x * scale, origin.y + s.y * scale);
    dl->AddCircle(at, 6.0f, ring, 16, 1.5f);
    char buf[32];
    snprintf(buf, sizeof(buf), "%s %+.1f", label, qi->second * 180.0 / M_PI);
    dl->AddText(ImVec2(at.x + offset.x, at.y + offset.y), text, buf);
  }
}

void RobotView::check_fk(State & state, const LinkPoses & poses)
{
  const double now = now_s();
  if (now - fk_checked_at_ < 1.0 || !state.tf || !model_ || !poses.count("gripper_frame_link")) {
    return;
  }
  fk_checked_at_ = now;
  try {
    const auto t = state.tf->lookupTransform(model_->getRoot()->name, "gripper_frame_link", tf2::TimePointZero);
    const Vector3 fk = translation(poses.at("gripper_frame_link"));
    const Vector3 tf{static_cast<float>(t.transform.translation.x), static_cast<float>(t.transform.translation.y),
      static_cast<float>(t.transform.translation.z)};
    fk_error_mm_ = Vector3Distance(fk, tf) * 1000.0;
  } catch (const std::exception &) {
    fk_error_mm_ = -1.0;
  }
}

LinkPoses RobotView::ghost_poses(const Ghost & ghost, const JointValues & q, bool task_running)
{
  path_.clear();
  if (ghost.received < 0 || ghost.positions.empty() ||
    (!task_running && now_s() - ghost.received > GHOST_LINGER_S))
  {
    return {};
  }
  // the goal pose, and the tool path through every plan point
  auto at = [&](const std::vector<double> & p) {
      JointValues gq = q;
      for (size_t j = 0; j < ghost.joints.size() && j < p.size(); ++j) {
        gq[ghost.joints[j]] = p[j];
      }
      return forward_kinematics(gq);
    };
  for (const auto & p : ghost.positions) {
    const LinkPoses poses = at(p);
    const auto tool = poses.find("gripper_frame_link");
    if (tool != poses.end()) {
      path_.push_back(translation(tool->second));
    }
  }
  return at(ghost.positions.back());
}

void RobotView::draw_cubes() const
{
  for (const auto & vc : cubes_) {
    const CubeMarker & c = vc.marker;
    const auto u8 = [](float v) {return static_cast<unsigned char>(std::clamp(v, 0.0f, 1.0f) * 255.0f);};
    const bool fresh = c.rgba[3] >= 0.5f;
    const bool held = vc.index == held_;   // still drawn where it was seen last, faint
    const Color fill{u8(c.rgba[0]), u8(c.rgba[1]), u8(c.rgba[2]),
      static_cast<unsigned char>(held ? 30 : fresh ? 200 : 60)};
    const Color edge{u8(c.rgba[0]), u8(c.rgba[1]), u8(c.rgba[2]), static_cast<unsigned char>(held ? 90 : 255)};
    rlPushMatrix();
    rlTranslatef(c.position[0], c.position[1], c.position[2]);
    rlRotatef(c.yaw * RAD2DEG, 0.0f, 0.0f, 1.0f);
    DrawCube(Vector3Zero(), c.edge, c.edge, c.edge, fill);
    DrawCubeWires(Vector3Zero(), c.edge, c.edge, c.edge, edge);
    // selection: a double outline that glows; stack target in amber
    const bool selected = !holding_ && vc.index == selection_.cube;
    const bool stack = holding_ && vc.index == selection_.stack_on;
    if (selected || stack) {
      const ImVec4 & hl = selected ? theme::CYAN : theme::AMBER;
      const float pulse = 0.65f + 0.35f * std::sin(static_cast<float>(now_s()) * 5.0f);
      DrawCubeWires(Vector3Zero(), c.edge * 1.2f, c.edge * 1.2f, c.edge * 1.2f, theme::to_raylib(hl));
      DrawCubeWires(Vector3Zero(), c.edge * 1.4f, c.edge * 1.4f, c.edge * 1.4f,
        theme::to_raylib(ImVec4(hl.x, hl.y, hl.z, 0.5f * pulse)));
      DrawCube(Vector3Zero(), c.edge * 1.2f, c.edge * 1.2f, c.edge * 1.2f,
        theme::to_raylib(ImVec4(hl.x, hl.y, hl.z, 0.12f * pulse)));
    }
    rlPopMatrix();
  }
}

bool RobotView::screen_of(Vector3 p, ImVec2 origin, ImVec2 & out) const
{
  if (Vector3DotProduct(Vector3Subtract(p, camera_.position),
    Vector3Subtract(camera_.target, camera_.position)) <= 0)
  {
    return false;
  }
  const Vector2 s = GetWorldToScreenEx(p, camera_, target_w_, target_h_);
  out = ImVec2(origin.x + s.x / SUPERSAMPLE, origin.y + s.y / SUPERSAMPLE);
  return true;
}

void RobotView::draw_cube_labels(ImVec2 origin, float scale) const
{
  (void)scale;
  ImDrawList * dl = ImGui::GetWindowDrawList();
  for (const auto & vc : cubes_) {
    const CubeMarker & c = vc.marker;
    ImVec2 at;
    if (!screen_of(Vector3{c.position[0], c.position[1], c.position[2] + c.edge}, origin, at)) {
      continue;
    }
    char buf[96];
    const char * tag = vc.index == held_ ? "  (held)" :
      !holding_ && vc.index == selection_.cube ? "  SELECTED" :
      holding_ && vc.index == selection_.stack_on ? "  STACK HERE?" : "";
    snprintf(buf, sizeof(buf), "%s  %.1f, %.1f cm%s", c.label.c_str(), c.position[0] * 100.0f,
      c.position[1] * 100.0f, tag);
    const ImVec2 text_at(at.x + 6, at.y - 16);
    if (*tag && vc.index != held_) {   // a glowing plate behind the selected cube's label
      const ImVec4 & hl = holding_ ? theme::AMBER : theme::CYAN;
      const ImVec2 size = ImGui::CalcTextSize(buf);
      dl->AddRectFilled(ImVec2(text_at.x - 4, text_at.y - 2), ImVec2(text_at.x + size.x + 4, text_at.y + size.y + 2),
        ImGui::GetColorU32(ImVec4(hl.x, hl.y, hl.z, 0.18f)), 3.0f);
      dl->AddRect(ImVec2(text_at.x - 4, text_at.y - 2), ImVec2(text_at.x + size.x + 4, text_at.y + size.y + 2),
        ImGui::GetColorU32(hl), 3.0f);
      dl->AddText(text_at, ImGui::GetColorU32(hl), buf);
      continue;
    }
    const ImVec4 col(c.rgba[0], c.rgba[1], c.rgba[2], c.rgba[3] >= 0.5f && vc.index != held_ ? 1.0f : 0.5f);
    dl->AddText(text_at, ImGui::GetColorU32(col), buf);
  }
}

void RobotView::draw_pick_place_3d() const
{
  // the reach ring: where the pick node can put a cube down; brighter while a cube is held
  const float a = holding_ ? 0.55f : 0.16f;
  const Color ring = theme::to_raylib(ImVec4(theme::OK.x, theme::OK.y, theme::OK.z, a));
  const Color spoke = theme::to_raylib(ImVec4(theme::OK.x, theme::OK.y, theme::OK.z, a * 0.4f));
  const float z = table_top_ + 0.001f;
  rlPushMatrix();
  rlTranslatef(0.0f, 0.0f, z);
  DrawCircle3D(Vector3Zero(), REACH_MIN_M, Vector3{1, 0, 0}, 0.0f, ring);
  DrawCircle3D(Vector3Zero(), REACH_MAX_M, Vector3{1, 0, 0}, 0.0f, ring);
  for (int i = 0; i < 24; ++i) {   // light hatching between the circles
    const float t = i * 2.0f * PI / 24;
    DrawLine3D(Vector3{REACH_MIN_M * std::cos(t), REACH_MIN_M * std::sin(t), 0},
      Vector3{REACH_MAX_M * std::cos(t), REACH_MAX_M * std::sin(t), 0}, spoke);
  }
  rlPopMatrix();
  // the place target: a ghost cube where the held one would go
  if (holding_ && selection_.has_target && selection_.stack_on < 0) {
    constexpr float EDGE = 0.03f;
    const Color fill = theme::to_raylib(ImVec4(theme::OK.x, theme::OK.y, theme::OK.z, 0.22f));
    const Color edge = theme::to_raylib(theme::OK);
    rlPushMatrix();
    rlTranslatef(selection_.x, selection_.y, table_top_ + EDGE / 2);
    rlRotatef(selection_.any_yaw ? 0.0f : selection_.yaw_deg, 0.0f, 0.0f, 1.0f);
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    DrawCube(Vector3Zero(), EDGE, EDGE, EDGE, fill);
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    DrawCubeWires(Vector3Zero(), EDGE, EDGE, EDGE, edge);
    if (!selection_.any_yaw) {   // the yaw: a tick on the top face along the cube's x
      DrawLine3D(Vector3{0, 0, EDGE / 2}, Vector3{EDGE * 0.8f, 0, EDGE / 2}, edge);
    }
    rlPopMatrix();
    DrawLine3D(Vector3{selection_.x, selection_.y, table_top_ + EDGE},
      Vector3{selection_.x, selection_.y, table_top_ + 0.10f}, theme::to_raylib(ImVec4(theme::OK.x, theme::OK.y,
      theme::OK.z, 0.5f)));
  }
}

std::string RobotView::click(State & state, Vector2 px)
{
  const Ray ray = GetScreenToWorldRayEx(px, camera_, target_w_, target_h_);
  // the nearest cube box (enlarged a little: they are small on screen), in the cube's own frame
  int hit_cube = -1;
  float hit_distance = 1e9f;
  for (const auto & vc : cubes_) {
    const CubeMarker & c = vc.marker;
    const float cs = std::cos(-c.yaw), sn = std::sin(-c.yaw);
    const auto to_cube = [&](Vector3 v) {return Vector3{cs * v.x - sn * v.y, sn * v.x + cs * v.y, v.z};};
    const Ray local{to_cube(Vector3Subtract(ray.position, Vector3{c.position[0], c.position[1], c.position[2]})),
      to_cube(ray.direction)};
    const float h = c.edge * 0.5f * 1.3f;
    const RayCollision hit = GetRayCollisionBox(local, BoundingBox{Vector3{-h, -h, -h}, Vector3{h, h, h}});
    if (hit.hit && hit.distance < hit_distance) {
      hit_distance = hit.distance;
      hit_cube = vc.index;
    }
  }
  // the table plane
  bool hit_table = false;
  float tx = 0.0f, ty = 0.0f;
  if (std::abs(ray.direction.z) > 1e-6f) {
    const float t = (table_top_ - ray.position.z) / ray.direction.z;
    if (t > 0 && t < hit_distance) {
      hit_table = true;
      tx = ray.position.x + t * ray.direction.x;
      ty = ray.position.y + t * ray.direction.y;
    }
  }
  if (hit_cube >= 0 && hit_table) {
    hit_table = false;   // the cube is nearer than the table point behind it
  }

  std::lock_guard<std::mutex> lock(state.mutex);
  CubeSelection & sel = state.selection;
  const bool holding = !state.holding.cube.empty();
  const int held = cube_index(state.holding.cube);
  char buf[160];
  sel.message.clear();
  if (!holding) {
    if (hit_cube >= 0) {
      sel.cube = hit_cube;
      snprintf(buf, sizeof(buf), "selected K%d", hit_cube);
    } else {
      sel.cube = -1;
      snprintf(buf, sizeof(buf), "deselected");
    }
    return buf;
  }
  if (hit_cube >= 0) {
    if (hit_cube == held) {
      sel.message = "K" + std::to_string(held) + " is the held cube";
    } else {
      sel.stack_on = hit_cube;
      sel.has_target = false;
      snprintf(buf, sizeof(buf), "stack target K%d", hit_cube);
      return buf;
    }
  } else if (hit_table) {
    const std::string problem = reach_problem(tx, ty);
    if (problem.empty()) {
      sel.has_target = true;
      sel.x = tx;
      sel.y = ty;
      sel.stack_on = -1;
      snprintf(buf, sizeof(buf), "place target %.1f, %.1f cm", tx * 100.0f, ty * 100.0f);
      return buf;
    }
    sel.message = problem;
    sel.has_target = false;
    sel.stack_on = -1;
  } else {
    sel.has_target = false;
    sel.stack_on = -1;
    return "target cleared";
  }
  sel.message_at = now_s();
  return "refused: " + sel.message;
}

void RobotView::draw_pick_place_popup(State & state, TaskRunner & tasks, ImVec2 origin, ImVec2 size)
{
  const CubeUi ui = cube_ui(state, tasks);
  // anchor: the selected cube, the stack target or the place target
  Vector3 anchor{};
  bool show = false;
  const int cube = holding(ui) ? ui.sel.stack_on : ui.sel.cube;
  if (cube >= 0) {
    for (const auto & vc : cubes_) {
      if (vc.index == cube) {
        anchor = Vector3{vc.marker.position[0], vc.marker.position[1], vc.marker.position[2]};
        show = true;
      }
    }
  } else if (holding(ui) && ui.sel.has_target) {
    anchor = Vector3{ui.sel.x, ui.sel.y, table_top_ + 0.015f};
    show = true;
  }
  ImVec2 at;
  if (show && screen_of(anchor, origin, at)) {
    const ImVec2 popup(300, holding(ui) && cube < 0 ? 104 : 76);   // for the clamping; the height fits itself
    at = ImVec2(std::clamp(at.x + 24, origin.x + 4, origin.x + size.x - popup.x - 4),
      std::clamp(at.y + 12, origin.y + 36, origin.y + size.y - popup.y - 30));
    ImGui::SetCursorScreenPos(at);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(theme::PANEL.x, theme::PANEL.y, theme::PANEL.z, 0.9f));
    ImGui::PushStyleColor(ImGuiCol_Border, holding(ui) ? (cube >= 0 ? theme::AMBER : theme::OK) : theme::CYAN);
    if (ImGui::BeginChild("##pick_place_popup", ImVec2(popup.x, 0), ImGuiChildFlags_Borders |
      ImGuiChildFlags_AlwaysUseWindowPadding | ImGuiChildFlags_AutoResizeY,
      ImGuiWindowFlags_NoScrollbar))
    {
      if (holding(ui)) {
        if (cube >= 0) {
          ImGui::TextColored(theme::AMBER, "K%d as the base of a stack", cube);
        } else {
          ImGui::TextColored(theme::OK, "put %s here   r %.1f cm", held_name(ui).c_str(),
            std::hypot(ui.sel.x, ui.sel.y) * 100.0f);
        }
      } else {
        ImGui::TextColored(theme::CYAN, "K%d", cube);
        if (ui.selected_seen) {
          ImGui::SameLine();
          ImGui::TextColored(theme::DIM, "%.1f, %.1f cm   r %.1f cm", ui.selected_x * 100.0f, ui.selected_y * 100.0f,
            std::hypot(ui.selected_x, ui.selected_y) * 100.0f);
        }
      }
      draw_cube_actions(state, tasks, ui, "popup");
    }
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
  }
  // a refused click: say why for a few seconds
  if (!ui.sel.message.empty() && now_s() - ui.sel.message_at < 4.0) {
    ImGui::SetCursorScreenPos(ImVec2(origin.x + 12, origin.y + size.y - 48));
    ImGui::TextColored(theme::AMBER, "%s", ui.sel.message.c_str());
  }
}

void RobotView::run_test_commands(State & state, TaskRunner & tasks)
{
  std::deque<std::string> commands;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    commands.swap(state.test_commands);
  }
  for (const auto & line : commands) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    std::string result = "ok";
    if (cmd == "click_world") {   // a click where this base_link point shows up on screen
      float x = 0, y = 0, z = 0;
      in >> x >> y >> z;
      const Vector2 s = GetWorldToScreenEx(Vector3{x, y, z}, camera_, target_w_, target_h_);
      result = click(state, s);
    } else if (cmd == "click_view") {   // a click at a fraction of the view
      float u = 0, v = 0;
      in >> u >> v;
      result = click(state, Vector2{u * target_w_, v * target_h_});
    } else if (cmd == "pick") {
      result = pick_selected(state, tasks) ? "pick started" : "pick not started";
    } else if (cmd == "place") {
      result = place_at_target(state, tasks) ? "place started" : "place not started";
    } else if (cmd == "stack") {
      result = stack_on_target(state, tasks) ? "stack started" : "stack not started";
    } else if (cmd == "yaw") {
      float yaw = 0;
      in >> yaw;
      std::lock_guard<std::mutex> lock(state.mutex);
      state.selection.yaw_deg = yaw;
      state.selection.any_yaw = false;
    } else if (cmd == "look") {
      result = tasks.start_look() ? "look started" : "look not started";
    } else if (cmd == "stop") {
      tasks.stop();
    } else if (cmd == "shot") {   // a screenshot of this window, into the working directory
      std::string name;
      in >> name;
      std::lock_guard<std::mutex> lock(state.mutex);
      state.screenshot = name.empty() ? "so101_dashboard.png" : name;
    } else {
      result = "unknown command";
    }
    TraceLog(LOG_INFO, "test hook: %s -> %s", line.c_str(), result.c_str());
  }
}

void RobotView::update_fov(State & state)
{
  LineMarker fov;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    fov = state.fov;
  }
  fov_world_.clear();
  // the camera node republishes it every second; older means the camera is gone
  if (fov.received < 0 || now_s() - fov.received > 3.0 || !state.tf || !model_) {
    return;
  }
  try {
    const auto t = state.tf->lookupTransform(model_->getRoot()->name, fov.frame, tf2::TimePointZero);
    urdf::Pose p;
    p.position = urdf::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
    p.rotation = urdf::Rotation(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
      t.transform.rotation.w);
    const Matrix m = pose_matrix(p);
    for (const auto & pt : fov.points) {
      fov_world_.push_back(Vector3Transform(Vector3{pt[0], pt[1], pt[2]}, m));
    }
  } catch (const std::exception &) {
  }
}

void RobotView::draw(State & state, TaskRunner & tasks, bool * open)
{
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  const bool visible = ImGui::Begin(WIN_VIEW, open);
  ImGui::PopStyleVar();
  if (!visible) {
    ImGui::End();
    return;
  }
  JointValues q;
  Ghost ghost;
  bool task_running;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.description_version != description_version_) {
      description_version_ = state.description_version;
      load_model(state.robot_description);
    }
    for (const auto & [name, joint] : state.joints) {
      q[name] = joint.position;
    }
    ghost = state.ghost;
    task_running = state.task.running;
    cubes_.clear();
    for (const auto & [id, cube] : state.cubes) {
      // the detector publishes in base_link, which is the model root here
      if (model_ && cube.frame == model_->getRoot()->name) {
        cubes_.push_back(ViewCube{id, cube});
        if (cubes_.back().marker.label.empty()) {
          cubes_.back().marker.label = "K" + std::to_string(id);
        }
      }
    }
    // cubes only in /cubes/tracked (no markers: the FAKE cube on mock, or a detector without them)
    for (const auto & [id, t] : state.tracked) {
      if (!model_ || model_->getRoot()->name != "base_link" || state.cubes.count(id)) {
        continue;
      }
      CubeMarker m;
      m.frame = "base_link";
      m.position = t.position;
      m.yaw = t.yaw;
      m.edge = t.edge;
      m.rgba = {theme::TEXT.x, theme::TEXT.y, theme::TEXT.z, 0.9f};
      m.label = "K" + std::to_string(id) + (t.score >= 0.5f ? "" : " ~");
      cubes_.push_back(ViewCube{id, m});
    }
    // a new held cube: the selection becomes the place target; nothing held: no target
    if (state.holding.cube != held_cube_) {
      held_cube_ = state.holding.cube;
      state.selection.cube = -1;
      state.selection.stack_on = -1;
      state.selection.has_target = false;
    }
    holding_ = !state.holding.cube.empty();
    held_ = cube_index(state.holding.cube);
    selection_ = state.selection;
  }
  if (!model_) {
    ImGui::SetCursorPos(ImVec2(12, 36));
    ImGui::TextColored(theme::DIM, "waiting for /robot_description ...");
    ImGui::End();
    return;
  }
  const LinkPoses poses = forward_kinematics(q);
  check_fk(state, poses);
  update_fov(state);

  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  const int w = std::max(1, static_cast<int>(avail.x)) * SUPERSAMPLE;
  const int h = std::max(1, static_cast<int>(avail.y)) * SUPERSAMPLE;
  if (w != target_w_ || h != target_h_) {
    if (target_w_ > 0) {
      UnloadRenderTexture(target_);
    }
    target_ = LoadRenderTexture(w, h);
    SetTextureFilter(target_.texture, TEXTURE_FILTER_BILINEAR);
    target_w_ = w;
    target_h_ = h;
  }

  // the overlays drawn later (toggles, the pick/place popup) take the mouse before the view does
  ImGui::SetNextItemAllowOverlap();
  ImGui::InvisibleButton("##view", avail, ImGuiButtonFlags_MouseButtonLeft |
    ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
  handle_input(ImGui::IsItemHovered(), ImGui::IsItemActive());
  // a left click that did not drag (< 4 px of travel) selects; a drag orbits
  const bool clicked = ImGui::IsItemDeactivated() && ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
    ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 16.0f;
  update_camera();
  if (clicked) {
    const ImVec2 m = ImGui::GetIO().MousePos;
    const std::string what = click(state, Vector2{(m.x - pos.x) * SUPERSAMPLE, (m.y - pos.y) * SUPERSAMPLE});
    TraceLog(LOG_INFO, "3D view click: %s", what.c_str());
    std::lock_guard<std::mutex> lock(state.mutex);
    selection_ = state.selection;
  }
  run_test_commands(state, tasks);
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    selection_ = state.selection;
  }
  render(poses, ghost_poses(ghost, q, task_running));

  ImGui::SetCursorScreenPos(pos);
  rlImGuiImageRect(&target_.texture, static_cast<int>(avail.x), static_cast<int>(avail.y),
    Rectangle{0, 0, static_cast<float>(target_w_), -static_cast<float>(target_h_)});
  if (labels_) {
    draw_labels(poses, q, pos, 1.0f / SUPERSAMPLE);
    draw_cube_labels(pos, 1.0f / SUPERSAMPLE);
  }

  // overlay: toggles left, FK check right
  ImGui::SetCursorScreenPos(ImVec2(pos.x + 10, pos.y + 8));
  ImGui::Checkbox("labels", &labels_);
  ImGui::SameLine();
  ImGui::Checkbox("scanlines", &scanlines_);
  if (fk_error_mm_ >= 0) {
    char buf[48];
    snprintf(buf, sizeof(buf), "FK/TF %.1f mm", fk_error_mm_);
    ImGui::SetCursorScreenPos(ImVec2(pos.x + avail.x - ImGui::CalcTextSize(buf).x - 12, pos.y + 12));
    ImGui::TextColored(fk_error_mm_ < 1.0 ? theme::DIM : theme::AMBER, "%s", buf);
  }
  draw_pick_place_popup(state, tasks, pos, avail);
  ImGui::SetCursorScreenPos(ImVec2(pos.x + 12, pos.y + avail.y - 24));
  ImGui::TextColored(theme::DIM, holding_ ?
    "click: table = place target, cube = stack   drag: orbit   right drag: pan   wheel: zoom" :
    "click: select a cube   drag: orbit   right drag: pan   wheel: zoom   double-click: reset");
  ImGui::End();
}

}  // namespace so101_dashboard
