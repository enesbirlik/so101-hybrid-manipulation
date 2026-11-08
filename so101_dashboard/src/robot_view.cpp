#include "robot_view.hpp"

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

void RobotView::draw(State & state)
{
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  const bool open = ImGui::Begin(WIN_VIEW);
  ImGui::PopStyleVar();
  if (!open) {
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

  ImGui::InvisibleButton("##view", avail, ImGuiButtonFlags_MouseButtonLeft |
    ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
  handle_input(ImGui::IsItemHovered(), ImGui::IsItemActive());
  update_camera();
  render(poses, ghost_poses(ghost, q, task_running));

  ImGui::SetCursorScreenPos(pos);
  rlImGuiImageRect(&target_.texture, static_cast<int>(avail.x), static_cast<int>(avail.y),
    Rectangle{0, 0, static_cast<float>(target_w_), -static_cast<float>(target_h_)});
  if (labels_) {
    draw_labels(poses, q, pos, 1.0f / SUPERSAMPLE);
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
  ImGui::SetCursorScreenPos(ImVec2(pos.x + 12, pos.y + avail.y - 24));
  ImGui::TextColored(theme::DIM, "drag: orbit   right drag: pan   wheel: zoom   double-click: reset");
  ImGui::End();
}

}  // namespace so101_dashboard
