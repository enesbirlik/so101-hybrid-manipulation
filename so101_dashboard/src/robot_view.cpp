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
constexpr float GRID_STEP = 0.05f;      // minor grid lines; every second one is a major line
constexpr double GHOST_LINGER_S = 3.0;   // the plan stays this long after the task ends
constexpr float RINGS_M[] = {0.10f, 0.20f, 0.30f, 0.40f};   // distance rings around the base axis

// Cubes
constexpr double LIVE_S = 0.5;            // seen within this long: drawn solid, else as a ghost
constexpr double SMOOTH_TAU_S = 0.05;     // pose smoothing: ~95 % of a change in 150 ms
constexpr double FADE_TAU_S = 0.08;       // opacity smoothing
constexpr double MEMORY_S = 30.0;         // the detector's memory_s: ghosts fade over this long
constexpr double PLACED_CONFIRM_S = 0.3;  // detections this long after the release can confirm a place
constexpr float PLACED_MATCH_M = 0.02f;   // ... when they are this close to where it was put
constexpr double PLACED_DISAGREE_S = 4.0; // fresh detections elsewhere for this long: the detector wins
constexpr float GRASP_TIP_HEIGHT = 0.011f;   // fixed jaw tip above the cube's bottom (pick_place.yaml)
constexpr float HOLD_GRIPPER_RAD = 12.5f * DEG2RAD;   // jaws closed on a 30 mm cube
const char * MOVING_JAW_LINK = "moving_jaw_so101_v1_link";

// Colour-blind friendly cube colours by id (Okabe-Ito, without the black and the dark blue that
// vanish on the dark background).
Color cube_color(int k)
{
  static const Color palette[] = {{230, 159, 0, 255}, {86, 180, 233, 255}, {0, 158, 115, 255},
    {240, 228, 66, 255}, {213, 94, 0, 255}, {204, 121, 167, 255}};
  return k < 0 ? Color{190, 200, 204, 255} : palette[k % 6];
}

// The cube looks the same every 90 deg: fold an angle into [-45, 45) deg.
float fold_quarter(float a)
{
  const float q = PI / 2;
  return a - q * std::floor((a + q / 2) / q);
}

Color with_alpha(Color c, float a)
{
  return Color{c.r, c.g, c.b, static_cast<unsigned char>(std::clamp(a, 0.0f, 1.0f) * 255.0f)};
}

Color scaled(Color c, float k, float a)
{
  const auto ch = [k](unsigned char v) {return static_cast<unsigned char>(std::clamp(v * k, 0.0f, 255.0f));};
  return with_alpha(Color{ch(c.r), ch(c.g), ch(c.b), 255}, a);
}

Color mixed(Color a, Color b, float t)
{
  const auto ch = [t](unsigned char x, unsigned char y) {return static_cast<unsigned char>(x + (y - x) * t);};
  return Color{ch(a.r, b.r), ch(a.g, b.g), ch(a.b, b.b), ch(a.a, b.a)};
}

Vector3 column(const Matrix & m, int i)
{
  return i == 0 ? Vector3{m.m0, m.m1, m.m2} : i == 1 ? Vector3{m.m4, m.m5, m.m6} : Vector3{m.m8, m.m9, m.m10};
}

Matrix frame_matrix(Vector3 x, Vector3 y, Vector3 z, Vector3 p)
{
  Matrix m = MatrixIdentity();
  m.m0 = x.x; m.m1 = x.y; m.m2 = x.z;
  m.m4 = y.x; m.m5 = y.y; m.m6 = y.z;
  m.m8 = z.x; m.m9 = z.y; m.m10 = z.z;
  m.m12 = p.x; m.m13 = p.y; m.m14 = p.z;
  return m;
}

void vertex(Vector3 p) {rlVertex3f(p.x, p.y, p.z);}

void quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color col)   // counter-clockwise seen from the front
{
  rlColor4ub(col.r, col.g, col.b, col.a);
  vertex(a); vertex(b); vertex(c);
  vertex(a); vertex(c); vertex(d);
}

// A box of edge e around the origin of the current matrix, each face lit by the key light
// (`world` turns the face normals for the lighting).
void draw_shaded_box(float e, Color base, float alpha, const Matrix & world)
{
  static const Vector3 L = Vector3Normalize(Vector3{0.3f, -0.4f, 1.0f});
  static const Vector3 faces[6][3] = {   // normal, u, v with u x v = normal
    {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}},
    {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}, {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
    {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}, {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}};
  const float h = e / 2;
  rlBegin(RL_TRIANGLES);
  for (const auto & f : faces) {
    const Vector3 n = f[0], u = Vector3Scale(f[1], h), v = Vector3Scale(f[2], h), c = Vector3Scale(n, h);
    const Vector3 nw = Vector3Normalize(Vector3Subtract(Vector3Transform(n, world), Vector3Transform(Vector3Zero(), world)));
    const float k = 0.42f + 0.58f * std::max(0.0f, Vector3DotProduct(nw, L)) + (nw.z > 0.7f ? 0.08f : 0.0f);
    quad(Vector3Subtract(Vector3Subtract(c, u), v), Vector3Subtract(Vector3Add(c, u), v),
      Vector3Add(Vector3Add(c, u), v), Vector3Add(Vector3Subtract(c, u), v), scaled(base, k, alpha));
  }
  rlEnd();
}

// The 12 edges of a box of edge e, dashed when the estimate is not stable.
void draw_box_edges(float e, Color c, bool dashed)
{
  const float h = e / 2 * 1.004f;
  for (int axis = 0; axis < 3; ++axis) {
    for (int s = 0; s < 4; ++s) {
      const float a = (s & 1) ? h : -h, b = (s & 2) ? h : -h;
      Vector3 p0, p1;
      if (axis == 0) {p0 = {-h, a, b}; p1 = {h, a, b};}
      else if (axis == 1) {p0 = {a, -h, b}; p1 = {a, h, b};}
      else {p0 = {a, b, -h}; p1 = {a, b, h};}
      if (!dashed) {
        DrawLine3D(p0, p1, c);
        continue;
      }
      constexpr int DASHES = 4;
      for (int i = 0; i < DASHES; ++i) {
        const float t0 = (i + 0.15f) / DASHES, t1 = (i + 0.75f) / DASHES;
        DrawLine3D(Vector3Lerp(p0, p1, t0), Vector3Lerp(p0, p1, t1), c);
      }
    }
  }
}

// An ArUco-like marker on the top face: black square, a 4x4 bit pattern from the id.
void draw_top_marker(float e, int k, float alpha)
{
  const float s = e * (22.0f / 30.0f) / 2, z = e / 2 + 0.0003f, cell = 2 * s / 6;
  rlBegin(RL_TRIANGLES);
  quad({-s, -s, z}, {s, -s, z}, {s, s, z}, {-s, s, z}, Color{12, 14, 16, static_cast<unsigned char>(alpha * 235)});
  uint32_t bits = static_cast<uint32_t>(k + 7) * 2654435761u;
  bits ^= bits >> 15;
  const Color white{226, 232, 232, static_cast<unsigned char>(alpha * 255)};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (!((bits >> (i * 4 + j)) & 1u)) {
        continue;
      }
      const float x0 = -s + (i + 1) * cell, y0 = -s + (j + 1) * cell, zz = z + 0.0001f;
      quad({x0, y0, zz}, {x0 + cell, y0, zz}, {x0 + cell, y0 + cell, zz}, {x0, y0 + cell, zz}, white);
    }
  }
  rlEnd();
}

// A soft round shadow on the plane z: dark in the middle, fading out at the rim.
void draw_soft_disc(Vector3 c, float r, float alpha)
{
  constexpr int N = 28;
  const unsigned char a = static_cast<unsigned char>(std::clamp(alpha, 0.0f, 1.0f) * 255);
  rlBegin(RL_TRIANGLES);
  for (int i = 0; i < N; ++i) {
    const float t0 = i * 2 * PI / N, t1 = (i + 1) * 2 * PI / N;
    rlColor4ub(0, 0, 0, a);
    vertex(c);
    rlColor4ub(0, 0, 0, 0);
    vertex(Vector3{c.x + r * std::cos(t0), c.y + r * std::sin(t0), c.z});
    vertex(Vector3{c.x + r * std::cos(t1), c.y + r * std::sin(t1), c.z});
  }
  rlEnd();
}

// A contact shadow: a dark square (yaw) under a resting cube, soft towards `outer`.
void draw_soft_square(Vector3 c, float inner, float outer, float yaw, float alpha)
{
  const unsigned char a = static_cast<unsigned char>(std::clamp(alpha, 0.0f, 1.0f) * 255);
  const float cs = std::cos(yaw), sn = std::sin(yaw);
  const auto at = [&](float x, float y) {return Vector3{c.x + cs * x - sn * y, c.y + sn * x + cs * y, c.z};};
  const float sx[4] = {-1, 1, 1, -1}, sy[4] = {-1, -1, 1, 1};
  rlBegin(RL_TRIANGLES);
  rlColor4ub(0, 0, 0, a);
  vertex(at(-inner, -inner)); vertex(at(inner, -inner)); vertex(at(inner, inner));
  vertex(at(-inner, -inner)); vertex(at(inner, inner)); vertex(at(-inner, inner));
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    const Vector3 i0 = at(sx[i] * inner, sy[i] * inner), i1 = at(sx[j] * inner, sy[j] * inner);
    const Vector3 o0 = at(sx[i] * outer, sy[i] * outer), o1 = at(sx[j] * outer, sy[j] * outer);
    rlColor4ub(0, 0, 0, a); vertex(i0);
    rlColor4ub(0, 0, 0, 0); vertex(o0); vertex(o1);
    rlColor4ub(0, 0, 0, a); vertex(i0);
    rlColor4ub(0, 0, 0, 0); vertex(o1);
    rlColor4ub(0, 0, 0, a); vertex(i1);
  }
  rlEnd();
}

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
  // what the cube view needs from the meshes: the moving jaw's tip (the vertex farthest from its
  // joint, as so101_manipulation's kinematics take it) and the base's footprint on the table
  have_moving_tip_ = false;
  float tip_norm = -1.0f;
  const std::string root = model_->getRoot() ? model_->getRoot()->name : "";
  std::vector<Vector3> base_points;
  float base_low = 1e9f;
  for (const Visual & v : visuals_) {
    if (v.table || (v.link != MOVING_JAW_LINK && v.link != root)) {
      continue;
    }
    const Mesh & mesh = meshes_[v.mesh];
    for (int i = 0; mesh.vertices && i < mesh.vertexCount; ++i) {
      const Vector3 p = Vector3Transform(Vector3{mesh.vertices[3 * i], mesh.vertices[3 * i + 1],
          mesh.vertices[3 * i + 2]}, v.origin);
      if (v.link == MOVING_JAW_LINK) {
        if (Vector3Length(p) > tip_norm) {
          tip_norm = Vector3Length(p);
          moving_tip_local_ = p;
          have_moving_tip_ = true;
        }
      } else {
        base_points.push_back(p);
        base_low = std::min(base_low, p.z);
      }
    }
  }
  base_min_ = base_max_ = Vector2{0, 0};
  bool first = true;
  for (const Vector3 & p : base_points) {   // the footprint: the lowest 15 mm of the base
    if (p.z > base_low + 0.015f) {
      continue;
    }
    base_min_ = first ? Vector2{p.x, p.y} : Vector2{std::min(base_min_.x, p.x), std::min(base_min_.y, p.y)};
    base_max_ = first ? Vector2{p.x, p.y} : Vector2{std::max(base_max_.x, p.x), std::max(base_max_.y, p.y)};
    first = false;
  }
  if (have_moving_tip_) {   // the tip distance with the jaws closed on a cube
    const LinkPoses p = forward_kinematics(JointValues{{"gripper", HOLD_GRIPPER_RAD}});
    if (p.count("gripper_frame_link") && p.count(MOVING_JAW_LINK)) {
      hold_gap_ = Vector3Distance(translation(p.at("gripper_frame_link")),
          Vector3Transform(moving_tip_local_, p.at(MOVING_JAW_LINK)));
    }
  }
  TraceLog(LOG_INFO, "3D view: %s, %zu links, %zu visuals, jaw tip gap at %.1f deg %.1f mm, base %.0fx%.0f mm",
    model_->getName().c_str(), model_->links_.size(), visuals_.size(), HOLD_GRIPPER_RAD * RAD2DEG,
    hold_gap_ * 1000.0f, (base_max_.x - base_min_.x) * 1000.0f, (base_max_.y - base_min_.y) * 1000.0f);
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
    const float surface = c.z + table_size_.z / 2;
    table_top_ = surface;   // clicks on the table hit this plane
    table_centre_ = Vector2{c.x, c.y};
    const float top = surface + 0.0004f;   // lines just above the surface
    const float hx = table_size_.x / 2, hy = table_size_.y / 2;
    const float x0 = c.x - hx, x1 = c.x + hx, y0 = c.y - hy, y1 = c.y + hy;

    // the surface: a solid slab, lit a little brighter around the work area in front of the base
    const Color lit{36, 62, 70, 255}, dark{15, 24, 30, 255};
    const auto shade = [&](float x, float y) {
        const float d = std::hypot(x - 0.2f, y) / 0.5f;
        return mixed(lit, dark, std::clamp(d * d, 0.0f, 1.0f));
      };
    constexpr int N = 16;
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < N; ++j) {
        const float xa = x0 + (x1 - x0) * i / N, xb = x0 + (x1 - x0) * (i + 1) / N;
        const float ya = y0 + (y1 - y0) * j / N, yb = y0 + (y1 - y0) * (j + 1) / N;
        const Vector3 p[4] = {{xa, ya, surface}, {xb, ya, surface}, {xb, yb, surface}, {xa, yb, surface}};
        const Color col[4] = {shade(xa, ya), shade(xb, ya), shade(xb, yb), shade(xa, yb)};
        for (int k : {0, 1, 2, 0, 2, 3}) {
          rlColor4ub(col[k].r, col[k].g, col[k].b, 255);
          vertex(p[k]);
        }
      }
    }
    rlEnd();
    // its edge faces, darker, so it reads as a slab
    const float bottom = surface - table_size_.z;
    const Color side{9, 15, 19, 255};
    rlBegin(RL_TRIANGLES);
    quad({x0, y0, bottom}, {x1, y0, bottom}, {x1, y0, surface}, {x0, y0, surface}, side);
    quad({x1, y0, bottom}, {x1, y1, bottom}, {x1, y1, surface}, {x1, y0, surface}, side);
    quad({x1, y1, bottom}, {x0, y1, bottom}, {x0, y1, surface}, {x1, y1, surface}, side);
    quad({x0, y1, bottom}, {x0, y0, bottom}, {x0, y0, surface}, {x0, y1, surface}, side);
    rlEnd();

    // grid: 5 cm minor, 10 cm major, on base_link multiples
    const ImVec4 & cy = theme::CYAN;
    const Color minor = theme::to_raylib(ImVec4(cy.x, cy.y, cy.z, 0.07f));
    const Color major = theme::to_raylib(ImVec4(cy.x, cy.y, cy.z, 0.17f));
    for (int i = static_cast<int>(std::ceil(x0 / GRID_STEP - 1e-3f)); i * GRID_STEP <= x1 + 1e-4f; ++i) {
      DrawLine3D(Vector3{i * GRID_STEP, y0, top}, Vector3{i * GRID_STEP, y1, top}, i % 2 ? minor : major);
    }
    for (int i = static_cast<int>(std::ceil(y0 / GRID_STEP - 1e-3f)); i * GRID_STEP <= y1 + 1e-4f; ++i) {
      DrawLine3D(Vector3{x0, i * GRID_STEP, top}, Vector3{x1, i * GRID_STEP, top}, i % 2 ? minor : major);
    }
    // distance rings around the base axis (labelled in draw_table_labels), clipped to the table
    const Color ring = theme::to_raylib(ImVec4(theme::TEXT.x, theme::TEXT.y, theme::TEXT.z, 0.16f));
    for (float r : RINGS_M) {
      constexpr int SEG = 144;
      for (int s = 0; s < SEG; ++s) {
        const float t0 = s * 2 * PI / SEG, t1 = (s + 1) * 2 * PI / SEG;
        const Vector3 a{r * std::cos(t0), r * std::sin(t0), top}, b{r * std::cos(t1), r * std::sin(t1), top};
        if (a.x >= x0 && a.x <= x1 && a.y >= y0 && a.y <= y1 && b.x >= x0 && b.x <= x1 && b.y >= y0 && b.y <= y1) {
          DrawLine3D(a, b, ring);
        }
      }
    }
    const Color edge = theme::to_raylib(ImVec4(cy.x, cy.y, cy.z, 0.5f));
    const Vector3 corners[4] = {{x0, y0, top}, {x1, y0, top}, {x1, y1, top}, {x0, y1, top}};
    for (int i = 0; i < 4; ++i) {
      DrawLine3D(corners[i], corners[(i + 1) % 4], edge);
    }
  }
  // the base's footprint: a dark plate with an outline
  if (base_max_.x > base_min_.x) {
    const float z = table_top_ + 0.0007f, pad = 0.004f;   // above the grid lines and the shadows
    const Vector3 a{base_min_.x - pad, base_min_.y - pad, z}, b{base_max_.x + pad, base_min_.y - pad, z},
      c{base_max_.x + pad, base_max_.y + pad, z}, d{base_min_.x - pad, base_max_.y + pad, z};
    rlBegin(RL_TRIANGLES);
    quad(a, b, c, d, Color{6, 12, 16, 150});
    rlEnd();
    const Color outline = theme::to_raylib(ImVec4(theme::CYAN.x, theme::CYAN.y, theme::CYAN.z, 0.35f));
    DrawLine3D(a, b, outline);
    DrawLine3D(b, c, outline);
    DrawLine3D(c, d, outline);
    DrawLine3D(d, a, outline);
  }
  // base frame axes, 5 cm
  DrawLine3D(Vector3{0, 0, 0}, Vector3{0.05f, 0, 0}, theme::to_raylib(theme::ALERT));
  DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 0.05f, 0}, theme::to_raylib(theme::OK));
  DrawLine3D(Vector3{0, 0, 0}, Vector3{0, 0, 0.05f}, theme::to_raylib(theme::CYAN));
}

void RobotView::draw_shadows(const LinkPoses & poses) const
{
  const float z = table_top_ + 0.0005f;
  rlDrawRenderBatchActive();
  rlDisableDepthMask();   // they overlap each other and must not hide the grid behind them
  for (const ViewCube & vc : cubes_) {
    const Vector3 p = translation(vc.pose);
    const float e = vc.marker.edge;
    const float h = std::max(0.0f, p.z - e / 2 - table_top_);   // the cube's bottom above the table
    const float fade = std::clamp(1.0f - h / 0.25f, 0.0f, 1.0f) * vc.alpha;
    if (fade <= 0.01f) {
      continue;
    }
    if (h < 0.004f) {   // resting on the table: a contact shadow hugging its footprint
      draw_soft_square(Vector3{p.x, p.y, z}, e * 0.5f, e * 0.85f, vc.marker.yaw, 0.55f * fade);
    }
    draw_soft_disc(Vector3{p.x + 0.004f, p.y - 0.004f, z}, e * 0.9f + h * 0.35f, 0.38f * fade);
  }
  // the gripper: under the jaws and, wider and fainter, under the wrist
  const auto tip = poses.find("gripper_frame_link");
  const auto wrist = poses.find("gripper_link");
  if (tip != poses.end() && wrist != poses.end()) {
    Vector3 jaws = translation(tip->second);
    const auto jaw = poses.find(MOVING_JAW_LINK);
    if (have_moving_tip_ && jaw != poses.end()) {
      jaws = Vector3Lerp(jaws, Vector3Transform(moving_tip_local_, jaw->second), 0.5f);
    }
    const Vector3 w = translation(wrist->second);
    const float hj = std::max(0.0f, jaws.z - table_top_), hw = std::max(0.0f, w.z - table_top_);
    draw_soft_disc(Vector3{jaws.x, jaws.y, z}, 0.022f + hj * 0.3f, 0.5f * std::clamp(1.0f - hj / 0.3f, 0.0f, 1.0f));
    draw_soft_disc(Vector3{w.x, w.y, z}, 0.04f + hw * 0.3f, 0.3f * std::clamp(1.0f - hw / 0.35f, 0.0f, 1.0f));
  }
  rlDrawRenderBatchActive();
  rlEnableDepthMask();
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
  draw_shadows(poses);
  draw_robot(poses, Color{14, 34, 42, 255}, theme::to_raylib(theme::CYAN));
  draw_cubes(false);   // solid ones first, they write depth
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
  draw_cubes(true);    // ghosts, placed and fading cubes
  draw_pick_place_3d();
  // the camera's field of view: thin and faint, it must not hide the cubes
  const Color fov = theme::to_raylib(ImVec4(theme::ROSE.x, theme::ROSE.y, theme::ROSE.z, 0.22f));
  rlDrawRenderBatchActive();
  rlDisableDepthMask();
  for (size_t i = 0; i + 1 < fov_world_.size(); i += 2) {
    DrawLine3D(fov_world_[i], fov_world_[i + 1], fov);
  }
  rlDrawRenderBatchActive();
  rlEnableDepthMask();
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

bool RobotView::jaw_cube_pose(const LinkPoses & poses, float edge, Matrix & out) const
{
  const auto tip = poses.find("gripper_frame_link");
  const auto jaw = poses.find(MOVING_JAW_LINK);
  if (!have_moving_tip_ || tip == poses.end() || jaw == poses.end()) {
    return false;
  }
  const Vector3 fixed = translation(tip->second);
  const Vector3 moving = Vector3Transform(moving_tip_local_, jaw->second);
  const float gap = Vector3Distance(fixed, moving);
  if (gap < 1e-4f) {
    return false;
  }
  const Vector3 close = Vector3Scale(Vector3Subtract(moving, fixed), 1.0f / gap);
  // approach: wrist -> tip = +z of gripper_frame_link; the tips sit GRASP_TIP_HEIGHT above the
  // cube's bottom, so the centre is a little behind them. Between the jaws: the gap midpoint as
  // the pick node grasps, the gap capped at the closed-on-a-cube width (opening jaws do not move it)
  const Vector3 approach = Vector3Normalize(column(tip->second, 2));
  const Vector3 centre = Vector3Subtract(Vector3Add(fixed, Vector3Scale(close, std::min(gap, hold_gap_) / 2)),
      Vector3Scale(approach, edge / 2 - GRASP_TIP_HEIGHT));
  const Vector3 z = Vector3Negate(approach);
  const Vector3 x = Vector3Normalize(Vector3Subtract(close, Vector3Scale(z, Vector3DotProduct(close, z))));
  out = frame_matrix(x, Vector3CrossProduct(z, x), z, centre);
  return true;
}

void RobotView::update_cubes(const LinkPoses & poses)
{
  const double now = now_s();
  const float dt = last_update_ < 0 ? 1.0f : static_cast<float>(std::clamp(now - last_update_, 0.0, 0.5));
  last_update_ = now;
  const float k_pose = 1.0f - std::exp(-dt / static_cast<float>(SMOOTH_TAU_S));
  const float k_fade = 1.0f - std::exp(-dt / static_cast<float>(FADE_TAU_S));
  const std::string root = model_->getRoot()->name;

  // what the detector says, per cube: /cubes/tracked first (the source of truth), the markers
  // only for cubes it does not list
  struct Obs {Vector3 p; float yaw, edge; double age; bool stable;};
  std::map<int, Obs> obs;
  if (root == "base_link") {
    for (const auto & [id, t] : in_tracked_) {
      obs[id] = Obs{{t.position[0], t.position[1], t.position[2]}, t.yaw, t.edge,
        t.received >= 0 ? std::max(0.0, now - t.seen) : 0.0, t.score >= 0.5f};   // (seen < 0 is valid early on)
    }
  }
  for (const auto & [id, m] : in_markers_) {
    if (obs.count(id) || m.frame != root) {
      continue;
    }
    obs[id] = Obs{{m.position[0], m.position[1], m.position[2]}, m.yaw, m.edge,
      m.rgba[3] >= 0.5f ? 0.0 : LIVE_S + 1.0, m.label.find('~') == std::string::npos};
  }

  // the cube between the jaws
  const float held_edge = held_ >= 0 && obs.count(held_) ? obs.at(held_).edge : 0.03f;
  Matrix jaw{};
  const bool jaw_ok = holding_ && jaw_cube_pose(poses, held_edge, jaw);
  const int held_key = held_ >= 0 ? held_ : -1;   // -1: a cube held but not known which

  // released: put down by a place (the node clears `held` at the release, or the dashboard's own
  // record at the place result) -> where the jaws let it go, on the table or on a stack
  const bool placing = in_pick_node_.action == "place" ||
    (in_task_running_ && (in_task_label_.rfind("PLACE", 0) == 0 || in_task_label_.rfind("STACK", 0) == 0)) ||
    (in_placed_.seq != placed_seq_ && in_placed_.cube == prev_held_) ||
    (in_pick_node_.last_action == "place" && in_pick_node_.last_success &&
    cube_index(in_pick_node_.last_cube) == prev_held_);
  if (prev_held_ >= 0 && prev_held_ != held_key && placing && last_jaw_valid_) {
    const Vector3 c = translation(last_jaw_pose_);
    const float e = held_edge, level = std::max(0.0f, std::round((c.z - table_top_ - e / 2) / e));
    const Vector3 x = column(last_jaw_pose_, 0);
    placed_[prev_held_] = Placed{{c.x, c.y, table_top_ + e / 2 + level * e}, fold_quarter(std::atan2(x.y, x.x)),
      now, -1.0};
    TraceLog(LOG_INFO, "3D view: K%d released at %.1f, %.1f cm (level %.0f): placed", prev_held_, c.x * 100.0f,
      c.y * 100.0f, level);
  }
  // the place result: where the node says the cube centre was released
  if (in_placed_.seq != placed_seq_) {
    placed_seq_ = in_placed_.seq;
    if (in_placed_.cube >= 0 && in_placed_.cube != held_key) {
      auto it = placed_.find(in_placed_.cube);
      Placed p = it != placed_.end() ? it->second : Placed{{}, 0.0f, in_placed_.at, -1.0};
      p.pos = Vector3{in_placed_.position[0], in_placed_.position[1], in_placed_.position[2]};
      if (std::isfinite(in_placed_.yaw_deg)) {
        p.yaw = fold_quarter(in_placed_.yaw_deg * DEG2RAD);
      }
      placed_[in_placed_.cube] = p;
      TraceLog(LOG_INFO, "3D view: K%d placed at %.1f, %.1f, %.1f cm (place result)", in_placed_.cube,
        p.pos.x * 100.0f, p.pos.y * 100.0f, p.pos.z * 100.0f);
    }
  }
  if (held_key != prev_held_ && held_key >= 0) {
    placed_.erase(held_key);   // picked up again
  }
  prev_held_ = held_key;
  last_jaw_valid_ = jaw_ok;
  if (jaw_ok) {
    last_jaw_pose_ = jaw;
  }
  // a placed cube until the detector sees it there (then its estimate takes over), or keeps
  // seeing it elsewhere for PLACED_DISAGREE_S (moved by hand, or the place went wrong)
  for (auto it = placed_.begin(); it != placed_.end(); ) {
    const auto o = obs.find(it->first);
    bool drop = false;
    if (o != obs.end() && now - o->second.age > it->second.at + PLACED_CONFIRM_S) {
      const Vector3 d = Vector3Subtract(o->second.p, it->second.pos);
      if (std::hypot(d.x, d.y) < PLACED_MATCH_M && std::abs(d.z) < PLACED_MATCH_M) {
        drop = true;
        TraceLog(LOG_INFO, "3D view: K%d placed: confirmed by the detector", it->first);
      } else {
        if (it->second.disagree_since < 0) {
          it->second.disagree_since = now;
        }
        drop = now - it->second.disagree_since > PLACED_DISAGREE_S;
        if (drop) {
          TraceLog(LOG_INFO, "3D view: K%d placed: the detector sees it %.1f cm away, using its estimate", it->first,
            std::hypot(d.x, d.y) * 100.0f);
        }
      }
    }
    it = drop ? placed_.erase(it) : std::next(it);
  }

  // every cube to draw: its state, target pose and opacity; smoothed towards them
  std::vector<int> ids;
  for (const auto & [id, o] : obs) {
    ids.push_back(id);
  }
  for (const auto & [id, p] : placed_) {
    if (!obs.count(id)) {
      ids.push_back(id);
    }
  }
  if (jaw_ok && !obs.count(held_key) && !placed_.count(held_key)) {
    ids.push_back(held_key);
  }
  std::vector<ViewCube> cubes;
  for (int id : ids) {
    ViewCube vc;
    vc.index = id;
    Vector3 target;
    float yaw = 0.0f, alpha = 1.0f, edge = 0.03f;
    const auto o = obs.find(id);
    if (o != obs.end()) {
      edge = o->second.edge;
      vc.stable = o->second.stable;
      vc.age = o->second.age;
    }
    const bool in_jaws = jaw_ok && id == held_key;
    if (in_jaws) {
      vc.state = CubeState::Held;
      target = translation(jaw);
      const Vector3 x = column(jaw, 0);
      yaw = std::atan2(x.y, x.x);
      vc.stable = true;
    } else if (placed_.count(id)) {
      vc.state = CubeState::Placed;
      target = placed_.at(id).pos;
      yaw = placed_.at(id).yaw;
      alpha = 0.5f;
      vc.stable = true;
    } else if (o != obs.end()) {
      target = o->second.p;
      yaw = o->second.yaw;
      vc.state = o->second.age < LIVE_S ? CubeState::Live : CubeState::Remembered;
      alpha = vc.state == CubeState::Live ? 1.0f :
        0.42f - 0.22f * static_cast<float>(std::min(1.0, o->second.age / MEMORY_S));
    } else {
      continue;
    }
    const bool fresh = !tracks_.count(id);
    CubeTrack & t = tracks_[id];
    if (fresh) {   // a new cube: in place, fading in
      t.pos = target;
      t.yaw = yaw;
      t.alpha = 0.0f;
    }
    if (in_jaws) {
      if (t.held_since < 0) {
        t.held_since = now;
      }
      // a short slide into the jaws, then it moves with them exactly
      t.pos = now - t.held_since > 0.15 ? target : Vector3Lerp(t.pos, target, k_pose);
      t.yaw = yaw;
    } else {
      t.held_since = -1.0;
      t.pos = Vector3Lerp(t.pos, target, k_pose);
      t.yaw += fold_quarter(yaw - t.yaw) * k_pose;   // the shorter way (the cube repeats every 90 deg)
    }
    t.alpha += (alpha - t.alpha) * k_fade;
    vc.alpha = t.alpha;
    vc.pose = in_jaws ? frame_matrix(column(jaw, 0), column(jaw, 1), column(jaw, 2), t.pos) :
      MatrixMultiply(MatrixRotateZ(t.yaw), MatrixTranslate(t.pos.x, t.pos.y, t.pos.z));
    vc.marker.frame = root;
    vc.marker.position = {t.pos.x, t.pos.y, t.pos.z};
    vc.marker.yaw = t.yaw;
    vc.marker.edge = edge;
    const Color col = cube_color(id);
    vc.marker.rgba = {col.r / 255.0f, col.g / 255.0f, col.b / 255.0f, vc.alpha};
    vc.marker.label = id >= 0 ? "K" + std::to_string(id) : "cube";
    t.last = vc;
    cubes.push_back(vc);
  }
  // gone from every source (dropped from the tracker's memory, or a held cube without a pose):
  // fade out where it was last drawn, then forget it
  for (auto it = tracks_.begin(); it != tracks_.end(); ) {
    const bool shown = std::any_of(cubes.begin(), cubes.end(), [&](const ViewCube & c) {return c.index == it->first;});
    if (shown) {
      ++it;
      continue;
    }
    CubeTrack & t = it->second;
    t.alpha -= t.alpha * k_fade;
    if (t.alpha < 0.02f) {
      it = tracks_.erase(it);
      continue;
    }
    ViewCube vc = t.last;
    vc.alpha = t.alpha;
    vc.marker.rgba[3] = t.alpha;
    if (vc.state == CubeState::Live) {
      vc.state = CubeState::Remembered;
    }
    cubes.push_back(vc);
    ++it;
  }
  cubes_.swap(cubes);
}

void RobotView::draw_cubes(bool translucent) const
{
  constexpr float SOLID = 0.97f;   // opacity from which a cube is drawn solid (writes depth)
  std::vector<const ViewCube *> pass;
  for (const auto & vc : cubes_) {
    if ((vc.alpha >= SOLID) != translucent && vc.alpha >= 0.01f) {
      pass.push_back(&vc);
    }
  }
  if (pass.empty()) {
    return;
  }
  rlDrawRenderBatchActive();
  if (translucent) {
    rlDisableDepthMask();
  }
  // faces and top markers, then all edges with one line width change, then the selection
  for (const ViewCube * vc : pass) {
    const float a = translucent ? vc->alpha : 1.0f;
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloatV(vc->pose).v);
    // remembered cubes: a ghost, faces fainter than the edges
    draw_shaded_box(vc->marker.edge, cube_color(vc->index), vc->state == CubeState::Remembered ? a * 0.55f : a,
      vc->pose);
    draw_top_marker(vc->marker.edge, vc->index, vc->state == CubeState::Remembered ? a * 0.5f : a);
    rlPopMatrix();
  }
  rlDrawRenderBatchActive();
  rlSetLineWidth(2.0f);
  for (const ViewCube * vc : pass) {
    const float a = translucent ? vc->alpha : 1.0f;
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloatV(vc->pose).v);
    draw_box_edges(vc->marker.edge, with_alpha(mixed(cube_color(vc->index), Color{255, 255, 255, 255}, 0.45f),
      std::min(1.0f, a * 1.6f)), !vc->stable);
    rlPopMatrix();
  }
  rlDrawRenderBatchActive();
  rlSetLineWidth(1.0f);
  for (const ViewCube * vc : pass) {
    // selection: a double outline that glows; stack target in amber
    const bool selected = !holding_ && vc->index >= 0 && vc->index == selection_.cube;
    const bool stack = holding_ && vc->index >= 0 && vc->index == selection_.stack_on;
    if (!selected && !stack) {
      continue;
    }
    const float e = vc->marker.edge;
    const ImVec4 & hl = selected ? theme::CYAN : theme::AMBER;
    const float pulse = 0.65f + 0.35f * std::sin(static_cast<float>(now_s()) * 5.0f);
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloatV(vc->pose).v);
    DrawCubeWires(Vector3Zero(), e * 1.2f, e * 1.2f, e * 1.2f, theme::to_raylib(hl));
    DrawCubeWires(Vector3Zero(), e * 1.4f, e * 1.4f, e * 1.4f, theme::to_raylib(ImVec4(hl.x, hl.y, hl.z, 0.5f * pulse)));
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    DrawCube(Vector3Zero(), e * 1.2f, e * 1.2f, e * 1.2f, theme::to_raylib(ImVec4(hl.x, hl.y, hl.z, 0.12f * pulse)));
    rlDrawRenderBatchActive();
    if (!translucent) {
      rlEnableDepthMask();
    }
    rlPopMatrix();
  }
  rlDrawRenderBatchActive();
  rlEnableDepthMask();
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
  struct Label {const ViewCube * cube; ImVec2 at, size; std::string text; bool tagged;};
  std::vector<Label> labels;
  for (const auto & vc : cubes_) {
    const CubeMarker & c = vc.marker;
    ImVec2 at;
    if (vc.alpha < 0.05f || !screen_of(Vector3{c.position[0], c.position[1], c.position[2] + c.edge}, origin, at)) {
      continue;
    }
    char info[64];
    const float h = c.position[2] - c.edge / 2 - table_top_;   // bottom above the table
    switch (vc.state) {
      case CubeState::Held:
        snprintf(info, sizeof(info), "in the jaws");
        break;
      case CubeState::Placed:
        snprintf(info, sizeof(info), "placed, not seen yet");
        break;
      case CubeState::Remembered:
        snprintf(info, sizeof(info), "last seen %.0f s ago", vc.age);
        break;
      default:
        if (h > 0.01f) {
          snprintf(info, sizeof(info), "%.1f, %.1f cm  z+%.0f", c.position[0] * 100.0f, c.position[1] * 100.0f,
            h * 100.0f);
        } else {
          snprintf(info, sizeof(info), "%.1f, %.1f cm", c.position[0] * 100.0f, c.position[1] * 100.0f);
        }
    }
    const char * tag = !holding_ && vc.index >= 0 && vc.index == selection_.cube ? "  SELECTED" :
      holding_ && vc.index >= 0 && vc.index == selection_.stack_on ? "  STACK HERE?" : "";
    char buf[128];
    snprintf(buf, sizeof(buf), "%s%s  %s%s", c.label.c_str(), vc.stable ? "" : " ~", info, tag);
    labels.push_back(Label{&vc, ImVec2(at.x + 6, at.y - 16), ImGui::CalcTextSize(buf), buf, *tag != 0});
  }
  // no overlaps (a stack puts its labels on top of each other): from the bottom of the view up,
  // a label that hits one already placed moves up above it
  std::sort(labels.begin(), labels.end(), [](const Label & l, const Label & r) {return l.at.y > r.at.y;});
  for (size_t i = 0; i < labels.size(); ++i) {
    for (bool moved = true; moved; ) {
      moved = false;
      for (size_t j = 0; j < i; ++j) {
        const Label & o = labels[j];
        Label & l = labels[i];
        if (l.at.x < o.at.x + o.size.x + 8 && o.at.x < l.at.x + l.size.x + 8 &&
          l.at.y < o.at.y + o.size.y + 3 && o.at.y < l.at.y + l.size.y + 3)
        {
          l.at.y = o.at.y - l.size.y - 3;
          moved = true;
        }
      }
    }
  }
  for (const Label & l : labels) {
    const ViewCube & vc = *l.cube;
    const CubeMarker & c = vc.marker;
    const ImVec2 text_at = l.at;
    const char * buf = l.text.c_str();
    if (l.tagged) {   // a glowing plate behind the selected cube's label
      const ImVec4 & hl = holding_ ? theme::AMBER : theme::CYAN;
      const ImVec2 size = l.size;
      dl->AddRectFilled(ImVec2(text_at.x - 4, text_at.y - 2), ImVec2(text_at.x + size.x + 4, text_at.y + size.y + 2),
        ImGui::GetColorU32(ImVec4(hl.x, hl.y, hl.z, 0.18f)), 3.0f);
      dl->AddRect(ImVec2(text_at.x - 4, text_at.y - 2), ImVec2(text_at.x + size.x + 4, text_at.y + size.y + 2),
        ImGui::GetColorU32(hl), 3.0f);
      dl->AddText(text_at, ImGui::GetColorU32(hl), buf);
      continue;
    }
    // the id in the cube's colour, the rest dimmer; ghosts and unconfirmed places fainter
    const float a = vc.state == CubeState::Live || vc.state == CubeState::Held ? 1.0f : 0.6f;
    const ImVec4 id_col(c.rgba[0], c.rgba[1], c.rgba[2], a);
    dl->AddRectFilled(ImVec2(text_at.x - 3, text_at.y - 1), ImVec2(text_at.x + l.size.x + 3, text_at.y + l.size.y + 1),
      ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.5f * a)), 3.0f);
    const std::string id = c.label + (vc.stable ? "" : " ~");
    dl->AddText(text_at, ImGui::GetColorU32(id_col), id.c_str());
    const float w = ImGui::CalcTextSize(id.c_str()).x;
    dl->AddText(ImVec2(text_at.x + w, text_at.y), ImGui::GetColorU32(ImVec4(theme::TEXT.x, theme::TEXT.y, theme::TEXT.z,
      0.75f * a)), buf + id.size());
  }
}

void RobotView::draw_table_labels(ImVec2 origin) const
{
  // on the ring, on the side facing the camera, turned a little off the arm
  const float t = std::atan2(camera_.position.y, camera_.position.x) + 0.35f;
  ImDrawList * dl = ImGui::GetWindowDrawList();
  const ImU32 col = ImGui::GetColorU32(ImVec4(theme::TEXT.x, theme::TEXT.y, theme::TEXT.z, 0.45f));
  for (float r : RINGS_M) {
    ImVec2 at;
    const Vector3 p{r * std::cos(t), r * std::sin(t), table_top_};
    if (std::abs(p.x - table_centre_.x) > table_size_.x / 2 || std::abs(p.y - table_centre_.y) > table_size_.y / 2 ||
      !screen_of(p, origin, at))
    {
      continue;
    }
    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f cm", r * 100.0f);
    dl->AddText(ImVec2(at.x + 3, at.y - 7), col, buf);
  }
}

void RobotView::draw_pick_place_3d() const
{
  // the reach band: where the pick node can grasp / put down a cube, per pan from its reach table
  // (the "every yaw" band clear, the "some yaw" band fainter), or the fixed ring without one;
  // brighter while a cube is held
  const float a = holding_ ? 0.55f : 0.24f;
  const ImVec4 & ok = theme::OK;
  const float z = table_top_ + 0.001f;
  const bool table = !in_reach_.rows.empty();
  const float pan0 = table ? in_reach_.rows.front().pan_deg : -180.0f;
  const float pan1 = table ? in_reach_.rows.back().pan_deg : 180.0f;
  const int steps = std::max(8, static_cast<int>((pan1 - pan0) / 2.5f));
  const auto at = [z](float pan_deg, float r) {
      const float t = pan_deg * DEG2RAD;
      return Vector3{r * std::cos(t), r * std::sin(t), z};
    };
  rlDrawRenderBatchActive();
  rlDisableDepthMask();
  for (int pass = 0; pass < (table ? 2 : 1); ++pass) {
    const bool every = pass == 1 || !table;
    const float k = every ? 1.0f : 0.45f;   // the some-yaw band fainter
    const Color fill = theme::to_raylib(ImVec4(ok.x, ok.y, ok.z, a * (every ? 0.28f : 0.14f)));
    const Color edge = theme::to_raylib(ImVec4(ok.x, ok.y, ok.z, a * k));
    float pn = 0, pf = 0;
    bool prev = false;
    for (int i = 0; i <= steps; ++i) {
      const float pan = pan0 + (pan1 - pan0) * i / steps;
      float n = 0, f = 0;
      const bool ok_here = reach_band(in_reach_, pan, every, n, f);
      if (ok_here && prev) {
        const float pp = pan0 + (pan1 - pan0) * (i - 1) / steps;
        rlBegin(RL_TRIANGLES);
        quad(at(pp, pn), at(pp, pf), at(pan, f), at(pan, n), fill);   // counter-clockwise from above
        rlEnd();
        DrawLine3D(at(pp, pn), at(pan, n), edge);
        DrawLine3D(at(pp, pf), at(pan, f), edge);
      }
      if (ok_here != prev || (ok_here && (i == 0 || i == steps))) {   // the band's ends
        if (ok_here) {
          DrawLine3D(at(pan, n), at(pan, f), edge);
        } else {
          const float pp = pan0 + (pan1 - pan0) * (i - 1) / steps;
          DrawLine3D(at(pp, pn), at(pp, pf), edge);
        }
      }
      prev = ok_here;
      pn = n;
      pf = f;
    }
  }
  rlDrawRenderBatchActive();
  rlEnableDepthMask();
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
    if (vc.state == CubeState::Held) {   // in the jaws: not a target, and must not block the table
      continue;
    }
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
    const std::string problem = reach_problem(state.reach, tx, ty);
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
    } else if (cmd == "perf") {   // frame rate and time spent in this view, means over the last frames
      char buf[160];
      const double frame = PerfStats::mean(state.perf.frame_ms, state.perf.frames);
      snprintf(buf, sizeof(buf), "%.0f fps, frame %.2f ms, 3D view %.2f ms (CPU, %d frames)",
        frame > 0 ? 1000.0 / frame : 0.0, frame, PerfStats::mean(state.perf.view_ms, state.perf.views),
        std::min(state.perf.frames, PerfStats::N));
      result = buf;
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
    // the cube sources, for update_cubes() after the forward kinematics
    in_markers_ = state.cubes;
    in_tracked_ = state.tracked;
    in_placed_ = state.placed;
    in_pick_node_ = state.pick_node;
    in_reach_ = state.reach;
    in_task_label_ = state.task.label;
    in_task_running_ = state.task.running;
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
  update_cubes(poses);

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
    draw_table_labels(pos);
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
