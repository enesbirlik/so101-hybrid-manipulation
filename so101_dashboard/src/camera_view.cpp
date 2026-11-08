#include "camera_view.hpp"

#include <algorithm>

#include <imgui.h>
#include <rlImGui.h>

#include "panels.hpp"
#include "theme.hpp"

namespace so101_dashboard
{

namespace
{

constexpr double STALE_S = 1.0;

// Viewfinder corners around the image.
void brackets(ImDrawList * dl, ImVec2 a, ImVec2 b, ImU32 colour)
{
  const float l = std::min(b.x - a.x, b.y - a.y) * 0.08f;
  dl->AddLine(a, ImVec2(a.x + l, a.y), colour, 2.0f);
  dl->AddLine(a, ImVec2(a.x, a.y + l), colour, 2.0f);
  dl->AddLine(ImVec2(b.x, a.y), ImVec2(b.x - l, a.y), colour, 2.0f);
  dl->AddLine(ImVec2(b.x, a.y), ImVec2(b.x, a.y + l), colour, 2.0f);
  dl->AddLine(ImVec2(a.x, b.y), ImVec2(a.x + l, b.y), colour, 2.0f);
  dl->AddLine(ImVec2(a.x, b.y), ImVec2(a.x, b.y - l), colour, 2.0f);
  dl->AddLine(b, ImVec2(b.x - l, b.y), colour, 2.0f);
  dl->AddLine(b, ImVec2(b.x, b.y - l), colour, 2.0f);
}

}  // namespace

CameraView::~CameraView()
{
  if (texture_.id != 0) {
    UnloadTexture(texture_);
  }
}

void CameraView::upload(const CameraFrame & frame)
{
  if (texture_.id == 0 || texture_.width != frame.width || texture_.height != frame.height) {
    if (texture_.id != 0) {
      UnloadTexture(texture_);
    }
    Image image{const_cast<uint8_t *>(frame.rgb.data()), frame.width, frame.height, 1,
      PIXELFORMAT_UNCOMPRESSED_R8G8B8};
    texture_ = LoadTextureFromImage(image);
    SetTextureFilter(texture_, TEXTURE_FILTER_BILINEAR);
  } else {
    UpdateTexture(texture_, frame.rgb.data());
  }
  version_ = frame.version;
}

void CameraView::draw(State & state, bool * open)
{
  if (!ImGui::Begin(WIN_CAMERA, open)) {
    ImGui::End();
    return;
  }
  const double now = now_s();
  std::vector<std::string> topics;
  std::string topic;
  double fps, last;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    topics = state.image_topics;
    topic = state.image_topic;
    fps = state.image_rate.hz(now);
    last = state.image_rate.last();
    if (state.frame.version != version_ && state.frame.width > 0) {
      upload(state.frame);
    }
  }

  // toolbar: topic, crosshair, rate
  ImGui::SetNextItemWidth(std::min(360.0f, ImGui::GetContentRegionAvail().x * 0.6f));
  if (ImGui::BeginCombo("##topic", topic.empty() ? "(no image topic)" : topic.c_str())) {
    for (const auto & t : topics) {
      if (ImGui::Selectable(t.c_str(), t == topic)) {
        std::lock_guard<std::mutex> lock(state.mutex);
        state.image_topic = t;
        state.frame = CameraFrame{};
      }
    }
    ImGui::EndCombo();
  }
  ImGui::SameLine();
  ImGui::Checkbox("crosshair", &crosshair_);
  char info[64];
  snprintf(info, sizeof(info), "%dx%d  %4.1f fps", texture_.width, texture_.height, fps);
  ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize(info).x - 12);
  ImGui::TextColored(fps > 10 ? theme::OK : fps > 0 ? theme::AMBER : theme::DIM, "%s", info);

  const ImVec2 pos = ImGui::GetCursorScreenPos();
  const ImVec2 avail = ImGui::GetContentRegionAvail();
  ImDrawList * dl = ImGui::GetWindowDrawList();
  const bool live = last >= 0 && now - last < STALE_S && texture_.id != 0;
  if (texture_.id != 0 && avail.x > 1 && avail.y > 1) {
    const float scale = std::min(avail.x / texture_.width, avail.y / texture_.height);
    const ImVec2 size(texture_.width * scale, texture_.height * scale);
    const ImVec2 a(pos.x + (avail.x - size.x) / 2, pos.y + (avail.y - size.y) / 2);
    const ImVec2 b(a.x + size.x, a.y + size.y);
    ImGui::SetCursorScreenPos(a);
    rlImGuiImageSize(&texture_, static_cast<int>(size.x), static_cast<int>(size.y));
    if (!live) {
      dl->AddRectFilled(a, b, ImGui::GetColorU32(ImVec4(0, 0, 0, 0.6f)));
    }
    brackets(dl, ImVec2(a.x + 6, a.y + 6), ImVec2(b.x - 6, b.y - 6), ImGui::GetColorU32(live ? theme::CYAN : theme::DIM));
    if (crosshair_) {
      const ImVec2 c((a.x + b.x) / 2, (a.y + b.y) / 2);
      const ImU32 col = ImGui::GetColorU32(ImVec4(theme::CYAN.x, theme::CYAN.y, theme::CYAN.z, 0.7f));
      dl->AddLine(ImVec2(c.x - 18, c.y), ImVec2(c.x - 5, c.y), col, 1.5f);
      dl->AddLine(ImVec2(c.x + 5, c.y), ImVec2(c.x + 18, c.y), col, 1.5f);
      dl->AddLine(ImVec2(c.x, c.y - 18), ImVec2(c.x, c.y - 5), col, 1.5f);
      dl->AddLine(ImVec2(c.x, c.y + 5), ImVec2(c.x, c.y + 18), col, 1.5f);
    }
    dl->AddRectFilled(ImVec2(a.x + 12, a.y + 12), ImVec2(a.x + 40 + ImGui::CalcTextSize("NO SIGNAL").x, a.y + 33),
      ImGui::GetColorU32(ImVec4(theme::BG.x, theme::BG.y, theme::BG.z, 0.75f)), 2.0f);
    if (live && static_cast<int>(now * 1.5) % 2 == 0) {
      dl->AddCircleFilled(ImVec2(a.x + 22, a.y + 22), 5.0f, ImGui::GetColorU32(theme::ALERT));
    }
    dl->AddText(ImVec2(a.x + 32, a.y + 13), ImGui::GetColorU32(live ? theme::TEXT : theme::AMBER),
      live ? "LIVE" : "NO SIGNAL");
  } else {
    const char * text = topic.empty() ? "no image topic" : "waiting for frames ...";
    const ImVec2 size = ImGui::CalcTextSize(text);
    dl->AddText(ImVec2(pos.x + (avail.x - size.x) / 2, pos.y + (avail.y - size.y) / 2),
      ImGui::GetColorU32(theme::DIM), text);
  }
  ImGui::End();
}

}  // namespace so101_dashboard
