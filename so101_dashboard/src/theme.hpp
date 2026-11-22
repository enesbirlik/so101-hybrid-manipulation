// Retro-futuristic look: dark glass panels, neon accents, a mono font.
#pragma once

#include <imgui.h>
#include <raylib.h>

namespace so101_dashboard::theme
{

// Palette (ImGui colours; to_raylib() for the 3D view).
inline const ImVec4 BG{0.027f, 0.035f, 0.051f, 1.0f};        // #07090d
inline const ImVec4 PANEL{0.047f, 0.067f, 0.094f, 0.96f};    // #0c1118
inline const ImVec4 BORDER{0.114f, 0.231f, 0.267f, 1.0f};    // #1d3b44
inline const ImVec4 TEXT{0.81f, 0.914f, 0.925f, 1.0f};       // #cfe9ec
inline const ImVec4 DIM{0.373f, 0.478f, 0.502f, 1.0f};       // #5f7a80
inline const ImVec4 CYAN{0.208f, 0.941f, 0.878f, 1.0f};      // #35f0e0
inline const ImVec4 AMBER{1.0f, 0.702f, 0.278f, 1.0f};       // #ffb347
inline const ImVec4 ROSE{1.0f, 0.31f, 0.639f, 1.0f};        // #ff4fa3
inline const ImVec4 OK{0.427f, 1.0f, 0.541f, 1.0f};          // #6dff8a
inline const ImVec4 ALERT{1.0f, 0.302f, 0.369f, 1.0f};       // #ff4d5e
// (not MAGENTA/GREEN/RED: raylib defines those as macros)

constexpr float FONT_SIZE = 17.0f;
constexpr float TITLE_SIZE = 26.0f;

inline Color to_raylib(const ImVec4 & c)
{
  return Color{static_cast<unsigned char>(c.x * 255), static_cast<unsigned char>(c.y * 255),
    static_cast<unsigned char>(c.z * 255), static_cast<unsigned char>(c.w * 255)};
}

// rlImGui font callback: the mono font from the package share folder.
void load_fonts();
void apply_style();

}  // namespace so101_dashboard::theme
