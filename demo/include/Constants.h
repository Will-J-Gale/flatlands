#pragma once
#include <math/Vector2.h>
#include <imgui.h>

const float TIME_STEP = 1.0f / 150.0f;
const int NUM_ITRERATIONS = 20;
// const int NUM_ITRERATIONS = 1;
// const Vector2 GRAVITY = Vector2(0, 9.81 * 100.0f);
const Vector2 GRAVITY = Vector2(0,0);

//Colours
const ImU32 WHITE = ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
const ImU32 WHITE_A = ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 0.3f));
const ImU32 BLACK = ImGui::ColorConvertFloat4ToU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
const ImU32 RED = ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
const ImU32 GREEN = ImGui::ColorConvertFloat4ToU32(ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
const ImU32 BLUE = ImGui::ColorConvertFloat4ToU32(ImVec4(0.0f, 0.0f, 1.0f, 1.0f));
const ImU32 YELLOW = ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
const ImU32 MAGENTA = ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 0.0f, 1.0f, 1.0f));
const ImU32 CYAN = ImGui::ColorConvertFloat4ToU32(ImVec4(0.0f, 1.0f, 1.0f, 1.0f));