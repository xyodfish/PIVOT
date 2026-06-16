#include "kinematic_viewer/kinematic_viewport_hud.h"

#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_ui_feedback.h"

#include "imgui.h"

#include <algorithm>
#include <string>

namespace kinematic_viewer {
    namespace {

        constexpr float kHudPad           = 10.0f;
        constexpr float kHudBgAlpha       = 0.90f;
        constexpr float kHudBottomHeight    = 105.0f;
        constexpr ImVec4 kHudAccentCyan     = ImVec4(0.60f, 0.80f, 1.00f, 1.00f);
        constexpr ImVec4 kHudPanelBg        = ImVec4(0.10f, 0.11f, 0.14f, 0.92f);
        constexpr ImVec4 kHudPanelBorder    = ImVec4(0.28f, 0.34f, 0.44f, 0.55f);

        std::string TrajectoryDisplayName(const DebugPlaybackState& playback_state) {
            if (playback_state.selected_trajectory_index >= 0 &&
                playback_state.selected_trajectory_index < static_cast<int>(playback_state.trajectory_files.size())) {
                const auto& entry = playback_state.trajectory_files[static_cast<size_t>(playback_state.selected_trajectory_index)];
                if (!entry.path.empty()) {
                    return PathBasename(entry.path);
                }
            }
            if (playback_state.trajectory_file_path[0] != '\0') {
                return PathBasename(playback_state.trajectory_file_path);
            }
            return "未加载轨迹";
        }

        const char* PlaybackStateLabel(const PlaybackStateMachine* playback_sm) {
            if (playback_sm == nullptr) {
                return "STOP";
            }
            if (playback_sm->IsPlaying()) {
                return "PLAY";
            }
            if (playback_sm->IsPaused()) {
                return "PAUSE";
            }
            return "STOP";
        }

        UiSemanticLevel PlaybackSemanticLevel(const PlaybackStateMachine* playback_sm) {
            if (playback_sm == nullptr) {
                return UiSemanticLevel::Info;
            }
            if (playback_sm->IsPlaying()) {
                return UiSemanticLevel::Success;
            }
            if (playback_sm->IsPaused()) {
                return UiSemanticLevel::Warning;
            }
            return UiSemanticLevel::Info;
        }

        struct HudPanelStyle {
            static constexpr int kColorCount = 8;

            void Push() {
                ImGui::PushStyleColor(ImGuiCol_WindowBg, kHudPanelBg);
                ImGui::PushStyleColor(ImGuiCol_Border, kHudPanelBorder);
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.93f, 0.95f, 0.98f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_TextDisabled, ImVec4(0.55f, 0.62f, 0.72f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.16f, 0.19f, 0.26f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.20f, 0.26f, 0.36f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_FrameBgActive, ImVec4(0.24f, 0.32f, 0.44f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImVec4(0.45f, 0.68f, 0.94f, 1.0f));
                ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
                ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);
            }

            void Pop() {
                ImGui::PopStyleVar(3);
                ImGui::PopStyleColor(kColorCount);
            }
        };

        void RenderStatusChipOnHud(const char* label, UiSemanticLevel level) {
            if (label == nullptr) {
                return;
            }
            const ImVec4 color = KinematicUiFeedback::SemanticColor(level);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(color.x, color.y, color.z, 0.28f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(color.x, color.y, color.z, 0.28f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(color.x, color.y, color.z, 0.28f));
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 999.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10.0f, 4.0f));
            ImGui::Button(label);
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(4);
        }

        ImGuiWindowFlags HudOverlayFlags() {
            return ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                   ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs |
                   ImGuiWindowFlags_AlwaysAutoResize;
        }

        ImGuiWindowFlags HudBottomFlags() {
            return ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                   ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav;
        }

        void RenderPlaybackBottomBar(const ViewportHudContext& ctx, float bottom_w) {
            const bool has_playback = ctx.playback_state != nullptr && ctx.playback_sm != nullptr && ctx.playback_player != nullptr &&
                                      ctx.scene != nullptr && ctx.playback_sm->HasKeyframes();

            const float total = has_playback ? std::max(0.0f, ctx.playback_sm->TotalDuration()) : 0.0f;

            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("时间 (s):");
            ImGui::SameLine();
            const float reserved_right = 72.0f;
            ImGui::SetNextItemWidth(std::max(120.0f, bottom_w - 150.0f - reserved_right));
            if (has_playback) {
                float scrub_time = ctx.playback_state->play_time;
                if (ImGui::SliderFloat("##viewport_timeline", &scrub_time, 0.0f, std::max(0.01f, total), "%.2f")) {
                    ctx.playback_sm->Seek(scrub_time);
                    ctx.playback_player->SampleAtCurrentTime(*ctx.playback_state, ctx.scene);
                }
            } else {
                ImGui::BeginDisabled();
                float placeholder = 0.0f;
                ImGui::SliderFloat("##viewport_timeline_disabled", &placeholder, 0.0f, 1.0f, "%.2f");
                ImGui::EndDisabled();
            }
            ImGui::SameLine();
            ImGui::TextColored(kHudAccentCyan, "%.2fs", total);

            if (!has_playback) {
                ImGui::BeginDisabled();
            }
            const bool playing = has_playback && ctx.playback_sm->IsPlaying();
            if (playing) {
                if (ImGui::Button("暂停")) {
                    ctx.playback_sm->Pause();
                }
            } else {
                if (ImGui::Button("播放")) {
                    ctx.playback_sm->Play();
                }
            }
            if (!has_playback) {
                ImGui::EndDisabled();
            }

            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("速度");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            if (has_playback) {
                float speed = ctx.playback_state->play_speed;
                if (ImGui::SliderFloat("##viewport_speed", &speed, -2.0f, 2.0f, "%.2f")) {
                    ctx.playback_state->play_speed = speed;
                }
            } else {
                ImGui::BeginDisabled();
                float speed_placeholder = 1.0f;
                ImGui::SliderFloat("##viewport_speed_disabled", &speed_placeholder, -2.0f, 2.0f, "%.2f");
                ImGui::EndDisabled();
            }

            ImGui::SameLine();
            if (has_playback) {
                ImGui::Checkbox("循环", &ctx.playback_state->loop);
            } else {
                ImGui::BeginDisabled();
                bool loop_placeholder = false;
                ImGui::Checkbox("循环", &loop_placeholder);
                ImGui::EndDisabled();
            }
        }

    }  // namespace

    void RenderViewportHud(const ViewportHudContext& ctx) {
        if (ctx.viewport_w <= 0 || ctx.viewport_h <= 0 || ctx.ui_state == nullptr) {
            return;
        }

        const float bottom_y = static_cast<float>(ctx.viewport_h) - kHudBottomHeight - kHudPad;
        const float bottom_w = std::max(320.0f, static_cast<float>(ctx.viewport_w) - kHudPad * 2.0f);

        HudPanelStyle hud_style;
        hud_style.Push();
        ImGui::SetNextWindowBgAlpha(kHudBgAlpha);

        ImGui::SetNextWindowPos(ImVec2(kHudPad, kHudPad), ImGuiCond_Always);
        if (ImGui::Begin("##viewport_hud_top", nullptr, HudOverlayFlags())) {
            const std::string traj_name =
                ctx.playback_state != nullptr ? TrajectoryDisplayName(*ctx.playback_state) : std::string("未加载轨迹");
            ImGui::TextUnformatted(traj_name.c_str());

            if (ctx.playback_state != nullptr && ctx.playback_sm != nullptr && ctx.playback_sm->HasKeyframes()) {
                const float total = std::max(0.0f, ctx.playback_sm->TotalDuration());
                ImGui::Text("%.2f / %.2f s   ·   %.2fx", ctx.playback_state->play_time, total, ctx.playback_state->play_speed);
            } else {
                ImGui::TextDisabled("无轨迹 — 在回放页加载 CSV");
            }

            RenderStatusChipOnHud(PlaybackStateLabel(ctx.playback_sm), PlaybackSemanticLevel(ctx.playback_sm));
            ImGui::SameLine();
            if (ctx.ui_state->demo_visual_mode) {
                RenderStatusChipOnHud("演示模式", UiSemanticLevel::Info);
                ImGui::SameLine();
            }
            if (ctx.ui_state->sidebar_hidden) {
                RenderStatusChipOnHud("全屏视窗", UiSemanticLevel::Info);
                ImGui::SameLine();
            }
            if (ctx.collision_state != nullptr && ctx.collision_state->enable && ctx.collision_result != nullptr &&
                ctx.collision_result->valid) {
                UiSemanticLevel collision_level = UiSemanticLevel::Success;
                const char* collision_label     = "碰撞 SAFE";
                if (ctx.collision_state->nearest_surface_distance_m <= ctx.collision_state->danger_distance_m) {
                    collision_level = UiSemanticLevel::Error;
                    collision_label = "碰撞 DANGER";
                } else if (ctx.collision_state->nearest_surface_distance_m <= ctx.collision_state->warning_distance_m) {
                    collision_level = UiSemanticLevel::Warning;
                    collision_label = "碰撞 WARN";
                }
                RenderStatusChipOnHud(collision_label, collision_level);
                ImGui::SameLine();
            }
            if (ctx.video_recorder != nullptr && ctx.video_recorder->IsRecording()) {
                RenderStatusChipOnHud("录制中", UiSemanticLevel::Error);
            }
        }
        ImGui::End();

        ImGui::SetNextWindowPos(ImVec2(kHudPad, bottom_y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(bottom_w, kHudBottomHeight), ImGuiCond_Always);
        if (ImGui::Begin("##viewport_hud_bottom", nullptr, HudBottomFlags())) {
            RenderPlaybackBottomBar(ctx, bottom_w);
        }
        ImGui::End();
        hud_style.Pop();
    }

}  // namespace kinematic_viewer
