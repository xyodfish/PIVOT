#include "kinematic_viewer/kinematic_video_panel.h"

#include "kinematic_viewer/kinematic_string_utils.h"
#include "kinematic_viewer/kinematic_video_recorder.h"
#include "kinematic_viewer/kinematic_viewer_state.h"

#include "imgui.h"

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <vector>

#include <glad/glad.h>

namespace kinematic_viewer {

    void CaptureFrameForRecorder(VideoRecorder* video_recorder, int viewport_w, int viewport_h) {
        if (video_recorder == nullptr || !video_recorder->IsRecording() || viewport_w <= 0 || viewport_h <= 0) {
            return;
        }
        std::vector<uint8_t> pixels(static_cast<size_t>(viewport_w) * viewport_h * 4);
        glReadPixels(0, 0, viewport_w, viewport_h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        std::vector<uint8_t> flipped(static_cast<size_t>(viewport_w) * viewport_h * 4);
        for (int y = 0; y < viewport_h; ++y) {
            int src_row = viewport_h - 1 - y;
            std::memcpy(flipped.data() + y * viewport_w * 4, pixels.data() + src_row * viewport_w * 4, static_cast<size_t>(viewport_w) * 4);
        }
        video_recorder->SubmitFrame(flipped.data(), viewport_w, viewport_h);
    }

    void RenderVideoRecorderPanel(ViewerState* ui_state, VideoRecorder* video_recorder, KinematicUiFeedback* ui_feedback, double now_sec,
                                  int viewport_w, int viewport_h) {
        if (ui_state == nullptr || video_recorder == nullptr || ui_feedback == nullptr) {
            return;
        }
        const char* format_items[] = {"MP4", "GIF"};
        ImGui::Combo("格式", &ui_state->record_format, format_items, IM_ARRAYSIZE(format_items));
        ImGui::InputInt("帧率", &ui_state->record_fps);
        ui_state->record_fps = std::clamp(ui_state->record_fps, 1, 120);

        ImGui::InputText("输出目录", ui_state->record_output_dir, sizeof(ui_state->record_output_dir));
        ImGui::SameLine();
        if (ImGui::SmallButton("浏览..")) {
            ImGui::OpenPopup("record_output_dir_browser");
        }

        {
            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            const float popup_w           = std::clamp(viewport->Size.x * 0.55f, 640.0f, 1100.0f);
            const float popup_h           = std::clamp(viewport->Size.y * 0.60f, 420.0f, 800.0f);
            ImGui::SetNextWindowSize(ImVec2(popup_w, popup_h), ImGuiCond_Appearing);
            ImGui::SetNextWindowSizeConstraints(ImVec2(520.0f, 380.0f), ImVec2(1400.0f, 1000.0f));

            if (ImGui::BeginPopupModal("record_output_dir_browser", nullptr)) {
                static char browser_dir[512] = "";
                static std::vector<FileBrowserEntry> s_cached_entries;
                static bool s_browser_force_refresh = true;

                if (browser_dir[0] == '\0') {
                    const char* home = std::getenv("HOME");
                    std::snprintf(browser_dir, sizeof(browser_dir), "%s", home ? home : "/");
                }

                bool dir_changed = false;
                ImGui::InputText("目录", browser_dir, sizeof(browser_dir));
                ImGui::SameLine();
                if (ImGui::Button("进入目录")) {
                    std::string normalized = NormalizePath(browser_dir);
                    std::snprintf(browser_dir, sizeof(browser_dir), "%s", normalized.c_str());
                    dir_changed = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("上一级")) {
                    std::filesystem::path current = std::filesystem::path(NormalizePath(browser_dir));
                    std::filesystem::path parent  = current.parent_path();
                    if (parent.empty()) {
                        parent = std::filesystem::path("/");
                    }
                    std::snprintf(browser_dir, sizeof(browser_dir), "%s", parent.string().c_str());
                    dir_changed = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("根目录/")) {
                    std::snprintf(browser_dir, sizeof(browser_dir), "%s", "/");
                    dir_changed = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("HOME")) {
                    const char* home = std::getenv("HOME");
                    if (home != nullptr && home[0] != '\0') {
                        std::snprintf(browser_dir, sizeof(browser_dir), "%s", home);
                        dir_changed = true;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("刷新")) {
                    s_browser_force_refresh = true;
                }

                const std::string normalized_dir = NormalizePath(browser_dir);
                if (dir_changed) {
                    s_browser_force_refresh = true;
                }

                if (s_browser_force_refresh || s_cached_entries.empty()) {
                    s_browser_force_refresh = false;
                    s_cached_entries        = ScanDirectoryForBrowser(
                        normalized_dir, [](const std::filesystem::path&) { return true; }, FileBrowserSortBy::NameAsc);
                }

                if (ImGui::BeginChild("record_dir_browser_list", ImVec2(0, -48), true)) {
                    ImGuiTableFlags table_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
                    if (ImGui::BeginTable("record_dir_table", 3, table_flags)) {
                        ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 3.5f);
                        ImGui::TableSetupColumn("大小", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                        ImGui::TableSetupColumn("修改时间", ImGuiTableColumnFlags_WidthStretch, 1.5f);
                        ImGui::TableHeadersRow();

                        for (const auto& entry : s_cached_entries) {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);

                            std::string display_name = entry.is_directory ? ("[DIR] " + entry.name) : entry.name;
                            ImGui::PushStyleColor(ImGuiCol_Text,
                                                  entry.is_directory ? ImVec4(0.45f, 0.75f, 1.0f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            if (ImGui::Selectable(display_name.c_str(), false,
                                                  ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick)) {
                                if (entry.is_directory) {
                                    std::string new_dir = normalized_dir + "/" + entry.name;
                                    std::snprintf(browser_dir, sizeof(browser_dir), "%s", NormalizePath(new_dir).c_str());
                                    s_browser_force_refresh = true;
                                }
                            }
                            ImGui::PopStyleColor();

                            ImGui::TableSetColumnIndex(1);
                            ImGui::TextDisabled("%s", entry.size.c_str());
                            ImGui::TableSetColumnIndex(2);
                            ImGui::TextDisabled("%s", entry.mtime.c_str());
                        }
                        ImGui::EndTable();
                    }
                    ImGui::EndChild();
                }

                if (ImGui::Button("选择当前目录", ImVec2(140, 0))) {
                    std::snprintf(ui_state->record_output_dir, sizeof(ui_state->record_output_dir), "%s", normalized_dir.c_str());
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("取消", ImVec2(80, 0))) {
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("双击目录进入; 单击选择当前目录");
                ImGui::EndPopup();
            }
        }

        ImGui::InputText("文件名 (空=自动)", ui_state->record_filename, sizeof(ui_state->record_filename));

        if (video_recorder->IsRecording()) {
            if (ImGui::Button("停止录制", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
                video_recorder->StopRecording();
                ui_state->is_recording = false;
                ui_feedback->Push(UiSemanticLevel::Success, "录制已停止并保存", now_sec, 3.0);
            }
            ImGui::TextDisabled("%s", video_recorder->GetStatus().c_str());
            return;
        }

        if (ImGui::Button("开始录制", ImVec2(ImGui::GetContentRegionAvail().x, 0))) {
            std::string out_dir = ui_state->record_output_dir;
            if (out_dir.empty()) {
                out_dir = VideoRecorder::DefaultOutputDirectory();
            }
            std::filesystem::create_directories(out_dir);
            std::string ext = (ui_state->record_format == 0) ? ".mp4" : ".gif";

            std::string filename;
            if (ui_state->record_filename[0] != '\0') {
                filename = ui_state->record_filename;
                if (filename.size() < ext.size() || filename.compare(filename.size() - ext.size(), ext.size(), ext) != 0) {
                    filename += ext;
                }
            } else {
                filename = "recording_" + VideoRecorder::TimestampString() + ext;
            }
            std::string full_path = out_dir + "/" + filename;

            auto fmt = (ui_state->record_format == 0) ? VideoRecorder::Format::MP4 : VideoRecorder::Format::GIF;
            if (video_recorder->StartRecording(full_path, viewport_w, viewport_h, ui_state->record_fps, fmt)) {
                ui_state->is_recording = true;
                ui_feedback->Push(UiSemanticLevel::Success, "开始录制: " + filename, now_sec, 3.0);
            } else {
                ui_feedback->Push(UiSemanticLevel::Error, "录制启动失败", now_sec, 4.0);
            }
        }
    }

}  // namespace kinematic_viewer
