#include "kinematic_viewer/kinematic_sidebar_panels.h"

#include "kinematic_viewer/kinematic_ik_controller.h"
#include "kinematic_viewer/kinematic_marker_utils.h"
#include "kinematic_viewer/kinematic_angle_units.h"
#include "kinematic_viewer/kinematic_demo_visual.h"
#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_path_planner.h"
#include "kinematic_viewer/kinematic_string_utils.h"

#include "kinematic_viewer/kinematic_playback_bundle.h"
#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"
#include "kinematic_viewer/kinematic_viewer_state.h"

#include "imgui.h"
#include "imgui_internal.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kinematic_viewer {
    namespace kinematic_sidebar_panels_internal {

        using kinematic_viewer::NormalizePath;

        bool IsTrajectoryFileExt(const std::filesystem::path& path) {
            const std::string ext = LowerFileExtension(path.string());
            return ext == ".csv";
        }

        bool IsYamlFileExt(const std::filesystem::path& path) {
            const std::string ext = LowerFileExtension(path.string());
            return ext == ".yaml" || ext == ".yml";
        }

        void RenderPlaybackBundleImport(DebugPlaybackState* playbackState, ViewerState* viewerState) {
            if (playbackState == nullptr || viewerState == nullptr) {
                return;
            }

            static char bundle_path[512]        = "config/playback_bundles/demo_playback_bundle.yaml";
            static char bundle_browser_dir[512] = "";
            static std::string bundle_status;

            ImGui::Separator();
            ImGui::TextUnformatted("轨迹+障碍物离线导入");
            ImGui::TextDisabled("导入包 YAML 引用现有 CSV 与障碍物 YAML；也可选择会话目录");
            ImGui::InputText("导入包路径", bundle_path, sizeof(bundle_path));
            if (ImGui::Button("浏览导入包")) {
                const std::string default_dir = NormalizePath(std::filesystem::current_path().string());
                std::snprintf(bundle_browser_dir, sizeof(bundle_browser_dir), "%s", default_dir.c_str());
                ImGui::OpenPopup("playback_bundle_import_popup");
            }
            ImGui::SameLine();
            if (ImGui::Button("一键导入轨迹+障碍物")) {
                PlaybackBundleSpec spec;
                std::string parse_error;
                if (!ParsePlaybackBundle(bundle_path, &spec, &parse_error)) {
                    bundle_status = parse_error;
                } else {
                    std::string apply_status;
                    std::string apply_error;
                    if (ApplyPlaybackBundle(spec, &viewerState->user_obstacles, playbackState, &apply_status, &apply_error)) {
                        bundle_status = apply_status;
                    } else {
                        bundle_status = apply_error;
                    }
                }
            }

            if (ImGui::BeginPopupModal("playback_bundle_import_popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::InputText("目录", bundle_browser_dir, sizeof(bundle_browser_dir));
                ImGui::SameLine();
                if (ImGui::Button("进入目录##bundle_import")) {
                    const std::string normalized = NormalizePath(bundle_browser_dir);
                    std::snprintf(bundle_browser_dir, sizeof(bundle_browser_dir), "%s", normalized.c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("上一级##bundle_import")) {
                    std::filesystem::path current = std::filesystem::path(NormalizePath(bundle_browser_dir));
                    std::filesystem::path parent  = current.parent_path();
                    if (parent.empty()) {
                        parent = std::filesystem::path("/");
                    }
                    std::snprintf(bundle_browser_dir, sizeof(bundle_browser_dir), "%s", parent.string().c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("HOME##bundle_import")) {
                    const char* home = std::getenv("HOME");
                    if (home != nullptr && home[0] != '\0') {
                        std::snprintf(bundle_browser_dir, sizeof(bundle_browser_dir), "%s", home);
                    }
                }

                std::error_code ec;
                const std::filesystem::path browse_path(bundle_browser_dir);
                if (!std::filesystem::exists(browse_path, ec) || !std::filesystem::is_directory(browse_path, ec)) {
                    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "目录不可用");
                } else {
                    std::vector<std::filesystem::path> dirs;
                    std::vector<std::filesystem::path> files;
                    for (auto it = std::filesystem::directory_iterator(browse_path, ec); !ec && it != std::filesystem::directory_iterator();
                         ++it) {
                        if (it->is_directory(ec)) {
                            dirs.push_back(it->path());
                        } else if (it->is_regular_file(ec) && IsYamlFileExt(it->path())) {
                            files.push_back(it->path());
                        }
                    }
                    std::sort(dirs.begin(), dirs.end());
                    std::sort(files.begin(), files.end());
                    if (ImGui::BeginChild("playback_bundle_import_list", ImVec2(620, 280), true)) {
                        for (const auto& d : dirs) {
                            std::string label = "[DIR] " + d.filename().string();
                            if (ImGui::Selectable(label.c_str(), false)) {
                                std::snprintf(bundle_browser_dir, sizeof(bundle_browser_dir), "%s", d.string().c_str());
                            }
                        }
                        for (const auto& f : files) {
                            std::string label = f.filename().string();
                            if (ImGui::Selectable(label.c_str(), false)) {
                                std::snprintf(bundle_path, sizeof(bundle_path), "%s", f.string().c_str());
                                ImGui::CloseCurrentPopup();
                            }
                        }
                        ImGui::EndChild();
                    }
                    ImGui::TextDisabled("也可在上方目录栏输入会话目录路径后关闭弹窗，再点一键导入");
                }
                if (ImGui::Button("使用当前目录##bundle_import")) {
                    std::snprintf(bundle_path, sizeof(bundle_path), "%s", NormalizePath(bundle_browser_dir).c_str());
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("关闭##bundle_import")) {
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }

            if (!bundle_status.empty()) {
                const bool ok = bundle_status.find("成功") != std::string::npos;
                ImGui::TextColored(ok ? ImVec4(0.60f, 0.92f, 0.60f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s",
                                    bundle_status.c_str());
            }
        }

        std::string TrimCopy(const std::string& text) {
            size_t begin = 0;
            while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
                ++begin;
            }
            size_t end = text.size();
            while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
                --end;
            }
            return text.substr(begin, end - begin);
        }

        std::vector<std::string> SplitJointGroupCommands(const std::string& rawText) {
            std::string normalized = rawText;
            std::replace(normalized.begin(), normalized.end(), ';', '\n');
            std::vector<std::string> lines;
            std::stringstream ss(normalized);
            std::string line;
            while (std::getline(ss, line)) {
                const std::string trimmed = TrimCopy(line);
                if (!trimmed.empty()) {
                    lines.push_back(trimmed);
                }
            }
            return lines;
        }

        bool ParseJointValuesRad(const std::string& rawValues, std::vector<float>* outValues, std::string* errorMessage) {
            if (outValues == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：输出缓冲为空";
                }
                return false;
            }
            std::string normalized = rawValues;
            const auto replace_all = [](std::string* text, const char* from, const char* to) {
                if (text == nullptr || from == nullptr || to == nullptr) {
                    return;
                }
                const size_t from_len = std::strlen(from);
                size_t pos            = 0;
                while ((pos = text->find(from, pos)) != std::string::npos) {
                    text->replace(pos, from_len, to);
                    pos += 1;
                }
            };
            replace_all(&normalized, "、", " ");
            replace_all(&normalized, "，", " ");
            replace_all(&normalized, "；", " ");
            for (char& ch : normalized) {
                if (ch == ',' || ch == ';' || ch == '[' || ch == ']' || ch == '\t') {
                    ch = ' ';
                }
            }
            std::stringstream ss(normalized);
            std::vector<float> values;
            float value = 0.0f;
            while (ss >> value) {
                if (!std::isfinite(value)) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "存在非有限数值(NaN/Inf)";
                    }
                    return false;
                }
                values.push_back(value);
            }
            if (values.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "未解析到任何关节角数值";
                }
                return false;
            }
            *outValues = std::move(values);
            return true;
        }

        bool ParseJointValuesToRad(const std::string& rawValues, bool angle_unit_deg, std::vector<float>* outRad,
                                   std::string* errorMessage) {
            std::vector<float> ui_values;
            if (!ParseJointValuesRad(rawValues, &ui_values, errorMessage)) {
                return false;
            }
            if (outRad == nullptr) {
                return true;
            }
            outRad->resize(ui_values.size());
            for (size_t i = 0; i < ui_values.size(); ++i) {
                (*outRad)[i] = AngleUiToRad(ui_values[i], angle_unit_deg);
            }
            return true;
        }

        bool TryReadJointPositionRad(rkv::RobotScene* scene, const std::string& joint_name, float* out_rad) {
            if (scene == nullptr || out_rad == nullptr) {
                return false;
            }
            rkv::RobotScene::JointInfo info;
            if (!scene->getJointInfo(joint_name, &info)) {
                return false;
            }
            *out_rad = info.position;
            return true;
        }

        std::string FormatJointGroupLine(const ViewerState::JointInputGroup& group, rkv::RobotScene* scene, bool use_deg) {
            const int precision = use_deg ? 2 : 4;
            std::ostringstream ss;
            ss << group.name << ": ";
            for (size_t i = 0; i < group.joint_names.size(); ++i) {
                if (i > 0) {
                    ss << "、";
                }
                float rad = 0.0f;
                if (!TryReadJointPositionRad(scene, group.joint_names[i], &rad)) {
                    ss << (use_deg ? "0.00" : "0.0000");
                    continue;
                }
                ss << std::fixed << std::setprecision(precision) << AngleUiFromRad(rad, use_deg);
            }
            return ss.str();
        }

        std::string FormatAllJointGroups(const ViewerState& uiState, rkv::RobotScene* scene, bool use_deg) {
            std::ostringstream ss;
            for (size_t g = 0; g < uiState.joint_input_groups.size(); ++g) {
                if (g > 0) {
                    ss << '\n';
                }
                ss << FormatJointGroupLine(uiState.joint_input_groups[g], scene, use_deg);
            }
            return ss.str();
        }

        void CopyTextToClipboard(const std::string& text) {
            if (!text.empty()) {
                ImGui::SetClipboardText(text.c_str());
            }
        }

        void CopyJointGroupLineToInput(char* dest, size_t dest_size, const std::string& line) {
            if (dest == nullptr || dest_size == 0) {
                return;
            }
            std::snprintf(dest, dest_size, "%s", line.c_str());
        }

        const ViewerState::JointInputGroup* FindJointInputGroup(const ViewerState& uiState, const std::string& groupNameRaw) {
            std::string target = groupNameRaw;
            std::transform(target.begin(), target.end(), target.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            for (const auto& group : uiState.joint_input_groups) {
                std::string current = group.name;
                std::transform(current.begin(), current.end(), current.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (current == target) {
                    return &group;
                }
            }
            return nullptr;
        }

        bool ApplyJointGroupTextCommand(const std::string& commandLine, const ViewerState& uiState, rkv::RobotScene* scene,
                                        std::string* errorMessage) {
            if (scene == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：scene为空";
                }
                return false;
            }
            const std::string line = TrimCopy(commandLine);
            if (line.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "空输入";
                }
                return false;
            }

            std::string groupName;
            std::string valuesRaw;
            const size_t colon_pos = line.find(':');
            if (colon_pos != std::string::npos) {
                groupName = TrimCopy(line.substr(0, colon_pos));
                valuesRaw = TrimCopy(line.substr(colon_pos + 1));
            } else {
                const size_t firstSpace = line.find_first_of(" \t");
                if (firstSpace == std::string::npos) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "格式错误，应为: group: v1、v2,... 或 group v1 v2 ...";
                    }
                    return false;
                }
                groupName = TrimCopy(line.substr(0, firstSpace));
                valuesRaw = TrimCopy(line.substr(firstSpace + 1));
            }
            if (valuesRaw.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "缺少关节角列表";
                }
                return false;
            }

            const ViewerState::JointInputGroup* group = FindJointInputGroup(uiState, groupName);
            if (group == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "未知group: " + groupName;
                }
                return false;
            }
            if (group->joint_names.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "group未配置关节名: " + groupName;
                }
                return false;
            }

            std::vector<float> values_rad;
            std::string parseError;
            if (!ParseJointValuesToRad(valuesRaw, uiState.angle_unit_deg, &values_rad, &parseError)) {
                if (errorMessage != nullptr) {
                    *errorMessage = parseError;
                }
                return false;
            }
            if (values_rad.size() != group->joint_names.size()) {
                if (errorMessage != nullptr) {
                    std::stringstream ss;
                    ss << "group[" << group->name << "] 维度不匹配，期望 " << group->joint_names.size() << "，实际 " << values_rad.size();
                    *errorMessage = ss.str();
                }
                return false;
            }

            for (size_t i = 0; i < group->joint_names.size(); ++i) {
                if (!scene->setJointPositionByName(group->joint_names[i], values_rad[i])) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "场景不存在关节: " + group->joint_names[i];
                    }
                    return false;
                }
            }
            return true;
        }

        bool ApplyJointGroupValues(const ViewerState::JointInputGroup& group, const std::vector<float>& values,
                                   rkv::RobotScene* scene, std::string* errorMessage) {
            if (scene == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：scene为空";
                }
                return false;
            }
            if (group.joint_names.empty()) {
                if (errorMessage != nullptr) {
                    *errorMessage = "group未配置关节名: " + group.name;
                }
                return false;
            }
            if (values.size() != group.joint_names.size()) {
                if (errorMessage != nullptr) {
                    std::stringstream ss;
                    ss << "group[" << group.name << "] 维度不匹配，期望 " << group.joint_names.size() << "，实际 " << values.size();
                    *errorMessage = ss.str();
                }
                return false;
            }
            for (size_t i = 0; i < group.joint_names.size(); ++i) {
                if (!scene->setJointPositionByName(group.joint_names[i], values[i])) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "场景不存在关节: " + group.joint_names[i];
                    }
                    return false;
                }
            }
            return true;
        }

        using kinematic_viewer::FileBrowserSortBy;
        using kinematic_viewer::FormatFileSize;
        using kinematic_viewer::FormatFileTime;

        bool TrajectoryPathAlreadyInList(const DebugPlaybackState& playbackState, const std::string& path) {
            const std::string normalized = NormalizePath(path);
            for (const auto& entry : playbackState.trajectory_files) {
                if (NormalizePath(entry.path) == normalized) {
                    return true;
                }
            }
            return false;
        }

        int AddTrajectoryPathsToPlayback(DebugPlaybackState* playbackState, const std::vector<std::string>& paths,
                                         bool select_and_load_last) {
            if (playbackState == nullptr || paths.empty()) {
                return 0;
            }
            int added_count      = 0;
            int last_added_index = -1;
            for (const std::string& path : paths) {
                const std::string normalized = NormalizePath(path);
                if (TrajectoryPathAlreadyInList(*playbackState, normalized)) {
                    continue;
                }
                TrajectoryFileEntry newEntry;
                newEntry.path   = normalized;
                newEntry.status = "未加载";
                newEntry.loaded = false;
                playbackState->trajectory_files.push_back(std::move(newEntry));
                last_added_index = static_cast<int>(playbackState->trajectory_files.size()) - 1;
                ++added_count;
            }
            if (added_count > 0 && select_and_load_last && last_added_index >= 0) {
                playbackState->selected_trajectory_index = last_added_index;
                std::snprintf(playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path), "%s",
                              playbackState->trajectory_files[last_added_index].path.c_str());
                playbackState->pending_trajectory_load_index      = last_added_index;
                playbackState->pending_trajectory_play_after_load = false;
            }
            return added_count;
        }

        void RenderTrajectoryFileBrowser(DebugPlaybackState* playbackState) {
            if (playbackState == nullptr) {
                return;
            }

            struct DirEntry {
                std::filesystem::path path;
                std::string name;
                std::string size;
                std::string mtime;
                bool isDir = false;
            };

            // Persistent sort state for this browser instance
            static FileBrowserSortBy s_sort_by = FileBrowserSortBy::NameAsc;
            static std::string s_cached_dir;
            static std::vector<DirEntry> s_cached_entries;
            static bool s_force_refresh  = false;
            static bool s_has_scan_error = false;
            static std::unordered_set<std::string> s_selected_paths;

            if (ImGui::Button("浏览本地文件")) {
                s_force_refresh = true;
                ImGui::OpenPopup("trajectory_file_browser_popup");
            }

            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            const float popup_width       = std::clamp(viewport->Size.x * 0.72f, 900.0f, 1500.0f);
            const float popup_height      = std::clamp(viewport->Size.y * 0.70f, 460.0f, 920.0f);
            ImGui::SetNextWindowSize(ImVec2(popup_width, popup_height), ImGuiCond_Appearing);
            ImGui::SetNextWindowSizeConstraints(ImVec2(860.0f, 440.0f), ImVec2(1800.0f, 1200.0f));

            if (!ImGui::BeginPopupModal("trajectory_file_browser_popup", nullptr)) {
                return;
            }

            bool dirChanged = false;
            ImGui::InputText("目录", playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir));
            ImGui::SameLine();
            if (ImGui::Button("进入目录")) {
                const std::string normalized = NormalizePath(playbackState->trajectory_browser_dir);
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                              normalized.c_str());
                dirChanged = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("上一级")) {
                std::filesystem::path current = std::filesystem::path(NormalizePath(playbackState->trajectory_browser_dir));
                std::filesystem::path parent  = current.parent_path();
                if (parent.empty()) {
                    parent = std::filesystem::path("/");
                }
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                              parent.string().c_str());
                dirChanged = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("根目录/")) {
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", "/");
                dirChanged = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("HOME")) {
                const char* home = std::getenv("HOME");
                if (home != nullptr && home[0] != '\0') {
                    std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s", home);
                    dirChanged = true;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("刷新")) {
                s_force_refresh = true;
            }
            ImGui::TextDisabled("支持输入任意绝对路径，例如 /home/user/data");

            const std::string normalizedDir = NormalizePath(playbackState->trajectory_browser_dir);
            if (dirChanged) {
                s_force_refresh = true;
            }

            auto refreshBrowserCache = [&](const std::string& dir) {
                s_cached_dir = dir;
                s_cached_entries.clear();
                s_has_scan_error = false;

                std::error_code ec;
                const std::filesystem::path browsePath(dir);
                if (!std::filesystem::exists(browsePath, ec) || !std::filesystem::is_directory(browsePath, ec)) {
                    s_has_scan_error = true;
                    return;
                }

                for (auto it = std::filesystem::directory_iterator(browsePath, ec); !ec && it != std::filesystem::directory_iterator();
                     ++it) {
                    DirEntry entry;
                    entry.path  = it->path();
                    entry.name  = it->path().filename().string();
                    entry.isDir = it->is_directory(ec);
                    if (entry.isDir) {
                        entry.size  = "-";
                        entry.mtime = "-";
                    } else {
                        if (!IsTrajectoryFileExt(it->path())) {
                            continue;
                        }
                        auto fsize  = it->file_size(ec);
                        entry.size  = ec ? "-" : FormatFileSize(fsize);
                        auto ftime  = it->last_write_time(ec);
                        entry.mtime = ec ? "-" : FormatFileTime(ftime);
                    }
                    s_cached_entries.push_back(std::move(entry));
                }

                const FileBrowserSortBy sort_by = s_sort_by;
                std::sort(s_cached_entries.begin(), s_cached_entries.end(), [sort_by](const DirEntry& a, const DirEntry& b) {
                    if (a.isDir != b.isDir) {
                        return a.isDir > b.isDir;
                    }
                    switch (sort_by) {
                        case FileBrowserSortBy::SizeAsc:
                            if (a.size != b.size) {
                                return a.size < b.size;
                            }
                            break;
                        case FileBrowserSortBy::SizeDesc:
                            if (a.size != b.size) {
                                return a.size > b.size;
                            }
                            break;
                        case FileBrowserSortBy::TimeAsc:
                            if (a.mtime != b.mtime) {
                                return a.mtime < b.mtime;
                            }
                            break;
                        case FileBrowserSortBy::TimeDesc:
                            if (a.mtime != b.mtime) {
                                return a.mtime > b.mtime;
                            }
                            break;
                        case FileBrowserSortBy::NameDesc:
                            return a.name > b.name;
                        case FileBrowserSortBy::NameAsc:
                        default:
                            break;
                    }
                    return a.name < b.name;
                });
            };

            if (s_force_refresh || s_cached_dir != normalizedDir) {
                s_force_refresh = false;
                refreshBrowserCache(normalizedDir);
            }

            if (s_has_scan_error) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "目录不可用");
            } else {
                if (ImGui::BeginChild("trajectory_file_browser_list", ImVec2(0, -72), true)) {
                    ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
                    if (ImGui::BeginTable("file_browser_table", 4, tableFlags)) {
                        ImGui::TableSetupColumn("选", ImGuiTableColumnFlags_WidthFixed, 28.0f);
                        ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 3.5f);
                        ImGui::TableSetupColumn("大小", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                        ImGui::TableSetupColumn("修改时间", ImGuiTableColumnFlags_WidthStretch, 1.5f);
                        ImGui::TableHeadersRow();

                        // Clickable headers for sorting (skip checkbox column)
                        for (int col = 1; col < 4; ++col) {
                            ImGui::TableSetColumnIndex(col);
                            const char* labels[3]           = {"名称", "大小", "修改时间"};
                            const FileBrowserSortBy asc[3]  = {FileBrowserSortBy::NameAsc, FileBrowserSortBy::SizeAsc,
                                                               FileBrowserSortBy::TimeAsc};
                            const FileBrowserSortBy desc[3] = {FileBrowserSortBy::NameDesc, FileBrowserSortBy::SizeDesc,
                                                               FileBrowserSortBy::TimeDesc};
                            const int sort_col              = col - 1;
                            bool is_asc                     = (s_sort_by == asc[sort_col]);
                            bool is_desc                    = (s_sort_by == desc[sort_col]);
                            const char* arrow               = is_asc ? "▲" : (is_desc ? "▼" : "");
                            char buf[32];
                            std::snprintf(buf, sizeof(buf), "%s %s", labels[sort_col], arrow);
                            ImGui::TableHeader(buf);
                            if (ImGui::IsItemClicked()) {
                                if (is_asc) {
                                    s_sort_by = desc[sort_col];
                                } else {
                                    s_sort_by = asc[sort_col];
                                }
                                s_force_refresh = true;
                            }
                        }

                        for (const auto& entry : s_cached_entries) {
                            ImGui::TableNextRow();
                            const std::string abs_path = NormalizePath(entry.path.string());

                            ImGui::TableSetColumnIndex(0);
                            if (entry.isDir) {
                                ImGui::TextDisabled("-");
                            } else {
                                bool checked = s_selected_paths.count(abs_path) > 0;
                                ImGui::PushID(abs_path.c_str());
                                if (ImGui::Checkbox("##pick", &checked)) {
                                    if (checked) {
                                        s_selected_paths.insert(abs_path);
                                    } else {
                                        s_selected_paths.erase(abs_path);
                                    }
                                }
                                ImGui::PopID();
                            }

                            ImGui::TableSetColumnIndex(1);
                            std::string displayName = entry.isDir ? ("[DIR] " + entry.name) : entry.name;
                            const bool row_selected = !entry.isDir && s_selected_paths.count(abs_path) > 0;
                            ImGui::PushStyleColor(ImGuiCol_Text,
                                                  entry.isDir ? ImVec4(0.45f, 0.75f, 1.0f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            const ImGuiSelectableFlags row_flags =
                                ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick;
                            if (ImGui::Selectable(displayName.c_str(), row_selected, row_flags)) {
                                if (entry.isDir) {
                                    std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir),
                                                  "%s", abs_path.c_str());
                                } else if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                    AddTrajectoryPathsToPlayback(playbackState, {abs_path}, true);
                                    s_selected_paths.erase(abs_path);
                                    ImGui::CloseCurrentPopup();
                                } else {
                                    if (s_selected_paths.count(abs_path) > 0) {
                                        s_selected_paths.erase(abs_path);
                                    } else {
                                        s_selected_paths.insert(abs_path);
                                    }
                                }
                            }
                            ImGui::PopStyleColor();

                            ImGui::TableSetColumnIndex(2);
                            ImGui::TextDisabled("%s", entry.size.c_str());

                            ImGui::TableSetColumnIndex(3);
                            ImGui::TextDisabled("%s", entry.mtime.c_str());
                        }
                        ImGui::EndTable();
                    }
                    ImGui::EndChild();
                }
            }

            auto addSelectedPaths = [&](bool close_popup) {
                std::vector<std::string> paths;
                paths.reserve(s_selected_paths.size());
                for (const auto& path : s_selected_paths) {
                    paths.push_back(path);
                }
                std::sort(paths.begin(), paths.end());
                if (AddTrajectoryPathsToPlayback(playbackState, paths, true) > 0) {
                    s_selected_paths.clear();
                    if (close_popup) {
                        ImGui::CloseCurrentPopup();
                    }
                }
            };

            const int selected_count = static_cast<int>(s_selected_paths.size());
            char add_selected_label[48];
            std::snprintf(add_selected_label, sizeof(add_selected_label), "添加选中(%d)", selected_count);
            ImGui::BeginDisabled(selected_count == 0);
            if (ImGui::Button(add_selected_label)) {
                addSelectedPaths(false);
            }
            ImGui::SameLine();
            if (ImGui::Button("添加并关闭")) {
                addSelectedPaths(true);
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("全选")) {
                for (const auto& entry : s_cached_entries) {
                    if (!entry.isDir) {
                        s_selected_paths.insert(NormalizePath(entry.path.string()));
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("清除选择")) {
                s_selected_paths.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("关闭")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::TextDisabled("单击切换选中；双击单个文件可立即添加并关闭");
            ImGui::EndPopup();
        }

        void FillMobileBasePoseInput(ViewerState* uiState, rkv::RobotScene* scene) {
            if (uiState == nullptr || scene == nullptr) {
                return;
            }
            float base_x_m = 0.0f;
            float base_y_m = 0.0f;
            float base_yaw = 0.0f;
            if (!scene->getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw)) {
                return;
            }
            std::string text;
            if (uiState->mobile_base_pose_input_format == 1) {
                const glm::vec3 pos(base_x_m, base_y_m, 0.0f);
                const glm::quat quat = glm::angleAxis(base_yaw, glm::vec3(0.0f, 0.0f, 1.0f));
                text                 = FormatPoseInputXyzQuat(pos, quat);
            } else {
                text = FormatPoseInputXyYaw(base_x_m, base_y_m, base_yaw, uiState->angle_unit_deg);
            }
            std::snprintf(uiState->mobile_base_pose_input, sizeof(uiState->mobile_base_pose_input), "%s", text.c_str());
        }

        bool ApplyMobileBasePoseInput(ViewerState* uiState, rkv::RobotScene* scene) {
            if (uiState == nullptr || scene == nullptr) {
                return false;
            }
            if (uiState->mobile_base_pose_input_format == 1) {
                glm::vec3 parsed_pos(0.0f);
                glm::quat parsed_quat(1.0f, 0.0f, 0.0f, 0.0f);
                std::string parse_error;
                if (!ParsePoseInputXyzQuat(uiState->mobile_base_pose_input, &parsed_pos, &parsed_quat, &parse_error)) {
                    uiState->mobile_base_pose_input_status = "应用失败: " + parse_error;
                    return false;
                }
                const glm::mat3 rot_mat(parsed_quat);
                const float yaw_rad = std::atan2(rot_mat[0][1], rot_mat[0][0]);
                scene->setVirtualBasePose2D(parsed_pos.x, parsed_pos.y, yaw_rad);
                uiState->mobile_base_pose_input_status = "已应用 (平面: x,y,yaw; z/roll/pitch 不生效)";
                return true;
            }

            float parsed_x   = 0.0f;
            float parsed_y   = 0.0f;
            float parsed_yaw = 0.0f;
            std::string parse_error;
            if (!ParsePoseInputXyYaw(uiState->mobile_base_pose_input, &parsed_x, &parsed_y, &parsed_yaw, uiState->angle_unit_deg,
                                     &parse_error)) {
                uiState->mobile_base_pose_input_status = "应用失败: " + parse_error;
                return false;
            }
            scene->setVirtualBasePose2D(parsed_x, parsed_y, parsed_yaw);
            uiState->mobile_base_pose_input_status =
                std::string("已应用: x=") + std::to_string(parsed_x) + " y=" + std::to_string(parsed_y) + " yaw=" +
                (uiState->angle_unit_deg ? std::to_string(glm::degrees(parsed_yaw)) + " deg" : std::to_string(parsed_yaw) + " rad");
            return true;
        }

    }  // namespace kinematic_sidebar_panels_internal

    void RenderScenePanel(ViewerState* uiState, rkv::RobotScene* scene) {
        if (uiState == nullptr) {
            return;
        }
        if (!ImGui::CollapsingHeader("场景显示", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        {
            bool demo_enabled = uiState->demo_visual_mode;
            if (ImGui::Checkbox("演示视觉（暗底·隐藏轴网）", &demo_enabled)) {
                SetDemoVisualMode(uiState, demo_enabled);
            }
            if (uiState->demo_visual_mode) {
                ImGui::SameLine();
                ImGui::TextDisabled("录屏/展示推荐");
            }
        }
        SidebarCheckboxRow4("网格", &uiState->show_visual_meshes, "线框", &uiState->show_wireframe, "碰撞体",
                            &uiState->show_collision_bodies, "质心", &uiState->show_com);
        SidebarCheckboxRow4("关节轴", &uiState->show_axes, "仅旋转轴", &uiState->show_revolute_only, "非旋转轴",
                            &uiState->show_non_revolute, "世界轴", &uiState->show_world_axes);
        ImGui::Checkbox("固定底座", &uiState->lock_base);
        ImGui::Checkbox("Link悬停高亮", &uiState->enable_link_hover_highlight);
        if (ImGui::Checkbox("3D点选Link", &uiState->enable_link_click_select) && !uiState->enable_link_click_select) {
            uiState->selected_link.clear();
        }
        if (uiState->mobile_base_drag_available) {
            if (ImGui::Checkbox("底盘拖动Gizmo", &uiState->mobile_base_drag_enabled) && uiState->mobile_base_drag_enabled) {
                uiState->lock_base = false;
            }
            if (uiState->mobile_base_drag_enabled) {
                SidebarRadioRow3("底盘模式", &uiState->mobile_base_gizmo_operation, "平移", "旋转", "全");
                if (scene != nullptr) {
                    float base_x_m = 0.0f;
                    float base_y_m = 0.0f;
                    float base_yaw = 0.0f;
                    if (scene->getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw)) {
                        const float yaw_ui = AngleUiFromRad(base_yaw, uiState->angle_unit_deg);
                        ImGui::Text("底盘位姿: x=%.3f m  y=%.3f m  yaw=%.2f %s", base_x_m, base_y_m, yaw_ui,
                                    AngleUnitLabel(uiState->angle_unit_deg));
                    }
                }
                ImGui::Separator();
                ImGui::TextUnformatted("底盘位姿手动输入（yaw 单位跟随顶部角度单位）");
                ImGui::RadioButton("x,y,yaw##base_fmt_xy_yaw", &uiState->mobile_base_pose_input_format, 0);
                ImGui::SameLine();
                ImGui::RadioButton("x,y,z,qx,qy,qz,qw##base_fmt_xyz_quat", &uiState->mobile_base_pose_input_format, 1);
                const char* input_label =
                    (uiState->mobile_base_pose_input_format == 1) ? "x,y,z,qx,qy,qz,qw##base_pose_input" : "x,y,yaw##base_pose_input";
                if (uiState->mobile_base_pose_input[0] == '\0') {
                    kinematic_sidebar_panels_internal::FillMobileBasePoseInput(uiState, scene);
                }
                PushSidebarFullWidth();
                ImGui::InputText(input_label, uiState->mobile_base_pose_input, sizeof(uiState->mobile_base_pose_input));
                PopSidebarWidth();
                if (ImGui::SmallButton("填充当前")) {
                    kinematic_sidebar_panels_internal::FillMobileBasePoseInput(uiState, scene);
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("应用") && scene != nullptr) {
                    uiState->lock_base = false;
                    kinematic_sidebar_panels_internal::ApplyMobileBasePoseInput(uiState, scene);
                }
                if (!uiState->mobile_base_pose_input_status.empty()) {
                    ImGui::TextWrapped("%s", uiState->mobile_base_pose_input_status.c_str());
                }
                ImGui::TextDisabled("在左侧3D视窗：绿色圆环处拖动 Gizmo（仅“场景”页）");
            }
        } else {
            ImGui::TextDisabled("当前机器人未启用底盘拖动（见配置 ui.mobile_base_robots）");
        }
        if (ImGui::TreeNode("轴与网格")) {
            ImGui::Checkbox("地面网格", &uiState->show_grid);
            SidebarSliderFloat("轴长", &uiState->axis_length, 0.03f, 0.5f, "%.3f");
            SidebarSliderFloat("线宽", &uiState->axis_line_width, 1.0f, 6.0f, "%.1f");
            SidebarSliderFloat("世界轴长", &uiState->world_axis_length, 0.1f, 1.5f, "%.2f");
            SidebarSliderFloat("网格尺寸", &uiState->grid_size, 1.0f, 20.0f, "%.1f");
            PushSidebarFullWidth();
            ImGui::SliderInt("网格密度", &uiState->grid_count, 10, 120);
            PopSidebarWidth();
            ImGui::TreePop();
        }
    }

    void RenderObstaclePanel(ViewerState* uiState) {
        if (uiState == nullptr) {
            return;
        }
        ImGui::Separator();
        RenderUserObstaclePanel(&uiState->user_obstacles, uiState->angle_unit_deg);
    }

    void RenderIkPanel(ViewerState* uiState, IkState* ikState, KinematicIkController* ikController, rkv::RobotScene* scene) {
        if (uiState == nullptr || ikState == nullptr || ikController == nullptr || scene == nullptr) {
            return;
        }
        const bool use_deg = uiState->angle_unit_deg;
        if (ikState->chains.empty()) {
            ImGui::TextDisabled("无 IK 链配置");
            return;
        }
        if (!ImGui::CollapsingHeader("末端 Marker IK", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }

        std::vector<const char*> chain_labels;
        chain_labels.reserve(ikState->chains.size());
        for (const auto& c : ikState->chains) {
            chain_labels.push_back(c.config.label.c_str());
        }
        if (kinematic_viewer::SidebarCombo("控制链", &ikState->selected_chain, chain_labels.data(),
                                           static_cast<int>(chain_labels.size()))) {
            ikState->marker_initialized = false;
            ikController->LoadActiveMarkerFromTarget(scene);
        }
        int solve_mode_index           = (ikState->solve_mode == "full_body") ? 1 : 0;
        const char* solve_mode_items[] = {"单链", "全身"};
        if (kinematic_viewer::SidebarCombo("求解模式", &solve_mode_index, solve_mode_items, IM_ARRAYSIZE(solve_mode_items))) {
            ikState->solve_mode  = (solve_mode_index == 1) ? "full_body" : "single_chain";
            ikState->last_status = "已切换IK求解模式";
        }
        if (ikState->solve_mode == "full_body") {
            int backend_index           = (ikState->full_body_backend == "wbc_chain_ik") ? 1 : 0;
            const char* backend_items[] = {"flex_ik", "wbc_chain_ik"};
            if (kinematic_viewer::SidebarCombo("全身后端", &backend_index, backend_items, IM_ARRAYSIZE(backend_items))) {
                ikState->full_body_backend = (backend_index == 1) ? "wbc_chain_ik" : "flex_ik";
                ikState->solver.setFullBodyBackend(ikState->full_body_backend);
                ikState->last_status = std::string("已切换全身IK后端: ") + ikState->full_body_backend;
            }
            kinematic_viewer::PushSidebarHalfWidth();
            ImGui::DragInt("迭代", &ikState->full_body_iterations, 1.0f, 1, 30);
            kinematic_viewer::PopSidebarWidth();
            ikState->full_body_iterations = std::max(1, ikState->full_body_iterations);
            ImGui::TextDisabled("彩色目标轴: 亮色=当前控制链，淡色=其他链");
        }
        ikState->use_external_target = false;
        ImGui::TextDisabled("外部目标位姿输入已禁用（ROS 功能已移除）");

        const auto& chain_status = ikState->chains[ikState->selected_chain];
        if (ikState->solve_mode == "single_chain" && !chain_status.ready) {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "IK链不可用: %s", chain_status.error.c_str());
            return;
        }
        if (!ikState->marker_initialized) {
            ikController->LoadActiveMarkerFromTarget(scene);
        }
        if (ImGui::CollapsingHeader("Gizmo与拖动")) {
            kinematic_viewer::SidebarCheckboxRow2("锁姿态", &ikState->lock_orientation, "拖动实时IK", &ikState->realtime_ik_during_drag);
            kinematic_viewer::SidebarRadioRow3("模式", &ikState->gizmo_operation, "平移", "旋转", "全");
            ImGui::TextUnformatted("坐标系");
            ImGui::SameLine();
            if (ImGui::RadioButton("W", ikState->gizmo_world_mode)) {
                ikState->gizmo_world_mode = true;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("L", !ikState->gizmo_world_mode)) {
                ikState->gizmo_world_mode = false;
            }
            kinematic_viewer::SidebarSliderFloat("Gizmo尺寸", &ikState->gizmo_size_clip_space, 0.10f, 0.40f, "%.2f");
            if (ikState->realtime_ik_during_drag) {
                ImGui::Checkbox("旋转拖动实时IK", &ikState->realtime_ik_rotate_during_drag);
                kinematic_viewer::SidebarSliderFloat("IK Hz", &ikState->realtime_ik_hz, 5.0f, 120.0f, "%.0f");
                if (!ikState->realtime_ik_rotate_during_drag) {
                    ImGui::TextDisabled("旋转拖动默认仅松手求解（更稳）");
                }
                if (ikState->solve_mode == "full_body") {
                    ImGui::TextDisabled("full_body 拖动时频率自动上限：平移12Hz，姿态4Hz");
                    ImGui::TextDisabled("full_body 平移拖动走位置优先，旋转拖动走姿态求解");
                }
            }
            if (ikState->solve_mode == "full_body") {
                ImGui::Checkbox("松手后末端精修(single_chain)", &ikState->refine_single_chain_on_drag_end);
                if (ikState->refine_single_chain_on_drag_end) {
                    ImGui::Checkbox("仅旋转拖动时触发精修", &ikState->refine_only_when_rotation);
                }
            }
        }

        if (ImGui::CollapsingHeader("吸附与增益")) {
            kinematic_viewer::SidebarCheckboxRow2("平移吸附", &ikState->translate_snap_enabled, "旋转吸附", &ikState->rotate_snap_enabled);
            if (ikState->translate_snap_enabled) {
                kinematic_viewer::SidebarDragFloat("平移步长", &ikState->translate_snap_step_m, 0.001f, 0.001f, 0.20f, "%.3f");
            }
            if (ikState->rotate_snap_enabled) {
                float rotate_snap_ui = AngleUiFromDegStored(ikState->rotate_snap_step_deg, use_deg);
                const float snap_max = use_deg ? 45.0f : glm::radians(45.0f);
                char snap_label[32];
                std::snprintf(snap_label, sizeof(snap_label), "旋转步长(%s)", AngleUnitLabel(use_deg));
                kinematic_viewer::SidebarDragFloat(snap_label, &rotate_snap_ui, use_deg ? 0.2f : 0.01f, use_deg ? 0.2f : 0.01f, snap_max,
                                                   AngleInputFormat(use_deg));
                ikState->rotate_snap_step_deg = AngleUiToDegStored(rotate_snap_ui, use_deg);
            }
            ImGui::Columns(2, "##ik_gain_cols", false);
            kinematic_viewer::SidebarSliderFloat("Tx", &ikState->translate_channel_gain[0], 0.0f, 2.0f, "%.2f");
            ImGui::NextColumn();
            kinematic_viewer::SidebarSliderFloat("Ty", &ikState->translate_channel_gain[1], 0.0f, 2.0f, "%.2f");
            ImGui::NextColumn();
            kinematic_viewer::SidebarSliderFloat("Tz", &ikState->translate_channel_gain[2], 0.0f, 2.0f, "%.2f");
            ImGui::NextColumn();
            kinematic_viewer::SidebarSliderFloat("Rx", &ikState->rotate_channel_gain[0], 0.0f, 2.0f, "%.2f");
            ImGui::NextColumn();
            kinematic_viewer::SidebarSliderFloat("Ry", &ikState->rotate_channel_gain[1], 0.0f, 2.0f, "%.2f");
            ImGui::NextColumn();
            kinematic_viewer::SidebarSliderFloat("Rz", &ikState->rotate_channel_gain[2], 0.0f, 2.0f, "%.2f");
            ImGui::Columns(1);
        }

        if (ImGui::CollapsingHeader("目标位姿与求解", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextDisabled("直接在3D视窗抓取 Gizmo 轴/圆环进行平移或旋转");
            const glm::vec3 marker_pos_now(ikState->marker_pos[0], ikState->marker_pos[1], ikState->marker_pos[2]);
            const glm::quat marker_q_now       = glm::normalize(glm::quat_cast(markerWorldMatrix(
                glm::vec3(0.0f), glm::vec3(ikState->marker_rpy_deg[0], ikState->marker_rpy_deg[1], ikState->marker_rpy_deg[2]))));
            const std::string marker_pose_text = FormatPoseInputXyzQuat(marker_pos_now, marker_q_now);
            char marker_pose_display[256]      = {0};
            std::snprintf(marker_pose_display, sizeof(marker_pose_display), "%s", marker_pose_text.c_str());
            kinematic_viewer::PushSidebarFullWidth();
            ImGui::InputText("当前位姿", marker_pose_display, sizeof(marker_pose_display), ImGuiInputTextFlags_ReadOnly);
            kinematic_viewer::PopSidebarWidth();
            if (ImGui::SmallButton("复制")) {
                ImGui::SetClipboardText(marker_pose_display);
            }
            static char marker_pose_input[256] = "";
            if (marker_pose_input[0] == '\0') {
                const std::string init_pose_text = FormatPoseInputXyzQuat(marker_pos_now, marker_q_now);
                std::snprintf(marker_pose_input, sizeof(marker_pose_input), "%s", init_pose_text.c_str());
            }
            kinematic_viewer::PushSidebarFullWidth();
            ImGui::InputText("目标位姿", marker_pose_input, sizeof(marker_pose_input));
            kinematic_viewer::PopSidebarWidth();
            if (ImGui::SmallButton("填充")) {
                const glm::vec3 marker_pos_now(ikState->marker_pos[0], ikState->marker_pos[1], ikState->marker_pos[2]);
                const glm::quat marker_q_now = glm::normalize(glm::quat_cast(markerWorldMatrix(
                    glm::vec3(0.0f), glm::vec3(ikState->marker_rpy_deg[0], ikState->marker_rpy_deg[1], ikState->marker_rpy_deg[2]))));
                const std::string pose_text  = FormatPoseInputXyzQuat(marker_pos_now, marker_q_now);
                std::snprintf(marker_pose_input, sizeof(marker_pose_input), "%s", pose_text.c_str());
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("应用")) {
                glm::vec3 parsed_pos(0.0f);
                glm::quat parsed_quat(1.0f, 0.0f, 0.0f, 0.0f);
                std::string parse_error;
                if (ParsePoseInputXyzQuat(marker_pose_input, &parsed_pos, &parsed_quat, &parse_error)) {
                    ikState->marker_pos[0] = parsed_pos.x;
                    ikState->marker_pos[1] = parsed_pos.y;
                    ikState->marker_pos[2] = parsed_pos.z;
                    if (!ikState->lock_orientation) {
                        const glm::vec3 parsed_rpy = glm::degrees(glm::eulerAngles(parsed_quat));
                        ikState->marker_rpy_deg[0] = parsed_rpy.x;
                        ikState->marker_rpy_deg[1] = parsed_rpy.y;
                        ikState->marker_rpy_deg[2] = parsed_rpy.z;
                    }
                    ikController->SaveActiveMarkerToTarget();
                    ikController->ApplyIkForActiveChain(scene, false, false, false);
                    if (ikState->lock_orientation) {
                        ikState->last_status = "位姿串已应用：姿态锁定，仅更新了位置";
                    } else {
                        ikState->last_status = "位姿串已应用";
                    }
                } else {
                    ikState->last_status = std::string("位姿串应用失败: ") + parse_error;
                }
            }
            kinematic_viewer::PushSidebarFullWidth();
            bool marker_pos_edited = ImGui::DragFloat3("位置", ikState->marker_pos, 0.002f, -2.0f, 2.0f, "%.3f");
            bool marker_pos_commit = ImGui::IsItemDeactivatedAfterEdit();
            ImGui::BeginDisabled(ikState->lock_orientation);
            glm::vec3 marker_rpy_ui =
                RpyUiFromDegStored(glm::vec3(ikState->marker_rpy_deg[0], ikState->marker_rpy_deg[1], ikState->marker_rpy_deg[2]), use_deg);
            char rpy_label[24];
            std::snprintf(rpy_label, sizeof(rpy_label), "RPY(%s)", AngleUnitLabel(use_deg));
            bool marker_rot_edited = ImGui::DragFloat3(rpy_label, glm::value_ptr(marker_rpy_ui), use_deg ? 0.2f : 0.01f,
                                                       AngleDragMin(use_deg), AngleDragMax(use_deg), AngleInputFormat(use_deg));
            if (marker_rot_edited) {
                const glm::vec3 stored_rpy = RpyUiToDegStored(marker_rpy_ui, use_deg);
                ikState->marker_rpy_deg[0] = stored_rpy.x;
                ikState->marker_rpy_deg[1] = stored_rpy.y;
                ikState->marker_rpy_deg[2] = stored_rpy.z;
            }
            kinematic_viewer::PopSidebarWidth();
            bool marker_rot_commit = ImGui::IsItemDeactivatedAfterEdit();
            ImGui::EndDisabled();
            if (marker_pos_edited || marker_rot_edited) {
                ikController->SaveActiveMarkerToTarget();
                if (ikState->realtime_ik_during_drag) {
                    const bool position_only_target = marker_pos_edited && !marker_rot_edited;
                    ikController->ApplyIkForActiveChain(scene, false, true, position_only_target);
                }
            } else if (!ikState->realtime_ik_during_drag && (marker_pos_commit || marker_rot_commit)) {
                const bool position_only_target = marker_pos_commit && !marker_rot_commit;
                ikController->ApplyIkForActiveChain(scene, false, false, position_only_target);
            }
            if (ImGui::SmallButton("同步末端")) {
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ikState->solver.fetchTipWorldPose(*scene, ikState->selected_chain, &tip_pos, &tip_rpy)) {
                    ikState->marker_pos[0]      = tip_pos.x;
                    ikState->marker_pos[1]      = tip_pos.y;
                    ikState->marker_pos[2]      = tip_pos.z;
                    ikState->marker_rpy_deg[0]  = glm::degrees(tip_rpy.x);
                    ikState->marker_rpy_deg[1]  = glm::degrees(tip_rpy.y);
                    ikState->marker_rpy_deg[2]  = glm::degrees(tip_rpy.z);
                    ikState->marker_initialized = true;
                    ikController->SaveActiveMarkerToTarget();
                }
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("求解IK")) {
                ikController->ApplyIkForActiveChain(scene, false, false, false);
            }
            if (!ikState->last_status.empty()) {
                ImGui::TextUnformatted(ikState->last_status.c_str());
            }
            ImGui::TextDisabled("base=%s  tip=%s", chain_status.config.base_link.c_str(), chain_status.config.tip_link.c_str());

            glm::vec3 tip_pos(0.0f);
            glm::vec3 tip_rpy(0.0f);
            if (ikState->solver.fetchTipWorldPose(*scene, ikState->selected_chain, &tip_pos, &tip_rpy)) {
                glm::vec3 marker_p(ikState->marker_pos[0], ikState->marker_pos[1], ikState->marker_pos[2]);
                float pos_err_mm = glm::length(marker_p - tip_pos) * 1000.0f;
                const glm::vec3 tip_rpy_ui(AngleUiFromRad(tip_rpy.x, use_deg), AngleUiFromRad(tip_rpy.y, use_deg),
                                           AngleUiFromRad(tip_rpy.z, use_deg));
                const glm::vec3 marker_rpy_ui = RpyUiFromDegStored(
                    glm::vec3(ikState->marker_rpy_deg[0], ikState->marker_rpy_deg[1], ikState->marker_rpy_deg[2]), use_deg);
                const glm::vec3 drpy = glm::abs(marker_rpy_ui - tip_rpy_ui);
                ImVec4 ce            = (pos_err_mm < 2.0f)
                                           ? ImVec4(0.6f, 0.95f, 0.6f, 1.0f)
                                           : ((pos_err_mm < 8.0f) ? ImVec4(1.0f, 0.85f, 0.3f, 1.0f) : ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
                ImGui::TextColored(ce, "末端误差: 位置 %.2f mm, 姿态 %.2f/%.2f/%.2f %s", pos_err_mm, drpy.x, drpy.y, drpy.z,
                                   AngleUnitLabel(use_deg));
            }
        }
    }

    void RenderJointPanel(ViewerState* uiState, rkv::RobotScene* scene,
                          const std::vector<rkv::RobotScene::JointInfo>& joints) {
        if (uiState == nullptr || scene == nullptr) {
            return;
        }
        if (!ImGui::CollapsingHeader("关节调试", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }

        int revoluteCount   = 0;
        int clampedCount    = 0;
        float minMarginDeg  = 1e9f;
        std::string minName = "";
        for (const auto& j : joints) {
            if (j.revolute) {
                ++revoluteCount;
            }
            if (j.position < j.min_angle - 1e-5f || j.position > j.max_angle + 1e-5f) {
                ++clampedCount;
            }
            if (j.revolute) {
                float d0 = std::fabs(j.position - j.min_angle);
                float d1 = std::fabs(j.max_angle - j.position);
                float m  = glm::degrees(std::min(d0, d1));
                if (m < minMarginDeg) {
                    minMarginDeg = m;
                    minName      = j.name;
                }
            }
        }

        SidebarInputText("过滤", uiState->joint_filter, sizeof(uiState->joint_filter));
        std::string filter = uiState->joint_filter;
        std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        const bool joint_use_deg     = uiState->angle_unit_deg;
        const char* joint_unit_label = AngleUnitLabel(joint_use_deg);
        const char* angle_input_fmt  = AngleInputFormat(joint_use_deg);

        ImGui::Text("共 %d | 旋转 %d | 越界 %d", static_cast<int>(joints.size()), revoluteCount, clampedCount);
        if (!minName.empty()) {
            ImVec4 c = (minMarginDeg < 3.0f) ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)
                                             : ((minMarginDeg < 8.0f) ? ImVec4(1.0f, 0.75f, 0.25f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f));
            if (joint_use_deg) {
                ImGui::TextColored(c, "最小限位裕量: %.2f %s (%s)", minMarginDeg, joint_unit_label, minName.c_str());
            } else {
                ImGui::TextColored(c, "最小限位裕量: %.4f %s (%s)", glm::radians(minMarginDeg), joint_unit_label, minName.c_str());
            }
        }

        if (uiState->joint_input_groups.empty()) {
            ImGui::TextDisabled("未配置关节分组（见 config.initial_pose）");
        } else if (ImGui::CollapsingHeader("分组复制 / 应用", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (uiState->selected_joint_input_group < 0 ||
                uiState->selected_joint_input_group >= static_cast<int>(uiState->joint_input_groups.size())) {
                uiState->selected_joint_input_group = 0;
            }
            std::vector<const char*> groupNames;
            groupNames.reserve(uiState->joint_input_groups.size());
            for (const auto& group : uiState->joint_input_groups) {
                groupNames.push_back(group.name.c_str());
            }
            SidebarCombo("分组", &uiState->selected_joint_input_group, groupNames.data(), static_cast<int>(groupNames.size()));

            const auto& selectedGroup = uiState->joint_input_groups[static_cast<size_t>(uiState->selected_joint_input_group)];
            ImGui::Text("%s (%d 关节)", selectedGroup.name.c_str(), static_cast<int>(selectedGroup.joint_names.size()));
            const std::string current_group_line =
                kinematic_sidebar_panels_internal::FormatJointGroupLine(selectedGroup, scene, joint_use_deg);

            ImGui::TextUnformatted("当前分组数据（复制即此格式）");
            ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(40, 48, 58, 255));
            char group_preview[512] = {0};
            std::snprintf(group_preview, sizeof(group_preview), "%s", current_group_line.c_str());
            ImGui::InputTextMultiline("##group_preview", group_preview, sizeof(group_preview),
                                      ImVec2(-1.0f, ImGui::GetTextLineHeight() * 2.2f), ImGuiInputTextFlags_ReadOnly);
            ImGui::PopStyleColor();

            if (ImGui::Button("复制当前分组到剪贴板")) {
                kinematic_sidebar_panels_internal::CopyTextToClipboard(current_group_line);
                uiState->joint_group_input_last_ok = true;
                uiState->joint_group_input_status  = "已复制: " + selectedGroup.name;
            }
            ImGui::SameLine();
            if (ImGui::Button("复制全部分组")) {
                kinematic_sidebar_panels_internal::CopyTextToClipboard(
                    kinematic_sidebar_panels_internal::FormatAllJointGroups(*uiState, scene, joint_use_deg));
                uiState->joint_group_input_last_ok = true;
                uiState->joint_group_input_status =
                    "已复制 head/leg/left_arm/right_arm 共 " + std::to_string(uiState->joint_input_groups.size()) + " 行";
            }

            ImGui::Separator();
            ImGui::Text("编辑后应用（单位：%s）", joint_unit_label);
            if (ImGui::SmallButton("用当前姿态填充")) {
                kinematic_sidebar_panels_internal::CopyJointGroupLineToInput(uiState->joint_group_values_input,
                                                                             sizeof(uiState->joint_group_values_input), current_group_line);
                uiState->joint_group_input_last_ok = true;
                uiState->joint_group_input_status  = "已填充当前分组角度";
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("从剪贴板填入")) {
                const char* clip = ImGui::GetClipboardText();
                if (clip != nullptr && clip[0] != '\0') {
                    kinematic_sidebar_panels_internal::CopyJointGroupLineToInput(uiState->joint_group_values_input,
                                                                                 sizeof(uiState->joint_group_values_input), clip);
                    uiState->joint_group_input_last_ok = true;
                    uiState->joint_group_input_status  = "已从剪贴板填入下方文本框";
                } else {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "剪贴板为空";
                }
            }
            {
                char angle_field_label[32];
                std::snprintf(angle_field_label, sizeof(angle_field_label), "角度(%s)", joint_unit_label);
                SidebarInputTextMultiline(angle_field_label, uiState->joint_group_values_input, sizeof(uiState->joint_group_values_input),
                                          52.0f);
            }
            if (ImGui::Button("应用下方文本到当前分组")) {
                std::vector<float> values_rad;
                std::string error;
                if (!kinematic_sidebar_panels_internal::ParseJointValuesToRad(uiState->joint_group_values_input, joint_use_deg, &values_rad,
                                                                              &error)) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                } else if (!kinematic_sidebar_panels_internal::ApplyJointGroupValues(selectedGroup, values_rad, scene, &error)) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                } else {
                    uiState->joint_group_input_last_ok = true;
                    uiState->joint_group_input_status  = "应用成功: " + selectedGroup.name;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("直接应用剪贴板一行") && ImGui::GetClipboardText() != nullptr) {
                std::string error;
                if (kinematic_sidebar_panels_internal::ApplyJointGroupTextCommand(ImGui::GetClipboardText(), *uiState, scene, &error)) {
                    uiState->joint_group_input_last_ok = true;
                    uiState->joint_group_input_status  = "已从剪贴板应用";
                } else {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                }
            }

            if (ImGui::TreeNode("一次设置多组（高级）")) {
                ImGui::TextDisabled("每行一组，数值单位跟随上方 deg/rad 选项：");
                ImGui::TextDisabled(joint_use_deg ? "left_arm: 18.25、-60.08、43.12、..." : "left_arm: 0.32、-1.05、0.75、...");
                ImGui::TextDisabled(joint_use_deg ? "head: 0.00、5.00" : "head: 0.00、0.09");
                SidebarInputTextMultiline("多组命令", uiState->joint_group_input, sizeof(uiState->joint_group_input), 56.0f);
                if (ImGui::Button("应用多组命令")) {
                    const std::string rawText               = uiState->joint_group_input;
                    const std::vector<std::string> commands = kinematic_sidebar_panels_internal::SplitJointGroupCommands(rawText);
                    if (commands.empty()) {
                        uiState->joint_group_input_last_ok = false;
                        uiState->joint_group_input_status  = "输入为空";
                    } else {
                        bool allOk  = true;
                        int okCount = 0;
                        std::vector<std::string> failures;
                        failures.reserve(commands.size());
                        for (size_t i = 0; i < commands.size(); ++i) {
                            std::string error;
                            if (kinematic_sidebar_panels_internal::ApplyJointGroupTextCommand(commands[i], *uiState, scene, &error)) {
                                ++okCount;
                            } else {
                                allOk = false;
                                std::stringstream ss;
                                ss << "第" << (i + 1) << "条失败: " << error;
                                failures.push_back(ss.str());
                            }
                        }
                        uiState->joint_group_input_last_ok = allOk;
                        std::stringstream status;
                        status << "已应用 " << okCount << "/" << commands.size() << " 条";
                        if (!failures.empty()) {
                            status << " | " << failures.front();
                        }
                        uiState->joint_group_input_status = status.str();
                    }
                }
                ImGui::TreePop();
            }
            if (!uiState->joint_group_input_status.empty()) {
                const ImVec4 statusColor =
                    uiState->joint_group_input_last_ok ? ImVec4(0.45f, 0.95f, 0.45f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f);
                ImGui::TextColored(statusColor, "%s", uiState->joint_group_input_status.c_str());
            }
        }

        if (ImGui::Button("归零")) {
            for (const auto& j : joints) {
                if (j.revolute) {
                    scene->setJointPositionByName(j.name, 0.0f);
                }
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("夹紧限位")) {
            for (const auto& j : joints) {
                if (!j.revolute) {
                    continue;
                }
                float v = std::clamp(j.position, j.min_angle, j.max_angle);
                scene->setJointPositionByName(j.name, v);
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("存姿态")) {
            uiState->pose_snapshot.clear();
            for (const auto& j : joints) {
                uiState->pose_snapshot[j.name] = j.position;
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("恢复姿态") && !uiState->pose_snapshot.empty()) {
            for (const auto& [name, value] : uiState->pose_snapshot) {
                scene->setJointPositionByName(name, value);
            }
        }

        if (!ImGui::CollapsingHeader("关节表", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        std::string parent_joint_for_selected_link;
        if (!uiState->selected_link.empty()) {
            scene->getParentJointNameForLink(uiState->selected_link, &parent_joint_for_selected_link);
        }
        const ImVec2 joint_table_size = SidebarTableFillHeight(uiState->joint_section_height);
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(6.0f, 5.0f));
        if (ImGui::BeginTable("joint_table", 4,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit,
                              joint_table_size)) {
            ImGui::TableSetupColumn("关节", ImGuiTableColumnFlags_WidthFixed, 108.0f);
            ImGui::TableSetupColumn("滑条", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn(joint_use_deg ? "deg###joint_val" : "rad###joint_val", ImGuiTableColumnFlags_WidthFixed, 84.0f);
            ImGui::TableSetupColumn("限位", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            for (const auto& j : joints) {
                if (!uiState->show_non_revolute && !j.revolute) {
                    continue;
                }
                std::string nameLower = j.name;
                std::transform(nameLower.begin(), nameLower.end(), nameLower.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (!filter.empty() && nameLower.find(filter) == std::string::npos) {
                    continue;
                }

                ImGui::TableNextRow();
                ImGui::PushID(j.name.c_str());

                ImGui::TableSetColumnIndex(0);
                const bool joint_row_selected = (!parent_joint_for_selected_link.empty() && j.name == parent_joint_for_selected_link);
                if (ImGui::Selectable(j.name.c_str(), joint_row_selected)) {
                    rkv::RobotScene::JointDetailInfo detail;
                    if (scene->getJointDetail(j.name, &detail)) {
                        uiState->selected_link            = detail.child_link;
                        uiState->trajectory_min_surface_m = -1.0f;
                    }
                }

                ImGui::TableSetColumnIndex(1);
                if (j.revolute) {
                    const float min_ui      = AngleUiFromRad(j.min_angle, joint_use_deg);
                    const float max_ui      = AngleUiFromRad(j.max_angle, joint_use_deg);
                    const ImGuiID slider_id = ImGui::GetID("##slider");
                    const ImGuiID input_id  = ImGui::GetID("##joint_in");
                    float uiValue           = AngleUiFromRad(j.position, joint_use_deg);
                    ImGuiContext* imgui_ctx = ImGui::GetCurrentContext();
                    if (imgui_ctx != nullptr && imgui_ctx->ActiveId != slider_id && imgui_ctx->ActiveId != input_id) {
                        rkv::RobotScene::JointInfo live;
                        if (scene->getJointInfo(j.name, &live)) {
                            uiValue = AngleUiFromRad(live.position, joint_use_deg);
                        }
                    }

                    if (SidebarSliderFloatNoInput("##slider", &uiValue, min_ui, max_ui)) {
                        uiValue = std::clamp(uiValue, min_ui, max_ui);
                        scene->setJointPositionByName(j.name, AngleUiToRad(uiValue, joint_use_deg));
                    }

                    ImGui::TableSetColumnIndex(2);
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                    ImGui::InputFloat("##joint_in", &uiValue, 0.0f, 0.0f, angle_input_fmt);
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        uiValue = std::clamp(uiValue, min_ui, max_ui);
                        scene->setJointPositionByName(j.name, AngleUiToRad(uiValue, joint_use_deg));
                    }
                    ImGui::PopStyleVar();
                } else {
                    ImGui::TextDisabled("-");
                    ImGui::TableSetColumnIndex(2);
                    float pos_m = j.position;
                    rkv::RobotScene::JointInfo live;
                    if (scene->getJointInfo(j.name, &live)) {
                        pos_m = live.position;
                    }
                    ImGui::SetNextItemWidth(-1.0f);
                    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4.0f, 4.0f));
                    ImGui::InputFloat("##m_in", &pos_m, 0.0f, 0.0f, "%.4f");
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        scene->setJointPositionByName(j.name, pos_m);
                    }
                    ImGui::PopStyleVar();
                }

                ImGui::PopID();

                ImGui::TableSetColumnIndex(3);
                if (j.revolute) {
                    if (joint_use_deg) {
                        ImGui::Text("%.0f~%.0f", glm::degrees(j.min_angle), glm::degrees(j.max_angle));
                    } else {
                        ImGui::Text("%.2f~%.2f", j.min_angle, j.max_angle);
                    }
                } else {
                    ImGui::TextDisabled("-");
                }
            }
            ImGui::EndTable();
        }
        ImGui::PopStyleVar();
    }

    void RenderPlaybackPanel(DebugPlaybackState* playbackState, TrajectoryPlayer* playbackPlayer, PlaybackStateMachine* playback_sm,
                             rkv::RobotScene* scene, const std::vector<rkv::RobotScene::JointInfo>& joints, ViewerState* viewerState) {
        if (playbackState == nullptr || playbackPlayer == nullptr || playback_sm == nullptr || scene == nullptr) {
            return;
        }

        constexpr const char* kTrajectoryAlertPopupId = "轨迹告警##trajectory_incompatible_alert_popup";
        if (playbackState->trajectory_alert_popup_pending) {
            ImGui::OpenPopup(kTrajectoryAlertPopupId);
            playbackState->trajectory_alert_popup_pending = false;
        }
        ImGui::SetNextWindowSize(ImVec2(520.0f, 0.0f), ImGuiCond_Appearing);
        if (ImGui::BeginPopupModal(kTrajectoryAlertPopupId, nullptr, ImGuiWindowFlags_NoResize)) {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.22f, 1.0f), "轨迹文件不适用");
            ImGui::Separator();
            ImGui::TextWrapped("%s", playbackState->trajectory_alert_message.empty() ? "该轨迹无法用于当前机器人。"
                                                                                     : playbackState->trajectory_alert_message.c_str());
            if (!playbackState->trajectory_alert_detail.empty() && ImGui::CollapsingHeader("查看详情")) {
                ImGui::TextWrapped("%s", playbackState->trajectory_alert_detail.c_str());
            }
            ImGui::Spacing();
            if (ImGui::Button("知道了", ImVec2(120.0f, 0.0f))) {
                playbackState->trajectory_alert_detail.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::Separator();
        ImGui::TextUnformatted("轨迹关键帧回放");

        if (ImGui::CollapsingHeader("文件", ImGuiTreeNodeFlags_DefaultOpen)) {
            ProcessPendingTrajectoryLoad(playbackState, joints, playbackPlayer, scene, playback_sm);
            SidebarDragFloat("关键帧间隔", &playbackState->keyframe_interval_sec, 0.02f, 0.02f, 5.0f, "%.2f");

            ImGui::TextDisabled("单击选中并自动加载；双击播放；勾选多项后点连播");
            // Trajectory file list
            if (!playbackState->trajectory_files.empty()) {
                ImGui::TextUnformatted("轨迹文件列表:");
                if (ImGui::BeginListBox("##trajectory_file_list", ImVec2(-1, 140))) {
                    for (int i = 0; i < static_cast<int>(playbackState->trajectory_files.size()); ++i) {
                        auto& entry = playbackState->trajectory_files[static_cast<size_t>(i)];
                        std::filesystem::path p(entry.path);
                        std::string label = p.filename().string();
                        if (!entry.status.empty() && entry.status != "未加载") {
                            label += " (" + entry.status + ")";
                        }
                        const bool isSelected = (playbackState->selected_trajectory_index == i);

                        char queueId[32];
                        std::snprintf(queueId, sizeof(queueId), "##queue_%d", i);
                        ImGui::Checkbox(queueId, &entry.queued);
                        ImGui::SameLine();

                        ImVec4 textColor = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
                        if (entry.loaded) {
                            textColor = ImVec4(0.40f, 0.84f, 0.52f, 1.0f);
                        } else if (!entry.status.empty() && entry.status.find("失败") != std::string::npos) {
                            textColor = ImVec4(0.95f, 0.42f, 0.42f, 1.0f);
                        }
                        ImGui::PushStyleColor(ImGuiCol_Text, textColor);
                        const bool clicked       = ImGui::Selectable(label.c_str(), isSelected, ImGuiSelectableFlags_AllowDoubleClick);
                        const bool doubleClicked = ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
                        if (clicked || doubleClicked) {
                            playbackState->selected_trajectory_index = i;
                            if (ImGui::GetIO().KeyCtrl) {
                                entry.queued = !entry.queued;
                            } else if (doubleClicked) {
                                if (LoadTrajectoryListEntry(playbackState, i, joints, playbackPlayer, scene)) {
                                    CancelTrajectorySequence(playbackState);
                                    playbackState->play_time = 0.0f;
                                    playback_sm->Play();
                                }
                            } else {
                                LoadTrajectoryListEntry(playbackState, i, joints, playbackPlayer, scene);
                            }
                        }
                        ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered() && !entry.path.empty()) {
                            ImGui::SetTooltip("%s\n单击加载 | 双击播放 | Ctrl+单击切换连播勾选", entry.path.c_str());
                        }
                    }
                    ImGui::EndListBox();
                }
            }

            // Buttons row
            if (ImGui::Button("+ 添加")) {
                kinematic_sidebar_panels_internal::RenderTrajectoryFileBrowser(playbackState);
            }
            ImGui::SameLine();
            if (ImGui::Button("- 删除") && playbackState->selected_trajectory_index >= 0) {
                playbackState->trajectory_files.erase(playbackState->trajectory_files.begin() + playbackState->selected_trajectory_index);
                if (playbackState->selected_trajectory_index >= static_cast<int>(playbackState->trajectory_files.size())) {
                    playbackState->selected_trajectory_index = static_cast<int>(playbackState->trajectory_files.size()) - 1;
                }
            }
            ImGui::SameLine();
            int queuedCount = 0;
            for (const auto& entry : playbackState->trajectory_files) {
                if (entry.queued) {
                    ++queuedCount;
                }
            }
            char sequenceLabel[48];
            std::snprintf(sequenceLabel, sizeof(sequenceLabel), "连播选中 (%d)", queuedCount);
            if (ImGui::Button(sequenceLabel) && queuedCount > 0) {
                StartTrajectorySequence(playbackState, joints, playbackPlayer, scene, playback_sm);
            }
            ImGui::SameLine();
            if (ImGui::Button("清空列表")) {
                CancelTrajectorySequence(playbackState);
                playbackState->trajectory_files.clear();
                playbackState->selected_trajectory_index = -1;
            }

            ImGui::InputText("轨迹文件", playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path));
            ImGui::SameLine();
            kinematic_sidebar_panels_internal::RenderTrajectoryFileBrowser(playbackState);
            ProcessPendingTrajectoryLoad(playbackState, joints, playbackPlayer, scene, playback_sm);

            if (ImGui::Button("重新加载") && playbackState->selected_trajectory_index >= 0) {
                LoadTrajectoryListEntry(playbackState, playbackState->selected_trajectory_index, joints, playbackPlayer, scene);
            }
            ImGui::SameLine();
            if (ImGui::Button("保存当前轨迹")) {
                std::string ioError;
                if (SaveTrajectoryToFile(playbackState->trajectory_file_path, *playbackState, &ioError)) {
                    playbackState->trajectory_io_status = "保存成功";
                } else {
                    playbackState->trajectory_io_status = "保存失败: " + ioError;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("生成Demo轨迹")) {
                BuildDemoTrajectoryFromCurrentPose(playbackState, joints, *scene);
                std::string ioError;
                if (SaveTrajectoryToFile(playbackState->trajectory_file_path, *playbackState, &ioError)) {
                    playbackState->trajectory_io_status = "Demo轨迹已生成并保存";
                } else {
                    playbackState->trajectory_io_status = "Demo生成成功但保存失败: " + ioError;
                }
                playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
            }

            if (!playbackState->trajectory_io_status.empty()) {
                ImVec4 color(0.66f, 0.72f, 0.80f, 1.0f);
                if (playbackState->trajectory_io_status.find("失败") != std::string::npos) {
                    color = ImVec4(0.95f, 0.42f, 0.42f, 1.0f);
                } else if (playbackState->trajectory_io_status.find("成功") != std::string::npos) {
                    color = ImVec4(0.40f, 0.84f, 0.52f, 1.0f);
                }
                ImGui::TextColored(color, "%s", playbackState->trajectory_io_status.c_str());
            }

            kinematic_sidebar_panels_internal::RenderPlaybackBundleImport(playbackState, viewerState);
        }

        if (ImGui::CollapsingHeader("播放控制", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("记录关键帧")) {
                playbackPlayer->RecordKeyframe(playbackState, joints, *scene);
            }
            ImGui::SameLine();
            const bool hasKeyframes = !playbackState->keyframes.empty();
            const bool playing      = playback_sm->IsPlaying();
            if (!hasKeyframes) {
                ImGui::BeginDisabled();
            }
            if (playing) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("播放")) {
                playback_sm->Play();
            }
            if (playing) {
                ImGui::EndDisabled();
            }
            ImGui::SameLine();
            if (!playing) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("暂停")) {
                playback_sm->Pause();
            }
            if (!playing) {
                ImGui::EndDisabled();
            }
            if (!hasKeyframes) {
                ImGui::EndDisabled();
            }
            ImGui::SameLine();
            if (ImGui::Button("停止")) {
                CancelTrajectorySequence(playbackState);
                playback_sm->Stop();
            }
            ImGui::SameLine();
            if (ImGui::Button("清空")) {
                playbackPlayer->Clear(playbackState);
            }

            ImGui::Checkbox("循环回放", &playbackState->loop);
            ImGui::SliderFloat("回放倍速", &playbackState->play_speed, 0.1f, 3.0f, "%.2fx");

            if (!playbackState->keyframes.empty()) {
                float total = TrajectoryPlayer::TotalDuration(*playbackState);
                if (ImGui::SliderFloat("回放时间", &playbackState->play_time, 0.0f, std::max(0.0f, total), "%.2f s")) {
                    playbackState->timeline_edited_this_ui = true;
                }
                if (playbackState->timeline_edited_this_ui) {
                    playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                    playbackState->timeline_edited_this_ui = false;
                }

                const char* modeLabel = "Stopped";
                if (playback_sm->IsPlaying()) {
                    modeLabel = "Playing";
                } else if (playback_sm->IsPaused()) {
                    modeLabel = "Paused";
                }
                int baseKeyframeCount = 0;
                for (const auto& keyframe : playbackState->keyframes) {
                    if (keyframe.has_base_pose_2d) {
                        ++baseKeyframeCount;
                    }
                }
                ImGui::Text("状态: %s  总时长: %.2fs  当前段: %d", modeLabel, total, playbackState->current_segment_index);
                ImGui::Text("关键帧数: %d", static_cast<int>(playbackState->keyframes.size()));
                ImGui::Text("底盘关键帧: %d", baseKeyframeCount);
                if (baseKeyframeCount > 0 && scene->fixedBaseMode()) {
                    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.30f, 1.0f), "提示: 当前固定底座模式开启，底盘轨迹不会体现在模型位姿上");
                }
            } else {
                ImGui::TextDisabled("暂无关键帧，点击“记录关键帧”开始。");
            }
        }

        if (ImGui::CollapsingHeader("关键帧列表", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!playbackState->keyframes.empty() && playbackState->selected_keyframe_index >= 0 &&
                playbackState->selected_keyframe_index < static_cast<int>(playbackState->keyframes.size())) {
                if (ImGui::Button("删除选中关键帧")) {
                    playbackPlayer->RemoveSelectedKeyframe(playbackState);
                }
            }

            if (playbackState->keyframes.empty()) {
                ImGui::TextDisabled("暂无关键帧。");
            } else if (ImGui::BeginTable("keyframe_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                                         ImVec2(0.0f, 200.0f))) {
                ImGui::TableSetupColumn("索引");
                ImGui::TableSetupColumn("时间(s)");
                ImGui::TableSetupColumn("关节数");
                ImGui::TableSetupColumn("底盘");
                ImGui::TableHeadersRow();
                for (int i = 0; i < static_cast<int>(playbackState->keyframes.size()); ++i) {
                    const auto& keyframe = playbackState->keyframes[static_cast<size_t>(i)];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    char selectLabel[32];
                    snprintf(selectLabel, sizeof(selectLabel), "KF %d", i);
                    if (ImGui::Selectable(selectLabel, playbackState->selected_keyframe_index == i, ImGuiSelectableFlags_SpanAllColumns)) {
                        playbackState->selected_keyframe_index = i;
                        playbackState->play_time               = static_cast<float>(keyframe.t);
                        playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
                    }
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.2f", keyframe.t);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%d", static_cast<int>(keyframe.joints.size()));
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(keyframe.has_base_pose_2d ? "有" : "-");
                }
                ImGui::EndTable();
            }
        }
    }

    void RenderSafetyPanel(CollisionMonitorState* collisionState, const CollisionMonitorResult& collisionResult) {
        if (collisionState == nullptr) {
            return;
        }
        if (!ImGui::CollapsingHeader("碰撞监控", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        SidebarCheckboxRow2("启用", &collisionState->enable, "最近对连线", &collisionState->show_closest_pair_line);
        SidebarCheckboxRow2("忽略同Link", &collisionState->ignore_same_link, "忽略相连", &collisionState->ignore_parent_child);
        SidebarDragFloat("Danger (m)", &collisionState->danger_distance_m, 0.002f, -0.20f, 0.30f, "%.3f");
        SidebarDragFloat("Warning (m)", &collisionState->warning_distance_m, 0.002f, -0.20f, 0.50f, "%.3f");
        if (collisionState->warning_distance_m < collisionState->danger_distance_m) {
            collisionState->warning_distance_m = collisionState->danger_distance_m;
        }

        if (ImGui::BeginTable("safety_stats", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("评估Pair数");
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("Warning对数");
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("Danger对数");
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("%d", collisionState->evaluated_pair_count);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%d", collisionResult.warning_pair_count);
            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%d", collisionResult.danger_pair_count);
            ImGui::EndTable();
        }
        if (!collisionState->has_valid_distance) {
            ImGui::TextDisabled("暂无可用距离数据（可能proxy不足或全部被过滤）");
            return;
        }

        ImVec4 color(0.60f, 0.95f, 0.60f, 1.0f);
        if (collisionState->nearest_surface_distance_m <= collisionState->danger_distance_m) {
            color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
        } else if (collisionState->nearest_surface_distance_m <= collisionState->warning_distance_m) {
            color = ImVec4(1.0f, 0.80f, 0.30f, 1.0f);
        }

        ImGui::Text("最近Link对: %s <-> %s", collisionState->nearest_link_a.c_str(), collisionState->nearest_link_b.c_str());
        ImGui::TextColored(color, "最近表面距离: %.3f m", collisionState->nearest_surface_distance_m);
        ImGui::Text("中心距离: %.3f m", collisionState->nearest_center_distance_m);
    }

    void RenderTfPanel(ViewerState* uiState, const std::vector<rkv::RobotScene::LinkTfInfo>& tfs) {
        if (uiState == nullptr) {
            return;
        }
        if (!ImGui::CollapsingHeader("TF 列表", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        SidebarInputText("过滤", uiState->tf_filter, sizeof(uiState->tf_filter));
        std::string tfFilter = uiState->tf_filter;
        std::transform(tfFilter.begin(), tfFilter.end(), tfFilter.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

        if (ImGui::BeginTable("tf_table", 3,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingFixedFit,
                              SidebarListSize(uiState->joint_section_height))) {
            ImGui::TableSetupColumn("Link", ImGuiTableColumnFlags_WidthFixed, 0.0f, 88.0f);
            ImGui::TableSetupColumn("父", ImGuiTableColumnFlags_WidthFixed, 0.0f, 72.0f);
            ImGui::TableSetupColumn("位姿", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();
            for (const auto& tf : tfs) {
                std::string key      = tf.name + " " + tf.parent_name;
                std::string keyLower = key;
                std::transform(keyLower.begin(), keyLower.end(), keyLower.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (!tfFilter.empty() && keyLower.find(tfFilter) == std::string::npos) {
                    continue;
                }
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                const bool link_selected = (uiState->selected_link == tf.name);
                if (ImGui::Selectable(tf.name.c_str(), link_selected, ImGuiSelectableFlags_SpanAllColumns)) {
                    uiState->selected_link            = tf.name;
                    uiState->selected_joint           = -1;
                    uiState->trajectory_min_surface_m = -1.0f;
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(tf.parent_name.empty() ? "-" : tf.parent_name.c_str());
                ImGui::TableSetColumnIndex(2);
                const float rpy_x = AngleUiFromRad(tf.world_rpy.x, uiState->angle_unit_deg);
                const float rpy_y = AngleUiFromRad(tf.world_rpy.y, uiState->angle_unit_deg);
                const float rpy_z = AngleUiFromRad(tf.world_rpy.z, uiState->angle_unit_deg);
                if (uiState->angle_unit_deg) {
                    ImGui::Text("%.2f,%.2f,%.2f | %.0f,%.0f,%.0f", tf.world_position.x, tf.world_position.y, tf.world_position.z, rpy_x,
                                rpy_y, rpy_z);
                } else {
                    ImGui::Text("%.2f,%.2f,%.2f | %.3f,%.3f,%.3f", tf.world_position.x, tf.world_position.y, tf.world_position.z, rpy_x,
                                rpy_y, rpy_z);
                }
            }
            ImGui::EndTable();
        }
    }

    // ------------------------------------------------------------------
    // Path Planner Panel
    // ------------------------------------------------------------------
    void RenderPathPlannerPanel(ViewerState* uiState, PathPlannerUiState* ui, DebugPlaybackState* playbackState,
                                rkv::RobotScene* scene, rkv::IkSolver* solver,
                                const std::vector<rkv::IkChainStatus>& chains) {
        if (uiState == nullptr || ui == nullptr || playbackState == nullptr || scene == nullptr || solver == nullptr) {
            return;
        }
        const bool use_deg = uiState->angle_unit_deg;

        if (!ImGui::CollapsingHeader("路径规划", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }

        if (!chains.empty()) {
            std::vector<const char*> chain_labels;
            for (const auto& c : chains) {
                chain_labels.push_back(c.config.label.c_str());
            }
            SidebarCombo("控制链", &ui->selected_chain, chain_labels.data(), static_cast<int>(chain_labels.size()));
        }

        const char* path_types[] = {"画圆", "画方", "头部往复", "直线", "关节空间PTP"};
        SidebarCombo("路径类型", &ui->selected_path_type, path_types, IM_ARRAYSIZE(path_types));

        // IK mode selection (only for Cartesian path types)
        if (ui->selected_path_type != 4) {
            const char* ik_modes[] = {"单链 TRAC-IK", "全身 wbc"};
            SidebarCombo("IK模式", &ui->ik_mode, ik_modes, IM_ARRAYSIZE(ik_modes));
            if (ui->ik_mode == 0) {
                ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.4f, 1.0f), "⚠ 单链模式可能出现关节跳变");
            } else {
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "✓ 全身模式求解更稳定");
            }
        }

        ImGui::Separator();

        // Parameter panels based on path type
        if (ui->selected_path_type == 0) {
            ImGui::TextUnformatted("圆参数");
            ImGui::Checkbox("圆心使用当前末端位置", &ui->circle_center_use_current_tip);
            if (ui->circle_center_use_current_tip) {
                ImGui::TextDisabled("圆心将使用当前末端位置");
            } else {
                ImGui::InputFloat3("圆心 (m)", ui->circle_center, "%.3f");
            }
            ImGui::InputFloat("半径 (m)", &ui->circle_radius, 0.01f, 0.05f, "%.3f");
            ui->circle_radius = std::max(0.01f, ui->circle_radius);
            ImGui::InputFloat("周期 (s)", &ui->circle_period, 0.5f, 1.0f, "%.1f");
            ui->circle_period = std::max(0.1f, ui->circle_period);
            ImGui::InputInt("采样点数", &ui->circle_points);
            ui->circle_points = std::max(3, ui->circle_points);
        } else if (ui->selected_path_type == 1) {
            ImGui::TextUnformatted("方参数");
            ImGui::Checkbox("方形中心使用当前末端位置", &ui->square_center_use_current_tip);
            if (ui->square_center_use_current_tip) {
                ImGui::TextDisabled("方形中心将使用当前末端位置");
            } else {
                ImGui::InputFloat3("中心 (m)", ui->square_center, "%.3f");
            }
            ImGui::InputFloat("边长 (m)", &ui->square_side, 0.01f, 0.05f, "%.3f");
            ui->square_side = std::max(0.01f, ui->square_side);
            ImGui::InputFloat("圆角半径 (m)", &ui->square_corner_r, 0.0f, 0.01f, "%.3f");
            ui->square_corner_r = std::max(0.0f, std::min(ui->square_corner_r, ui->square_side * 0.4f));
            ImGui::InputFloat("周期 (s)", &ui->square_period, 0.5f, 1.0f, "%.1f");
            ui->square_period = std::max(0.1f, ui->square_period);
            ImGui::InputInt("采样点数", &ui->square_points);
            ui->square_points = std::max(4, ui->square_points);
        } else if (ui->selected_path_type == 2) {
            ImGui::TextUnformatted("头部往复参数");
            float head_pitch_ui = AngleUiFromDegStored(ui->head_pitch_amp_deg, use_deg);
            char head_pitch_label[40];
            std::snprintf(head_pitch_label, sizeof(head_pitch_label), "俯仰幅度 (%s)", AngleUnitLabel(use_deg));
            ImGui::InputFloat(head_pitch_label, &head_pitch_ui, use_deg ? 1.0f : 0.02f, use_deg ? 5.0f : 0.1f, AngleInputFormat(use_deg));
            head_pitch_ui = std::max(use_deg ? 1.0f : glm::radians(1.0f), std::min(head_pitch_ui, use_deg ? 45.0f : glm::radians(45.0f)));
            ui->head_pitch_amp_deg = AngleUiToDegStored(head_pitch_ui, use_deg);
            ImGui::InputFloat("周期 (s)", &ui->head_period, 0.5f, 1.0f, "%.1f");
            ui->head_period = std::max(0.1f, ui->head_period);
            ImGui::InputInt("采样点数", &ui->head_points);
            ui->head_points = std::max(2, ui->head_points);
        } else if (ui->selected_path_type == 3) {
            ImGui::TextUnformatted("直线参数");
            ImGui::Checkbox("使用相对位移", &ui->straight_use_relative);
            if (ui->straight_use_relative) {
                ImGui::InputFloat3("相对位移 Δxyz (m)", ui->straight_offset, "%.3f");
                ImGui::TextDisabled("目标点 = 当前末端位置 + Δxyz");
            } else {
                ImGui::InputFloat3("绝对目标位置 (m)", ui->straight_goal, "%.3f");
            }
            float rot_off_ui[3] = {AngleUiFromDegStored(ui->straight_rot_offset_deg[0], use_deg),
                                   AngleUiFromDegStored(ui->straight_rot_offset_deg[1], use_deg),
                                   AngleUiFromDegStored(ui->straight_rot_offset_deg[2], use_deg)};
            char rot_label[64];
            std::snprintf(rot_label, sizeof(rot_label), "相对旋转 Δrpy (%s)", AngleUnitLabel(use_deg));
            if (ImGui::InputFloat3(rot_label, rot_off_ui, AngleInputFormat(use_deg))) {
                ui->straight_rot_offset_deg[0] = AngleUiToDegStored(rot_off_ui[0], use_deg);
                ui->straight_rot_offset_deg[1] = AngleUiToDegStored(rot_off_ui[1], use_deg);
                ui->straight_rot_offset_deg[2] = AngleUiToDegStored(rot_off_ui[2], use_deg);
            }
            ImGui::InputFloat("最大速度 (m/s)", &ui->straight_max_vel, 0.01f, 0.05f, "%.2f");
            ui->straight_max_vel = std::max(0.01f, ui->straight_max_vel);
            ImGui::InputFloat("最大加速度 (m/s^2)", &ui->straight_max_acc, 0.01f, 0.05f, "%.2f");
            ui->straight_max_acc = std::max(0.01f, ui->straight_max_acc);
        } else if (ui->selected_path_type == 4) {
            ImGui::TextUnformatted("关节空间PTP参数");
            if (use_deg) {
                ImGui::TextDisabled("角速度/角加速度限幅仍为 rad/s（工程惯例）");
            }
            ImGui::InputFloat("最大速度 (rad/s)", &ui->ptp_max_vel, 0.1f, 0.5f, "%.2f");
            ui->ptp_max_vel = std::max(0.01f, ui->ptp_max_vel);
            ImGui::InputFloat("最大加速度 (rad/s^2)", &ui->ptp_max_acc, 0.1f, 0.5f, "%.2f");
            ui->ptp_max_acc = std::max(0.01f, ui->ptp_max_acc);
            ImGui::InputFloat("最大加加速度 (rad/s^3)", &ui->ptp_max_jerk, 0.5f, 2.0f, "%.1f");
            ui->ptp_max_jerk = std::max(0.1f, ui->ptp_max_jerk);
            ImGui::InputFloat("采样步长 (s)", &ui->ptp_delta_t, 0.001f, 0.01f, "%.3f");
            ui->ptp_delta_t            = std::max(0.001f, std::min(ui->ptp_delta_t, 0.1f));
            const char* ptp_profiles[] = {"TVP (梯形)", "DSVP (双S)"};
            ImGui::Combo("速度曲线", &ui->ptp_profile, ptp_profiles, IM_ARRAYSIZE(ptp_profiles));
            const char* ptp_sync_modes[] = {"先到先停 (hold)", "时间缩放 (time_scaling)"};
            ImGui::Combo("同步方式", &ui->ptp_sync_mode, ptp_sync_modes, IM_ARRAYSIZE(ptp_sync_modes));
            if (ui->ptp_sync_mode == 1) {
                ImGui::TextDisabled("时间缩放固定使用 DSVP，快关节全程协调运动");
            }

            // Per-joint goal offset inputs
            auto joints = scene->getJointInfos();
            if (!joints.empty()) {
                ImGui::Separator();
                ImGui::Text("各关节目标偏移 (%s)", AngleUnitLabel(use_deg));
                if (ui->ptp_goal_offsets.size() != joints.size()) {
                    ui->ptp_goal_offsets.resize(joints.size(), 0.0f);
                }
                for (size_t i = 0; i < joints.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    float offset_ui = AngleUiFromRad(ui->ptp_goal_offsets[i], use_deg);
                    char offset_fmt[16];
                    std::snprintf(offset_fmt, sizeof(offset_fmt), "%s %s", AngleInputFormat(use_deg), AngleUnitLabel(use_deg));
                    if (ImGui::DragFloat(joints[i].name.c_str(), &offset_ui, use_deg ? 0.5f : 0.01f, AngleDragMin(use_deg),
                                         AngleDragMax(use_deg), offset_fmt)) {
                        ui->ptp_goal_offsets[i] = AngleUiToRad(offset_ui, use_deg);
                    }
                    ImGui::PopID();
                }
            }
        }

        ImGui::Separator();

        // Action buttons
        const bool planning_pending_before_button = ui->planning_pending;
        if (planning_pending_before_button) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("生成路径并求解 IK") && !ui->planning_pending) {
            ui->last_status              = "开始规划...";
            ui->planning_pending         = true;
            ui->planning_defer_one_frame = true;
        }
        if (planning_pending_before_button) {
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.35f, 1.0f), "规划中...");
        }

        if (ui->planning_pending && !ui->planning_defer_one_frame) {
            const auto plan_begin_time = std::chrono::steady_clock::now();
            int timing_points_count    = 0;
            if (ui->selected_path_type == 4) {
                // Joint-space PTP: no IK needed, plan directly in joint space
                auto joints = scene->getJointInfos();
                if (joints.empty()) {
                    ui->last_status = "错误: 场景没有关节";
                } else {
                    if (ui->ptp_goal_offsets.size() != joints.size()) {
                        ui->ptp_goal_offsets.resize(joints.size(), 0.0f);
                    }

                    JointSpacePTPParams ptp_params;
                    ptp_params.joint_names.reserve(joints.size());
                    ptp_params.start_positions.reserve(joints.size());
                    ptp_params.goal_positions.reserve(joints.size());
                    for (size_t i = 0; i < joints.size(); ++i) {
                        ptp_params.joint_names.push_back(joints[i].name);
                        ptp_params.start_positions.push_back(joints[i].position);
                        ptp_params.goal_positions.push_back(joints[i].position + ui->ptp_goal_offsets[i]);
                    }
                    ptp_params.max_vel  = ui->ptp_max_vel;
                    ptp_params.max_acc  = ui->ptp_max_acc;
                    ptp_params.max_jerk = ui->ptp_max_jerk;
                    ptp_params.delta_t  = ui->ptp_delta_t;
                    ptp_params.profile   = (ui->ptp_profile == 0) ? "TVP" : "DSVP";
                    ptp_params.sync_mode = (ui->ptp_sync_mode == 0) ? "hold" : "time_scaling";

                    auto joint_traj = planJointSpacePTP(ptp_params);
                    if (!joint_traj.success) {
                        ui->last_status     = joint_traj.status;
                        timing_points_count = static_cast<int>(joints.size());
                    } else {
                        timing_points_count = static_cast<int>(joint_traj.times.size());
                        // Convert to playback keyframes
                        playbackState->keyframes.clear();
                        for (size_t i = 0; i < joint_traj.times.size(); ++i) {
                            PoseKeyframe kf;
                            kf.t = joint_traj.times[i];
                            for (size_t j = 0; j < joint_traj.joint_names.size(); ++j) {
                                kf.joints[joint_traj.joint_names[j]] = joint_traj.joint_positions[i][j];
                            }
                            playbackState->keyframes.push_back(std::move(kf));
                        }
                        playbackState->selected_keyframe_index = 0;
                        playbackState->play_time               = 0.0f;
                        ui->preview_waypoints.clear();  // No Cartesian preview for joint-space PTP
                        ui->last_status = joint_traj.status + ", 已加载到回放";
                    }
                }
            } else {
                // Cartesian path planning + IK (async)
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (!solver->fetchTipWorldPose(*scene, ui->selected_chain, &tip_pos, &tip_rpy)) {
                    ui->last_status = "错误: 无法获取当前末端位姿";
                } else {
                    glm::quat tip_quat = glm::quat(glm::vec3(tip_rpy.x, tip_rpy.y, tip_rpy.z));

                    std::unique_ptr<CartesianPathPlanner> planner;
                    if (ui->selected_path_type == 0) {
                        CirclePathParams params;
                        if (ui->circle_center_use_current_tip) {
                            params.center = tip_pos;
                        } else {
                            params.center = glm::vec3(ui->circle_center[0], ui->circle_center[1], ui->circle_center[2]);
                        }
                        params.radius     = ui->circle_radius;
                        params.period_sec = ui->circle_period;
                        params.num_points = ui->circle_points;
                        planner           = makeCirclePlanner(params);
                    } else if (ui->selected_path_type == 1) {
                        SquarePathParams params;
                        if (ui->square_center_use_current_tip) {
                            params.center = tip_pos;
                        } else {
                            params.center = glm::vec3(ui->square_center[0], ui->square_center[1], ui->square_center[2]);
                        }
                        params.side_length   = ui->square_side;
                        params.corner_radius = ui->square_corner_r;
                        params.period_sec    = ui->square_period;
                        params.num_points    = ui->square_points;
                        planner              = makeSquarePlanner(params);
                    } else if (ui->selected_path_type == 2) {
                        HeadBobParams params;
                        params.pitch_amplitude_deg = ui->head_pitch_amp_deg;
                        params.period_sec          = ui->head_period;
                        params.num_points          = ui->head_points;
                        planner                    = makeHeadBobPlanner(params);
                    } else {
                        StraightPathParams params;
                        params.start_pos = tip_pos;
                        if (ui->straight_use_relative) {
                            params.goal_pos = tip_pos + glm::vec3(ui->straight_offset[0], ui->straight_offset[1], ui->straight_offset[2]);
                        } else {
                            params.goal_pos = glm::vec3(ui->straight_goal[0], ui->straight_goal[1], ui->straight_goal[2]);
                        }
                        params.start_quat           = tip_quat;
                        const glm::vec3 rot_off_rad = glm::radians(
                            glm::vec3(ui->straight_rot_offset_deg[0], ui->straight_rot_offset_deg[1], ui->straight_rot_offset_deg[2]));
                        params.goal_quat = glm::normalize(tip_quat * glm::quat(rot_off_rad));
                        params.max_vel   = ui->straight_max_vel;
                        params.max_acc   = ui->straight_max_acc;
                        planner          = makeStraightPlanner(params);
                    }

                    auto cart_result = planner->plan(tip_pos, tip_quat);
                    if (!cart_result.success) {
                        ui->last_status = cart_result.status;
                        ui->preview_waypoints.clear();
                        timing_points_count = 0;
                    } else {
                        timing_points_count = static_cast<int>(cart_result.waypoints.size());
                        // Store Cartesian path for 3D preview (fallback when IK fails or preview remap is disabled)
                        if (ui->show_preview) {
                            ui->preview_waypoints = cart_result.waypoints;
                        }

                        // Solve IK synchronously
                        JointSpaceTrajectory joint_traj;
                        if (ui->ik_mode == 1) {
                            joint_traj = solveIkForCartesianPathFullBody(cart_result, scene, solver, ui->selected_chain);
                        } else {
                            joint_traj = solveIkForCartesianPath(cart_result, scene, solver, ui->selected_chain);
                        }

                        if (!joint_traj.success) {
                            ui->last_status = joint_traj.status;
                        } else {
                            timing_points_count = static_cast<int>(joint_traj.times.size());
                            if (ui->show_preview) {
                                // Build preview from solved joint trajectory so 3D preview matches actual motion.
                                // Downsample to keep UI responsive on long trajectories.
                                std::unordered_map<std::string, float> original_joint_pos;
                                const auto scene_joints_before = scene->getJointInfos();
                                for (const auto& j : scene_joints_before) {
                                    original_joint_pos[j.name] = j.position;
                                }

                                constexpr size_t kMaxPreviewSamples = 180;
                                const size_t total_samples          = std::min(joint_traj.times.size(), joint_traj.joint_positions.size());
                                const size_t sample_stride          = (total_samples <= kMaxPreviewSamples)
                                                                          ? 1
                                                                          : ((total_samples + kMaxPreviewSamples - 1) / kMaxPreviewSamples);

                                std::vector<CartesianWaypoint> solved_preview;
                                solved_preview.reserve((total_samples + sample_stride - 1) / sample_stride);
                                for (size_t i = 0; i < total_samples; i += sample_stride) {
                                    const auto& q = joint_traj.joint_positions[i];
                                    for (size_t j = 0; j < joint_traj.joint_names.size() && j < q.size(); ++j) {
                                        scene->setJointPositionByName(joint_traj.joint_names[j], q[j]);
                                    }
                                    scene->updateTransforms();
                                    glm::vec3 solved_tip_pos(0.0f);
                                    glm::vec3 solved_tip_rpy(0.0f);
                                    if (solver->fetchTipWorldPose(*scene, ui->selected_chain, &solved_tip_pos, &solved_tip_rpy)) {
                                        CartesianWaypoint wp;
                                        wp.time_sec    = joint_traj.times[i];
                                        wp.position    = solved_tip_pos;
                                        wp.orientation = glm::quat(glm::vec3(solved_tip_rpy.x, solved_tip_rpy.y, solved_tip_rpy.z));
                                        solved_preview.push_back(wp);
                                    }
                                }

                                if (total_samples > 0 && (total_samples - 1) % sample_stride != 0) {
                                    const size_t i = total_samples - 1;
                                    const auto& q  = joint_traj.joint_positions[i];
                                    for (size_t j = 0; j < joint_traj.joint_names.size() && j < q.size(); ++j) {
                                        scene->setJointPositionByName(joint_traj.joint_names[j], q[j]);
                                    }
                                    scene->updateTransforms();
                                    glm::vec3 solved_tip_pos(0.0f);
                                    glm::vec3 solved_tip_rpy(0.0f);
                                    if (solver->fetchTipWorldPose(*scene, ui->selected_chain, &solved_tip_pos, &solved_tip_rpy)) {
                                        CartesianWaypoint wp;
                                        wp.time_sec    = joint_traj.times[i];
                                        wp.position    = solved_tip_pos;
                                        wp.orientation = glm::quat(glm::vec3(solved_tip_rpy.x, solved_tip_rpy.y, solved_tip_rpy.z));
                                        solved_preview.push_back(wp);
                                    }
                                }

                                for (const auto& kv : original_joint_pos) {
                                    scene->setJointPositionByName(kv.first, kv.second);
                                }
                                scene->updateTransforms();

                                if (!solved_preview.empty()) {
                                    ui->preview_waypoints = std::move(solved_preview);
                                }
                            }

                            // Convert to playback keyframes
                            playbackState->keyframes.clear();
                            for (size_t i = 0; i < joint_traj.times.size(); ++i) {
                                PoseKeyframe kf;
                                kf.t = joint_traj.times[i];
                                for (size_t j = 0; j < joint_traj.joint_names.size(); ++j) {
                                    kf.joints[joint_traj.joint_names[j]] = joint_traj.joint_positions[i][j];
                                }
                                playbackState->keyframes.push_back(std::move(kf));
                            }
                            playbackState->selected_keyframe_index = 0;
                            playbackState->play_time               = 0.0f;
                            ui->last_status                        = joint_traj.status + ", 已加载到回放";
                        }
                    }
                }
            }

            const auto plan_end_time             = std::chrono::steady_clock::now();
            const double elapsed_ms              = std::chrono::duration<double, std::milli>(plan_end_time - plan_begin_time).count();
            const double avg_ms                  = timing_points_count > 0 ? (elapsed_ms / static_cast<double>(timing_points_count)) : 0.0;
            const std::string plan_result_status = ui->last_status;
            char timing_buf[160];
            std::snprintf(timing_buf, sizeof(timing_buf), " | 用时 %.1f ms, 平均 %.2f ms/点", elapsed_ms, avg_ms);
            ui->last_status      = std::string("开始规划 -> ") + plan_result_status + timing_buf;
            ui->planning_pending = false;
        } else if (ui->planning_pending && ui->planning_defer_one_frame) {
            ui->planning_defer_one_frame = false;
        }

        ImGui::SameLine();
        if (ImGui::Button("导出 CSV")) {
            if (playbackState->keyframes.empty()) {
                ui->last_status = "错误: 没有可导出的轨迹";
            } else {
                // Convert keyframes to JointSpaceTrajectory and export
                JointSpaceTrajectory traj;
                for (const auto& kf : playbackState->keyframes) {
                    traj.times.push_back(static_cast<float>(kf.t));
                }
                // ... export logic
                ui->last_status = "导出功能待完善";
            }
        }

        ImGui::Checkbox("3D 预览", &ui->show_preview);
        if (ui->planning_pending) {
            ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.35f, 1.0f), "状态: 开始规划...");
        }

        if (!ui->last_status.empty()) {
            ImVec4 color = ImVec4(0.6f, 0.95f, 0.6f, 1.0f);
            if (ui->last_status.find("错误") != std::string::npos || ui->last_status.find("失败") != std::string::npos) {
                color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
            }
            ImGui::TextColored(color, "%s", ui->last_status.c_str());
        }
    }

}  // namespace kinematic_viewer
