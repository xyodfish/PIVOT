#include "kinematic_viewer/kinematic_sidebar_panels.h"

#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_path_planner.h"
#include "kinematic_viewer/kinematic_string_utils.h"

#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"

#include "imgui.h"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace kinematic_viewer {
    namespace kinematic_sidebar_panels_internal {

        using kinematic_viewer::NormalizePath;

        bool IsTrajectoryFileExt(const std::filesystem::path& path) {
            const std::string ext = LowerFileExtension(path.string());
            return ext == ".csv";
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
            for (char& ch : normalized) {
                if (ch == ',' || ch == '[' || ch == ']' || ch == '\t') {
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

        bool ApplyJointGroupTextCommand(const std::string& commandLine, const ViewerState& uiState,
                                        teleop_viewer::RobotScene* scene, std::string* errorMessage) {
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

            size_t firstSpace = line.find_first_of(" \t");
            if (firstSpace == std::string::npos) {
                if (errorMessage != nullptr) {
                    *errorMessage = "格式错误，应为: group v1,v2,...";
                }
                return false;
            }
            const std::string groupName = line.substr(0, firstSpace);
            const std::string valuesRaw = TrimCopy(line.substr(firstSpace + 1));
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

            std::vector<float> values;
            std::string parseError;
            if (!ParseJointValuesRad(valuesRaw, &values, &parseError)) {
                if (errorMessage != nullptr) {
                    *errorMessage = parseError;
                }
                return false;
            }
            if (values.size() != group->joint_names.size()) {
                if (errorMessage != nullptr) {
                    std::stringstream ss;
                    ss << "group[" << group->name << "] 维度不匹配，期望 " << group->joint_names.size() << "，实际 " << values.size();
                    *errorMessage = ss.str();
                }
                return false;
            }

            for (size_t i = 0; i < group->joint_names.size(); ++i) {
                if (!scene->setJointPositionByName(group->joint_names[i], values[i])) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "场景不存在关节: " + group->joint_names[i];
                    }
                    return false;
                }
            }
            return true;
        }

        bool ApplyJointGroupValues(const ViewerState::JointInputGroup& group, const std::vector<float>& values,
                                   teleop_viewer::RobotScene* scene, std::string* errorMessage) {
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
            static bool s_force_refresh = false;
            static bool s_has_scan_error = false;

            if (ImGui::Button("浏览本地文件")) {
                const std::string defaultDir = NormalizePath(std::filesystem::current_path().string());
                std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir), "%s",
                              defaultDir.c_str());
                s_force_refresh = true;
                ImGui::OpenPopup("trajectory_file_browser_popup");
            }

            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            const float popup_width = std::clamp(viewport->Size.x * 0.72f, 900.0f, 1500.0f);
            const float popup_height = std::clamp(viewport->Size.y * 0.70f, 460.0f, 920.0f);
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
                if (ImGui::BeginChild("trajectory_file_browser_list", ImVec2(0, -38), true)) {
                    ImGuiTableFlags tableFlags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
                    if (ImGui::BeginTable("file_browser_table", 3, tableFlags)) {
                        ImGui::TableSetupColumn("名称", ImGuiTableColumnFlags_WidthStretch, 3.5f);
                        ImGui::TableSetupColumn("大小", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                        ImGui::TableSetupColumn("修改时间", ImGuiTableColumnFlags_WidthStretch, 1.5f);
                        ImGui::TableHeadersRow();

                        // Clickable headers for sorting
                        for (int col = 0; col < 3; ++col) {
                            ImGui::TableSetColumnIndex(col);
                            const char* labels[3]           = {"名称", "大小", "修改时间"};
                            const FileBrowserSortBy asc[3]  = {FileBrowserSortBy::NameAsc, FileBrowserSortBy::SizeAsc,
                                                               FileBrowserSortBy::TimeAsc};
                            const FileBrowserSortBy desc[3] = {FileBrowserSortBy::NameDesc, FileBrowserSortBy::SizeDesc,
                                                               FileBrowserSortBy::TimeDesc};
                            bool is_asc                     = (s_sort_by == asc[col]);
                            bool is_desc                    = (s_sort_by == desc[col]);
                            const char* arrow               = is_asc ? "▲" : (is_desc ? "▼" : "");
                            char buf[32];
                            std::snprintf(buf, sizeof(buf), "%s %s", labels[col], arrow);
                            ImGui::TableHeader(buf);
                            if (ImGui::IsItemClicked()) {
                                if (is_asc) {
                                    s_sort_by = desc[col];
                                } else {
                                    s_sort_by = asc[col];
                                }
                                s_force_refresh = true;
                            }
                        }

                        for (const auto& entry : s_cached_entries) {
                            ImGui::TableNextRow();

                            ImGui::TableSetColumnIndex(0);
                            std::string displayName = entry.isDir ? ("[DIR] " + entry.name) : entry.name;
                            ImGui::PushStyleColor(ImGuiCol_Text,
                                                  entry.isDir ? ImVec4(0.45f, 0.75f, 1.0f, 1.0f) : ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                            if (ImGui::Selectable(displayName.c_str(), false, ImGuiSelectableFlags_SpanAllColumns)) {
                                if (entry.isDir) {
                                    std::snprintf(playbackState->trajectory_browser_dir, sizeof(playbackState->trajectory_browser_dir),
                                                  "%s", entry.path.string().c_str());
                                } else {
                                    // Add to trajectory file list instead of replacing single path
                                    TrajectoryFileEntry newEntry;
                                    newEntry.path   = entry.path.string();
                                    newEntry.status = "未加载";
                                    newEntry.loaded = false;
                                    playbackState->trajectory_files.push_back(std::move(newEntry));
                                    playbackState->selected_trajectory_index = static_cast<int>(playbackState->trajectory_files.size()) - 1;
                                    std::snprintf(playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path), "%s",
                                                  entry.path.string().c_str());
                                    playbackState->pending_trajectory_load_index      = playbackState->selected_trajectory_index;
                                    playbackState->pending_trajectory_play_after_load = false;
                                    ImGui::CloseCurrentPopup();
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
            }

            if (ImGui::Button("关闭")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

    }  // namespace kinematic_sidebar_panels_internal

    void RenderScenePanel(ViewerState* uiState) {
        if (uiState == nullptr) {
            return;
        }
        if (!ImGui::CollapsingHeader("场景显示", ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        SidebarCheckboxRow4("网格", &uiState->show_visual_meshes, "线框", &uiState->show_wireframe, "碰撞体",
                              &uiState->show_collision_bodies, "质心", &uiState->show_com);
        SidebarCheckboxRow4("关节轴", &uiState->show_axes, "仅旋转轴", &uiState->show_revolute_only, "非旋转轴",
                              &uiState->show_non_revolute, "世界轴", &uiState->show_world_axes);
        ImGui::Checkbox("固定底座", &uiState->lock_base);
        if (ImGui::TreeNode("轴与网格")) {
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
        RenderUserObstaclePanel(&uiState->user_obstacles);
    }

    void RenderJointPanel(ViewerState* uiState, teleop_viewer::RobotScene* scene,
                          const std::vector<teleop_viewer::RobotScene::JointInfo>& joints) {
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

        ImGui::Text("共 %d | 旋转 %d | 越界 %d", static_cast<int>(joints.size()), revoluteCount, clampedCount);
        if (!minName.empty()) {
            ImVec4 c = (minMarginDeg < 3.0f) ? ImVec4(1.0f, 0.25f, 0.25f, 1.0f)
                                             : ((minMarginDeg < 8.0f) ? ImVec4(1.0f, 0.75f, 0.25f, 1.0f) : ImVec4(0.6f, 0.9f, 0.6f, 1.0f));
            ImGui::TextColored(c, "最小限位裕量: %.2f deg (%s)", minMarginDeg, minName.c_str());
        }

        if (uiState->joint_input_groups.empty()) {
            ImGui::TextDisabled("未配置关节分组（见 config.initial_pose）");
        } else if (ImGui::CollapsingHeader("分组批量输入")) {
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
            SidebarInputTextMultiline("角度(rad)", uiState->joint_group_values_input, sizeof(uiState->joint_group_values_input), 52.0f);
            if (ImGui::SmallButton("应用分组")) {
                std::vector<float> values;
                std::string error;
                if (!kinematic_sidebar_panels_internal::ParseJointValuesRad(uiState->joint_group_values_input, &values, &error)) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                } else if (!kinematic_sidebar_panels_internal::ApplyJointGroupValues(selectedGroup, values, scene, &error)) {
                    uiState->joint_group_input_last_ok = false;
                    uiState->joint_group_input_status  = "应用失败: " + error;
                } else {
                    uiState->joint_group_input_last_ok = true;
                    uiState->joint_group_input_status  = "应用成功: " + selectedGroup.name;
                }
            }

            if (ImGui::TreeNode("多组命令")) {
                SidebarInputTextMultiline("命令", uiState->joint_group_input, sizeof(uiState->joint_group_input), 56.0f);
                if (ImGui::SmallButton("应用全部")) {
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
        if (ImGui::BeginTable("joint_table", 4,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingFixedFit,
                              SidebarListSize(uiState->joint_section_height))) {
            ImGui::TableSetupColumn("关节", ImGuiTableColumnFlags_WidthFixed, 0.0f, 96.0f);
            ImGui::TableSetupColumn("deg", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("值", ImGuiTableColumnFlags_WidthFixed, 0.0f, 52.0f);
            ImGui::TableSetupColumn("限位", ImGuiTableColumnFlags_WidthFixed, 0.0f, 58.0f);
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
                ImGui::TableSetColumnIndex(0);
                const bool joint_row_selected = (!parent_joint_for_selected_link.empty() && j.name == parent_joint_for_selected_link);
                if (ImGui::Selectable(j.name.c_str(), joint_row_selected, ImGuiSelectableFlags_SpanAllColumns)) {
                    teleop_viewer::RobotScene::JointDetailInfo detail;
                    if (scene->getJointDetail(j.name, &detail)) {
                        uiState->selected_link            = detail.child_link;
                        uiState->trajectory_min_surface_m = -1.0f;
                    }
                }
                ImGui::TableSetColumnIndex(1);
                float radValue = j.position;
                if (j.revolute) {
                    PushSidebarFullWidth();
                    const std::string slider_deg_id = "##slider_deg_" + j.name;
                    if (ImGui::SliderAngle(slider_deg_id.c_str(), &radValue, glm::degrees(j.min_angle), glm::degrees(j.max_angle))) {
                        scene->setJointPositionByName(j.name, radValue);
                    }
                    PopSidebarWidth();
                } else {
                    ImGui::TextDisabled("-");
                }

                ImGui::TableSetColumnIndex(2);
                if (j.revolute) {
                    float input_deg         = glm::degrees(j.position);
                    const std::string input_deg_id = "##input_deg_" + j.name;
                    if (ImGui::InputFloat(input_deg_id.c_str(), &input_deg, 0.0f, 0.0f, "%.1f")) {
                        scene->setJointPositionByName(j.name, glm::radians(input_deg));
                    }
                } else {
                    float input_m = j.position;
                    if (ImGui::InputFloat(("##m_" + j.name).c_str(), &input_m, 0.0f, 0.0f, "%.3f")) {
                        scene->setJointPositionByName(j.name, input_m);
                    }
                }

                ImGui::TableSetColumnIndex(3);
                if (j.revolute) {
                    ImGui::Text("%.0f~%.0f", glm::degrees(j.min_angle), glm::degrees(j.max_angle));
                } else {
                    ImGui::TextDisabled("-");
                }
            }
            ImGui::EndTable();
        }
    }

    void RenderPlaybackPanel(DebugPlaybackState* playbackState, TrajectoryPlayer* playbackPlayer, PlaybackStateMachine* playback_sm,
                             teleop_viewer::RobotScene* scene,
                             const std::vector<teleop_viewer::RobotScene::JointInfo>& joints) {
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
                        const bool clicked =
                            ImGui::Selectable(label.c_str(), isSelected, ImGuiSelectableFlags_AllowDoubleClick);
                        const bool doubleClicked =
                            ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
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
        SidebarCheckboxRow2("忽略同Link", &collisionState->ignore_same_link, "忽略父子", &collisionState->ignore_parent_child);
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

    void RenderTfPanel(ViewerState* uiState, const std::vector<teleop_viewer::RobotScene::LinkTfInfo>& tfs) {
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
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingFixedFit,
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
                ImGui::Text("%.2f,%.2f,%.2f | %.0f,%.0f,%.0f", tf.world_position.x, tf.world_position.y, tf.world_position.z,
                            glm::degrees(tf.world_rpy.x), glm::degrees(tf.world_rpy.y), glm::degrees(tf.world_rpy.z));
            }
            ImGui::EndTable();
        }
    }

    // ------------------------------------------------------------------
    // Path Planner Panel
    // ------------------------------------------------------------------
    void RenderPathPlannerPanel(PathPlannerUiState* ui, DebugPlaybackState* playbackState, teleop_viewer::RobotScene* scene,
                                teleop_viewer::IkSolver* solver,
                                const std::vector<teleop_viewer::IkChainStatus>& chains) {
        if (ui == nullptr || playbackState == nullptr || scene == nullptr || solver == nullptr) {
            return;
        }

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
            ImGui::InputFloat3("圆心 (m)", ui->circle_center, "%.3f");
            ImGui::InputFloat("半径 (m)", &ui->circle_radius, 0.01f, 0.05f, "%.3f");
            ui->circle_radius = std::max(0.01f, ui->circle_radius);
            ImGui::InputFloat("周期 (s)", &ui->circle_period, 0.5f, 1.0f, "%.1f");
            ui->circle_period = std::max(0.1f, ui->circle_period);
            ImGui::InputInt("采样点数", &ui->circle_points);
            ui->circle_points = std::max(3, ui->circle_points);
        } else if (ui->selected_path_type == 1) {
            ImGui::TextUnformatted("方参数");
            ImGui::InputFloat3("中心 (m)", ui->square_center, "%.3f");
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
            ImGui::InputFloat("俯仰幅度 (deg)", &ui->head_pitch_amp_deg, 1.0f, 5.0f, "%.1f");
            ui->head_pitch_amp_deg = std::max(1.0f, std::min(ui->head_pitch_amp_deg, 45.0f));
            ImGui::InputFloat("周期 (s)", &ui->head_period, 0.5f, 1.0f, "%.1f");
            ui->head_period = std::max(0.1f, ui->head_period);
            ImGui::InputInt("采样点数", &ui->head_points);
            ui->head_points = std::max(2, ui->head_points);
        } else if (ui->selected_path_type == 3) {
            ImGui::TextUnformatted("直线参数");
            ImGui::InputFloat3("目标位置 (m)", ui->straight_goal, "%.3f");
            ImGui::InputFloat("最大速度 (m/s)", &ui->straight_max_vel, 0.01f, 0.05f, "%.2f");
            ui->straight_max_vel = std::max(0.01f, ui->straight_max_vel);
            ImGui::InputFloat("最大加速度 (m/s^2)", &ui->straight_max_acc, 0.01f, 0.05f, "%.2f");
            ui->straight_max_acc = std::max(0.01f, ui->straight_max_acc);
        } else if (ui->selected_path_type == 4) {
            ImGui::TextUnformatted("关节空间PTP参数");
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

            // Per-joint goal offset inputs
            auto joints = scene->getJointInfos();
            if (!joints.empty()) {
                ImGui::Separator();
                ImGui::TextUnformatted("各关节目标偏移 (rad)");
                if (ui->ptp_goal_offsets.size() != joints.size()) {
                    ui->ptp_goal_offsets.resize(joints.size(), 0.0f);
                }
                for (size_t i = 0; i < joints.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    float offset_deg = glm::degrees(ui->ptp_goal_offsets[i]);
                    if (ImGui::DragFloat(joints[i].name.c_str(), &offset_deg, 0.5f, -180.0f, 180.0f, "%.1f deg")) {
                        ui->ptp_goal_offsets[i] = glm::radians(offset_deg);
                    }
                    ImGui::PopID();
                }
            }
        }

        ImGui::Separator();

        // Action buttons
        if (ImGui::Button("生成路径并求解 IK")) {
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
                    ptp_params.profile  = (ui->ptp_profile == 0) ? "TVP" : "DSVP";

                    auto joint_traj = planJointSpacePTP(ptp_params);
                    if (!joint_traj.success) {
                        ui->last_status = joint_traj.status;
                    } else {
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
                        params.center     = glm::vec3(ui->circle_center[0], ui->circle_center[1], ui->circle_center[2]);
                        params.radius     = ui->circle_radius;
                        params.period_sec = ui->circle_period;
                        params.num_points = ui->circle_points;
                        planner           = makeCirclePlanner(params);
                    } else if (ui->selected_path_type == 1) {
                        SquarePathParams params;
                        params.center        = glm::vec3(ui->square_center[0], ui->square_center[1], ui->square_center[2]);
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
                        params.start_pos  = tip_pos;
                        params.goal_pos   = glm::vec3(ui->straight_goal[0], ui->straight_goal[1], ui->straight_goal[2]);
                        params.start_quat = tip_quat;
                        params.max_vel    = ui->straight_max_vel;
                        params.max_acc    = ui->straight_max_acc;
                        planner           = makeStraightPlanner(params);
                    }

                    auto cart_result = planner->plan(tip_pos, tip_quat);
                    if (!cart_result.success) {
                        ui->last_status = cart_result.status;
                        ui->preview_waypoints.clear();
                    } else {
                        // Store Cartesian path for 3D preview
                        ui->preview_waypoints = cart_result.waypoints;

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

        if (!ui->last_status.empty()) {
            ImVec4 color = ImVec4(0.6f, 0.95f, 0.6f, 1.0f);
            if (ui->last_status.find("错误") != std::string::npos || ui->last_status.find("失败") != std::string::npos) {
                color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
            }
            ImGui::TextColored(color, "%s", ui->last_status.c_str());
        }
    }

}  // namespace kinematic_viewer
