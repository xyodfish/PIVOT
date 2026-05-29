#include "kinematic_viewer/kinematic_teach_panel.h"

#include "kinematic_viewer/kinematic_path_planner.h"
#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_string_utils.h"
#include "kinematic_viewer/kinematic_teach.h"

#include "imgui.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <unordered_set>

namespace kinematic_viewer {
    namespace kinematic_teach_panel_internal {

        using kinematic_viewer::NormalizePath;

        bool IsTeachFileExt(const std::filesystem::path& path) {
            const std::string ext = kinematic_viewer::LowerFileExtension(path.string());
            return ext == ".yaml" || ext == ".yml";
        }

        bool ProgramPathInList(const TeachProgramState& teach, const std::string& path) {
            const std::string normalized = NormalizePath(path);
            for (const auto& entry : teach.program_files) {
                if (NormalizePath(entry.path) == normalized) {
                    return true;
                }
            }
            return false;
        }

        bool LoadProgramListEntry(TeachProgramState* teach, int index) {
            if (teach == nullptr || index < 0 || index >= static_cast<int>(teach->program_files.size())) {
                return false;
            }
            const std::string& path = teach->program_files[static_cast<size_t>(index)].path;
            std::string error;
            if (!LoadTeachProgramFromYaml(path, teach, &error)) {
                teach->program_files[static_cast<size_t>(index)].status = "加载失败: " + error;
                teach->program_files[static_cast<size_t>(index)].loaded = false;
                teach->io_status                                        = teach->program_files[static_cast<size_t>(index)].status;
                return false;
            }
            teach->program_files[static_cast<size_t>(index)].status = "加载成功";
            teach->program_files[static_cast<size_t>(index)].loaded = true;
            teach->selected_program_index                             = index;
            std::snprintf(teach->program_file_path, sizeof(teach->program_file_path), "%s", path.c_str());
            teach->io_status = "已加载: " + teach->program_name;
            return true;
        }

        void RenderTeachFileBrowserPopup(TeachProgramState* teach) {
            if (teach == nullptr) {
                return;
            }
            if (!ImGui::BeginPopupModal("teach_file_browser_popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                return;
            }
            ImGui::InputText("目录", teach->program_browser_dir, sizeof(teach->program_browser_dir));
            const std::string dir = NormalizePath(teach->program_browser_dir);
            std::error_code ec;
            if (ImGui::BeginChild("teach_browser_list", ImVec2(520.0f, 320.0f), true)) {
                if (std::filesystem::is_directory(dir, ec)) {
                    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
                        if (entry.is_directory(ec)) {
                            continue;
                        }
                        if (!IsTeachFileExt(entry.path())) {
                            continue;
                        }
                        const std::string path = NormalizePath(entry.path().string());
                        if (ImGui::Selectable(entry.path().filename().string().c_str())) {
                            if (!ProgramPathInList(*teach, path)) {
                                TeachFileEntry file_entry;
                                file_entry.path   = path;
                                file_entry.status = "未加载";
                                teach->program_files.push_back(std::move(file_entry));
                            }
                            std::snprintf(teach->program_file_path, sizeof(teach->program_file_path), "%s", path.c_str());
                            teach->selected_program_index = static_cast<int>(teach->program_files.size()) - 1;
                            for (int i = 0; i < static_cast<int>(teach->program_files.size()); ++i) {
                                if (NormalizePath(teach->program_files[static_cast<size_t>(i)].path) == path) {
                                    teach->selected_program_index = i;
                                    LoadProgramListEntry(teach, i);
                                    break;
                                }
                            }
                            ImGui::CloseCurrentPopup();
                        }
                    }
                } else {
                    ImGui::TextDisabled("目录无效");
                }
            }
            ImGui::EndChild();
            if (ImGui::Button("关闭")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        void OpenTeachFileBrowser(TeachProgramState* teach) {
            if (teach == nullptr) {
                return;
            }
            ImGui::OpenPopup("teach_file_browser_popup");
            RenderTeachFileBrowserPopup(teach);
        }

    }  // namespace kinematic_teach_panel_internal

    void RenderTeachPanel(TeachProgramState* teach, DebugPlaybackState* playback_state, teleop_viewer::RobotScene* scene,
                          teleop_viewer::IkSolver* solver, const std::vector<teleop_viewer::IkChainStatus>& chains,
                          const std::vector<teleop_viewer::RobotScene::JointInfo>& joints) {
        if (teach == nullptr || scene == nullptr) {
            return;
        }

        ImGui::Separator();
        ImGui::TextUnformatted("示教器");
        ImGui::TextDisabled("拖动关节或 IK 末端到位后，记录示教点；可命名、调序并保存 YAML。");

        if (ImGui::CollapsingHeader("示教程序", ImGuiTreeNodeFlags_DefaultOpen)) {
            char program_name_buf[128] = {};
            std::snprintf(program_name_buf, sizeof(program_name_buf), "%s", teach->program_name.c_str());
            if (ImGui::InputText("程序名", program_name_buf, sizeof(program_name_buf))) {
                teach->program_name = program_name_buf;
            }

            if (!teach->program_files.empty() && ImGui::BeginListBox("##teach_program_list", ImVec2(-1, 100))) {
                for (int i = 0; i < static_cast<int>(teach->program_files.size()); ++i) {
                    auto& entry = teach->program_files[static_cast<size_t>(i)];
                    std::string label = std::filesystem::path(entry.path).filename().string();
                    if (!entry.status.empty() && entry.status != "未加载") {
                        label += " (" + entry.status + ")";
                    }
                    const bool selected = (teach->selected_program_index == i);
                    if (ImGui::Selectable(label.c_str(), selected)) {
                        teach->selected_program_index = i;
                        kinematic_teach_panel_internal::LoadProgramListEntry(teach, i);
                    }
                }
                ImGui::EndListBox();
            }

            if (ImGui::Button("+ 添加文件")) {
                kinematic_teach_panel_internal::OpenTeachFileBrowser(teach);
            }
            ImGui::SameLine();
            if (ImGui::Button("- 删除选中") && teach->selected_program_index >= 0) {
                teach->program_files.erase(teach->program_files.begin() + teach->selected_program_index);
                teach->selected_program_index = std::min(teach->selected_program_index, static_cast<int>(teach->program_files.size()) - 1);
            }

            ImGui::InputText("文件路径", teach->program_file_path, sizeof(teach->program_file_path));
            ImGui::SameLine();
            if (ImGui::Button("浏览示教文件")) {
                kinematic_teach_panel_internal::OpenTeachFileBrowser(teach);
            }
            kinematic_teach_panel_internal::RenderTeachFileBrowserPopup(teach);

            if (ImGui::Button("加载")) {
                std::string error;
                if (LoadTeachProgramFromYaml(teach->program_file_path, teach, &error)) {
                    teach->io_status = "加载成功: " + teach->program_name;
                } else {
                    teach->io_status = "加载失败: " + error;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("保存")) {
                std::string error;
                if (SaveTeachProgramToYaml(teach->program_file_path, *teach, &error)) {
                    teach->io_status = "保存成功";
                    if (!kinematic_teach_panel_internal::ProgramPathInList(*teach, teach->program_file_path)) {
                        TeachFileEntry entry;
                        entry.path   = NormalizePath(teach->program_file_path);
                        entry.status = "加载成功";
                        entry.loaded = true;
                        teach->program_files.push_back(std::move(entry));
                        teach->selected_program_index = static_cast<int>(teach->program_files.size()) - 1;
                    }
                } else {
                    teach->io_status = "保存失败: " + error;
                }
            }

            if (!teach->io_status.empty()) {
                ImVec4 color(0.66f, 0.72f, 0.80f, 1.0f);
                if (teach->io_status.find("失败") != std::string::npos) {
                    color = ImVec4(0.95f, 0.42f, 0.42f, 1.0f);
                } else if (teach->io_status.find("成功") != std::string::npos) {
                    color = ImVec4(0.40f, 0.84f, 0.52f, 1.0f);
                }
                ImGui::TextColored(color, "%s", teach->io_status.c_str());
            }
        }

        if (ImGui::CollapsingHeader("示教点", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!chains.empty()) {
                const char* preview = chains[static_cast<size_t>(std::clamp(teach->record_chain_index, 0, static_cast<int>(chains.size()) - 1))]
                                          .config.label.c_str();
                if (ImGui::BeginCombo("记录末端链", preview)) {
                    for (int i = 0; i < static_cast<int>(chains.size()); ++i) {
                        const bool selected = (teach->record_chain_index == i);
                        if (ImGui::Selectable(chains[static_cast<size_t>(i)].config.label.c_str(), selected)) {
                            teach->record_chain_index = i;
                        }
                    }
                    ImGui::EndCombo();
                }
            }

            if (ImGui::Button("记录当前点")) {
                std::string tip_link;
                if (!chains.empty()) {
                    const int ci = std::clamp(teach->record_chain_index, 0, static_cast<int>(chains.size()) - 1);
                    tip_link     = chains[static_cast<size_t>(ci)].config.tip_link;
                }
                CaptureTeachPointFromScene(teach, joints, *scene, tip_link);
                teach->io_status = "已记录: " + teach->points.back().name;
            }
            ImGui::SameLine();
            if (ImGui::Button("删除选中")) {
                RemoveSelectedTeachPoint(teach);
            }
            ImGui::SameLine();
            if (ImGui::Button("清空全部")) {
                teach->points.clear();
                teach->selected_point_index = -1;
            }

            const bool has_selection = teach->selected_point_index >= 0 &&
                                       teach->selected_point_index < static_cast<int>(teach->points.size());
            if (has_selection) {
                if (ImGui::Button("运动到该点")) {
                    ApplyTeachPointToScene(teach->points[static_cast<size_t>(teach->selected_point_index)], scene);
                    scene->updateTransforms();
                }
                ImGui::SameLine();
                if (ImGui::Button("上移") && teach->selected_point_index > 0) {
                    MoveTeachPoint(teach, teach->selected_point_index, teach->selected_point_index - 1);
                }
                ImGui::SameLine();
                if (ImGui::Button("下移") && teach->selected_point_index + 1 < static_cast<int>(teach->points.size())) {
                    MoveTeachPoint(teach, teach->selected_point_index, teach->selected_point_index + 1);
                }
            }

            if (has_selection) {
                auto& point = teach->points[static_cast<size_t>(teach->selected_point_index)];
                char name_buf[128] = {};
                std::snprintf(name_buf, sizeof(name_buf), "%s", point.name.c_str());
                if (ImGui::InputText("点名", name_buf, sizeof(name_buf))) {
                    RenameTeachPoint(teach, teach->selected_point_index, name_buf);
                }
            }

            if (teach->points.empty()) {
                ImGui::TextDisabled("暂无示教点。");
            } else if (ImGui::BeginTable("teach_point_table", 4,
                                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY,
                                          ImVec2(0.0f, 200.0f))) {
                ImGui::TableSetupColumn("#");
                ImGui::TableSetupColumn("名称");
                ImGui::TableSetupColumn("关节");
                ImGui::TableSetupColumn("末端");
                ImGui::TableHeadersRow();
                for (int i = 0; i < static_cast<int>(teach->points.size()); ++i) {
                    const auto& point = teach->points[static_cast<size_t>(i)];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    const bool row_selected = (teach->selected_point_index == i);
                    if (ImGui::Selectable(std::to_string(i + 1).c_str(), row_selected, ImGuiSelectableFlags_SpanAllColumns)) {
                        teach->selected_point_index = i;
                        ApplyTeachPointToScene(point, scene);
                        scene->updateTransforms();
                    }
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(point.name.c_str());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%d", static_cast<int>(point.joints.size()));
                    ImGui::TableSetColumnIndex(3);
                    ImGui::TextUnformatted(point.has_ee_pose ? "有" : "-");
                }
                ImGui::EndTable();
            }
        }

        if (ImGui::CollapsingHeader("轨迹生成 (vp)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("moveJ 速度", &teach->movej_max_vel, 0.05f, 3.0f, "%.2f");
            ImGui::SliderFloat("moveJ 加速度", &teach->movej_max_acc, 0.05f, 10.0f, "%.2f");
            ImGui::SliderFloat("moveL 速度", &teach->movel_max_vel, 0.02f, 1.0f, "%.2f");

            if (ImGui::Button("生成 moveJ → 回放")) {
                std::string status;
                const JointSpaceTrajectory traj = BuildTeachMoveJTrajectory(*teach, &status);
                if (playback_state != nullptr && traj.success) {
                    std::string load_error;
                    if (LoadJointTrajectoryIntoPlayback(traj, playback_state, &load_error)) {
                        teach->io_status = status + "；已载入回放页";
                    } else {
                        teach->io_status = status + "；载入回放失败: " + load_error;
                    }
                } else {
                    teach->io_status = status;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("生成 moveL → 回放") && playback_state != nullptr) {
                std::string status;
                const CartesianPathResult cart = BuildTeachMoveLTrajectory(*teach, &status);
                if (!cart.success) {
                    teach->io_status = status;
                } else if (!chains.empty()) {
                    const int chain_index = std::clamp(teach->record_chain_index, 0, static_cast<int>(chains.size()) - 1);
                    JointSpaceTrajectory joint_traj = solveIkForCartesianPathFullBody(cart, scene, solver, chain_index);
                    if (!joint_traj.success) {
                        joint_traj = solveIkForCartesianPath(cart, scene, solver, chain_index);
                    }
                    std::string load_error;
                    if (joint_traj.success && LoadJointTrajectoryIntoPlayback(joint_traj, playback_state, &load_error)) {
                        teach->io_status = status + "；IK 后已载入回放";
                    } else {
                        teach->io_status = joint_traj.success ? ("载入回放失败: " + load_error) : joint_traj.status;
                    }
                } else {
                    teach->io_status = "无 IK 链，无法 moveL";
                }
            }

            ImGui::SameLine();
            if (ImGui::Button("示教点直连回放") && playback_state != nullptr) {
                std::string error;
                if (LoadTeachPointsIntoPlayback(*teach, 1.0f, playback_state, &error)) {
                    teach->io_status = "已按 1s/点 载入回放（无 vp 规划）";
                } else {
                    teach->io_status = error;
                }
            }
        }
    }

}  // namespace kinematic_viewer
