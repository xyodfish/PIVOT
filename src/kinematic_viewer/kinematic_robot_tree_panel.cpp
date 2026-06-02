#include "kinematic_viewer/kinematic_robot_tree_panel.h"

#include "imgui.h"

#include <algorithm>
#include <cctype>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kinematic_viewer {
    namespace {

        struct TreeNode {
            std::string name;
            std::vector<TreeNode> children;
        };

        void CollectRoots(const std::vector<teleop_viewer::RobotScene::LinkTfInfo>& tfs, std::vector<TreeNode>* out_roots) {
            if (out_roots == nullptr) {
                return;
            }
            out_roots->clear();
            std::unordered_map<std::string, size_t> index_by_name;
            std::vector<TreeNode> storage;
            storage.reserve(tfs.size());
            for (const auto& tf : tfs) {
                index_by_name[tf.name] = storage.size();
                storage.push_back(TreeNode{tf.name, {}});
            }

            std::unordered_set<std::string> has_parent;
            for (const auto& tf : tfs) {
                if (!tf.parent_name.empty() && index_by_name.find(tf.parent_name) != index_by_name.end()) {
                    storage[index_by_name[tf.parent_name]].children.push_back(storage[index_by_name[tf.name]]);
                    has_parent.insert(tf.name);
                }
            }

            for (const auto& tf : tfs) {
                if (has_parent.find(tf.name) == has_parent.end()) {
                    out_roots->push_back(storage[index_by_name[tf.name]]);
                }
            }
            if (out_roots->empty() && !storage.empty()) {
                out_roots->push_back(storage[0]);
            }
        }

        bool NameMatchesFilter(const std::string& name, const std::string& filter_lower) {
            if (filter_lower.empty()) {
                return true;
            }
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return lower.find(filter_lower) != std::string::npos;
        }

        bool TreeNodeMatchesFilter(const TreeNode& node, const std::string& filter_lower) {
            if (NameMatchesFilter(node.name, filter_lower)) {
                return true;
            }
            for (const auto& child : node.children) {
                if (TreeNodeMatchesFilter(child, filter_lower)) {
                    return true;
                }
            }
            return false;
        }

        void DrawTreeNode(const TreeNode& node, ViewerState* ui_state, teleop_viewer::RobotScene* scene, const std::string& filter_lower) {
            if (!TreeNodeMatchesFilter(node, filter_lower)) {
                return;
            }

            const bool selected      = (ui_state->selected_link == node.name);
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_DefaultOpen;
            if (selected) {
                flags |= ImGuiTreeNodeFlags_Selected;
            }
            if (node.children.empty()) {
                flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
            }

            const bool open = ImGui::TreeNodeEx(node.name.c_str(), flags);
            if (ImGui::IsItemClicked()) {
                ui_state->selected_link            = node.name;
                ui_state->trajectory_min_surface_m = -1.0f;
                ui_state->selected_joint           = -1;
                std::string joint_name;
                if (scene != nullptr && scene->getParentJointNameForLink(node.name, &joint_name)) {
                    const auto joints = scene->getJointInfos();
                    for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
                        if (joints[static_cast<size_t>(i)].name == joint_name) {
                            ui_state->selected_joint = i;
                            break;
                        }
                    }
                }
            }

            if (open && !node.children.empty()) {
                for (const auto& child : node.children) {
                    DrawTreeNode(child, ui_state, scene, filter_lower);
                }
                ImGui::TreePop();
            }
        }

    }  // namespace

    void RenderRobotTreePanel(ViewerState* ui_state, teleop_viewer::RobotScene* scene) {
        if (ui_state == nullptr || scene == nullptr) {
            return;
        }

        if (!ImGui::CollapsingHeader("结构树")) {
            return;
        }

        ImGui::SetNextItemWidth(-1.0f);
        ImGui::InputText("过滤", ui_state->tree_filter, sizeof(ui_state->tree_filter));
        std::string filter = ui_state->tree_filter;
        std::transform(filter.begin(), filter.end(), filter.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        const auto tfs = scene->getLinkTfInfos();
        std::vector<TreeNode> roots;
        CollectRoots(tfs, &roots);

        ImGui::BeginChild("robot_link_tree", ImVec2(0.0f, 120.0f), true);
        if (roots.empty()) {
            ImGui::TextDisabled("无 link 数据");
        } else {
            for (const auto& root : roots) {
                DrawTreeNode(root, ui_state, scene, filter);
            }
        }
        ImGui::EndChild();

        if (ImGui::TreeNode("关节列表")) {
            const auto joints = scene->getJointInfos();
            for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
                const auto& j   = joints[static_cast<size_t>(i)];
                std::string key = j.name;
                std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (!filter.empty() && key.find(filter) == std::string::npos) {
                    continue;
                }
                teleop_viewer::RobotScene::JointDetailInfo detail;
                const bool has_detail = scene->getJointDetail(j.name, &detail);
                const bool selected   = (ui_state->selected_joint == i);
                if (ImGui::Selectable(j.name.c_str(), selected)) {
                    ui_state->selected_joint = i;
                    if (has_detail) {
                        ui_state->selected_link            = detail.child_link;
                        ui_state->trajectory_min_surface_m = -1.0f;
                    }
                }
            }
            ImGui::TreePop();
        }
    }

}  // namespace kinematic_viewer
