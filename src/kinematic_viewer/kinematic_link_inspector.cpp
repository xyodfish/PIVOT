#include "kinematic_viewer/kinematic_link_inspector.h"

#include "kinematic_viewer/kinematic_angle_units.h"

#include "imgui.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace kinematic_viewer {
    namespace {

        constexpr double kKinematicsRefreshSec = 0.25;

    }  // namespace

    bool GetLinkWorldFocusPoint(const teleop_viewer::RobotScene& scene, const std::string& link_name, glm::vec3* out_position) {
        if (out_position == nullptr || link_name.empty()) {
            return false;
        }
        for (const auto& proxy : scene.getLinkCollisionProxies()) {
            if (proxy.link_name == link_name) {
                *out_position = proxy.world_center;
                return true;
            }
        }
        glm::mat4 world_tf(1.0f);
        if (scene.getLinkWorldTransform(link_name, &world_tf)) {
            *out_position = glm::vec3(world_tf[3]);
            return true;
        }
        return false;
    }

    void FocusCameraOnLink(teleop_viewer::OrbitCamera* camera, const teleop_viewer::RobotScene& scene, const std::string& link_name) {
        if (camera == nullptr) {
            return;
        }
        glm::vec3 focus(0.0f);
        if (GetLinkWorldFocusPoint(scene, link_name, &focus)) {
            camera->target = focus;
        }
    }

    LinkSafetyInspectInfo BuildLinkSafetyInfo(const std::string& link_name, const CollisionMonitorState& collision_state,
                                              const CollisionMonitorResult& collision_result,
                                              const teleop_viewer::RobotScene& scene) {
        LinkSafetyInspectInfo info;
        if (collision_result.valid) {
            info.has_global_closest = true;
            info.global_closest     = collision_result.closest_pair;
        }

        if (!collision_state.enable || link_name.empty()) {
            return info;
        }

        const auto proxies = scene.getLinkCollisionProxies();
        bool has_closest   = false;
        for (size_t i = 0; i < proxies.size(); ++i) {
            for (size_t j = i + 1; j < proxies.size(); ++j) {
                const auto& a = proxies[i];
                const auto& b = proxies[j];
                if (a.link_name != link_name && b.link_name != link_name) {
                    continue;
                }
                CollisionPairDistance distance;
                const glm::vec3 delta       = b.world_center - a.world_center;
                const float center_distance = glm::length(delta);
                distance.link_a             = a.link_name;
                distance.link_b             = b.link_name;
                distance.center_distance_m  = center_distance;
                glm::vec3 direction(1.0f, 0.0f, 0.0f);
                if (center_distance > 1e-6f) {
                    direction = delta / center_distance;
                }
                distance.point_a            = a.world_center + direction * a.radius_m;
                distance.point_b            = b.world_center - direction * b.radius_m;
                distance.surface_distance_m = center_distance - (a.radius_m + b.radius_m);

                if (!has_closest || distance.surface_distance_m < info.pair_involving_link.surface_distance_m) {
                    info.pair_involving_link = distance;
                    has_closest              = true;
                }
            }
        }
        info.has_pair_involving_link = has_closest;
        return info;
    }

    float ScanTrajectoryMinSurfaceDistanceForLink(const std::string& link_name, const DebugPlaybackState& playback,
                                                  const CollisionMonitorState& collision_state, CollisionMonitor* monitor,
                                                  teleop_viewer::RobotScene* scene) {
        if (link_name.empty() || monitor == nullptr || scene == nullptr || playback.keyframes.empty()) {
            return -1.0f;
        }

        std::unordered_map<std::string, float> saved_joints;
        for (const auto& joint : scene->getJointInfos()) {
            saved_joints[joint.name] = joint.position;
        }
        float saved_base_x = 0.0f;
        float saved_base_y = 0.0f;
        float saved_base_yaw = 0.0f;
        scene->getVirtualBasePose2D(&saved_base_x, &saved_base_y, &saved_base_yaw);

        float min_surface = 1e9f;
        for (const auto& kf : playback.keyframes) {
            for (const auto& kv : kf.joints) {
                scene->setJointPositionByName(kv.first, kv.second);
            }
            if (kf.has_base_pose_2d) {
                scene->setVirtualBasePose2D(kf.base_x_m, kf.base_y_m, kf.base_yaw_rad);
            }
            scene->updateTransforms();

            const auto safety = BuildLinkSafetyInfo(link_name, collision_state, monitor->Evaluate(collision_state, *scene), *scene);
            if (safety.has_pair_involving_link) {
                min_surface = std::min(min_surface, safety.pair_involving_link.surface_distance_m);
            }
        }

        for (const auto& kv : saved_joints) {
            scene->setJointPositionByName(kv.first, kv.second);
        }
        scene->setVirtualBasePose2D(saved_base_x, saved_base_y, saved_base_yaw);
        scene->updateTransforms();

        if (min_surface > 1e8f) {
            return -1.0f;
        }
        return min_surface;
    }

    void RenderLinkInspectorPanel(ViewerState* ui_state, teleop_viewer::RobotScene* scene, teleop_viewer::OrbitCamera* camera,
                                  const CollisionMonitorState* collision_state, const CollisionMonitorResult* collision_result,
                                  DebugPlaybackState* playback_state, CollisionMonitor* collision_monitor,
                                  LinkKinematicsAnalyzer* kinematics_analyzer) {
        if (ui_state == nullptr || scene == nullptr) {
            return;
        }

        if (!ImGui::CollapsingHeader("Link 检查器")) {
            return;
        }

        if (ui_state->selected_link.empty()) {
            ImGui::TextDisabled("未选中 link（3D 左键点选）");
            return;
        }

        const std::string& link_name = ui_state->selected_link;
        ImGui::TextColored(ImVec4(0.35f, 0.85f, 1.0f, 1.0f), "%s", link_name.c_str());
        if (ImGui::SmallButton("清除")) {
            ui_state->selected_link.clear();
            ui_state->selected_joint = -1;
            ui_state->trajectory_min_surface_m = -1.0f;
            return;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("对准") && camera != nullptr) {
            FocusCameraOnLink(camera, *scene, link_name);
        }

        std::string parent_link;
        scene->getLinkParentName(link_name, &parent_link);

        std::string parent_joint_name;
        teleop_viewer::RobotScene::JointDetailInfo joint_detail;
        const bool has_joint = scene->getParentJointNameForLink(link_name, &parent_joint_name) &&
                               scene->getJointDetail(parent_joint_name, &joint_detail);

        if (ImGui::CollapsingHeader("关节", ImGuiTreeNodeFlags_DefaultOpen) && has_joint) {
            ImGui::Text("joint: %s  type: %s", parent_joint_name.c_str(), joint_detail.type.c_str());
            if (joint_detail.revolute) {
                const float q_ui = AngleUiFromRad(joint_detail.position, ui_state->angle_unit_deg);
                ImGui::Text("q: %.4f %s", q_ui, AngleUnitLabel(ui_state->angle_unit_deg));
                if (joint_detail.has_limits) {
                    const float lo_ui = AngleUiFromRad(joint_detail.lower_limit, ui_state->angle_unit_deg);
                    const float hi_ui = AngleUiFromRad(joint_detail.upper_limit, ui_state->angle_unit_deg);
                    ImGui::Text("limit: [%.4f, %.4f] %s", lo_ui, hi_ui, AngleUnitLabel(ui_state->angle_unit_deg));
                }
            } else {
                ImGui::Text("q: %.4f m", joint_detail.position);
            }
        } else if (!has_joint && !parent_link.empty()) {
            ImGui::TextDisabled("parent: %s", parent_link.c_str());
        }

        if (ImGui::CollapsingHeader("运动学")) {
            glm::mat4 world_tf(1.0f);
            if (scene->getLinkWorldTransform(link_name, &world_tf)) {
                const glm::vec3 pos  = glm::vec3(world_tf[3]);
                const glm::vec3 rpy_rad = glm::eulerAngles(glm::normalize(glm::quat_cast(world_tf)));
                const glm::vec3 rpy_ui(AngleUiFromRad(rpy_rad.x, ui_state->angle_unit_deg), AngleUiFromRad(rpy_rad.y, ui_state->angle_unit_deg),
                                       AngleUiFromRad(rpy_rad.z, ui_state->angle_unit_deg));
                ImGui::Text("pos: %.3f, %.3f, %.3f m", pos.x, pos.y, pos.z);
                ImGui::Text("rpy: %.2f, %.2f, %.2f %s", rpy_ui.x, rpy_ui.y, rpy_ui.z, AngleUnitLabel(ui_state->angle_unit_deg));
            }

            if (kinematics_analyzer != nullptr) {
                static std::string metrics_link;
                static LinkKinematicsMetrics metrics_cache;
                static double metrics_last_sec = -1.0;
                const double now_sec           = ImGui::GetTime();
                const bool need_refresh        = metrics_link != link_name || metrics_last_sec < 0.0 ||
                                          (now_sec - metrics_last_sec) >= kKinematicsRefreshSec;
                if (need_refresh) {
                    metrics_cache = LinkKinematicsMetrics{};
                    if (kinematics_analyzer->compute(*scene, link_name, &metrics_cache)) {
                        metrics_link     = link_name;
                        metrics_last_sec = now_sec;
                    } else {
                        metrics_link.clear();
                    }
                }
                if (metrics_cache.valid) {
                    ImGui::Text("manip: %.4f", metrics_cache.translational_manip);
                    if (metrics_cache.jacobian_condition_6d > 0.0f) {
                        ImGui::Text("cond(J): %.1f", metrics_cache.jacobian_condition_6d);
                    }
                } else if (!metrics_cache.error.empty()) {
                    ImGui::TextDisabled("%s", metrics_cache.error.c_str());
                }
            }
        }

        if (ImGui::CollapsingHeader("安全") && collision_state != nullptr && collision_result != nullptr) {
            const LinkSafetyInspectInfo safety =
                BuildLinkSafetyInfo(link_name, *collision_state, *collision_result, *scene);

            if (safety.has_pair_involving_link) {
                ImGui::Text("%s <-> %s", safety.pair_involving_link.link_a.c_str(), safety.pair_involving_link.link_b.c_str());
                ImGui::Text("surface: %.1f mm", safety.pair_involving_link.surface_distance_m * 1000.0f);
            } else if (collision_state->enable) {
                ImGui::TextDisabled("无涉及该 link 的碰撞对");
            } else {
                ImGui::TextDisabled("碰撞监测未启用");
            }

            if (playback_state != nullptr && collision_monitor != nullptr) {
                if (ui_state->trajectory_min_surface_m >= 0.0f) {
                    ImGui::Text("轨迹最小间距: %.1f mm", ui_state->trajectory_min_surface_m * 1000.0f);
                }
                if (ImGui::SmallButton("扫描轨迹最小间距")) {
                    ui_state->trajectory_min_surface_m =
                        ScanTrajectoryMinSurfaceDistanceForLink(link_name, *playback_state, *collision_state, collision_monitor, scene);
                }
            }
        }
    }

}  // namespace kinematic_viewer
