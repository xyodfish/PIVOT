#include "kinematic_viewer/kinematic_ik_controller.h"
#include "kinematic_viewer/kinematic_marker_utils.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace kinematic_viewer {

    KinematicIkController::KinematicIkController(IkState* ik_state) : ik_state_(ik_state) {}

    bool KinematicIkController::InitializeSolver(const std::string& urdf_path, const ViewerIkConfig& ik_cfg) {
        if (!ik_state_) {
            return false;
        }
        ik_state_->solver.setFullBodyBackend(ik_state_->full_body_backend);
        if (!ik_state_->solver.initialize(urdf_path, ik_cfg.chains)) {
            return false;
        }
        ik_state_->chains.clear();
        for (int i = 0; i < ik_state_->solver.chainCount(); ++i) {
            ik_state_->chains.push_back(ik_state_->solver.chainStatus(i));
        }
        if (!ik_state_->chains.empty()) {
            ik_state_->selected_chain = std::clamp(ik_state_->selected_chain, 0, static_cast<int>(ik_state_->chains.size()) - 1);
        }
        ik_state_->marker_targets.resize(ik_state_->chains.size());
        return true;
    }

    bool KinematicIkController::EnsureMarkerTargetInitialized(teleop_viewer::RobotScene* scene, int chain_index) {
        if (!ik_state_ || !scene || chain_index < 0 || chain_index >= static_cast<int>(ik_state_->marker_targets.size())) {
            return false;
        }
        auto& target = ik_state_->marker_targets[chain_index];
        if (target.initialized) {
            return true;
        }
        glm::vec3 tip_pos(0.0f);
        glm::vec3 tip_rpy(0.0f);
        if (ik_state_->solver.fetchTipWorldPose(*scene, chain_index, &tip_pos, &tip_rpy)) {
            target.pos         = tip_pos;
            target.rpy_deg     = glm::vec3(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
            target.initialized = true;
            return true;
        }
        return false;
    }

    bool KinematicIkController::LoadActiveMarkerFromTarget(teleop_viewer::RobotScene* scene) {
        if (!ik_state_ || !scene || ik_state_->selected_chain < 0 ||
            ik_state_->selected_chain >= static_cast<int>(ik_state_->marker_targets.size())) {
            return false;
        }
        auto& target = ik_state_->marker_targets[ik_state_->selected_chain];
        if (!target.initialized) {
            if (!EnsureMarkerTargetInitialized(scene, ik_state_->selected_chain)) {
                return false;
            }
        }
        ik_state_->marker_pos[0]      = target.pos.x;
        ik_state_->marker_pos[1]      = target.pos.y;
        ik_state_->marker_pos[2]      = target.pos.z;
        ik_state_->marker_rpy_deg[0]  = target.rpy_deg.x;
        ik_state_->marker_rpy_deg[1]  = target.rpy_deg.y;
        ik_state_->marker_rpy_deg[2]  = target.rpy_deg.z;
        ik_state_->marker_initialized = true;
        return true;
    }

    void KinematicIkController::SaveActiveMarkerToTarget() {
        if (!ik_state_ || ik_state_->selected_chain < 0 ||
            ik_state_->selected_chain >= static_cast<int>(ik_state_->marker_targets.size())) {
            return;
        }
        auto& target       = ik_state_->marker_targets[ik_state_->selected_chain];
        target.pos         = glm::vec3(ik_state_->marker_pos[0], ik_state_->marker_pos[1], ik_state_->marker_pos[2]);
        target.rpy_deg     = glm::vec3(ik_state_->marker_rpy_deg[0], ik_state_->marker_rpy_deg[1], ik_state_->marker_rpy_deg[2]);
        target.initialized = true;
    }

    bool KinematicIkController::ApplyIkForActiveChain(teleop_viewer::RobotScene* scene, bool force_orientation_lock,
                                                      bool fast_mode, bool prefer_position_only_target) {
        if (!ik_state_ || !scene || ik_state_->selected_chain < 0 ||
            ik_state_->selected_chain >= static_cast<int>(ik_state_->chains.size())) {
            if (ik_state_) {
                ik_state_->last_status = "IK失败：未选择链";
            }
            return false;
        }
        const auto& chain_status = ik_state_->chains[ik_state_->selected_chain];
        if (ik_state_->solve_mode == "single_chain" && !chain_status.ready) {
            ik_state_->last_status = "IK失败：链未就绪";
            return false;
        }

        if (force_orientation_lock || ik_state_->lock_orientation) {
            glm::vec3 tip_pos(0.0f);
            glm::vec3 tip_rpy(0.0f);
            if (ik_state_->solver.fetchTipWorldPose(*scene, ik_state_->selected_chain, &tip_pos, &tip_rpy)) {
                ik_state_->marker_rpy_deg[0] = glm::degrees(tip_rpy.x);
                ik_state_->marker_rpy_deg[1] = glm::degrees(tip_rpy.y);
                ik_state_->marker_rpy_deg[2] = glm::degrees(tip_rpy.z);
            }
        }
        if (ik_state_->solve_mode == "full_body" && fast_mode && prefer_position_only_target && !ik_state_->lock_orientation) {
            glm::vec3 tip_pos(0.0f);
            glm::vec3 tip_rpy(0.0f);
            if (ik_state_->solver.fetchTipWorldPose(*scene, ik_state_->selected_chain, &tip_pos, &tip_rpy)) {
                ik_state_->marker_rpy_deg[0] = glm::degrees(tip_rpy.x);
                ik_state_->marker_rpy_deg[1] = glm::degrees(tip_rpy.y);
                ik_state_->marker_rpy_deg[2] = glm::degrees(tip_rpy.z);
            }
        }
        SaveActiveMarkerToTarget();

        const glm::vec3 marker_pos(ik_state_->marker_pos[0], ik_state_->marker_pos[1], ik_state_->marker_pos[2]);
        const glm::vec3 marker_rpy_deg(ik_state_->marker_rpy_deg[0], ik_state_->marker_rpy_deg[1], ik_state_->marker_rpy_deg[2]);
        const glm::mat4 active_target_world = markerWorldMatrix(marker_pos, marker_rpy_deg);

        if (ik_state_->solve_mode == "full_body") {
            std::vector<glm::mat4> targets_world(static_cast<size_t>(ik_state_->chains.size()), glm::mat4(1.0f));
            for (int i = 0; i < static_cast<int>(ik_state_->chains.size()); ++i) {
                if (i == ik_state_->selected_chain) {
                    targets_world[static_cast<size_t>(i)] = active_target_world;
                    continue;
                }
                glm::vec3 tip_pos(0.0f);
                glm::vec3 tip_rpy(0.0f);
                if (ik_state_->solver.fetchTipWorldPose(*scene, i, &tip_pos, &tip_rpy)) {
                    const glm::vec3 tip_rpy_deg(glm::degrees(tip_rpy.x), glm::degrees(tip_rpy.y), glm::degrees(tip_rpy.z));
                    targets_world[static_cast<size_t>(i)] = markerWorldMatrix(tip_pos, tip_rpy_deg);
                } else if (EnsureMarkerTargetInitialized(scene, i)) {
                    const auto& target                    = ik_state_->marker_targets[i];
                    targets_world[static_cast<size_t>(i)] = markerWorldMatrix(target.pos, target.rpy_deg);
                } else {
                    targets_world[static_cast<size_t>(i)] = active_target_world;
                }
            }

            teleop_viewer::IkSolveStats stats;
            const int solve_iters = fast_mode ? 1 : std::max(1, ik_state_->full_body_iterations);
            if (ik_state_->solver.solveFullBody(scene, targets_world, solve_iters, ik_state_->selected_chain, fast_mode,
                                                prefer_position_only_target, &stats, &ik_state_->last_status)) {
                return true;
            }
            return false;
        }

        return ik_state_->solver.solveSingleChain(scene, ik_state_->selected_chain, active_target_world, &ik_state_->last_status);
    }

    bool KinematicIkController::RefineActiveChainToMarker(teleop_viewer::RobotScene* scene) {
        if (!ik_state_ || !scene || ik_state_->selected_chain < 0 ||
            ik_state_->selected_chain >= static_cast<int>(ik_state_->chains.size())) {
            return false;
        }
        const auto& chain_status = ik_state_->chains[ik_state_->selected_chain];
        if (!chain_status.ready) {
            return false;
        }
        const glm::vec3 marker_pos(ik_state_->marker_pos[0], ik_state_->marker_pos[1], ik_state_->marker_pos[2]);
        const glm::vec3 marker_rpy_deg(ik_state_->marker_rpy_deg[0], ik_state_->marker_rpy_deg[1], ik_state_->marker_rpy_deg[2]);
        const glm::mat4 target_world = markerWorldMatrix(marker_pos, marker_rpy_deg);
        std::string refine_status;
        const bool refined = ik_state_->solver.solveSingleChain(scene, ik_state_->selected_chain, target_world, &refine_status);
        if (refined) {
            if (ik_state_->last_status.empty()) {
                ik_state_->last_status = "末端精修完成(single_chain)";
            } else {
                ik_state_->last_status += " | 末端精修完成(single_chain)";
            }
            return true;
        }
        if (!refine_status.empty()) {
            if (ik_state_->last_status.empty()) {
                ik_state_->last_status = "末端精修失败: " + refine_status;
            } else {
                ik_state_->last_status += " | 末端精修失败: " + refine_status;
            }
        }
        return false;
    }

    float KinematicIkController::ActiveChainPositionErrorMmToMarker(teleop_viewer::RobotScene* scene) const {
        if (!ik_state_ || !scene || ik_state_->selected_chain < 0 ||
            ik_state_->selected_chain >= static_cast<int>(ik_state_->chains.size())) {
            return 0.0f;
        }
        glm::vec3 tip_pos(0.0f);
        glm::vec3 tip_rpy(0.0f);
        if (!ik_state_->solver.fetchTipWorldPose(*scene, ik_state_->selected_chain, &tip_pos, &tip_rpy)) {
            return 0.0f;
        }
        const glm::vec3 marker_pos(ik_state_->marker_pos[0], ik_state_->marker_pos[1], ik_state_->marker_pos[2]);
        return glm::length(marker_pos - tip_pos) * 1000.0f;
    }

    void KinematicIkController::ApplyExternalTarget(teleop_viewer::RobotScene* scene) {
        if (!ik_state_ || !scene || !ik_state_->use_external_target || !ik_state_->external_target_dirty || ik_state_->dragging_marker) {
            return;
        }

        const glm::vec3 prev_marker_pos(ik_state_->marker_pos[0], ik_state_->marker_pos[1], ik_state_->marker_pos[2]);
        const glm::vec3 prev_marker_rpy_deg(ik_state_->marker_rpy_deg[0], ik_state_->marker_rpy_deg[1], ik_state_->marker_rpy_deg[2]);
        const glm::quat prev_marker_quat = glm::normalize(glm::quat_cast(markerWorldMatrix(prev_marker_pos, prev_marker_rpy_deg)));
        const glm::quat target_quat      = glm::normalize(ik_state_->external_target_quat);
        float external_rotation_delta    = glm::abs(glm::angle(glm::inverse(prev_marker_quat) * target_quat));
        if (external_rotation_delta > glm::pi<float>()) {
            external_rotation_delta = glm::two_pi<float>() - external_rotation_delta;
        }

        ik_state_->marker_pos[0] = ik_state_->external_target_pos.x;
        ik_state_->marker_pos[1] = ik_state_->external_target_pos.y;
        ik_state_->marker_pos[2] = ik_state_->external_target_pos.z;
        if (!ik_state_->lock_orientation) {
            glm::vec3 rpy                = glm::eulerAngles(ik_state_->external_target_quat);
            ik_state_->marker_rpy_deg[0] = glm::degrees(rpy.x);
            ik_state_->marker_rpy_deg[1] = glm::degrees(rpy.y);
            ik_state_->marker_rpy_deg[2] = glm::degrees(rpy.z);
        }
        SaveActiveMarkerToTarget();
        const bool external_position_only = ik_state_->external_target_position_only && external_rotation_delta < glm::radians(0.5f);
        ApplyIkForActiveChain(scene, false, false, external_position_only);
        ik_state_->external_target_dirty = false;
    }

}  // namespace kinematic_viewer
