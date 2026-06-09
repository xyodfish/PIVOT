#include "kinematic_viewer/kinematic_marker_target_state.h"

#include "rkv/scene.h"

namespace kinematic_viewer {

    bool EnsureMarkerTargetInitialized(IkState* ikState, rkv::RobotScene* scene, int chainIndex) {
        if (ikState == nullptr || scene == nullptr) {
            return false;
        }
        if (chainIndex < 0 || chainIndex >= static_cast<int>(ikState->marker_targets.size())) {
            return false;
        }
        auto& target = ikState->marker_targets[chainIndex];
        if (target.initialized) {
            return true;
        }
        glm::vec3 tipPos(0.0f);
        glm::vec3 tipRpy(0.0f);
        if (!ikState->solver.fetchTipWorldPose(*scene, chainIndex, &tipPos, &tipRpy)) {
            return false;
        }
        target.pos         = tipPos;
        target.rpy_deg     = glm::vec3(glm::degrees(tipRpy.x), glm::degrees(tipRpy.y), glm::degrees(tipRpy.z));
        target.initialized = true;
        return true;
    }

    bool LoadActiveMarkerFromTarget(IkState* ikState, rkv::RobotScene* scene) {
        if (ikState == nullptr || scene == nullptr) {
            return false;
        }
        if (!EnsureMarkerTargetInitialized(ikState, scene, ikState->selected_chain)) {
            return false;
        }
        const auto& target          = ikState->marker_targets[ikState->selected_chain];
        ikState->marker_pos[0]      = target.pos.x;
        ikState->marker_pos[1]      = target.pos.y;
        ikState->marker_pos[2]      = target.pos.z;
        ikState->marker_rpy_deg[0]  = target.rpy_deg.x;
        ikState->marker_rpy_deg[1]  = target.rpy_deg.y;
        ikState->marker_rpy_deg[2]  = target.rpy_deg.z;
        ikState->marker_initialized = true;
        return true;
    }

    void SaveActiveMarkerToTarget(IkState* ikState) {
        if (ikState == nullptr) {
            return;
        }
        if (ikState->selected_chain < 0 || ikState->selected_chain >= static_cast<int>(ikState->marker_targets.size())) {
            return;
        }
        auto& target       = ikState->marker_targets[ikState->selected_chain];
        target.pos         = glm::vec3(ikState->marker_pos[0], ikState->marker_pos[1], ikState->marker_pos[2]);
        target.rpy_deg     = glm::vec3(ikState->marker_rpy_deg[0], ikState->marker_rpy_deg[1], ikState->marker_rpy_deg[2]);
        target.initialized = true;
    }

}  // namespace kinematic_viewer
