#pragma once

#include "kinematic_viewer/kinematic_marker_target_state.h"
#include "kinematic_viewer/kinematic_viewer_config.h"
#include "kinematic_viewer/kinematic_viewer_state.h"
#include "rkv/ik_solver.h"
#include "rkv/scene.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <vector>

namespace kinematic_viewer {

    // Encapsulates IK solve logic, marker target management, and gizmo interaction helpers.
    class KinematicIkController {
       public:
        explicit KinematicIkController(IkState* ik_state);

        // Initialize solver from config
        bool InitializeSolver(const std::string& urdf_path, const ViewerIkConfig& ik_cfg);

        // Marker target helpers
        bool EnsureMarkerTargetInitialized(rkv::RobotScene* scene, int chain_index);
        bool LoadActiveMarkerFromTarget(rkv::RobotScene* scene);
        void SaveActiveMarkerToTarget();

        // IK solve entry points
        bool ApplyIkForActiveChain(rkv::RobotScene* scene, bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target);
        bool RefineActiveChainToMarker(rkv::RobotScene* scene);

        // Utility
        float ActiveChainPositionErrorMmToMarker(rkv::RobotScene* scene) const;

        // External target application
        void ApplyExternalTarget(rkv::RobotScene* scene);

        // Accessors
        IkState* State() const { return ik_state_; }

       private:
        IkState* ik_state_ = nullptr;
    };

}  // namespace kinematic_viewer
