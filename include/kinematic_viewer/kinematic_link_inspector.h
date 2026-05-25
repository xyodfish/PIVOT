#pragma once

#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_link_kinematics.h"
#include "kinematic_viewer/kinematic_playback_state.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

#include <glm/vec3.hpp>

namespace teleop_viewer {
class OrbitCamera;
}

namespace kinematic_viewer {

    struct LinkSafetyInspectInfo {
        bool has_pair_involving_link     = false;
        CollisionPairDistance pair_involving_link;
        bool has_global_closest          = false;
        CollisionPairDistance global_closest;
        float trajectory_min_surface_m   = -1.0f;
        bool trajectory_scan_done        = false;
        std::string trajectory_scan_note;
    };

    LinkSafetyInspectInfo BuildLinkSafetyInfo(const std::string& link_name, const CollisionMonitorState& collision_state,
                                              const CollisionMonitorResult& collision_result,
                                              const teleop_viewer::RobotScene& scene);

    float ScanTrajectoryMinSurfaceDistanceForLink(const std::string& link_name, const DebugPlaybackState& playback,
                                                  const CollisionMonitorState& collision_state, CollisionMonitor* monitor,
                                                  teleop_viewer::RobotScene* scene);

    bool GetLinkWorldFocusPoint(const teleop_viewer::RobotScene& scene, const std::string& link_name, glm::vec3* out_position);

    void FocusCameraOnLink(teleop_viewer::OrbitCamera* camera, const teleop_viewer::RobotScene& scene,
                           const std::string& link_name);

    void RenderLinkInspectorPanel(ViewerState* ui_state, teleop_viewer::RobotScene* scene,
                                  teleop_viewer::OrbitCamera* camera, const CollisionMonitorState* collision_state,
                                  const CollisionMonitorResult* collision_result, DebugPlaybackState* playback_state,
                                  CollisionMonitor* collision_monitor, LinkKinematicsAnalyzer* kinematics_analyzer);

}  // namespace kinematic_viewer
