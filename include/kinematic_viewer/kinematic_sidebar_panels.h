#pragma once

#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_link_inspector.h"
#include "kinematic_viewer/kinematic_link_kinematics.h"
#include "kinematic_viewer/kinematic_path_planner.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

#include <vector>

namespace kinematic_viewer {

    void RenderScenePanel(ViewerState* uiState);
    void RenderLinkInspectorPanel(ViewerState* uiState, teleop_viewer::RobotScene* scene,
                                  teleop_viewer::OrbitCamera* camera, const CollisionMonitorState* collisionState,
                                  const CollisionMonitorResult* collisionResult, DebugPlaybackState* playbackState,
                                  CollisionMonitor* collisionMonitor, LinkKinematicsAnalyzer* kinematicsAnalyzer);
    void RenderJointPanel(ViewerState* uiState, teleop_viewer::RobotScene* scene,
                          const std::vector<teleop_viewer::RobotScene::JointInfo>& joints);
    void RenderPlaybackPanel(DebugPlaybackState* playbackState, TrajectoryPlayer* playbackPlayer, PlaybackStateMachine* playback_sm,
                             teleop_viewer::RobotScene* scene,
                             const std::vector<teleop_viewer::RobotScene::JointInfo>& joints);
    void RenderSafetyPanel(CollisionMonitorState* collisionState, const CollisionMonitorResult& collisionResult);
    void RenderObstaclePanel(ViewerState* uiState);
    void RenderTfPanel(ViewerState* uiState, const std::vector<teleop_viewer::RobotScene::LinkTfInfo>& tfs);

    // Path planning panel
    struct PathPlannerUiState {
        int selected_path_type = 0;  // 0=circle, 1=square, 2=head_bob, 3=straight, 4=joint_ptp
        int selected_chain     = 0;  // Which IK chain to control
        std::string last_status;     // Last operation status
        bool show_preview = true;    // Show path preview in 3D

        // Stored Cartesian path for 3D preview (updated when planning succeeds)
        std::vector<CartesianWaypoint> preview_waypoints;

        // Circle params
        float circle_center[3] = {0.0f, 0.0f, 0.0f};
        float circle_radius    = 0.1f;
        float circle_period    = 4.0f;
        int circle_points      = 60;

        // Square params
        float square_center[3] = {0.0f, 0.0f, 0.0f};
        float square_side      = 0.15f;
        float square_corner_r  = 0.02f;
        float square_period    = 4.0f;
        int square_points      = 80;

        // Head bob params
        float head_pitch_amp_deg = 15.0f;
        float head_period        = 2.0f;
        int head_points          = 40;

        // Straight line params
        float straight_goal[3] = {0.0f, 0.0f, 0.0f};
        float straight_max_vel = 0.2f;
        float straight_max_acc = 0.1f;

        // Joint-space PTP params
        float ptp_max_vel  = 1.0f;   // rad/s or m/s
        float ptp_max_acc  = 2.0f;   // rad/s^2 or m/s^2
        float ptp_max_jerk = 10.0f;  // rad/s^3 or m/s^3
        float ptp_delta_t  = 0.02f;  // sampling time step
        int ptp_profile    = 1;      // 0=TVP, 1=DSVP
        // Goal offsets from current position (per joint, filled at runtime)
        std::vector<float> ptp_goal_offsets;

        // IK mode: 0=single chain (TRAC-IK), 1=full body (wbc_chain_ik)
        int ik_mode = 1;  // Default to full body for better solve quality

        // Export
        char export_name[128] = "planned_trajectory.csv";
    };

    void RenderPathPlannerPanel(PathPlannerUiState* planner_ui, DebugPlaybackState* playbackState,
                                teleop_viewer::RobotScene* scene, teleop_viewer::IkSolver* solver,
                                const std::vector<teleop_viewer::IkChainStatus>& chains);

}  // namespace kinematic_viewer
