// Panel plugin C ABI.
// Each panel .so must export two C symbols:
//   rkv_panel_info(RkvPanelInfo*)
//   rkv_panel_render(RkvPanelCtx*)
//
// All structs are POD / plain-C so the ABI stays stable across C++ boundary.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ------- forward declarations of viewer-side C++ objects, passed as void* ----
// Panel .so files cast them back to the real types via the headers they #include.
// They are guaranteed valid for the lifetime of the render call.

struct RkvPanelInfo {
    const char* id;     // unique ASCII id, e.g. "scene", "ik"
    const char* label;  // UTF-8 tab label shown in the sidebar
};

// All viewer state bundled into one opaque context bag.
// Panels access whatever they need; unused fields are left NULL.
struct RkvPanelCtx {
    // ------ core viewer objects (always non-null) ------
    void* viewer_state;        // kinematic_viewer::ViewerState*
    void* ik_state;            // kinematic_viewer::IkState*
    void* ik_controller;       // kinematic_viewer::KinematicIkController*
    void* scene;               // rkv::RobotScene*
    void* camera;              // rkv::OrbitCamera*

    // ------ collision ------
    void* collision_state;     // kinematic_viewer::CollisionMonitorState*
    void* collision_result;    // kinematic_viewer::CollisionMonitorResult*
    void* collision_monitor;   // kinematic_viewer::CollisionMonitor*

    // ------ playback / teach ------
    void* playback_state;      // kinematic_viewer::DebugPlaybackState*
    void* playback_player;     // kinematic_viewer::TrajectoryPlayer*
    void* playback_sm;         // kinematic_viewer::PlaybackStateMachine*
    void* teach_state;         // kinematic_viewer::TeachProgramState*

    // ------ point cloud ------
    void* point_cloud_state;   // kinematic_viewer::PointCloudUiState*
    void* point_cloud_layer;   // kinematic_viewer::KinematicPointCloudLayer*

    // ------ path planner ------
    void* path_planner_ui;     // kinematic_viewer::PathPlannerUiState*

    // ------ link inspector helpers ------
    void* link_kinematics_analyzer; // kinematic_viewer::LinkKinematicsAnalyzer*

    // ------ misc ------
    void* joints;              // std::vector<rkv::RobotScene::JointInfo>*
    void* tf_infos;            // std::vector<rkv::RobotScene::LinkTfInfo>*
    void* ik_solver;           // rkv::IkSolver*
    void* ik_chains;           // std::vector<rkv::IkChainStatus>*
};

typedef void (*RkvPanelInfoFn)(RkvPanelInfo*);
typedef void (*RkvPanelRenderFn)(RkvPanelCtx*);

#ifdef __cplusplus
}
#endif
