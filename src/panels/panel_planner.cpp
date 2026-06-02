#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "teleop_viewer/scene.h"
#include "teleop_viewer/ik_solver.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "planner";
    out->label = "规划";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    using ChainVec = std::vector<teleop_viewer::IkChainStatus>;
    kinematic_viewer::RenderPathPlannerPanel(
        static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state),
        static_cast<kinematic_viewer::PathPlannerUiState*>(ctx->path_planner_ui),
        static_cast<kinematic_viewer::DebugPlaybackState*>(ctx->playback_state),
        static_cast<teleop_viewer::RobotScene*>(ctx->scene),
        static_cast<teleop_viewer::IkSolver*>(ctx->ik_solver),
        *static_cast<const ChainVec*>(ctx->ik_chains));
}

}  // extern "C"
