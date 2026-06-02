#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_teach_panel.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "teleop_viewer/scene.h"
#include "teleop_viewer/ik_solver.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "teach";
    out->label = "示教";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    using JointVec = std::vector<teleop_viewer::RobotScene::JointInfo>;
    using ChainVec = std::vector<teleop_viewer::IkChainStatus>;
    kinematic_viewer::RenderTeachPanel(
        static_cast<kinematic_viewer::TeachProgramState*>(ctx->teach_state),
        static_cast<kinematic_viewer::DebugPlaybackState*>(ctx->playback_state),
        static_cast<teleop_viewer::RobotScene*>(ctx->scene),
        static_cast<teleop_viewer::IkSolver*>(ctx->ik_solver),
        *static_cast<const ChainVec*>(ctx->ik_chains),
        *static_cast<const JointVec*>(ctx->joints));
}

}  // extern "C"
