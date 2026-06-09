#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_teach_panel.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "rkv/scene.h"
#include "rkv/ik_solver.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "teach";
    out->label = "示教";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    using JointVec = std::vector<rkv::RobotScene::JointInfo>;
    using ChainVec = std::vector<rkv::IkChainStatus>;
    kinematic_viewer::RenderTeachPanel(
        static_cast<kinematic_viewer::TeachProgramState*>(ctx->teach_state),
        static_cast<kinematic_viewer::DebugPlaybackState*>(ctx->playback_state),
        static_cast<rkv::RobotScene*>(ctx->scene),
        static_cast<rkv::IkSolver*>(ctx->ik_solver),
        *static_cast<const ChainVec*>(ctx->ik_chains),
        *static_cast<const JointVec*>(ctx->joints));
}

}  // extern "C"
