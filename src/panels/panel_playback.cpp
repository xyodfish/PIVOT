#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_viewer_state.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "rkv/scene.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "playback";
    out->label = "回放";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    using JointVec = std::vector<rkv::RobotScene::JointInfo>;
    kinematic_viewer::RenderPlaybackPanel(static_cast<kinematic_viewer::DebugPlaybackState*>(ctx->playback_state),
                                          static_cast<kinematic_viewer::TrajectoryPlayer*>(ctx->playback_player),
                                          static_cast<kinematic_viewer::PlaybackStateMachine*>(ctx->playback_sm),
                                          static_cast<rkv::RobotScene*>(ctx->scene), *static_cast<const JointVec*>(ctx->joints),
                                          static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state));
}

}  // extern "C"
