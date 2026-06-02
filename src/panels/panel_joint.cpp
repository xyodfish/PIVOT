#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "joint";
    out->label = "关节";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    using JointVec = std::vector<teleop_viewer::RobotScene::JointInfo>;
    kinematic_viewer::RenderJointPanel(
        static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state),
        static_cast<teleop_viewer::RobotScene*>(ctx->scene),
        *static_cast<const JointVec*>(ctx->joints));
}

}  // extern "C"
