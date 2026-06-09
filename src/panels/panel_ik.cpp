#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_ik_controller.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "rkv/scene.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "ik";
    out->label = "IK";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    kinematic_viewer::RenderIkPanel(
        static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state),
        static_cast<kinematic_viewer::IkState*>(ctx->ik_state),
        static_cast<kinematic_viewer::KinematicIkController*>(ctx->ik_controller),
        static_cast<rkv::RobotScene*>(ctx->scene));
}

}  // extern "C"
