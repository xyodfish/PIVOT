#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "rkv/scene.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "tf";
    out->label = "TF";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    using TfVec = std::vector<rkv::RobotScene::LinkTfInfo>;
    kinematic_viewer::RenderTfPanel(
        static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state),
        *static_cast<const TfVec*>(ctx->tf_infos));
}

}  // extern "C"
