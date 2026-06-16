#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_runtime_state.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "obstacle";
    out->label = "障碍";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    kinematic_viewer::RenderObstaclePanel(static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state));
}

}  // extern "C"
