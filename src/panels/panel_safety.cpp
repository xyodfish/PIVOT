#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_collision_monitor.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "safety";
    out->label = "安全";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    kinematic_viewer::RenderSafetyPanel(static_cast<kinematic_viewer::CollisionMonitorState*>(ctx->collision_state),
                                        *static_cast<const kinematic_viewer::CollisionMonitorResult*>(ctx->collision_result));
}

}  // extern "C"
