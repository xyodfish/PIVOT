#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_point_cloud.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "point_cloud";
    out->label = "点云";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    kinematic_viewer::RenderPointCloudPanel(
        static_cast<kinematic_viewer::PointCloudUiState*>(ctx->point_cloud_state),
        static_cast<kinematic_viewer::KinematicPointCloudLayer*>(ctx->point_cloud_layer));
}

}  // extern "C"
