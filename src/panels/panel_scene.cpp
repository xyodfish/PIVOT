#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_robot_tree_panel.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

extern "C" {

void rkv_panel_info(RkvPanelInfo* out) {
    out->id    = "scene";
    out->label = "场景";
}

void rkv_panel_render(RkvPanelCtx* ctx) {
    auto* ui    = static_cast<kinematic_viewer::ViewerState*>(ctx->viewer_state);
    auto* scene = static_cast<teleop_viewer::RobotScene*>(ctx->scene);
    kinematic_viewer::RenderScenePanel(ui, scene);
    kinematic_viewer::RenderRobotTreePanel(ui, scene);
}

}  // extern "C"
