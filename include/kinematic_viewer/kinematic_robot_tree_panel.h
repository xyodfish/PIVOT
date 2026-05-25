#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

namespace kinematic_viewer {

    void RenderRobotTreePanel(ViewerState* ui_state, teleop_viewer::RobotScene* scene);

}  // namespace kinematic_viewer
