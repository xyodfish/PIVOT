#pragma once

#include "kinematic_viewer/kinematic_viewer_state.h"

namespace kinematic_viewer {

    void SetDemoVisualMode(ViewerState* ui_state, bool enabled);
    void ToggleDemoVisualMode(ViewerState* ui_state);

}  // namespace kinematic_viewer
