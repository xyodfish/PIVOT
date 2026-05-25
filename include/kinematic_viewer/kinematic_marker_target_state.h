#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"

namespace teleop_viewer {
    class RobotScene;
}

namespace kinematic_viewer {

    bool EnsureMarkerTargetInitialized(IkState* ikState, teleop_viewer::RobotScene* scene, int chainIndex);
    bool LoadActiveMarkerFromTarget(IkState* ikState, teleop_viewer::RobotScene* scene);
    void SaveActiveMarkerToTarget(IkState* ikState);

}  // namespace kinematic_viewer
