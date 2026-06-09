#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"

namespace rkv {
    class RobotScene;
}

namespace kinematic_viewer {

    bool EnsureMarkerTargetInitialized(IkState* ikState, rkv::RobotScene* scene, int chainIndex);
    bool LoadActiveMarkerFromTarget(IkState* ikState, rkv::RobotScene* scene);
    void SaveActiveMarkerToTarget(IkState* ikState);

}  // namespace kinematic_viewer
