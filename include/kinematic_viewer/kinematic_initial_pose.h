#pragma once

#include "kinematic_viewer/kinematic_viewer_config.h"
#include "rkv/scene.h"

#include <string>

namespace kinematic_viewer {

    struct InitialPoseApplyResult {
        int requested_joint_count = 0;
        int applied_joint_count   = 0;
        int missing_joint_count   = 0;
        bool chassis_applied      = false;
        float chassis_x_m         = 0.0f;
        float chassis_y_m         = 0.0f;
        float chassis_yaw_rad     = 0.0f;
        std::string detail;
    };

    InitialPoseApplyResult ApplyConfiguredInitialPose(const KinematicInitialPoseConfig& config, rkv::RobotScene* scene);

}  // namespace kinematic_viewer
