#pragma once

#include "teleop_viewer/config_types.h"

#include <string>

namespace kinematic_viewer {

    using CameraConfig        = omnilink::teleop_viewer::CameraConfig;
    using RobotConfig         = omnilink::teleop_viewer::RobotConfig;
    using UiConfig            = omnilink::teleop_viewer::UiConfig;
    using ViewerIkConfig      = omnilink::teleop_viewer::ViewerIkConfig;
    using ViewerIkChainConfig = omnilink::teleop_viewer::ViewerIkChainConfig;
    using WindowConfig        = omnilink::teleop_viewer::WindowConfig;

    struct KinematicRosConfig {
        bool enable = true;
    };

    struct KinematicInitialPoseConfig {
        bool enable              = false;
        bool auto_apply_on_start = false;

        std::vector<std::string> head_joint_names;
        std::vector<std::string> leg_joint_names;
        std::vector<std::string> left_arm_joint_names;
        std::vector<std::string> right_arm_joint_names;

        std::vector<float> head;
        std::vector<float> leg;
        std::vector<float> left_arm;
        std::vector<float> right_arm;

        // Planar mobile base (m, m, rad); applied via RobotScene virtual base when enable_apply_chassis is true.
        bool apply_chassis = true;
        float chassis_x    = 0.0f;
        float chassis_y    = 0.0f;
        float chassis_yaw  = 0.0f;
    };

    struct KinematicPlaybackConfig {
        std::vector<std::string> trajectory_files;
        int selected_index = -1;
    };

    struct KinematicViewerConfig {
        WindowConfig window;
        RobotConfig robot;
        CameraConfig camera;
        UiConfig ui;
        ViewerIkConfig ik;
        KinematicRosConfig ros;
        KinematicInitialPoseConfig initial_pose;
        KinematicPlaybackConfig playback;

        static KinematicViewerConfig LoadFromFile(const std::string& yaml_path, bool* loaded_ok = nullptr);
    };

}  // namespace kinematic_viewer
