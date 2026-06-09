#pragma once

#include "rkv/config_types.h"

#include <string>

namespace kinematic_viewer {

    using CameraConfig        = rkv::CameraConfig;
    using RobotConfig         = rkv::RobotConfig;
    using UiConfig            = rkv::UiConfig;
    using ViewerIkConfig      = rkv::ViewerIkConfig;
    using ViewerIkChainConfig = rkv::ViewerIkChainConfig;
    using WindowConfig        = rkv::WindowConfig;

    struct KinematicRosConfig {
        bool enable = true;
    };

    struct KinematicPointCloudConfig {
        bool enable             = false;
        bool visible            = true;
        bool auto_load_on_start = false;
        std::string file_path;
        float voxel_size             = 0.05f;
        int max_points               = 500000;
        float x_min                  = -1e9f;
        float x_max                  = 1e9f;
        float y_min                  = -1e9f;
        float y_max                  = 1e9f;
        float z_min                  = -1.0f;
        float z_max                  = 2.0f;
        float offset_x               = 0.0f;
        float offset_y               = 0.0f;
        float offset_yaw             = 0.0f;
        float point_size_px          = 2.0f;
        std::string color_mode       = "height";  // flat | height | intensity
        bool build_esdf              = false;
        float esdf_resolution        = 0.05f;
        float esdf_map_origin[3]     = {-30.0f, -20.0f, -0.1f};
        float esdf_map_size[3]       = {60.0f, 40.0f, 2.1f};
        bool esdf_use_fixed_map      = true;
        float esdf_max_visual_dist   = 0.15f;
        int esdf_visual_stride       = 1;
        std::string esdf_visual_mode = "occupied";  // occupied | surface | distance
        std::string esdf_color_mode  = "height";    // flat | height | distance
        bool esdf_z_slice_enable     = false;
        float esdf_z_slice_m         = 0.5f;
        bool esdf_use_raycast        = true;
        float esdf_ray_origin[3]     = {0.0f, 0.0f, 0.0f};
        bool esdf_ray_origin_auto    = true;
        float esdf_min_ray_length    = 0.1f;
        float esdf_max_ray_length    = 50.0f;
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
        std::string last_browser_dir;  // remembered trajectory file browser directory
    };

    struct KinematicTeachConfig {
        std::vector<std::string> program_files;
        int selected_index = -1;
        std::string last_browser_dir;
    };

    struct KinematicViewerConfig {
        WindowConfig window;
        RobotConfig robot;
        CameraConfig camera;
        UiConfig ui;
        ViewerIkConfig ik;
        KinematicRosConfig ros;
        KinematicPointCloudConfig point_cloud;
        KinematicInitialPoseConfig initial_pose;
        KinematicPlaybackConfig playback;
        KinematicTeachConfig teach;

        static KinematicViewerConfig LoadFromFile(const std::string& yaml_path, bool* loaded_ok = nullptr);
    };

}  // namespace kinematic_viewer
