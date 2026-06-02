#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace teleop_viewer {

    struct WindowConfig {
        int width         = 1280;
        int height        = 720;
        std::string title = "遥操作机器人监控界面";
    };

    struct RobotConfig {
        std::string urdf_path =
            "/home/yuxia/Workspace/SingoriX/OmniLink/singorix_omnilink/config/galbot_description/galbot_one_golf_description/"
            "galbot_one_golf.urdf";
        std::string mujoco_xml_path = "";
    };

    struct CameraConfig {
        float distance   = 3.0f;
        float yaw        = 0.0f;
        float pitch      = 0.0f;
        glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);

        float rotate_speed = 0.005f;
        float zoom_scale   = 0.1f;
        float dolly_scale  = 0.02f;
        float pan_scale    = 0.0015f;
        float min_distance = 0.2f;
        float max_distance = 20.0f;
    };

    struct UiConfig {
        bool only_show_master_arm_groups = true;
        bool fix_base_like_mujoco        = true;
        bool sidebar_collapsed_default   = false;

        int side_panel_width           = 560;
        int collapsed_sidebar_width    = 34;
        float sidebar_width_drag_speed = 0.2f;

        double stale_timeout_seconds = 0.5;
        double out_of_range_margin   = 0.01;

        int waveform_history_size  = 600;
        float waveform_plot_height = 180.0f;

        int alarm_trigger_frames            = 5;
        bool alarm_show_only_active_default = true;
        float baseline_warn_delta_deg       = 15.0f;

        std::string record_output_dir = "logs";
        bool auto_start_recording     = false;

        std::string cjk_font_path = "";
        float cjk_font_size       = 18.0f;
        std::string theme_preset  = "current_dark";  // current_dark | moonlight | spectrum_light
        // Enable planar chassis gizmo only for robots matched by keywords.
        bool enable_mobile_base_drag                = true;
        std::vector<std::string> mobile_base_robots = {"galbot_g1"};
        // Sidebar panel ids shown in tabs (order defines tab order).
        // Supported ids: scene, ik, playback, safety, joint, tf, obstacle, planner, teach, point_cloud.
        std::vector<std::string> sidebar_panels = {
            "scene", "ik", "playback", "safety", "joint", "tf", "obstacle", "planner", "teach", "point_cloud"};
    };

    struct ViewerIkChainConfig {
        std::string label     = "";
        std::string base_link = "";
        std::string tip_link  = "";
        std::vector<std::string> base_link_candidates;
        std::vector<std::string> tip_link_candidates;
    };

    struct ViewerIkConfig {
        std::string mode                        = "single_chain";  // single_chain | full_body
        std::string full_body_backend           = "flex_ik";       // flex_ik | wbc_chain_ik
        int full_body_iterations                = 3;
        std::vector<ViewerIkChainConfig> chains = {
            {"右臂", "torso_base_link", "right_arm_end_effector_mount_link"},
            {"左臂", "torso_base_link", "left_arm_end_effector_mount_link"},
        };
    };

}  // namespace teleop_viewer
