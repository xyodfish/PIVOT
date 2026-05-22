#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace kinematic_viewer {

    struct PoseKeyframe {
        double t = 0.0;
        std::unordered_map<std::string, float> joints;
        bool has_base_pose_2d = false;
        float base_x_m        = 0.0f;
        float base_y_m        = 0.0f;
        float base_yaw_rad    = 0.0f;
    };

    struct TrajectoryFileEntry {
        std::string path;
        std::string status;  // "未加载" / "加载成功" / "加载失败: ..."
        bool loaded = false;
    };

    struct DebugPlaybackState {
        enum class Mode {
            Stopped = 0,
            Playing = 1,
            Paused  = 2,
        };

        std::vector<PoseKeyframe> keyframes;
        Mode mode                        = Mode::Stopped;
        bool loop                        = true;
        float play_speed                 = 1.0f;
        float play_time                  = 0.0f;
        float keyframe_interval_sec      = 1.0f;
        int selected_keyframe_index      = -1;
        int current_segment_index        = -1;
        bool timeline_edited_this_ui     = false;
        char trajectory_file_path[512]   = "config/trajectories/galbot_g1_dance_slide_in.csv";
        char trajectory_browser_dir[512] = "";
        std::string trajectory_io_status;
        bool trajectory_alert_popup_pending = false;
        std::string trajectory_alert_message;
        std::string trajectory_alert_detail;

        // Multi-trajectory file sequence
        std::vector<TrajectoryFileEntry> trajectory_files;
        int selected_trajectory_index = -1;
    };

}  // namespace kinematic_viewer
