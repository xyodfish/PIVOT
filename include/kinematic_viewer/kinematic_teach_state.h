#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace kinematic_viewer {

    struct TeachPoint {
        std::string name;
        std::unordered_map<std::string, float> joints;
        bool has_base_pose_2d = false;
        float base_x_m        = 0.0f;
        float base_y_m        = 0.0f;
        float base_yaw_rad    = 0.0f;
        bool has_ee_pose      = false;
        std::string ee_tip_link;
        glm::vec3 ee_position{0.0f};
        glm::quat ee_orientation{1.0f, 0.0f, 0.0f, 0.0f};
    };

    struct TeachFileEntry {
        std::string path;
        std::string status;  // "未加载" / "加载成功" / "加载失败: ..."
        bool loaded = false;
    };

    struct TeachProgramState {
        std::string program_name = "untitled";
        /// 程序级关节表；与 points[].q 或 joints 映射对应。加载/保存 YAML 时维护。
        std::vector<std::string> joint_names;
        std::vector<TeachPoint> points;
        int selected_point_index = -1;

        std::vector<TeachFileEntry> program_files;
        int selected_program_index    = -1;
        char program_file_path[512]   = "config/teach/demo.yaml";
        char program_browser_dir[512] = "";
        std::string io_status;

        /// 记录示教点时同时保存该 IK 链末端位姿（用于 moveL）。
        int record_chain_index = 0;

        // moveJ / moveL 规划参数（对接 vp）
        float movej_max_vel  = 1.0f;
        float movej_max_acc  = 2.0f;
        float movej_max_jerk = 10.0f;
        float movej_delta_t  = 0.02f;
        int movej_profile    = 1;  // 0=TVP, 1=DSVP
        float movel_max_vel  = 0.2f;
        float movel_max_acc  = 0.1f;
        float movel_delta_t  = 0.02f;
    };

}  // namespace kinematic_viewer
