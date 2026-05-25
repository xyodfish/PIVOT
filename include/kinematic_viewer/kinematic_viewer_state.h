#pragma once

#include "kinematic_viewer/kinematic_obstacle_state.h"
#include "teleop_viewer/ik_solver.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace kinematic_viewer {

    using TeleopIkChainStatus = teleop_viewer::IkChainStatus;
    using TeleopIkSolver      = teleop_viewer::IkSolver;

    struct ViewerState {
        struct JointInputGroup {
            std::string name;
            std::vector<std::string> joint_names;
        };

        bool show_axes                      = true;
        bool show_visual_meshes             = true;
        bool show_collision_bodies          = false;
        bool show_wireframe                 = false;
        bool show_com                       = false;
        bool show_world_axes                = true;
        bool show_revolute_only             = true;
        bool lock_base                      = true;
        bool show_non_revolute              = false;
        float axis_length                   = 0.12f;
        float axis_line_width               = 2.0f;
        float world_axis_length             = 0.4f;
        float grid_size                     = 4.0f;
        int grid_count                      = 40;
        float panel_width                   = 430.0f;
        float joint_section_height          = 260.0f;
        bool panel_resize_active            = false;
        char joint_filter[128]              = {0};
        char joint_group_input[4096]        = {0};
        char joint_group_values_input[4096] = {0};
        char tf_filter[128]                 = {0};
        char tree_filter[128]               = {0};
        int selected_joint                  = -1;
        int selected_joint_input_group      = 0;
        std::string selected_link;
        std::string hovered_link;
        float trajectory_min_surface_m      = -1.0f;
        std::unordered_map<std::string, float> pose_snapshot;
        std::vector<JointInputGroup> joint_input_groups;
        std::string joint_group_input_status;
        bool joint_group_input_last_ok = false;
        int sidebar_page               = 4;  // 0:场景 1:IK 2:回放 3:安全 4:关节 5:TF 6:障碍
        UserObstacleState user_obstacles;
    };

    struct IkState {
        struct MarkerTarget {
            bool initialized  = false;
            glm::vec3 pos     = glm::vec3(0.0f);
            glm::vec3 rpy_deg = glm::vec3(0.0f);
        };

        TeleopIkSolver solver;
        std::vector<TeleopIkChainStatus> chains;
        std::vector<MarkerTarget> marker_targets;
        std::string solve_mode        = "single_chain";  // single_chain | full_body
        std::string full_body_backend = "flex_ik";       // flex_ik | wbc_chain_ik
        int full_body_iterations      = 3;
        int selected_chain            = 0;
        bool marker_initialized       = false;
        bool lock_orientation         = false;
        float marker_pos[3]           = {0.0f, 0.0f, 0.0f};
        float marker_rpy_deg[3]       = {0.0f, 0.0f, 0.0f};
        std::string last_status;
        bool drag_mode_rotate                = false;
        bool dragging_marker                 = false;
        bool marker_hovered                  = false;
        bool left_mouse_prev                 = false;
        int active_axis                      = -1;  // 0:x 1:y 2:z
        bool active_rotate                   = false;
        float translate_sensitivity          = 0.35f;
        float rotate_sensitivity             = 0.20f;
        int drag_mode                        = 0;  // 0:view-plane move, 1/2/3 move x/y/z, 4/5/6 rotate x/y/z
        float drag_prev_x                    = 0.0f;
        float drag_prev_y                    = 0.0f;
        bool drag_prev_valid                 = false;
        int gizmo_operation                  = 0;  // 0 translate, 1 rotate, 2 universal
        bool gizmo_was_using                 = false;
        bool gizmo_pose_dirty                = false;
        bool gizmo_drag_interacted           = false;
        bool gizmo_drag_position_only        = true;
        bool gizmo_was_over                  = false;
        bool gizmo_world_mode                = true;
        bool realtime_ik_during_drag         = true;
        float realtime_ik_hz                 = 30.0f;
        double last_realtime_ik_apply_sec    = -1.0;
        float gizmo_size_clip_space          = 0.11f;
        bool translate_snap_enabled          = false;
        bool rotate_snap_enabled             = false;
        float translate_snap_step_m          = 0.01f;
        float rotate_snap_step_deg           = 5.0f;
        bool refine_single_chain_on_drag_end = true;
        bool refine_only_when_rotation       = false;
        float translate_channel_gain[3]      = {1.0f, 1.0f, 1.0f};  // x/y/z
        float rotate_channel_gain[3]         = {1.0f, 1.0f, 1.0f};  // roll/pitch/yaw
        bool use_external_target             = true;
        bool external_target_received        = false;
        bool external_target_dirty           = false;
        bool external_target_position_only   = true;
        std::string external_target_topic    = "/teleop_gui/ik_target_pose";
        std::string external_target_expected_frame;
        std::string external_target_last_frame;
        glm::vec3 external_target_pos        = glm::vec3(0.0f);
        glm::quat external_target_quat       = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);  // w, x, y, z
        double external_target_last_recv_sec = 0.0;
    };

}  // namespace kinematic_viewer
