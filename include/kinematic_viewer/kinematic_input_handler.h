#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"
#include "rkv/scene.h"

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <string>

namespace kinematic_viewer {

    // Encapsulates all input handling logic: camera orbit/pan/zoom, obstacle picking,
    // and sidebar hotkeys. Keeps mutable per-frame state internally.
    class KinematicInputHandler {
       public:
        struct CameraInputResult {
            bool consumed = false;  // true if input was used for camera control
        };

        struct ObstaclePickResult {
            bool picked        = false;
            int selected_index = -1;
        };

        struct LinkPickResult {
            bool picked        = false;
            bool throttle_skip = false;  // true: caller should keep previous hover
            std::string link_name;
        };

        struct JointDragResult {
            bool dragging     = false;
            bool started      = false;
            bool ended        = false;
            std::string joint_name;
            std::string link_name;
        };

        struct UpdateContext {
            double mouse_x = 0.0;
            double mouse_y = 0.0;
            int viewport_w = 0;
            int viewport_h = 0;

            bool imgui_wants_mouse    = false;
            bool imgui_wants_keyboard = false;
            bool panel_resize_active  = false;

            bool ik_gizmo_using     = false;
            bool ik_gizmo_over      = false;
            bool obs_gizmo_using    = false;
            bool obs_gizmo_over     = false;
            bool ik_dragging_marker = false;
            bool joint_drag_active  = false;
            bool enable_joint_drag  = true;
            bool hovered_link_draggable = false;

            const std::string* hovered_link = nullptr;

            int sidebar_page = 0;

            float scroll_delta = 0.0f;

            // GLFW button states (queried by caller)
            bool left_mouse_down   = false;
            bool middle_mouse_down = false;
            bool right_mouse_down  = false;
            bool shift_key_down    = false;
        };

        KinematicInputHandler() = default;

        // Call once per frame with current GLFW window and context.
        // Returns camera input result (whether camera consumed the input).
        CameraInputResult UpdateCamera(rkv::OrbitCamera* camera, const UpdateContext& ctx);

        // Check for obstacle click-pick in the 3D viewport.
        ObstaclePickResult UpdateObstaclePick(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                              const UserObstacleState& obstacles);

        LinkPickResult UpdateLinkPick(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                      rkv::RobotScene* scene);

        // Raycast link under cursor (fast proxy pick). When unthrottled=true, updates every frame.
        LinkPickResult UpdateLinkHover(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                       rkv::RobotScene* scene, double now_sec, bool unthrottled = false);

        // Left-click a revolute joint's child link, then drag to rotate the joint.
        JointDragResult UpdateJointDrag(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                        rkv::RobotScene* scene);

        bool IsJointDragActive() const { return joint_drag_active_; }

        // Handle sidebar page hotkeys (1-9), bounded by visible tab count.
        int HandleSidebarHotkeys(int current_page, int page_count, bool enable_hotkeys);

        struct ViewportHotkeyResult {
            bool toggled_sidebar = false;
            bool toggled_playback = false;
            int playback_step_count = 0;
            int playback_step_direction = 0;  // -1: prev keyframe, +1: next keyframe
            int playback_speed_adjust = 0;    // -1: slower, +1: faster
        };

        // Space: play/pause; A/D: step keyframes; W/S: adjust step/play speed; H: toggle sidebar.
        ViewportHotkeyResult HandleViewportHotkeys(bool enable_hotkeys, bool has_playable_trajectory, float dt_sec,
                                                   float play_speed);

        // Reset internal mouse tracking (e.g., after window focus change).
        void ResetMouseTracking();

       private:
        double prev_mouse_x_                             = 0.0;
        double prev_mouse_y_                             = 0.0;
        bool first_mouse_                                = true;
        bool obstacle_pick_left_prev_                    = false;
        bool link_pick_left_prev_                        = false;
        bool link_pick_drag_tracking_                    = false;
        double link_pick_press_x_                        = 0.0;
        double link_pick_press_y_                        = 0.0;
        double link_hover_last_sec_                      = -1.0;
        bool joint_drag_active_                          = false;
        bool joint_drag_suppress_link_pick_              = false;
        bool joint_drag_left_prev_                       = false;
        double joint_drag_last_mouse_x_                  = 0.0;
        double joint_drag_last_mouse_y_                  = 0.0;
        std::string joint_drag_joint_name_;
        std::string joint_drag_link_name_;
        float playback_step_accumulator_                 = 0.0f;
        static constexpr double kLinkHoverIntervalSec    = 0.05;
        static constexpr double kLinkPickDragThresholdPx = 6.0;
        static constexpr float kPlaybackScrubBaseIntervalSec = 0.10f;

        bool IsMouseInViewport(double x, double y, int viewport_w, int viewport_h) const;
    };

}  // namespace kinematic_viewer
