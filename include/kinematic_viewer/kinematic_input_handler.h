#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

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
        CameraInputResult UpdateCamera(teleop_viewer::OrbitCamera* camera, const UpdateContext& ctx);

        // Check for obstacle click-pick in the 3D viewport.
        ObstaclePickResult UpdateObstaclePick(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                              const UserObstacleState& obstacles);

        LinkPickResult UpdateLinkPick(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                      teleop_viewer::RobotScene* scene);

        // Raycast link under cursor (fast proxy pick, throttled). throttle_skip when not refreshed.
        LinkPickResult UpdateLinkHover(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                       teleop_viewer::RobotScene* scene, double now_sec);

        // Handle sidebar page hotkeys (1-9), bounded by visible tab count.
        int HandleSidebarHotkeys(int current_page, int page_count, bool enable_hotkeys);

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
        static constexpr double kLinkHoverIntervalSec    = 0.05;
        static constexpr double kLinkPickDragThresholdPx = 6.0;

        bool IsMouseInViewport(double x, double y, int viewport_w, int viewport_h) const;
    };

}  // namespace kinematic_viewer
