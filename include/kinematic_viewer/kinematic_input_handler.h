#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

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
        CameraInputResult UpdateCamera(omnilink::teleop_viewer::OrbitCamera* camera, const UpdateContext& ctx);

        // Check for obstacle click-pick in the 3D viewport.
        ObstaclePickResult UpdateObstaclePick(const UpdateContext& ctx, const glm::mat4& view, const glm::mat4& proj,
                                              const UserObstacleState& obstacles);

        // Handle sidebar page hotkeys (1-7).
        int HandleSidebarHotkeys(int current_page, bool enable_hotkeys);

        // Reset internal mouse tracking (e.g., after window focus change).
        void ResetMouseTracking();

       private:
        double prev_mouse_x_          = 0.0;
        double prev_mouse_y_          = 0.0;
        bool first_mouse_             = true;
        bool obstacle_pick_left_prev_ = false;

        bool IsMouseInViewport(double x, double y, int viewport_w, int viewport_h) const;
    };

}  // namespace kinematic_viewer
