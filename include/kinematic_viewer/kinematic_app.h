#pragma once

#include "kinematic_viewer/kinematic_viewer_config.h"

#include <string>

// Forward declaration to avoid including GLFW header in this public header.
struct GLFWwindow;

namespace kinematic_viewer {

    // Scroll delta helper (managed internally by KinematicApp)
    float GetScrollDelta();

    // Encapsulates GLFW window, OpenGL context, and ImGui initialization/shutdown.
    class KinematicApp {
       public:
        struct InitResult {
            bool success = false;
            std::string error;
        };

        KinematicApp() = default;
        ~KinematicApp() { Shutdown(); }

        // Non-copyable, non-movable
        KinematicApp(const KinematicApp&)            = delete;
        KinematicApp& operator=(const KinematicApp&) = delete;

        // Initialize GLFW, OpenGL, ImGui. Returns success/failure.
        InitResult Initialize(const KinematicViewerConfig& cfg);

        // Shutdown in reverse order
        void Shutdown();

        bool ShouldClose() const;
        void PollEvents() const;
        void SwapBuffers() const;
        void GetFramebufferSize(int* w, int* h) const;
        void GetCursorPos(double* x, double* y) const;
        bool IsMouseButtonPressed(int button) const;
        bool IsKeyPressed(int key) const;

        GLFWwindow* Window() const { return window_; }

       private:
        GLFWwindow* window_     = nullptr;
        bool imgui_initialized_ = false;
        bool glfw_initialized_  = false;
    };

}  // namespace kinematic_viewer
