#include "kinematic_viewer/kinematic_app.h"
#include "kinematic_viewer/kinematic_ui_theme.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>

namespace kinematic_viewer {

    namespace {
        float g_scroll_delta = 0.0f;

        void ScrollCallback(GLFWwindow*, double, double yoffset) {
            g_scroll_delta += static_cast<float>(yoffset);
        }
    }  // namespace

    float GetScrollDelta() {
        float d        = g_scroll_delta;
        g_scroll_delta = 0.0f;
        return d;
    }

    KinematicApp::InitResult KinematicApp::Initialize(const KinematicViewerConfig& cfg) {
        InitResult result;

        if (!glfwInit()) {
            result.error = "glfwInit failed";
            return result;
        }
        glfw_initialized_ = true;

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window_ = glfwCreateWindow(cfg.window.width, cfg.window.height, "Robot Kinematic Debug Viewer", nullptr, nullptr);
        if (!window_) {
            result.error = "create window failed";
            glfwTerminate();
            glfw_initialized_ = false;
            return result;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        glfwSetScrollCallback(window_, ScrollCallback);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            result.error = "glad init failed";
            glfwDestroyWindow(window_);
            window_ = nullptr;
            glfwTerminate();
            glfw_initialized_ = false;
            return result;
        }

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        SetupKinematicViewerFonts(cfg);
        int ui_theme_index = KinematicUiThemeIndexFromName(cfg.ui.theme_preset);
        ApplyKinematicUiStyleByIndex(ui_theme_index);
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init("#version 330");
        imgui_initialized_ = true;

        result.success = true;
        return result;
    }

    void KinematicApp::Shutdown() {
        if (imgui_initialized_) {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
            imgui_initialized_ = false;
        }
        if (window_) {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        if (glfw_initialized_) {
            glfwTerminate();
            glfw_initialized_ = false;
        }
    }

    bool KinematicApp::ShouldClose() const {
        return window_ ? glfwWindowShouldClose(window_) : true;
    }

    void KinematicApp::PollEvents() const {
        if (window_) {
            glfwPollEvents();
        }
    }

    void KinematicApp::SwapBuffers() const {
        if (window_) {
            glfwSwapBuffers(window_);
        }
    }

    void KinematicApp::GetFramebufferSize(int* w, int* h) const {
        if (window_) {
            glfwGetFramebufferSize(window_, w, h);
        } else {
            *w = 0;
            *h = 0;
        }
    }

    void KinematicApp::GetCursorPos(double* x, double* y) const {
        if (window_) {
            glfwGetCursorPos(window_, x, y);
        } else {
            *x = 0.0;
            *y = 0.0;
        }
    }

    bool KinematicApp::IsMouseButtonPressed(int button) const {
        return window_ && glfwGetMouseButton(window_, button) == GLFW_PRESS;
    }

    bool KinematicApp::IsKeyPressed(int key) const {
        return window_ && glfwGetKey(window_, key) == GLFW_PRESS;
    }

}  // namespace kinematic_viewer
