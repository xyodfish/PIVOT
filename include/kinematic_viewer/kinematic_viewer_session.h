#pragma once

#include "kinematic_viewer/kinematic_bootstrap.h"

#include <string>

namespace kinematic_viewer {

    class KinematicApp;

    // Owns viewer runtime state, initialization, and the per-frame main loop.
    class KinematicViewerSession {
       public:
        KinematicViewerSession();
        ~KinematicViewerSession();

        KinematicViewerSession(const KinematicViewerSession&)            = delete;
        KinematicViewerSession& operator=(const KinematicViewerSession&) = delete;

        // `app` must already be initialized. Returns false on fatal setup errors.
        bool Initialize(const LaunchConfig& launch, KinematicApp* app, std::string* error_message = nullptr);

        void Run(KinematicApp& app);

       private:
        void TickFrame(KinematicApp& app, double dt_sec, double now_sec, double mouse_x, double mouse_y);

        struct FrameContext;
        struct Impl;
        Impl* impl_ = nullptr;
    };

}  // namespace kinematic_viewer
