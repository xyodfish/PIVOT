#include "kinematic_viewer/kinematic_app.h"
#include "kinematic_viewer/kinematic_bootstrap.h"
#include "kinematic_viewer/kinematic_viewer_session.h"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    kinematic_viewer::LaunchConfig launch;
    std::string launch_error;
    if (!kinematic_viewer::LoadLaunchConfigFromArgs(argc, argv, &launch, &launch_error)) {
        if (!launch_error.empty()) {
            std::cerr << launch_error << std::endl;
        }
        return 1;
    }

    kinematic_viewer::KinematicApp app;
    {
        auto init_result = app.Initialize(launch.config);
        if (!init_result.success) {
            std::cerr << init_result.error << "\n";
            return 1;
        }
    }

    kinematic_viewer::KinematicViewerSession session;
    std::string session_error;
    if (!session.Initialize(launch, &app, &session_error)) {
        if (!session_error.empty()) {
            std::cerr << session_error << "\n";
        }
        return 1;
    }

    session.Run(app);
    return 0;
}
