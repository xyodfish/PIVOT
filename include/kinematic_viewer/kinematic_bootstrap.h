#pragma once

#include "kinematic_viewer/kinematic_viewer_config.h"

#include <string>

namespace kinematic_viewer {

    struct LaunchConfig {
        KinematicViewerConfig config;
        std::string configPath;
        std::string urdfPath;
    };

    bool LoadLaunchConfigFromArgs(int argc, char** argv, LaunchConfig* out, std::string* errorMessage);

}  // namespace kinematic_viewer
