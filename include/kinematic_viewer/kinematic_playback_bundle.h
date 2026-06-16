#pragma once

#include "kinematic_viewer/kinematic_obstacle_state.h"
#include "kinematic_viewer/kinematic_playback_state.h"

#include <string>

namespace kinematic_viewer {

    struct PlaybackBundleSpec {
        std::string trajectory_path;
        std::string obstacles_path;
        bool auto_play = false;
    };

    bool ParsePlaybackBundle(const std::string& path, PlaybackBundleSpec* out, std::string* errorMessage);
    bool ApplyPlaybackBundle(const PlaybackBundleSpec& spec, UserObstacleState* obstacles, DebugPlaybackState* playbackState,
                             std::string* statusMessage, std::string* errorMessage);

}  // namespace kinematic_viewer
