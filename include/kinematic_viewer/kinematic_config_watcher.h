#pragma once

#include "kinematic_viewer/kinematic_viewer_config.h"

#include <chrono>
#include <filesystem>
#include <functional>
#include <string>

namespace kinematic_viewer {

    // Monitors a YAML config file for changes and invokes a callback when modified.
    // Uses filesystem last_write_time with a configurable polling interval.
    class ConfigWatcher {
       public:
        using OnChangedCallback = std::function<void(const KinematicViewerConfig&)>;

        explicit ConfigWatcher(std::string yaml_path);

        // Set the minimum interval between file checks (default: 2 seconds).
        void SetPollIntervalSec(double seconds);

        // Register a callback invoked when the file changes.
        void SetOnChanged(OnChangedCallback callback);

        // Call once per frame or on a timer. Returns true if a change was detected
        // and the callback was invoked.
        bool Poll();

        // Force an immediate reload and callback invocation.
        bool ForceReload();

        const std::string& Path() const { return yaml_path_; }
        bool HasValidPath() const;

       private:
        std::string yaml_path_;
        OnChangedCallback on_changed_;
        double poll_interval_sec_ = 2.0;

        std::chrono::steady_clock::time_point last_poll_time_;
        std::filesystem::file_time_type last_write_time_;
        bool initialized_ = false;

        bool CheckAndReload();
    };

}  // namespace kinematic_viewer
