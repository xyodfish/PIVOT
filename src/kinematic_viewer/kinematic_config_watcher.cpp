#include "kinematic_viewer/kinematic_config_watcher.h"

#include <filesystem>

namespace kinematic_viewer {

    ConfigWatcher::ConfigWatcher(std::string yaml_path) : yaml_path_(std::move(yaml_path)) {}

    void ConfigWatcher::SetPollIntervalSec(double seconds) {
        poll_interval_sec_ = std::max(0.1, seconds);
    }

    void ConfigWatcher::SetOnChanged(OnChangedCallback callback) {
        on_changed_ = std::move(callback);
    }

    bool ConfigWatcher::HasValidPath() const {
        return !yaml_path_.empty() && std::filesystem::exists(yaml_path_);
    }

    bool ConfigWatcher::Poll() {
        if (!HasValidPath()) {
            return false;
        }
        auto now         = std::chrono::steady_clock::now();
        auto elapsed_sec = std::chrono::duration<double>(now - last_poll_time_).count();
        if (elapsed_sec < poll_interval_sec_) {
            return false;
        }
        last_poll_time_ = now;
        return CheckAndReload();
    }

    bool ConfigWatcher::ForceReload() {
        if (!HasValidPath()) {
            return false;
        }
        last_poll_time_ = std::chrono::steady_clock::now();
        return CheckAndReload();
    }

    bool ConfigWatcher::CheckAndReload() {
        try {
            auto current_write_time = std::filesystem::last_write_time(yaml_path_);
            if (!initialized_) {
                last_write_time_ = current_write_time;
                initialized_     = true;
                return false;
            }
            if (current_write_time == last_write_time_) {
                return false;
            }
            last_write_time_ = current_write_time;

            bool loaded_ok                = false;
            KinematicViewerConfig new_cfg = KinematicViewerConfig::LoadFromFile(yaml_path_, &loaded_ok);
            if (!loaded_ok) {
                return false;
            }
            if (on_changed_) {
                on_changed_(new_cfg);
            }
            return true;
        } catch (...) {
            return false;
        }
    }

}  // namespace kinematic_viewer
