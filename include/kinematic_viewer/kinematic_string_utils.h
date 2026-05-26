#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace kinematic_viewer {

    // ------------------------------------------------------------------
    // String utilities
    // ------------------------------------------------------------------

    std::string LowerString(std::string s);

    bool EndsWithCaseInsensitive(const std::string& value, const std::string& suffix);

    std::string LowerFileExtension(const std::string& path);

    // ------------------------------------------------------------------
    // Path utilities
    // ------------------------------------------------------------------

    std::string NormalizePath(const std::string& path);

    // ------------------------------------------------------------------
    // Pose input / output (x,y,z,qx,qy,qz,qw)
    // ------------------------------------------------------------------

    bool ParsePoseInputXyzQuat(const char* text, glm::vec3* out_pos, glm::quat* out_quat, std::string* out_error);

    std::string FormatPoseInputXyzQuat(const glm::vec3& pos, const glm::quat& quat);

    // Planar base: x,y,yaw (yaw in radians unless yaw_is_deg is true).
    bool ParsePoseInputXyYaw(const char* text, float* out_x, float* out_y, float* out_yaw, bool yaw_is_deg,
                             std::string* out_error);

    std::string FormatPoseInputXyYaw(float x, float y, float yaw_rad, bool yaw_as_deg);

    // ------------------------------------------------------------------
    // File browser helpers
    // ------------------------------------------------------------------

    enum class FileBrowserSortBy {
        NameAsc,
        NameDesc,
        SizeAsc,
        SizeDesc,
        TimeAsc,
        TimeDesc,
    };

    struct FileBrowserEntry {
        std::string name;
        std::string size;
        std::string mtime;
        bool is_directory       = false;
        std::uintmax_t raw_size = 0;
        std::filesystem::file_time_type raw_mtime;
    };

    std::string FormatFileSize(std::uintmax_t bytes);

    std::string FormatFileTime(std::filesystem::file_time_type ftime);

    std::vector<FileBrowserEntry> ScanDirectoryForBrowser(const std::string& dir_path, bool (*filter)(const std::filesystem::path&),
                                                          FileBrowserSortBy sort_by = FileBrowserSortBy::NameAsc);

}  // namespace kinematic_viewer
