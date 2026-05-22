#include "kinematic_viewer/kinematic_string_utils.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace kinematic_viewer {

    // ------------------------------------------------------------------
    // String utilities
    // ------------------------------------------------------------------

    std::string LowerString(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return s;
    }

    bool EndsWithCaseInsensitive(const std::string& value, const std::string& suffix) {
        if (value.size() < suffix.size()) {
            return false;
        }
        const std::string left  = LowerString(value.substr(value.size() - suffix.size()));
        const std::string right = LowerString(suffix);
        return left == right;
    }

    std::string LowerFileExtension(const std::string& path) {
        std::filesystem::path p(path);
        std::string ext = p.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return ext;
    }

    // ------------------------------------------------------------------
    // Path utilities
    // ------------------------------------------------------------------

    std::string NormalizePath(const std::string& path) {
        std::error_code ec;
        std::filesystem::path p(path);
        auto normalized = std::filesystem::weakly_canonical(p, ec);
        if (!ec) {
            return normalized.string();
        }
        return p.lexically_normal().string();
    }

    // ------------------------------------------------------------------
    // Pose input / output
    // ------------------------------------------------------------------

    bool ParsePoseInputXyzQuat(const char* text, glm::vec3* out_pos, glm::quat* out_quat, std::string* out_error) {
        if (text == nullptr || out_pos == nullptr || out_quat == nullptr) {
            if (out_error != nullptr) {
                *out_error = "输入为空";
            }
            return false;
        }
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
        int consumed      = 0;
        const int matched = std::sscanf(text, " %f , %f , %f , %f , %f , %f , %f %n", &x, &y, &z, &qx, &qy, &qz, &qw, &consumed);
        if (matched != 7) {
            if (out_error != nullptr) {
                *out_error = "格式错误，应为 x,y,z,qx,qy,qz,qw";
            }
            return false;
        }
        if (text[consumed] != '\0') {
            if (out_error != nullptr) {
                *out_error = "格式错误：包含多余字符";
            }
            return false;
        }

        const glm::quat q_in(qw, qx, qy, qz);
        const float norm = glm::length(q_in);
        if (norm < 1e-6f) {
            if (out_error != nullptr) {
                *out_error = "四元数范数过小";
            }
            return false;
        }
        if (std::fabs(norm - 1.0f) > 1e-3f) {
            if (out_error != nullptr) {
                char buf[128];
                std::snprintf(buf, sizeof(buf), "四元数未归一化，当前范数=%.6f", norm);
                *out_error = buf;
            }
            return false;
        }

        *out_pos  = glm::vec3(x, y, z);
        *out_quat = glm::normalize(q_in);
        return true;
    }

    std::string FormatPoseInputXyzQuat(const glm::vec3& pos, const glm::quat& quat) {
        const glm::quat q = glm::normalize(quat);
        char buf[196];
        std::snprintf(buf, sizeof(buf), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w);
        return std::string(buf);
    }

    // ------------------------------------------------------------------
    // File browser helpers
    // ------------------------------------------------------------------

    std::string FormatFileSize(std::uintmax_t bytes) {
        if (bytes < 1024) {
            return std::to_string(bytes) + " B";
        } else if (bytes < 1024 * 1024) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << (bytes / 1024.0) << " KB";
            return ss.str();
        } else {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << (bytes / (1024.0 * 1024.0)) << " MB";
            return ss.str();
        }
    }

    std::string FormatFileTime(std::filesystem::file_time_type ftime) {
        auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
            ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
        std::time_t tt = std::chrono::system_clock::to_time_t(sctp);
        std::tm* tm    = std::localtime(&tt);
        if (tm == nullptr) {
            return "-";
        }
        std::stringstream ss;
        ss << std::put_time(tm, "%Y-%m-%d %H:%M");
        return ss.str();
    }

    std::vector<FileBrowserEntry> ScanDirectoryForBrowser(const std::string& dir_path, bool (*filter)(const std::filesystem::path&),
                                                          FileBrowserSortBy sort_by) {
        std::vector<FileBrowserEntry> entries;
        std::error_code ec;

        // Parent directory entry
        {
            FileBrowserEntry parent;
            parent.name         = "..";
            parent.size         = "-";
            parent.mtime        = "-";
            parent.is_directory = true;
            entries.push_back(parent);
        }

        for (const auto& entry : std::filesystem::directory_iterator(dir_path, ec)) {
            if (ec) {
                continue;
            }
            const auto& p = entry.path();
            if (filter != nullptr && !filter(p)) {
                continue;
            }

            FileBrowserEntry e;
            e.name         = p.filename().string();
            e.is_directory = entry.is_directory(ec);

            if (!e.is_directory) {
                auto fsize  = entry.file_size(ec);
                auto ftime  = entry.last_write_time(ec);
                e.size      = ec ? "-" : FormatFileSize(fsize);
                e.mtime     = ec ? "-" : FormatFileTime(ftime);
                e.raw_size  = ec ? 0 : fsize;
                e.raw_mtime = ec ? std::filesystem::file_time_type{} : ftime;
            } else {
                e.size      = "<DIR>";
                e.mtime     = "-";
                e.raw_size  = 0;
                e.raw_mtime = std::filesystem::file_time_type{};
            }
            entries.push_back(e);
        }

        // Sort: directories first, then by selected criteria
        std::sort(entries.begin(), entries.end(), [sort_by](const FileBrowserEntry& a, const FileBrowserEntry& b) {
            if (a.is_directory != b.is_directory) {
                return a.is_directory > b.is_directory;
            }
            switch (sort_by) {
                case FileBrowserSortBy::SizeAsc:
                    if (a.raw_size != b.raw_size) {
                        return a.raw_size < b.raw_size;
                    }
                    break;
                case FileBrowserSortBy::SizeDesc:
                    if (a.raw_size != b.raw_size) {
                        return a.raw_size > b.raw_size;
                    }
                    break;
                case FileBrowserSortBy::TimeAsc:
                    if (a.raw_mtime != b.raw_mtime) {
                        return a.raw_mtime < b.raw_mtime;
                    }
                    break;
                case FileBrowserSortBy::TimeDesc:
                    if (a.raw_mtime != b.raw_mtime) {
                        return a.raw_mtime > b.raw_mtime;
                    }
                    break;
                case FileBrowserSortBy::NameDesc:
                    return a.name > b.name;
                case FileBrowserSortBy::NameAsc:
                default:
                    break;
            }
            return a.name < b.name;
        });

        return entries;
    }

}  // namespace kinematic_viewer
