#include "kinematic_viewer/kinematic_playback_bundle.h"

#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_string_utils.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace kinematic_viewer {
    namespace {

        std::string ResolveBundlePath(const std::filesystem::path& base_dir, const std::string& raw_path) {
            if (raw_path.empty()) {
                return {};
            }
            std::filesystem::path path(raw_path);
            if (path.is_absolute()) {
                return NormalizePath(path.string());
            }
            return NormalizePath((base_dir / path).string());
        }

        bool ReadScalarPath(const YAML::Node& root, const char* key, std::string* out) {
            if (out == nullptr || !root[key]) {
                return false;
            }
            const YAML::Node& node = root[key];
            if (!node.IsScalar()) {
                return false;
            }
            *out = node.as<std::string>();
            return !out->empty();
        }

        bool ReadFirstTrajectoryFromConfig(const std::filesystem::path& config_path, std::string* out) {
            if (out == nullptr || !std::filesystem::is_regular_file(config_path)) {
                return false;
            }
            try {
                const YAML::Node root  = YAML::LoadFile(config_path.string());
                const YAML::Node files = root["playback"]["trajectory_files"];
                if (!files || !files.IsSequence() || files.size() == 0) {
                    return false;
                }
                for (std::size_t i = 0; i < files.size(); ++i) {
                    if (!files[i].IsScalar()) {
                        continue;
                    }
                    const std::string rel = files[i].as<std::string>();
                    if (rel.empty()) {
                        continue;
                    }
                    *out = ResolveBundlePath(config_path.parent_path(), rel);
                    return true;
                }
            } catch (const std::exception&) {
                return false;
            }
            return false;
        }

        bool ReadFirstTrajectoryFromDirectory(const std::filesystem::path& dir, std::string* out) {
            if (out == nullptr || !std::filesystem::is_directory(dir)) {
                return false;
            }
            std::error_code ec;
            std::vector<std::filesystem::path> csv_files;
            for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
                if (ec || !entry.is_regular_file(ec)) {
                    continue;
                }
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                if (ext == ".csv") {
                    csv_files.push_back(entry.path());
                }
            }
            if (csv_files.empty()) {
                return false;
            }
            std::sort(csv_files.begin(), csv_files.end());
            *out = NormalizePath(csv_files.front().string());
            return true;
        }

        bool ParsePlaybackBundleFromYamlFile(const std::filesystem::path& manifest_path, PlaybackBundleSpec* out,
                                             std::string* errorMessage) {
            if (out == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：输出参数为空";
                }
                return false;
            }
            try {
                const YAML::Node root = YAML::LoadFile(manifest_path.string());
                if (root["obstacles"] && root["obstacles"].IsSequence()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "该文件是障碍物 YAML，请使用回放离线导入包（需包含 trajectory 字段）";
                    }
                    return false;
                }

                std::string trajectory_raw;
                std::string obstacles_raw;
                if (!ReadScalarPath(root, "trajectory", &trajectory_raw) && !ReadScalarPath(root, "trajectory_file", &trajectory_raw) &&
                    !ReadScalarPath(root, "trajectory_csv", &trajectory_raw)) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "导入包缺少 trajectory 字段";
                    }
                    return false;
                }
                if (!ReadScalarPath(root, "obstacles", &obstacles_raw) && !ReadScalarPath(root, "obstacle_file", &obstacles_raw) &&
                    !ReadScalarPath(root, "obstacles_file", &obstacles_raw)) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "导入包缺少 obstacles 字段";
                    }
                    return false;
                }

                const std::filesystem::path base_dir = manifest_path.parent_path();
                out->trajectory_path                 = ResolveBundlePath(base_dir, trajectory_raw);
                out->obstacles_path                  = ResolveBundlePath(base_dir, obstacles_raw);
                out->auto_play                       = root["auto_play"] ? root["auto_play"].as<bool>() : false;
                return true;
            } catch (const std::exception& e) {
                if (errorMessage != nullptr) {
                    *errorMessage = std::string("解析导入包失败: ") + e.what();
                }
                return false;
            }
        }

        bool ParsePlaybackBundleFromSessionDir(const std::filesystem::path& session_dir, PlaybackBundleSpec* out,
                                               std::string* errorMessage) {
            if (out == nullptr) {
                if (errorMessage != nullptr) {
                    *errorMessage = "内部错误：输出参数为空";
                }
                return false;
            }

            const std::filesystem::path obstacles_path = session_dir / "user_obstacles.yaml";
            if (!std::filesystem::is_regular_file(obstacles_path)) {
                if (errorMessage != nullptr) {
                    *errorMessage = "会话目录缺少 user_obstacles.yaml";
                }
                return false;
            }

            std::string trajectory_path;
            if (!ReadFirstTrajectoryFromConfig(session_dir / "config.yaml", &trajectory_path)) {
                if (!ReadFirstTrajectoryFromDirectory(session_dir / "trajectories", &trajectory_path)) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "会话目录未找到轨迹 CSV（config.yaml 或 trajectories/）";
                    }
                    return false;
                }
            }

            out->trajectory_path = trajectory_path;
            out->obstacles_path  = NormalizePath(obstacles_path.string());
            out->auto_play       = false;
            return true;
        }

        bool TrajectoryPathAlreadyInList(const DebugPlaybackState& playbackState, const std::string& path) {
            const std::string normalized = NormalizePath(path);
            for (const auto& entry : playbackState.trajectory_files) {
                if (NormalizePath(entry.path) == normalized) {
                    return true;
                }
            }
            return false;
        }

        void QueueTrajectoryLoad(DebugPlaybackState* playbackState, const std::string& trajectory_path, bool auto_play) {
            if (playbackState == nullptr) {
                return;
            }
            const std::string normalized = NormalizePath(trajectory_path);
            int target_index             = -1;
            if (!TrajectoryPathAlreadyInList(*playbackState, normalized)) {
                TrajectoryFileEntry entry;
                entry.path   = normalized;
                entry.status = "未加载";
                entry.loaded = false;
                playbackState->trajectory_files.push_back(std::move(entry));
                target_index = static_cast<int>(playbackState->trajectory_files.size()) - 1;
            } else {
                for (int i = 0; i < static_cast<int>(playbackState->trajectory_files.size()); ++i) {
                    if (NormalizePath(playbackState->trajectory_files[static_cast<size_t>(i)].path) == normalized) {
                        target_index = i;
                        break;
                    }
                }
            }
            if (target_index < 0) {
                return;
            }
            playbackState->selected_trajectory_index = target_index;
            std::snprintf(playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path), "%s",
                          playbackState->trajectory_files[static_cast<size_t>(target_index)].path.c_str());
            playbackState->pending_trajectory_load_index      = target_index;
            playbackState->pending_trajectory_play_after_load = auto_play;
        }

    }  // namespace

    bool ParsePlaybackBundle(const std::string& path, PlaybackBundleSpec* out, std::string* errorMessage) {
        if (path.empty()) {
            if (errorMessage != nullptr) {
                *errorMessage = "导入路径为空";
            }
            return false;
        }

        std::error_code ec;
        const std::filesystem::path input_path = NormalizePath(path);
        if (std::filesystem::is_directory(input_path, ec)) {
            return ParsePlaybackBundleFromSessionDir(input_path, out, errorMessage);
        }
        if (!std::filesystem::is_regular_file(input_path, ec)) {
            if (errorMessage != nullptr) {
                *errorMessage = "导入路径不存在: " + input_path.string();
            }
            return false;
        }
        return ParsePlaybackBundleFromYamlFile(input_path, out, errorMessage);
    }

    bool ApplyPlaybackBundle(const PlaybackBundleSpec& spec, UserObstacleState* obstacles, DebugPlaybackState* playbackState,
                             std::string* statusMessage, std::string* errorMessage) {
        if (obstacles == nullptr || playbackState == nullptr) {
            if (errorMessage != nullptr) {
                *errorMessage = "内部错误：状态对象为空";
            }
            return false;
        }
        if (spec.trajectory_path.empty() || spec.obstacles_path.empty()) {
            if (errorMessage != nullptr) {
                *errorMessage = "导入包路径不完整";
            }
            return false;
        }

        std::error_code ec;
        if (!std::filesystem::is_regular_file(spec.trajectory_path, ec)) {
            if (errorMessage != nullptr) {
                *errorMessage = "轨迹文件不存在: " + spec.trajectory_path;
            }
            return false;
        }
        if (!std::filesystem::is_regular_file(spec.obstacles_path, ec)) {
            if (errorMessage != nullptr) {
                *errorMessage = "障碍物文件不存在: " + spec.obstacles_path;
            }
            return false;
        }

        std::vector<UserObstacleItem> loaded_obstacles;
        std::string obstacle_error;
        if (!LoadUserObstaclesFromYaml(spec.obstacles_path, &loaded_obstacles, &obstacle_error)) {
            if (errorMessage != nullptr) {
                *errorMessage = obstacle_error;
            }
            return false;
        }

        obstacles->items          = std::move(loaded_obstacles);
        obstacles->selected_index = obstacles->items.empty() ? -1 : 0;
        UpdateUserObstacleNextSerial(obstacles);

        CancelTrajectorySequence(playbackState);
        QueueTrajectoryLoad(playbackState, spec.trajectory_path, spec.auto_play);

        if (statusMessage != nullptr) {
            *statusMessage = "导入成功: 轨迹=" + spec.trajectory_path + " | 障碍物=" + spec.obstacles_path + " (共 " +
                             std::to_string(obstacles->items.size()) + " 个)";
        }
        return true;
    }

}  // namespace kinematic_viewer
