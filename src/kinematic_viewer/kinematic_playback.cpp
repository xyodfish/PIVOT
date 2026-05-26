#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_string_utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace kinematic_viewer {
    namespace kinematic_playback_internal {

        using kinematic_viewer::LowerString;

        std::string NormalizeTrajectoryJointName(const std::string& raw_name) {
            static const std::unordered_map<std::string, std::string> kAliases = {
                {"leg1", "leg_joint1"},
                {"leg2", "leg_joint2"},
                {"leg3", "leg_joint3"},
                {"leg4", "leg_joint4"},
                {"leg5", "leg_joint5"},
                {"head1", "head_joint1"},
                {"head2", "head_joint2"},
                {"left_arm1", "left_arm_joint1"},
                {"left_arm2", "left_arm_joint2"},
                {"left_arm3", "left_arm_joint3"},
                {"left_arm4", "left_arm_joint4"},
                {"left_arm5", "left_arm_joint5"},
                {"left_arm6", "left_arm_joint6"},
                {"left_arm7", "left_arm_joint7"},
                {"right_arm1", "right_arm_joint1"},
                {"right_arm2", "right_arm_joint2"},
                {"right_arm3", "right_arm_joint3"},
                {"right_arm4", "right_arm_joint4"},
                {"right_arm5", "right_arm_joint5"},
                {"right_arm6", "right_arm_joint6"},
                {"right_arm7", "right_arm_joint7"},
            };
            const std::string key = LowerString(raw_name);
            const auto it         = kAliases.find(key);
            if (it != kAliases.end()) {
                return it->second;
            }
            return raw_name;
        }

        bool IsSkippedCsvColumn(const std::string& key_lower) {
            return key_lower == "idx" || key_lower == "id";
        }

        DebugPlaybackState::Mode NextPausedOrPlaying(DebugPlaybackState::Mode mode) {
            if (mode == DebugPlaybackState::Mode::Playing) {
                return DebugPlaybackState::Mode::Paused;
            }
            return DebugPlaybackState::Mode::Playing;
        }

        float WrapPhase(float phase) {
            const float twoPi = 6.283185307f;
            while (phase > twoPi) {
                phase -= twoPi;
            }
            while (phase < 0.0f) {
                phase += twoPi;
            }
            return phase;
        }

        std::vector<std::string> CollectJointNames(const DebugPlaybackState& playbackState) {
            std::vector<std::string> names;
            for (const auto& keyframe : playbackState.keyframes) {
                for (const auto& [jointName, _] : keyframe.joints) {
                    if (std::find(names.begin(), names.end(), jointName) == names.end()) {
                        names.push_back(jointName);
                    }
                }
            }

            // Preferred order for readability in trajectory csv:
            // leg -> head -> left_arm -> right_arm, with known robot-specific joint orders.
            const std::array<std::string, 12> leg_priority = {
                "left_hip_pitch_joint",   "left_hip_roll_joint",   "left_hip_yaw_joint",      "left_knee_joint",
                "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",   "right_hip_roll_joint",
                "right_hip_yaw_joint",    "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
            };
            const std::array<std::string, 3> head_priority = {
                "waist_yaw_joint",
                "waist_roll_joint",
                "waist_pitch_joint",
            };
            const std::array<std::string, 7> left_arm_priority = {
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
                "left_wrist_roll_joint",     "left_wrist_pitch_joint",   "left_wrist_yaw_joint",
            };
            const std::array<std::string, 7> right_arm_priority = {
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
                "right_wrist_roll_joint",     "right_wrist_pitch_joint",   "right_wrist_yaw_joint",
            };

            auto containsAny = [](const std::string& source, const std::initializer_list<const char*> tokens) {
                for (const char* token : tokens) {
                    if (source.find(token) != std::string::npos) {
                        return true;
                    }
                }
                return false;
            };

            auto groupRank = [&](const std::string& jointName) {
                const std::string lower = LowerString(jointName);
                if (containsAny(lower, {"leg_", "hip_", "knee_", "ankle_"})) {
                    return 0;
                }
                if (containsAny(lower, {"head_", "waist_"})) {
                    return 1;
                }
                if (containsAny(lower, {"left_arm_", "left_shoulder_", "left_elbow_", "left_wrist_"})) {
                    return 2;
                }
                if (containsAny(lower, {"right_arm_", "right_shoulder_", "right_elbow_", "right_wrist_"})) {
                    return 3;
                }
                return 4;
            };

            auto priorityIndex = [&](const std::string& jointName, const auto& arr) {
                for (size_t i = 0; i < arr.size(); ++i) {
                    if (arr[i] == jointName) {
                        return static_cast<int>(i);
                    }
                }
                return 1000000;
            };

            std::stable_sort(names.begin(), names.end(), [&](const std::string& a, const std::string& b) {
                const int ga = groupRank(a);
                const int gb = groupRank(b);
                if (ga != gb) {
                    return ga < gb;
                }

                if (ga == 0) {
                    const int ia = priorityIndex(a, leg_priority);
                    const int ib = priorityIndex(b, leg_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                } else if (ga == 1) {
                    const int ia = priorityIndex(a, head_priority);
                    const int ib = priorityIndex(b, head_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                } else if (ga == 2) {
                    const int ia = priorityIndex(a, left_arm_priority);
                    const int ib = priorityIndex(b, left_arm_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                } else if (ga == 3) {
                    const int ia = priorityIndex(a, right_arm_priority);
                    const int ib = priorityIndex(b, right_arm_priority);
                    if (ia != ib) {
                        return ia < ib;
                    }
                }

                return LowerString(a) < LowerString(b);
            });
            return names;
        }

        std::string Trim(const std::string& input) {
            size_t begin = 0;
            while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin])) != 0) {
                ++begin;
            }
            size_t end = input.size();
            while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
                --end;
            }
            return input.substr(begin, end - begin);
        }

        std::vector<std::string> SplitCsvLine(const std::string& line) {
            std::vector<std::string> out;
            out.reserve(64);  // Typical trajectory has <64 columns
            size_t start = 0;
            while (start <= line.size()) {
                size_t end = line.find(',', start);
                if (end == std::string::npos) {
                    end = line.size();
                }
                // Inline trim to avoid extra string copy
                size_t t_begin = start;
                while (t_begin < end && std::isspace(static_cast<unsigned char>(line[t_begin])) != 0) {
                    ++t_begin;
                }
                size_t t_end = end;
                while (t_end > t_begin && std::isspace(static_cast<unsigned char>(line[t_end - 1])) != 0) {
                    --t_end;
                }
                out.emplace_back(line.substr(t_begin, t_end - t_begin));
                start = end + 1;
            }
            return out;
        }

        std::string LowerFileExtension(const std::string& path) {
            std::filesystem::path p(path);
            std::string ext = p.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            return ext;
        }

    }  // namespace kinematic_playback_internal

    void LinearTrajectoryInterpolator::SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec,
                                                      int* currentSegmentIndex, teleop_viewer::RobotScene* scene) const {
        auto lerpAngleRad = [](float a0, float a1, float alpha) {
            const float delta = std::atan2(std::sin(a1 - a0), std::cos(a1 - a0));
            return a0 + delta * alpha;
        };

        if (scene == nullptr || playbackState.keyframes.empty()) {
            return;
        }

        if (playbackState.keyframes.size() == 1) {
            const auto& only = playbackState.keyframes.front();
            for (const auto& [jointName, value] : only.joints) {
                scene->setJointPositionByName(jointName, value);
            }
            if (only.has_base_pose_2d) {
                scene->setVirtualBasePose2D(only.base_x_m, only.base_y_m, only.base_yaw_rad);
            }
            if (currentSegmentIndex != nullptr) {
                *currentSegmentIndex = 0;
            }
            return;
        }

        size_t hi = 1;
        while (hi < playbackState.keyframes.size() && static_cast<float>(playbackState.keyframes[hi].t) < sampleTimeSec) {
            ++hi;
        }
        size_t lo = (hi == 0) ? 0 : (hi - 1);
        hi        = std::min(hi, playbackState.keyframes.size() - 1);

        const auto& k0 = playbackState.keyframes[lo];
        const auto& k1 = playbackState.keyframes[hi];
        const float t0 = static_cast<float>(k0.t);
        const float t1 = static_cast<float>(k1.t);
        float alpha    = (t1 > t0 + 1e-6f) ? ((sampleTimeSec - t0) / (t1 - t0)) : 0.0f;
        alpha          = std::clamp(alpha, 0.0f, 1.0f);

        for (const auto& [jointName, value0] : k0.joints) {
            auto it1 = k1.joints.find(jointName);
            if (it1 == k1.joints.end()) {
                continue;
            }
            float value = value0 * (1.0f - alpha) + it1->second * alpha;
            scene->setJointPositionByName(jointName, value);
        }
        if (k0.has_base_pose_2d && k1.has_base_pose_2d) {
            const float x_m = k0.base_x_m * (1.0f - alpha) + k1.base_x_m * alpha;
            const float y_m = k0.base_y_m * (1.0f - alpha) + k1.base_y_m * alpha;
            const float yaw = lerpAngleRad(k0.base_yaw_rad, k1.base_yaw_rad, alpha);
            scene->setVirtualBasePose2D(x_m, y_m, yaw);
        } else if (k0.has_base_pose_2d) {
            scene->setVirtualBasePose2D(k0.base_x_m, k0.base_y_m, k0.base_yaw_rad);
        } else if (k1.has_base_pose_2d) {
            scene->setVirtualBasePose2D(k1.base_x_m, k1.base_y_m, k1.base_yaw_rad);
        }

        if (currentSegmentIndex != nullptr) {
            *currentSegmentIndex = static_cast<int>(lo);
        }
    }

    TrajectoryPlayer::TrajectoryPlayer() : interpolator_(std::make_unique<LinearTrajectoryInterpolator>()) {}

    void TrajectoryPlayer::SetInterpolator(std::unique_ptr<TrajectoryInterpolator> interpolator) {
        if (interpolator) {
            interpolator_ = std::move(interpolator);
        }
    }

    void TrajectoryPlayer::RecordKeyframe(DebugPlaybackState* playbackState,
                                          const std::vector<teleop_viewer::RobotScene::JointInfo>& joints,
                                          const teleop_viewer::RobotScene& scene) const {
        if (playbackState == nullptr) {
            return;
        }

        PoseKeyframe keyframe;
        if (!playbackState->keyframes.empty()) {
            keyframe.t = playbackState->keyframes.back().t + std::max(0.02f, playbackState->keyframe_interval_sec);
        }
        for (const auto& joint : joints) {
            if (!joint.revolute) {
                continue;
            }
            keyframe.joints[joint.name] = joint.position;
        }
        float base_x_m     = 0.0f;
        float base_y_m     = 0.0f;
        float base_yaw_rad = 0.0f;
        if (scene.getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw_rad)) {
            keyframe.has_base_pose_2d = true;
            keyframe.base_x_m         = base_x_m;
            keyframe.base_y_m         = base_y_m;
            keyframe.base_yaw_rad     = base_yaw_rad;
        }
        playbackState->keyframes.push_back(std::move(keyframe));
        playbackState->selected_keyframe_index = static_cast<int>(playbackState->keyframes.size()) - 1;
        playbackState->play_time               = static_cast<float>(playbackState->keyframes.back().t);
    }

    void TrajectoryPlayer::RemoveSelectedKeyframe(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr || playbackState->keyframes.empty()) {
            return;
        }
        const int index = playbackState->selected_keyframe_index;
        if (index < 0 || index >= static_cast<int>(playbackState->keyframes.size())) {
            return;
        }
        playbackState->keyframes.erase(playbackState->keyframes.begin() + index);
        if (playbackState->keyframes.empty()) {
            playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
            playbackState->selected_keyframe_index = -1;
            playbackState->play_time               = 0.0f;
            playbackState->current_segment_index   = -1;
            return;
        }
        playbackState->selected_keyframe_index = std::clamp(index, 0, static_cast<int>(playbackState->keyframes.size()) - 1);
        playbackState->play_time               = std::min(playbackState->play_time, TotalDuration(*playbackState));
    }

    void TrajectoryPlayer::Clear(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr) {
            return;
        }
        playbackState->keyframes.clear();
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->play_time               = 0.0f;
        playbackState->selected_keyframe_index = -1;
        playbackState->current_segment_index   = -1;
    }

    void TrajectoryPlayer::TogglePlayPause(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr || !HasPlayableTrajectory(*playbackState)) {
            return;
        }
        if (playbackState->mode == DebugPlaybackState::Mode::Stopped) {
            playbackState->play_time = 0.0f;
            playbackState->mode      = DebugPlaybackState::Mode::Playing;
            return;
        }
        playbackState->mode = kinematic_playback_internal::NextPausedOrPlaying(playbackState->mode);
    }

    void TrajectoryPlayer::Stop(DebugPlaybackState* playbackState) const {
        if (playbackState == nullptr) {
            return;
        }
        playbackState->mode      = DebugPlaybackState::Mode::Stopped;
        playbackState->play_time = 0.0f;
    }

    void TrajectoryPlayer::AdvanceAndApply(DebugPlaybackState* playbackState, teleop_viewer::RobotScene* scene,
                                           double dtSec) const {
        if (playbackState == nullptr || scene == nullptr || !interpolator_) {
            return;
        }
        if (playbackState->mode != DebugPlaybackState::Mode::Playing || !HasPlayableTrajectory(*playbackState)) {
            return;
        }

        const float totalDuration = TotalDuration(*playbackState);
        if (totalDuration <= 1e-6f) {
            playbackState->mode = DebugPlaybackState::Mode::Stopped;
            return;
        }

        playbackState->play_time += static_cast<float>(dtSec) * playbackState->play_speed;
        if (playbackState->loop) {
            while (playbackState->play_time > totalDuration) {
                playbackState->play_time -= totalDuration;
            }
        } else if (playbackState->play_time > totalDuration) {
            playbackState->play_time = totalDuration;
            playbackState->mode      = DebugPlaybackState::Mode::Paused;
        }

        interpolator_->SampleAndApply(*playbackState, playbackState->play_time, &playbackState->current_segment_index, scene);
    }

    void TrajectoryPlayer::SampleAtCurrentTime(const DebugPlaybackState& playbackState, teleop_viewer::RobotScene* scene) const {
        if (scene == nullptr || !interpolator_ || playbackState.keyframes.empty()) {
            return;
        }
        int segmentIndex = -1;
        interpolator_->SampleAndApply(playbackState, playbackState.play_time, &segmentIndex, scene);
    }

    float TrajectoryPlayer::TotalDuration(const DebugPlaybackState& playbackState) {
        if (playbackState.keyframes.empty()) {
            return 0.0f;
        }
        return static_cast<float>(playbackState.keyframes.back().t);
    }

    bool TrajectoryPlayer::HasPlayableTrajectory(const DebugPlaybackState& playbackState) {
        return playbackState.keyframes.size() >= 2;
    }

    bool LoadTrajectoryFromCsv(const std::string& csvPath, DebugPlaybackState* playbackState, std::string* errorMessage) {
        if (playbackState == nullptr) {
            if (errorMessage != nullptr) {
                *errorMessage = "playbackState is null";
            }
            return false;
        }
        std::ifstream file(csvPath);
        if (!file.good()) {
            if (errorMessage != nullptr) {
                *errorMessage = "open csv failed";
            }
            return false;
        }

        std::string line;
        std::vector<std::string> header;
        while (std::getline(file, line)) {
            const std::string stripped = kinematic_playback_internal::Trim(line);
            if (stripped.empty() || stripped[0] == '#') {
                continue;
            }
            header = kinematic_playback_internal::SplitCsvLine(stripped);
            break;
        }
        if (header.size() < 2) {
            if (errorMessage != nullptr) {
                *errorMessage = "csv header invalid, expected: time,joint1,...";
            }
            return false;
        }

        int time_col = -1;
        for (size_t i = 0; i < header.size(); ++i) {
            if (header[i].empty()) {
                continue;
            }
            const std::string key = kinematic_playback_internal::LowerString(header[i]);
            if (key == "time" || key == "t" || key == "timestamp") {
                time_col = static_cast<int>(i);
                break;
            }
        }
        if (time_col < 0) {
            if (errorMessage != nullptr) {
                *errorMessage = "csv missing time column (time/t/timestamp)";
            }
            return false;
        }

        std::vector<std::pair<std::string, size_t>> jointColumns;
        jointColumns.reserve(header.size() - 1);
        int base_x_col    = -1;
        int base_y_col    = -1;
        int base_yaw_col  = -1;
        bool base_yaw_deg = false;
        for (size_t i = 0; i < header.size(); ++i) {
            if (static_cast<int>(i) == time_col || header[i].empty()) {
                continue;
            }
            const std::string key = kinematic_playback_internal::LowerString(header[i]);
            if (kinematic_playback_internal::IsSkippedCsvColumn(key)) {
                continue;
            }
            if (key == "chassis_x" || key == "base_x" || key == "base_x_m" || key == "mobile_x") {
                base_x_col = static_cast<int>(i);
            } else if (key == "chassis_y" || key == "base_y" || key == "base_y_m" || key == "mobile_y") {
                base_y_col = static_cast<int>(i);
            } else if (key == "chassis_yaw_deg" || key == "base_yaw_deg") {
                base_yaw_col = static_cast<int>(i);
                base_yaw_deg = true;
            } else if (key == "chassis_yaw" || key == "chassis_z" || key == "base_yaw" || key == "base_yaw_rad" || key == "mobile_yaw") {
                base_yaw_col = static_cast<int>(i);
                base_yaw_deg = false;
            } else {
                jointColumns.push_back({kinematic_playback_internal::NormalizeTrajectoryJointName(header[i]), i});
            }
        }
        if (jointColumns.empty()) {
            if (errorMessage != nullptr) {
                *errorMessage = "csv joints empty";
            }
            return false;
        }
        const bool has_base_cols = (base_x_col >= 0 && base_y_col >= 0 && base_yaw_col >= 0);

        std::vector<PoseKeyframe> loaded;
        loaded.reserve(4096);  // Pre-allocate for typical trajectory sizes
        while (std::getline(file, line)) {
            const std::string stripped = kinematic_playback_internal::Trim(line);
            if (stripped.empty() || stripped[0] == '#') {
                continue;
            }
            std::vector<std::string> cells = kinematic_playback_internal::SplitCsvLine(stripped);
            PoseKeyframe keyframe;
            bool row_ok = true;
            try {
                if (static_cast<size_t>(time_col) >= cells.size() || cells[time_col].empty()) {
                    row_ok = false;
                } else {
                    keyframe.t = std::stod(cells[time_col]);
                }
                for (const auto& [joint_name, col_idx] : jointColumns) {
                    if (col_idx >= cells.size() || cells[col_idx].empty()) {
                        row_ok = false;
                        break;
                    }
                    keyframe.joints[joint_name] = static_cast<float>(std::stod(cells[col_idx]));
                }
            } catch (const std::exception&) {
                row_ok = false;
            }
            if (!row_ok) {
                continue;
            }

            if (has_base_cols && static_cast<size_t>(base_x_col) < cells.size() && static_cast<size_t>(base_y_col) < cells.size() &&
                static_cast<size_t>(base_yaw_col) < cells.size() && !cells[base_x_col].empty() && !cells[base_y_col].empty() &&
                !cells[base_yaw_col].empty()) {
                try {
                    float base_x_m = static_cast<float>(std::stod(cells[base_x_col]));
                    float base_y_m = static_cast<float>(std::stod(cells[base_y_col]));
                    float base_yaw = static_cast<float>(std::stod(cells[base_yaw_col]));
                    if (base_yaw_deg) {
                        base_yaw *= 0.017453292519943295f;
                    }
                    keyframe.has_base_pose_2d = true;
                    keyframe.base_x_m         = base_x_m;
                    keyframe.base_y_m         = base_y_m;
                    keyframe.base_yaw_rad     = base_yaw;
                } catch (const std::exception&) {}
            }
            loaded.push_back(std::move(keyframe));
        }

        std::sort(loaded.begin(), loaded.end(), [](const PoseKeyframe& a, const PoseKeyframe& b) { return a.t < b.t; });
        playbackState->keyframes               = std::move(loaded);
        playbackState->selected_keyframe_index = playbackState->keyframes.empty() ? -1 : 0;
        playbackState->current_segment_index   = -1;
        playbackState->play_time               = 0.0f;
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->timeline_edited_this_ui = false;
        if (errorMessage != nullptr) {
            *errorMessage = "";
        }
        return !playbackState->keyframes.empty();
    }

    bool LoadTrajectoryFromFile(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage) {
        const std::string ext = kinematic_playback_internal::LowerFileExtension(path);
        if (ext == ".csv") {
            return LoadTrajectoryFromCsv(path, playbackState, errorMessage);
        }
        if (errorMessage != nullptr) {
            *errorMessage = "unsupported trajectory extension: " + ext + " (expected .csv)";
        }
        return false;
    }

    bool SaveTrajectoryToFile(const std::string& path, const DebugPlaybackState& playbackState, std::string* errorMessage) {
        const std::string ext = kinematic_playback_internal::LowerFileExtension(path);
        if (ext == ".csv") {
            try {
                const std::vector<std::string> jointNames = kinematic_playback_internal::CollectJointNames(playbackState);
                const bool hasAnyBase2d                   = std::any_of(playbackState.keyframes.begin(), playbackState.keyframes.end(),
                                                                        [](const PoseKeyframe& keyframe) { return keyframe.has_base_pose_2d; });
                std::ofstream file(path);
                if (!file.good()) {
                    if (errorMessage != nullptr) {
                        *errorMessage = "open csv failed";
                    }
                    return false;
                }

                file << "time";
                for (const auto& jointName : jointNames) {
                    file << "," << jointName;
                }
                if (hasAnyBase2d) {
                    file << ",chassis_x,chassis_y,chassis_yaw";
                }
                file << "\n";

                std::unordered_map<std::string, float> lastValues;
                bool lastBaseValid = false;
                float lastBaseX    = 0.0f;
                float lastBaseY    = 0.0f;
                float lastBaseYaw  = 0.0f;
                for (const auto& keyframe : playbackState.keyframes) {
                    file << keyframe.t;
                    for (const auto& jointName : jointNames) {
                        auto it = keyframe.joints.find(jointName);
                        if (it != keyframe.joints.end()) {
                            lastValues[jointName] = it->second;
                        }
                        float value = 0.0f;
                        auto last   = lastValues.find(jointName);
                        if (last != lastValues.end()) {
                            value = last->second;
                        }
                        file << "," << value;
                    }
                    if (hasAnyBase2d) {
                        if (keyframe.has_base_pose_2d) {
                            lastBaseValid = true;
                            lastBaseX     = keyframe.base_x_m;
                            lastBaseY     = keyframe.base_y_m;
                            lastBaseYaw   = keyframe.base_yaw_rad;
                        }
                        if (lastBaseValid) {
                            file << "," << lastBaseX << "," << lastBaseY << "," << lastBaseYaw;
                        } else {
                            file << ",0,0,0";
                        }
                    }
                    file << "\n";
                }
                file.close();
                if (errorMessage != nullptr) {
                    *errorMessage = "";
                }
                return true;
            } catch (const std::exception& e) {
                if (errorMessage != nullptr) {
                    *errorMessage = e.what();
                }
                return false;
            }
        }
        if (errorMessage != nullptr) {
            *errorMessage = "unsupported trajectory extension: " + ext + " (expected .csv)";
        }
        return false;
    }

    void BuildDemoTrajectoryFromCurrentPose(DebugPlaybackState* playbackState,
                                            const std::vector<teleop_viewer::RobotScene::JointInfo>& joints,
                                            const teleop_viewer::RobotScene& scene) {
        if (playbackState == nullptr) {
            return;
        }
        playbackState->keyframes.clear();
        playbackState->mode                    = DebugPlaybackState::Mode::Stopped;
        playbackState->play_time               = 0.0f;
        playbackState->selected_keyframe_index = -1;
        playbackState->current_segment_index   = -1;

        std::vector<teleop_viewer::RobotScene::JointInfo> revoluteJoints;
        revoluteJoints.reserve(joints.size());
        for (const auto& joint : joints) {
            if (joint.revolute) {
                revoluteJoints.push_back(joint);
            }
        }
        if (revoluteJoints.empty()) {
            return;
        }

        const int frameCount     = 16;
        const float dt           = std::max(0.1f, playbackState->keyframe_interval_sec);
        const float twoPi        = 6.283185307f;
        float start_base_x_m     = 0.0f;
        float start_base_y_m     = 0.0f;
        float start_base_yaw     = 0.0f;
        const bool has_base_pose = scene.getVirtualBasePose2D(&start_base_x_m, &start_base_y_m, &start_base_yaw);
        for (int frame = 0; frame < frameCount; ++frame) {
            PoseKeyframe keyframe;
            keyframe.t = static_cast<double>(frame) * static_cast<double>(dt);
            const float phase =
                kinematic_playback_internal::WrapPhase((static_cast<float>(frame) / static_cast<float>(frameCount - 1)) * twoPi);
            for (size_t i = 0; i < revoluteJoints.size(); ++i) {
                const auto& joint            = revoluteJoints[i];
                const float jointRange       = std::max(0.0f, joint.max_angle - joint.min_angle);
                const float amplitudeByRange = std::max(0.03f, std::min(0.25f, 0.12f * jointRange));
                const float amplitude        = std::min(amplitudeByRange, 0.35f);
                const float offsetPhase      = phase + static_cast<float>(i) * 0.35f;
                float value                  = joint.position + amplitude * std::sin(offsetPhase);
                value                        = std::clamp(value, joint.min_angle, joint.max_angle);
                keyframe.joints[joint.name]  = value;
            }
            if (has_base_pose) {
                const float x_wave        = 0.06f * std::sin(phase);
                const float y_wave        = 0.04f * std::cos(phase);
                const float yaw_wave      = 0.20f * std::sin(phase);
                keyframe.has_base_pose_2d = true;
                keyframe.base_x_m         = start_base_x_m + x_wave;
                keyframe.base_y_m         = start_base_y_m + y_wave;
                keyframe.base_yaw_rad     = start_base_yaw + yaw_wave;
            }
            playbackState->keyframes.push_back(std::move(keyframe));
        }

        playbackState->selected_keyframe_index = 0;
    }

    bool ValidateTrajectoryJointNames(const DebugPlaybackState& playbackState,
                                      const std::vector<teleop_viewer::RobotScene::JointInfo>& joints, std::string* errorMessage) {
        std::unordered_set<std::string> sceneJointNames;
        for (const auto& joint : joints) {
            sceneJointNames.insert(joint.name);
        }

        std::unordered_set<std::string> trajectoryJointNames;
        for (const auto& keyframe : playbackState.keyframes) {
            for (const auto& [jointName, _] : keyframe.joints) {
                trajectoryJointNames.insert(jointName);
            }
        }

        if (trajectoryJointNames.empty()) {
            if (errorMessage != nullptr) {
                *errorMessage = "轨迹文件中未找到任何关节名";
            }
            return false;
        }

        std::vector<std::string> unknown;
        int matchedCount = 0;
        for (const auto& jointName : trajectoryJointNames) {
            if (sceneJointNames.find(jointName) == sceneJointNames.end()) {
                unknown.push_back(jointName);
            } else {
                ++matchedCount;
            }
        }

        if (!unknown.empty()) {
            std::sort(unknown.begin(), unknown.end());
            std::stringstream ss;
            ss << "轨迹关节名与当前机器人不匹配，未知关节 " << unknown.size() << " 个: ";
            const size_t showCount = std::min<size_t>(unknown.size(), 8);
            for (size_t i = 0; i < showCount; ++i) {
                if (i > 0) {
                    ss << ", ";
                }
                ss << unknown[i];
            }
            if (unknown.size() > showCount) {
                ss << " ...";
            }
            if (errorMessage != nullptr) {
                *errorMessage = ss.str();
            }
            return false;
        }

        if (matchedCount <= 0) {
            if (errorMessage != nullptr) {
                *errorMessage = "轨迹关节名与当前机器人无任何匹配";
            }
            return false;
        }

        return true;
    }

    bool LoadTrajectoryListEntry(DebugPlaybackState* playbackState, int index,
                                 const std::vector<teleop_viewer::RobotScene::JointInfo>& joints, TrajectoryPlayer* playbackPlayer,
                                 teleop_viewer::RobotScene* scene) {
        if (playbackState == nullptr || playbackPlayer == nullptr || scene == nullptr || index < 0 ||
            index >= static_cast<int>(playbackState->trajectory_files.size())) {
            return false;
        }

        const std::string& path = playbackState->trajectory_files[static_cast<size_t>(index)].path;
        std::snprintf(playbackState->trajectory_file_path, sizeof(playbackState->trajectory_file_path), "%s", path.c_str());
        playbackState->selected_trajectory_index = index;

        const DebugPlaybackState previousState = *playbackState;
        std::string ioError;
        if (!LoadTrajectoryFromFile(path, playbackState, &ioError)) {
            *playbackState                                                     = previousState;
            playbackState->trajectory_files[static_cast<size_t>(index)].status = "加载失败: " + ioError;
            playbackState->trajectory_files[static_cast<size_t>(index)].loaded = false;
            playbackState->trajectory_io_status                                = "加载失败: " + ioError;
            playbackState->trajectory_alert_message                            = "轨迹文件加载失败，请检查路径或文件格式。";
            playbackState->trajectory_alert_detail                             = ioError;
            playbackState->trajectory_alert_popup_pending                      = true;
            return false;
        }

        std::string checkError;
        if (!ValidateTrajectoryJointNames(*playbackState, joints, &checkError)) {
            *playbackState                                                     = previousState;
            playbackState->trajectory_files[static_cast<size_t>(index)].status = "加载失败: " + checkError;
            playbackState->trajectory_files[static_cast<size_t>(index)].loaded = false;
            playbackState->trajectory_io_status                                = "加载失败: " + checkError;
            playbackState->trajectory_alert_message                            = "该轨迹与当前机器人关节定义不匹配。";
            playbackState->trajectory_alert_detail                             = checkError;
            playbackState->trajectory_alert_popup_pending                      = true;
            return false;
        }

        playbackState->trajectory_files[static_cast<size_t>(index)].status = "加载成功";
        playbackState->trajectory_files[static_cast<size_t>(index)].loaded = true;
        playbackPlayer->SampleAtCurrentTime(*playbackState, scene);
        playbackState->trajectory_io_status = "加载成功: " + path;
        return true;
    }

    void ProcessPendingTrajectoryLoad(DebugPlaybackState* playbackState,
                                      const std::vector<teleop_viewer::RobotScene::JointInfo>& joints, TrajectoryPlayer* playbackPlayer,
                                      teleop_viewer::RobotScene* scene, PlaybackStateMachine* playback_sm) {
        if (playbackState == nullptr || playbackState->pending_trajectory_load_index < 0) {
            return;
        }

        const int index = playbackState->pending_trajectory_load_index;
        const bool play = playbackState->pending_trajectory_play_after_load;
        playbackState->pending_trajectory_load_index      = -1;
        playbackState->pending_trajectory_play_after_load = false;

        if (!LoadTrajectoryListEntry(playbackState, index, joints, playbackPlayer, scene)) {
            return;
        }

        if (play && playback_sm != nullptr) {
            CancelTrajectorySequence(playbackState);
            playbackState->play_time = 0.0f;
            playback_sm->Play();
        }
    }

    void CancelTrajectorySequence(DebugPlaybackState* playbackState) {
        if (playbackState == nullptr || !playbackState->trajectory_sequence_active) {
            return;
        }
        playbackState->trajectory_sequence_active = false;
        playbackState->trajectory_sequence_indices.clear();
        playbackState->trajectory_sequence_position = 0;
        playbackState->loop                           = playbackState->trajectory_sequence_saved_loop;
    }

    void StartTrajectorySequence(DebugPlaybackState* playbackState,
                                 const std::vector<teleop_viewer::RobotScene::JointInfo>& joints, TrajectoryPlayer* playbackPlayer,
                                 teleop_viewer::RobotScene* scene, PlaybackStateMachine* playback_sm) {
        if (playbackState == nullptr || playbackPlayer == nullptr || scene == nullptr || playback_sm == nullptr) {
            return;
        }

        playbackState->trajectory_sequence_indices.clear();
        for (int i = 0; i < static_cast<int>(playbackState->trajectory_files.size()); ++i) {
            if (playbackState->trajectory_files[static_cast<size_t>(i)].queued) {
                playbackState->trajectory_sequence_indices.push_back(i);
            }
        }
        if (playbackState->trajectory_sequence_indices.empty()) {
            playbackState->trajectory_io_status = "连播: 请先勾选要播放的轨迹";
            return;
        }

        playbackState->trajectory_sequence_active       = true;
        playbackState->trajectory_sequence_position     = 0;
        playbackState->trajectory_sequence_saved_loop   = playbackState->loop;
        playbackState->loop                             = false;
        playbackState->trajectory_io_status             = "连播: 开始 (" + std::to_string(playbackState->trajectory_sequence_indices.size()) + " 个文件)";

        while (playbackState->trajectory_sequence_position < playbackState->trajectory_sequence_indices.size()) {
            const int index = playbackState->trajectory_sequence_indices[playbackState->trajectory_sequence_position];
            if (LoadTrajectoryListEntry(playbackState, index, joints, playbackPlayer, scene)) {
                playbackState->play_time = 0.0f;
                playback_sm->Play();
                return;
            }
            ++playbackState->trajectory_sequence_position;
        }

        CancelTrajectorySequence(playbackState);
        playbackState->trajectory_io_status = "连播: 所选轨迹均无法加载";
    }

    void TickTrajectorySequence(DebugPlaybackState* playbackState, PlaybackStateMachine* playback_sm, bool was_playing_last_frame,
                                const std::vector<teleop_viewer::RobotScene::JointInfo>& joints, TrajectoryPlayer* playbackPlayer,
                                teleop_viewer::RobotScene* scene) {
        if (playbackState == nullptr || playback_sm == nullptr || playbackPlayer == nullptr || scene == nullptr ||
            !playbackState->trajectory_sequence_active || !was_playing_last_frame || playback_sm->IsPlaying() || playback_sm->IsPaused()) {
            return;
        }

        ++playbackState->trajectory_sequence_position;
        while (playbackState->trajectory_sequence_position < playbackState->trajectory_sequence_indices.size()) {
            const int index = playbackState->trajectory_sequence_indices[playbackState->trajectory_sequence_position];
            if (LoadTrajectoryListEntry(playbackState, index, joints, playbackPlayer, scene)) {
                playbackState->play_time = 0.0f;
                playback_sm->Play();
                const size_t current = playbackState->trajectory_sequence_position + 1;
                const size_t total   = playbackState->trajectory_sequence_indices.size();
                playbackState->trajectory_io_status = "连播: " + std::to_string(current) + "/" + std::to_string(total);
                return;
            }
            ++playbackState->trajectory_sequence_position;
        }

        CancelTrajectorySequence(playbackState);
        playbackState->trajectory_io_status = "连播: 完成";
    }

}  // namespace kinematic_viewer
