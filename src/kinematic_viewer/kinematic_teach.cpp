#include "kinematic_viewer/kinematic_teach.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_set>

namespace kinematic_viewer {
    namespace kinematic_teach_internal {

        std::string YamlQuote(const std::string& text) {
            if (text.find_first_of(":#\"'\n\r\t") == std::string::npos) {
                return text;
            }
            std::string out = "\"";
            for (char ch : text) {
                if (ch == '"' || ch == '\\') {
                    out.push_back('\\');
                }
                out.push_back(ch);
            }
            out.push_back('"');
            return out;
        }

        bool FindLinkWorldPose(const teleop_viewer::RobotScene& scene, const std::string& link_name, glm::vec3* position,
                               glm::quat* orientation) {
            if (position == nullptr || orientation == nullptr || link_name.empty()) {
                return false;
            }
            for (const auto& tf : scene.getLinkTfInfos()) {
                if (tf.name != link_name) {
                    continue;
                }
                *position = tf.world_position;
                const glm::vec3 rpy_rad(tf.world_rpy.x, tf.world_rpy.y, tf.world_rpy.z);
                *orientation = glm::normalize(glm::quat(glm::vec3(rpy_rad.x, rpy_rad.y, rpy_rad.z)));
                return true;
            }
            return false;
        }

        std::vector<std::string> CollectJointNamesFromPoints(const TeachProgramState& teach) {
            std::unordered_set<std::string> names;
            for (const auto& point : teach.points) {
                for (const auto& [joint_name, _] : point.joints) {
                    names.insert(joint_name);
                }
            }
            std::vector<std::string> sorted(names.begin(), names.end());
            std::sort(sorted.begin(), sorted.end());
            return sorted;
        }

        std::vector<std::string> CollectJointNames(const TeachProgramState& teach) {
            if (!teach.joint_names.empty()) {
                return teach.joint_names;
            }
            return CollectJointNamesFromPoints(teach);
        }

        void WriteYamlStringList(std::ostream& out, const std::string& key, const std::vector<std::string>& values, int indent) {
            const std::string pad(static_cast<size_t>(indent), ' ');
            out << pad << key << ":\n";
            for (const auto& value : values) {
                out << pad << "  - " << YamlQuote(value) << "\n";
            }
        }

        void WriteYamlFloatFlowList(std::ostream& out, const std::string& key, const std::vector<float>& values, int indent) {
            const std::string pad(static_cast<size_t>(indent), ' ');
            out << pad << key << ": [";
            for (size_t i = 0; i < values.size(); ++i) {
                if (i > 0) {
                    out << ", ";
                }
                out << std::setprecision(8) << values[i];
            }
            out << "]\n";
        }

        void PointJointsFromQ(TeachPoint* point, const std::vector<std::string>& joint_names, const YAML::Node& q_node) {
            if (point == nullptr || !q_node || !q_node.IsSequence()) {
                return;
            }
            point->joints.clear();
            const size_t n = std::min(joint_names.size(), q_node.size());
            for (size_t i = 0; i < n; ++i) {
                point->joints[joint_names[i]] = q_node[static_cast<int>(i)].as<float>();
            }
        }

        void ReadChassisFromNode(const YAML::Node& node, TeachPoint* point) {
            if (point == nullptr) {
                return;
            }
            if (node["chassis"] && node["chassis"].IsSequence() && node["chassis"].size() >= 3) {
                point->has_base_pose_2d = true;
                point->base_x_m         = node["chassis"][0].as<float>();
                point->base_y_m         = node["chassis"][1].as<float>();
                point->base_yaw_rad     = node["chassis"][2].as<float>();
            } else if (node["chassis_x"] && node["chassis_y"] && node["chassis_yaw"]) {
                point->has_base_pose_2d = true;
                point->base_x_m         = node["chassis_x"].as<float>();
                point->base_y_m         = node["chassis_y"].as<float>();
                point->base_yaw_rad     = node["chassis_yaw"].as<float>();
            }
        }

        std::vector<float> JointVectorForPoint(const TeachPoint& point, const std::vector<std::string>& joint_names) {
            std::vector<float> values(joint_names.size(), 0.0f);
            for (size_t i = 0; i < joint_names.size(); ++i) {
                const auto it = point.joints.find(joint_names[i]);
                if (it != point.joints.end()) {
                    values[i] = it->second;
                }
            }
            return values;
        }

        std::string ProfileName(int profile_index) {
            return profile_index == 0 ? "TVP" : "DSVP";
        }

        void AppendJointTrajectory(JointSpaceTrajectory* combined, const JointSpaceTrajectory& segment, float time_offset_sec) {
            if (combined == nullptr || segment.joint_names.empty() || segment.joint_positions.empty()) {
                return;
            }
            if (combined->joint_names.empty()) {
                combined->joint_names = segment.joint_names;
            }
            const size_t n = combined->joint_names.size();
            for (size_t i = 0; i < segment.times.size() && i < segment.joint_positions.size(); ++i) {
                if (i == 0 && !combined->times.empty()) {
                    const float last_time = combined->times.back();
                    const float seg_time  = segment.times[i] + time_offset_sec;
                    if (std::fabs(seg_time - last_time) < 1e-6f) {
                        continue;
                    }
                }
                combined->times.push_back(segment.times[i] + time_offset_sec);
                std::vector<float> row(n, 0.0f);
                for (size_t j = 0; j < n; ++j) {
                    const auto& name = combined->joint_names[j];
                    const auto it    = std::find(segment.joint_names.begin(), segment.joint_names.end(), name);
                    if (it != segment.joint_names.end()) {
                        const size_t seg_j = static_cast<size_t>(std::distance(segment.joint_names.begin(), it));
                        if (seg_j < segment.joint_positions[i].size()) {
                            row[j] = segment.joint_positions[i][seg_j];
                        }
                    }
                }
                combined->joint_positions.push_back(std::move(row));
            }
        }

    }  // namespace kinematic_teach_internal

    void CaptureTeachPointFromScene(TeachProgramState* teach, const std::vector<teleop_viewer::RobotScene::JointInfo>& joints,
                                    const teleop_viewer::RobotScene& scene, const std::string& ee_tip_link) {
        if (teach == nullptr) {
            return;
        }
        TeachPoint point;
        point.name = "P" + std::to_string(teach->points.size() + 1);
        for (const auto& joint : joints) {
            if (joint.revolute) {
                point.joints[joint.name] = joint.position;
            }
        }
        float base_x   = 0.0f;
        float base_y   = 0.0f;
        float base_yaw = 0.0f;
        if (scene.getVirtualBasePose2D(&base_x, &base_y, &base_yaw)) {
            point.has_base_pose_2d = true;
            point.base_x_m         = base_x;
            point.base_y_m         = base_y;
            point.base_yaw_rad     = base_yaw;
        }
        if (!ee_tip_link.empty()) {
            glm::vec3 pos;
            glm::quat quat;
            if (kinematic_teach_internal::FindLinkWorldPose(scene, ee_tip_link, &pos, &quat)) {
                point.has_ee_pose    = true;
                point.ee_tip_link    = ee_tip_link;
                point.ee_position    = pos;
                point.ee_orientation = quat;
            }
        }
        teach->points.push_back(std::move(point));
        teach->selected_point_index = static_cast<int>(teach->points.size()) - 1;
        if (teach->joint_names.empty()) {
            teach->joint_names = kinematic_teach_internal::CollectJointNamesFromPoints(*teach);
        }
    }

    void ApplyTeachPointToScene(const TeachPoint& point, teleop_viewer::RobotScene* scene) {
        if (scene == nullptr) {
            return;
        }
        for (const auto& [joint_name, value] : point.joints) {
            scene->setJointPositionByName(joint_name, value);
        }
        if (point.has_base_pose_2d) {
            scene->setVirtualBasePose2D(point.base_x_m, point.base_y_m, point.base_yaw_rad);
        }
    }

    void RemoveSelectedTeachPoint(TeachProgramState* teach) {
        if (teach == nullptr || teach->selected_point_index < 0 || teach->selected_point_index >= static_cast<int>(teach->points.size())) {
            return;
        }
        teach->points.erase(teach->points.begin() + teach->selected_point_index);
        if (teach->points.empty()) {
            teach->selected_point_index = -1;
        } else {
            teach->selected_point_index = std::clamp(teach->selected_point_index, 0, static_cast<int>(teach->points.size()) - 1);
        }
    }

    void MoveTeachPoint(TeachProgramState* teach, int from_index, int to_index) {
        if (teach == nullptr || from_index < 0 || to_index < 0 || from_index >= static_cast<int>(teach->points.size()) ||
            to_index >= static_cast<int>(teach->points.size()) || from_index == to_index) {
            return;
        }
        TeachPoint moved = std::move(teach->points[static_cast<size_t>(from_index)]);
        teach->points.erase(teach->points.begin() + from_index);
        teach->points.insert(teach->points.begin() + to_index, std::move(moved));
        teach->selected_point_index = to_index;
    }

    void RenameTeachPoint(TeachProgramState* teach, int index, const std::string& new_name) {
        if (teach == nullptr || index < 0 || index >= static_cast<int>(teach->points.size()) || new_name.empty()) {
            return;
        }
        teach->points[static_cast<size_t>(index)].name = new_name;
    }

    bool SaveTeachProgramToYaml(const std::string& path, const TeachProgramState& teach, std::string* error_message) {
        try {
            std::ofstream file(path);
            if (!file.good()) {
                if (error_message != nullptr) {
                    *error_message = "无法写入文件";
                }
                return false;
            }
            const std::vector<std::string> joint_names = kinematic_teach_internal::CollectJointNames(teach);
            file << "program_name: " << kinematic_teach_internal::YamlQuote(teach.program_name) << "\n";
            if (!joint_names.empty()) {
                kinematic_teach_internal::WriteYamlStringList(file, "joint_names", joint_names, 0);
            }
            file << "points:\n";
            for (const auto& point : teach.points) {
                file << "  - name: " << kinematic_teach_internal::YamlQuote(point.name) << "\n";
                if (!joint_names.empty()) {
                    kinematic_teach_internal::WriteYamlFloatFlowList(file, "q",
                                                                     kinematic_teach_internal::JointVectorForPoint(point, joint_names), 4);
                }
                if (point.has_base_pose_2d) {
                    file << "    chassis: [" << std::setprecision(8) << point.base_x_m << ", " << point.base_y_m << ", "
                         << point.base_yaw_rad << "]\n";
                }
                if (point.has_ee_pose) {
                    file << "    ee_tip_link: " << kinematic_teach_internal::YamlQuote(point.ee_tip_link) << "\n";
                    file << "    ee_pos: [" << point.ee_position.x << ", " << point.ee_position.y << ", " << point.ee_position.z << "]\n";
                    file << "    ee_quat: [" << point.ee_orientation.w << ", " << point.ee_orientation.x << ", " << point.ee_orientation.y
                         << ", " << point.ee_orientation.z << "]\n";
                }
            }
            if (error_message != nullptr) {
                *error_message = "";
            }
            return true;
        } catch (const std::exception& e) {
            if (error_message != nullptr) {
                *error_message = e.what();
            }
            return false;
        }
    }

    bool LoadTeachProgramFromYaml(const std::string& path, TeachProgramState* teach, std::string* error_message) {
        if (teach == nullptr) {
            if (error_message != nullptr) {
                *error_message = "teach state is null";
            }
            return false;
        }
        try {
            const YAML::Node root = YAML::LoadFile(path);
            teach->points.clear();
            teach->joint_names.clear();
            teach->selected_point_index = -1;
            if (root["program_name"]) {
                teach->program_name = root["program_name"].as<std::string>();
            }
            if (root["joint_names"] && root["joint_names"].IsSequence()) {
                for (const auto& item : root["joint_names"]) {
                    if (item.IsScalar()) {
                        teach->joint_names.push_back(item.as<std::string>());
                    }
                }
            }
            const YAML::Node points_node = root["points"];
            if (!points_node || !points_node.IsSequence()) {
                if (error_message != nullptr) {
                    *error_message = "缺少 points 序列";
                }
                return false;
            }
            int index = 0;
            for (const auto& node : points_node) {
                TeachPoint point;
                point.name = node["name"] ? node["name"].as<std::string>() : ("P" + std::to_string(index + 1));
                if (node["q"] && node["q"].IsSequence() && !teach->joint_names.empty()) {
                    kinematic_teach_internal::PointJointsFromQ(&point, teach->joint_names, node["q"]);
                } else if (node["joints"] && node["joints"].IsMap()) {
                    for (const auto& item : node["joints"]) {
                        point.joints[item.first.as<std::string>()] = item.second.as<float>();
                    }
                }
                kinematic_teach_internal::ReadChassisFromNode(node, &point);
                if (node["ee_pos"] && node["ee_pos"].IsSequence() && node["ee_pos"].size() >= 3) {
                    point.ee_position.x = node["ee_pos"][0].as<float>();
                    point.ee_position.y = node["ee_pos"][1].as<float>();
                    point.ee_position.z = node["ee_pos"][2].as<float>();
                    point.has_ee_pose   = true;
                }
                if (node["ee_quat"] && node["ee_quat"].IsSequence() && node["ee_quat"].size() >= 4) {
                    point.ee_orientation.w = node["ee_quat"][0].as<float>();
                    point.ee_orientation.x = node["ee_quat"][1].as<float>();
                    point.ee_orientation.y = node["ee_quat"][2].as<float>();
                    point.ee_orientation.z = node["ee_quat"][3].as<float>();
                    point.has_ee_pose      = true;
                }
                if (node["ee_tip_link"]) {
                    point.ee_tip_link = node["ee_tip_link"].as<std::string>();
                }
                teach->points.push_back(std::move(point));
                ++index;
            }
            if (teach->joint_names.empty()) {
                teach->joint_names = kinematic_teach_internal::CollectJointNamesFromPoints(*teach);
            }
            if (!teach->points.empty()) {
                teach->selected_point_index = 0;
            }
            if (error_message != nullptr) {
                *error_message = "";
            }
            return !teach->points.empty();
        } catch (const std::exception& e) {
            if (error_message != nullptr) {
                *error_message = e.what();
            }
            return false;
        }
    }

    JointSpaceTrajectory BuildTeachMoveJTrajectory(const TeachProgramState& teach, std::string* status_message) {
        JointSpaceTrajectory combined;
        if (teach.points.size() < 2) {
            if (status_message != nullptr) {
                *status_message = "至少需要 2 个示教点";
            }
            return combined;
        }
        const std::vector<std::string> joint_names = kinematic_teach_internal::CollectJointNames(teach);
        if (joint_names.empty()) {
            if (status_message != nullptr) {
                *status_message = "示教点中无关节数据";
            }
            return combined;
        }

        float time_offset = 0.0f;
        for (size_t seg = 1; seg < teach.points.size(); ++seg) {
            JointSpacePTPParams params;
            params.joint_names     = joint_names;
            params.start_positions = kinematic_teach_internal::JointVectorForPoint(teach.points[seg - 1], joint_names);
            params.goal_positions  = kinematic_teach_internal::JointVectorForPoint(teach.points[seg], joint_names);
            params.max_vel         = teach.movej_max_vel;
            params.max_acc         = teach.movej_max_acc;
            params.max_jerk        = teach.movej_max_jerk;
            params.delta_t         = teach.movej_delta_t;
            params.profile         = kinematic_teach_internal::ProfileName(teach.movej_profile);

            const JointSpaceTrajectory segment = planJointSpacePTP(params);
            if (!segment.success) {
                if (status_message != nullptr) {
                    *status_message = "moveJ 段 " + std::to_string(seg) + " 失败: " + segment.status;
                }
                return JointSpaceTrajectory{};
            }
            kinematic_teach_internal::AppendJointTrajectory(&combined, segment, time_offset);
            if (!segment.times.empty()) {
                time_offset = combined.times.back();
            }
        }
        combined.success = !combined.joint_positions.empty();
        if (status_message != nullptr) {
            *status_message = combined.success ? ("moveJ 成功: " + std::to_string(combined.times.size()) + " 点, 时长 " +
                                                  std::to_string(combined.times.empty() ? 0.0f : combined.times.back()) + "s")
                                               : "moveJ 失败";
        }
        return combined;
    }

    CartesianPathResult BuildTeachMoveLTrajectory(const TeachProgramState& teach, std::string* status_message) {
        CartesianPathResult combined;
        if (teach.points.size() < 2) {
            if (status_message != nullptr) {
                *status_message = "至少需要 2 个示教点";
            }
            combined.status = *status_message;
            return combined;
        }

        float time_offset = 0.0f;
        for (size_t seg = 1; seg < teach.points.size(); ++seg) {
            const auto& p0 = teach.points[seg - 1];
            const auto& p1 = teach.points[seg];
            if (!p0.has_ee_pose || !p1.has_ee_pose) {
                if (status_message != nullptr) {
                    *status_message = "示教点 " + std::to_string(seg - 1) + "→" + std::to_string(seg) + " 缺少末端位姿，请用 IK 链记录";
                }
                combined.status = *status_message;
                return combined;
            }

            StraightPathParams params;
            params.start_pos  = p0.ee_position;
            params.goal_pos   = p1.ee_position;
            params.start_quat = p0.ee_orientation;
            params.goal_quat  = p1.ee_orientation;
            params.max_vel    = teach.movel_max_vel;
            params.max_acc    = teach.movel_max_acc;
            params.delta_t    = teach.movel_delta_t;
            params.profile    = "DSVP";

            auto planner                      = makeStraightPlanner(params);
            const CartesianPathResult segment = planner->plan(p0.ee_position, p0.ee_orientation);
            if (!segment.success || segment.waypoints.empty()) {
                if (status_message != nullptr) {
                    *status_message = "moveL 段 " + std::to_string(seg) + " 失败: " + segment.status;
                }
                combined.status = segment.status;
                return combined;
            }

            for (size_t i = 0; i < segment.waypoints.size(); ++i) {
                if (i == 0 && !combined.waypoints.empty()) {
                    continue;
                }
                CartesianWaypoint wp = segment.waypoints[i];
                wp.time_sec += time_offset;
                combined.waypoints.push_back(wp);
            }
            time_offset = combined.waypoints.empty() ? time_offset : combined.waypoints.back().time_sec;
        }

        combined.success = combined.waypoints.size() >= 2;
        combined.status  = combined.success ? ("moveL 成功: " + std::to_string(combined.waypoints.size()) + " 路点") : "moveL 失败";
        if (status_message != nullptr) {
            *status_message = combined.status;
        }
        return combined;
    }

    bool LoadJointTrajectoryIntoPlayback(const JointSpaceTrajectory& traj, DebugPlaybackState* playback_state, std::string* error_message) {
        if (playback_state == nullptr) {
            if (error_message != nullptr) {
                *error_message = "playback state is null";
            }
            return false;
        }
        if (!traj.success || traj.joint_positions.empty()) {
            if (error_message != nullptr) {
                *error_message = "轨迹为空";
            }
            return false;
        }
        playback_state->keyframes.clear();
        for (size_t i = 0; i < traj.joint_positions.size(); ++i) {
            PoseKeyframe kf;
            kf.t = (i < traj.times.size()) ? static_cast<double>(traj.times[i]) : static_cast<double>(i) * 0.02;
            for (size_t j = 0; j < traj.joint_names.size() && j < traj.joint_positions[i].size(); ++j) {
                kf.joints[traj.joint_names[j]] = traj.joint_positions[i][j];
            }
            playback_state->keyframes.push_back(std::move(kf));
        }
        playback_state->selected_keyframe_index = 0;
        playback_state->play_time               = 0.0f;
        playback_state->mode                    = DebugPlaybackState::Mode::Stopped;
        if (error_message != nullptr) {
            *error_message = "";
        }
        return true;
    }

    bool LoadTeachPointsIntoPlayback(const TeachProgramState& teach, float seconds_per_segment, DebugPlaybackState* playback_state,
                                     std::string* error_message) {
        if (playback_state == nullptr || teach.points.empty()) {
            if (error_message != nullptr) {
                *error_message = "无示教点";
            }
            return false;
        }
        playback_state->keyframes.clear();
        float t        = 0.0f;
        const float dt = std::max(0.02f, seconds_per_segment);
        for (const auto& point : teach.points) {
            PoseKeyframe kf;
            kf.t      = static_cast<double>(t);
            kf.joints = point.joints;
            if (point.has_base_pose_2d) {
                kf.has_base_pose_2d = true;
                kf.base_x_m         = point.base_x_m;
                kf.base_y_m         = point.base_y_m;
                kf.base_yaw_rad     = point.base_yaw_rad;
            }
            playback_state->keyframes.push_back(std::move(kf));
            t += dt;
        }
        playback_state->selected_keyframe_index = 0;
        playback_state->play_time               = 0.0f;
        playback_state->mode                    = DebugPlaybackState::Mode::Stopped;
        if (error_message != nullptr) {
            *error_message = "";
        }
        return true;
    }

}  // namespace kinematic_viewer
