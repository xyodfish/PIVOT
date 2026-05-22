#include "kinematic_viewer/kinematic_viewer_config.h"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace kinematic_viewer {
    namespace kinematic_viewer_config_internal {

        template <typename T>
        void ReadScalar(const YAML::Node& node, const char* key, T& out) {
            if (node && node[key]) {
                out = node[key].as<T>();
            }
        }

        void ReadFloatList(const YAML::Node& node, const char* key, std::vector<float>& out) {
            if (!node || !node[key] || !node[key].IsSequence()) {
                return;
            }
            out.clear();
            for (const auto& item : node[key]) {
                if (item.IsScalar()) {
                    out.push_back(item.as<float>());
                }
            }
        }

        void ReadVec3(const YAML::Node& node, const char* key, glm::vec3& out) {
            if (!node || !node[key] || !node[key].IsSequence() || node[key].size() < 3) {
                return;
            }
            out.x = node[key][0].as<float>();
            out.y = node[key][1].as<float>();
            out.z = node[key][2].as<float>();
        }

        void ReadStringListFromItem(const YAML::Node& item, const char* key, std::vector<std::string>& out) {
            if (!item || !item[key] || !item[key].IsSequence()) {
                return;
            }
            out.clear();
            for (const auto& entry : item[key]) {
                if (entry.IsScalar()) {
                    out.push_back(entry.as<std::string>());
                }
            }
        }

        void ReadIkChainList(const YAML::Node& node, const char* key, std::vector<ViewerIkChainConfig>& out) {
            if (!node || !node[key] || !node[key].IsSequence()) {
                return;
            }
            std::vector<ViewerIkChainConfig> parsed;
            for (const auto& item : node[key]) {
                if (!item || !item.IsMap()) {
                    continue;
                }
                ViewerIkChainConfig chain;
                ReadScalar(item, "label", chain.label);
                ReadScalar(item, "base_link", chain.base_link);
                ReadScalar(item, "tip_link", chain.tip_link);
                ReadStringListFromItem(item, "base_link_candidates", chain.base_link_candidates);
                ReadStringListFromItem(item, "tip_link_candidates", chain.tip_link_candidates);
                const bool hasBaseSelector = !chain.base_link.empty() || !chain.base_link_candidates.empty();
                const bool hasTipSelector  = !chain.tip_link.empty() || !chain.tip_link_candidates.empty();
                if (hasBaseSelector && hasTipSelector) {
                    parsed.push_back(std::move(chain));
                }
            }
            if (!parsed.empty()) {
                out = std::move(parsed);
            }
        }

        void ValidateTopLevelKeys(const YAML::Node& root) {
            if (!root || !root.IsMap()) {
                throw std::runtime_error("YAML 根节点必须是 map");
            }

            static const std::unordered_set<std::string> kAllowedKeys = {
                "window", "robot", "camera", "ui", "ik", "ros", "initial_pose", "playback",
            };

            std::vector<std::string> unknown;
            for (auto it = root.begin(); it != root.end(); ++it) {
                if (!it->first.IsScalar()) {
                    continue;
                }
                const std::string key = it->first.as<std::string>();
                if (kAllowedKeys.find(key) == kAllowedKeys.end()) {
                    unknown.push_back(key);
                }
            }

            if (!unknown.empty()) {
                std::ostringstream oss;
                oss << "检测到非 robot_kinematic_viewer 配置项: ";
                for (size_t i = 0; i < unknown.size(); ++i) {
                    if (i > 0) {
                        oss << ", ";
                    }
                    oss << unknown[i];
                }
                oss << "。请使用独立的 robot_kinematic_viewer 配置文件。";
                throw std::runtime_error(oss.str());
            }
        }

    }  // namespace kinematic_viewer_config_internal

    KinematicViewerConfig KinematicViewerConfig::LoadFromFile(const std::string& yaml_path, bool* loaded_ok) {
        using namespace kinematic_viewer_config_internal;
        KinematicViewerConfig cfg;
        bool ok = false;

        try {
            YAML::Node root = YAML::LoadFile(yaml_path);
            ValidateTopLevelKeys(root);

            ReadScalar(root["window"], "width", cfg.window.width);
            ReadScalar(root["window"], "height", cfg.window.height);
            ReadScalar(root["window"], "title", cfg.window.title);

            ReadScalar(root["robot"], "urdf_path", cfg.robot.urdf_path);
            ReadScalar(root["robot"], "mujoco_xml_path", cfg.robot.mujoco_xml_path);

            ReadScalar(root["camera"], "distance", cfg.camera.distance);
            ReadScalar(root["camera"], "yaw", cfg.camera.yaw);
            ReadScalar(root["camera"], "pitch", cfg.camera.pitch);
            ReadVec3(root["camera"], "target", cfg.camera.target);
            ReadScalar(root["camera"], "rotate_speed", cfg.camera.rotate_speed);
            ReadScalar(root["camera"], "zoom_scale", cfg.camera.zoom_scale);
            ReadScalar(root["camera"], "dolly_scale", cfg.camera.dolly_scale);
            ReadScalar(root["camera"], "pan_scale", cfg.camera.pan_scale);
            ReadScalar(root["camera"], "min_distance", cfg.camera.min_distance);
            ReadScalar(root["camera"], "max_distance", cfg.camera.max_distance);

            ReadScalar(root["ui"], "fix_base_like_mujoco", cfg.ui.fix_base_like_mujoco);
            ReadScalar(root["ui"], "cjk_font_path", cfg.ui.cjk_font_path);
            ReadScalar(root["ui"], "cjk_font_size", cfg.ui.cjk_font_size);
            ReadScalar(root["ui"], "theme_preset", cfg.ui.theme_preset);

            ReadScalar(root["ik"], "mode", cfg.ik.mode);
            ReadScalar(root["ik"], "full_body_backend", cfg.ik.full_body_backend);
            ReadScalar(root["ik"], "full_body_iterations", cfg.ik.full_body_iterations);
            ReadIkChainList(root["ik"], "chains", cfg.ik.chains);
            ReadScalar(root["ros"], "enable", cfg.ros.enable);

            ReadScalar(root["initial_pose"], "enable", cfg.initial_pose.enable);
            ReadScalar(root["initial_pose"], "auto_apply_on_start", cfg.initial_pose.auto_apply_on_start);
            ReadStringListFromItem(root["initial_pose"], "head_joint_names", cfg.initial_pose.head_joint_names);
            ReadStringListFromItem(root["initial_pose"], "leg_joint_names", cfg.initial_pose.leg_joint_names);
            ReadStringListFromItem(root["initial_pose"], "left_arm_joint_names", cfg.initial_pose.left_arm_joint_names);
            ReadStringListFromItem(root["initial_pose"], "right_arm_joint_names", cfg.initial_pose.right_arm_joint_names);
            ReadFloatList(root["initial_pose"], "head", cfg.initial_pose.head);
            ReadFloatList(root["initial_pose"], "leg", cfg.initial_pose.leg);
            ReadFloatList(root["initial_pose"], "left_arm", cfg.initial_pose.left_arm);
            ReadFloatList(root["initial_pose"], "right_arm", cfg.initial_pose.right_arm);
            ReadScalar(root["initial_pose"], "apply_chassis", cfg.initial_pose.apply_chassis);
            ReadScalar(root["initial_pose"], "chassis_x", cfg.initial_pose.chassis_x);
            ReadScalar(root["initial_pose"], "chassis_y", cfg.initial_pose.chassis_y);
            ReadScalar(root["initial_pose"], "chassis_yaw", cfg.initial_pose.chassis_yaw);
            if (root["initial_pose"] && root["initial_pose"]["chassis"] && root["initial_pose"]["chassis"].IsSequence() &&
                root["initial_pose"]["chassis"].size() >= 3) {
                cfg.initial_pose.chassis_x   = root["initial_pose"]["chassis"][0].as<float>();
                cfg.initial_pose.chassis_y   = root["initial_pose"]["chassis"][1].as<float>();
                cfg.initial_pose.chassis_yaw = root["initial_pose"]["chassis"][2].as<float>();
            }

            ReadStringListFromItem(root["playback"], "trajectory_files", cfg.playback.trajectory_files);
            ReadScalar(root["playback"], "selected_index", cfg.playback.selected_index);

            ok = true;
        } catch (const std::exception& e) {
            std::cerr << "[KinematicViewerConfig] Load failed for: " << yaml_path << ", reason: " << e.what() << ". Fallback to defaults."
                      << std::endl;
        }

        if (loaded_ok) {
            *loaded_ok = ok;
        }
        return cfg;
    }

}  // namespace kinematic_viewer
