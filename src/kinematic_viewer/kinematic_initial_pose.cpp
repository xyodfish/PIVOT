#include "kinematic_viewer/kinematic_initial_pose.h"

#include <algorithm>
#include <sstream>
#include <utility>
#include <vector>

namespace kinematic_viewer {
    namespace kinematic_initial_pose_internal {

        void ApplyJointGroup(const std::vector<std::string>& jointNames, const std::vector<float>& values, rkv::RobotScene* scene,
                             InitialPoseApplyResult* result, std::vector<std::string>* missingJointNames) {
            if (scene == nullptr || result == nullptr || missingJointNames == nullptr) {
                return;
            }
            const size_t applyCount = std::min(jointNames.size(), values.size());
            result->requested_joint_count += static_cast<int>(applyCount);
            for (size_t i = 0; i < applyCount; ++i) {
                if (scene->setJointPositionByName(jointNames[i], values[i])) {
                    ++result->applied_joint_count;
                } else {
                    ++result->missing_joint_count;
                    missingJointNames->push_back(jointNames[i]);
                }
            }
        }

    }  // namespace kinematic_initial_pose_internal

    InitialPoseApplyResult ApplyConfiguredInitialPose(const KinematicInitialPoseConfig& config, rkv::RobotScene* scene) {
        InitialPoseApplyResult result;
        if (scene == nullptr) {
            result.detail = "scene is null";
            return result;
        }

        std::vector<std::string> missingJointNames;
        kinematic_initial_pose_internal::ApplyJointGroup(config.head_joint_names, config.head, scene, &result, &missingJointNames);
        kinematic_initial_pose_internal::ApplyJointGroup(config.leg_joint_names, config.leg, scene, &result, &missingJointNames);
        kinematic_initial_pose_internal::ApplyJointGroup(config.left_arm_joint_names, config.left_arm, scene, &result, &missingJointNames);
        kinematic_initial_pose_internal::ApplyJointGroup(config.right_arm_joint_names, config.right_arm, scene, &result,
                                                         &missingJointNames);

        if (config.apply_chassis) {
            scene->setVirtualBasePose2D(config.chassis_x, config.chassis_y, config.chassis_yaw);
            result.chassis_applied = true;
            result.chassis_x_m     = config.chassis_x;
            result.chassis_y_m     = config.chassis_y;
            result.chassis_yaw_rad = config.chassis_yaw;
        }
        scene->updateTransforms();

        std::stringstream ss;
        ss << "已应用关节 " << result.applied_joint_count << "/" << result.requested_joint_count;
        if (result.chassis_applied) {
            ss << "，底盘(x=" << config.chassis_x << " m, y=" << config.chassis_y << " m, yaw=" << config.chassis_yaw << " rad)";
        }
        if (!missingJointNames.empty()) {
            std::sort(missingJointNames.begin(), missingJointNames.end());
            missingJointNames.erase(std::unique(missingJointNames.begin(), missingJointNames.end()), missingJointNames.end());
            ss << "，缺失关节 " << missingJointNames.size() << " 个";
            const size_t showCount = std::min<size_t>(missingJointNames.size(), 6);
            ss << ": ";
            for (size_t i = 0; i < showCount; ++i) {
                if (i > 0) {
                    ss << ", ";
                }
                ss << missingJointNames[i];
            }
            if (missingJointNames.size() > showCount) {
                ss << " ...";
            }
        }
        result.detail = ss.str();
        return result;
    }

}  // namespace kinematic_viewer
