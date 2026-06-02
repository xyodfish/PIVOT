#pragma once

#include "teleop_viewer/scene.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <unordered_map>

namespace kinematic_viewer {

    struct LinkKinematicsMetrics {
        bool valid                  = false;
        float translational_manip   = 0.0f;
        float jacobian_condition_6d = -1.0f;
        std::string error;
    };

    // Pinocchio-backed metrics for the selected link frame (lazy URDF model cache).
    class LinkKinematicsAnalyzer {
       public:
        bool compute(const teleop_viewer::RobotScene& scene, const std::string& link_name, LinkKinematicsMetrics* out);

       private:
        bool RebuildModelIfNeeded(const std::string& urdf_path);

        std::string cached_urdf_path_;
        bool model_ready_ = false;
        pinocchio::Model model_;
        pinocchio::Data data_;
        std::unordered_map<std::string, int> joint_q_index_;
    };

}  // namespace kinematic_viewer
