#include "kinematic_viewer/kinematic_link_kinematics.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <unordered_map>

namespace kinematic_viewer {

    bool LinkKinematicsAnalyzer::RebuildModelIfNeeded(const std::string& urdf_path) {
        if (model_ready_ && urdf_path == cached_urdf_path_) {
            return true;
        }
        try {
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
            joint_q_index_.clear();
            for (int j = 1; j < model_.njoints; ++j) {
                const auto& joint = model_.joints[j];
                if (joint.nq() == 1 && joint.nv() == 1) {
                    joint_q_index_[model_.names[j]] = joint.idx_q();
                }
            }
            cached_urdf_path_ = urdf_path;
            model_ready_      = true;
            return true;
        } catch (...) {
            model_ready_ = false;
            joint_q_index_.clear();
            return false;
        }
    }

    bool LinkKinematicsAnalyzer::compute(const teleop_viewer::RobotScene& scene, const std::string& link_name,
                                         LinkKinematicsMetrics* out) {
        if (out == nullptr) {
            return false;
        }
        *out = LinkKinematicsMetrics{};

        const std::string& urdf_path = scene.urdfFilePath();
        if (urdf_path.empty()) {
            out->error = "URDF path empty";
            return false;
        }
        if (!RebuildModelIfNeeded(urdf_path)) {
            out->error = "Failed to build Pinocchio model";
            return false;
        }

        if (!model_.existFrame(link_name)) {
            out->error = "Frame not found in Pinocchio model";
            return false;
        }
        const pinocchio::FrameIndex frame_id = model_.getFrameId(link_name);
        if (model_.nv <= 0) {
            out->error = "Pinocchio model has no velocity DoF";
            return false;
        }

        Eigen::VectorXd q = pinocchio::neutral(model_);
        for (const auto& joint : scene.getJointInfos()) {
            const auto it = joint_q_index_.find(joint.name);
            if (it == joint_q_index_.end()) {
                continue;
            }
            const int idx_q = it->second;
            if (idx_q >= 0 && idx_q < q.size()) {
                q[idx_q] = static_cast<double>(joint.position);
            }
        }

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        Eigen::MatrixXd J(6, model_.nv);
        pinocchio::computeFrameJacobian(model_, data_, q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

        // J.topRows(3) is 3 x nv; must not assign to fixed-size Matrix3d when nv != 3.
        const Eigen::MatrixXd Jt_pos = J.topRows(3);
        const Eigen::Matrix3d JJt    = Jt_pos * Jt_pos.transpose();
        const double det             = JJt.determinant();
        out->translational_manip      = det > 0.0 ? static_cast<float>(std::sqrt(det)) : 0.0f;

        const int sv_count = static_cast<int>(std::min(static_cast<Eigen::Index>(6), J.rows()));
        if (sv_count > 0 && model_.nv > 0) {
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const auto& sv = svd.singularValues();
            if (sv.size() > 0 && sv(sv.size() - 1) > 1e-9) {
                out->jacobian_condition_6d = static_cast<float>(sv(0) / sv(sv.size() - 1));
            }
        } else {
            out->jacobian_condition_6d = -1.0f;
        }

        out->valid = true;
        return true;
    }

}  // namespace kinematic_viewer
