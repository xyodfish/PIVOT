#pragma once

#include <optional>
#include "wbc/math/types.h"
#include "wbc/solver/hqp_solver.h"

namespace Wbc {
    namespace robot {

        struct IkResult {
            Vector solution;
            bool success;
            bool converged;
            double time_spent;
        };

        class ChainIkTrait {
           public:
            ChainIkTrait(const pinocchio::Model& model, const std::vector<Index>& fixed_joints = {});

            struct IkConfig {
                Index frame_id;
                SE3 target_pose;
                double weight = 1.0;
                std::optional<std::array<bool, 6>> mask;
            };

            void addIkTask(const std::string& frame_name, const SE3& target_pose = SE3::Identity(), double weight = 1.0,
                           const std::array<bool, 6>& mask = {1, 1, 1, 1, 1, 1});

            void removeIkTask(const std::string& frame_name) { tasks.erase(frame_name); }

            void removeAllIkTasks() { tasks.clear(); }

            void setFixedJoints(const std::vector<Index>& joints) { fixed_joints = joints; }

            void setJointBounds(const Vector& q_min_in, const Vector& q_max_in) {
                q_min = q_min_in;
                q_max = q_max_in;
            }

            void setTaskReference(const std::string& frame_name, const SE3& pose) {
                auto it = tasks.find(frame_name);
                if (it != tasks.end()) {
                    it->second.target_pose = pose;
                } else {
                    std::cout << "Task for frame " + frame_name + " not found!" << std::endl;
                }
            }

            void setTaskWeight(const std::string& frame_name, double weight) {
                auto it = tasks.find(frame_name);
                if (it != tasks.end()) {
                    it->second.weight = weight;
                } else {
                    std::cout << "Task for frame " + frame_name + " not found!" << std::endl;
                }
            }

            const std::unordered_map<std::string, IkConfig>& getTasks() const { return tasks; }
            const std::vector<Index>& getFixedJoints() const { return fixed_joints; }
            const Vector& qMin() const { return q_min; }
            const Vector& qMax() const { return q_max; }

            void setMaxIters(int iters) { max_iters = iters; }
            void setTolerance(double tolerance) { tol = tolerance; }
            void setJointWeights(double weight) { joint_weights = weight * Vector::Ones(nv); }
            void setJointWeights(const Vector& weights) {
                if (weights.size() == nv) {
                    joint_weights = weights;
                }
            }

            IkResult solveIK(const Vector& q_current, bool verbose = false);

           private:
            Size nv;
            std::unordered_map<std::string, IkConfig> tasks;  // key: frame name
            std::vector<Index> fixed_joints;

            Vector q_min, q_max;

            int max_iters = 100;
            double tol    = 1e-4;
            Vector joint_weights;

            Matrix6x J;
            solver::HQPData hqp_data;
            solver::HQPSolver hqp_solver;
            pinocchio::Model model_;
            pinocchio::Data data_;

           public:
            // void setLv1QRef(const Vector& q_ref) { q_reference = q_ref; }
            void setComRef(const Vector3& com_ref_) { com_ref = com_ref_; }
            void setComWeight(const Vector3& com_weight_) { com_weight = com_weight_; }

           private:
            void addComCost(solver::QPData& qp, const Vector& q);
            Vector3 com_ref, com_weight;
            Matrix3x J_com;
            //     Vector q_reference;
        };

    }  // namespace robot
}  // namespace Wbc
