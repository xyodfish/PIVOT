#include "kinematic_viewer/kinematic_path_planner.h"

#include "vp/velocity_planning.h"
#include "vp/geometry_trajectory/straight_trajectory.h"
#include "vp/multi_velocity_planner.h"

#include <cmath>
#include <future>
#include <fstream>
#include <glm/gtc/quaternion.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace kinematic_viewer {

    namespace {

        // Build an orthonormal basis from a normal vector
        void buildBasisFromNormal(const glm::vec3& n, glm::vec3* out_u, glm::vec3* out_v) {
            glm::vec3 norm = glm::normalize(n);
            glm::vec3 ref  = (std::abs(norm.z) < 0.9f) ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
            *out_u         = glm::normalize(glm::cross(norm, ref));
            *out_v         = glm::cross(norm, *out_u);
        }

        // Convert euler angles (roll, pitch, yaw in radians) to quaternion
        glm::quat eulerToQuat(float roll, float pitch, float yaw) {
            float cr = std::cos(roll * 0.5f);
            float sr = std::sin(roll * 0.5f);
            float cp = std::cos(pitch * 0.5f);
            float sp = std::sin(pitch * 0.5f);
            float cy = std::cos(yaw * 0.5f);
            float sy = std::sin(yaw * 0.5f);
            return glm::quat(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                             cr * cp * sy - sr * sp * cy);
        }

        std::vector<vp::KinematicState<double>> planArcLengthWithVp(double total_len, double period_sec, int num_points) {
            std::vector<vp::KinematicState<double>> samples;
            if (total_len <= 1e-9 || period_sec <= 1e-6 || num_points < 2) {
                return samples;
            }

            vp::BCs<double> bc;
            bc.start_state.pos = 0.0;
            bc.start_state.vel = 0.0;
            bc.start_state.acc = 0.0;
            bc.goal_state.pos  = total_len;
            bc.goal_state.vel  = 0.0;
            bc.goal_state.acc  = 0.0;

            const double avg_vel = total_len / period_sec;
            bc.max_vel           = std::max(1e-4, avg_vel * 1.5);
            bc.max_acc           = std::max(1e-4, bc.max_vel * 4.0 / period_sec);
            bc.max_jerk          = std::max(1e-4, bc.max_acc * 8.0 / period_sec);
            bc.delta_t           = period_sec / static_cast<double>(num_points);

            std::vector<vp::BCs<double>> bc_vec                           = {bc};
            std::shared_ptr<vp::VelocityPlannerInterface<double>> planner = std::make_shared<vp::DoubleSPlanner>(bc_vec, "DSVP");
            auto kstates                                                  = planner->planKStates(false);
            if (!kstates.empty() && !kstates[0].empty()) {
                samples = std::move(kstates[0]);
            }
            return samples;
        }

    }  // namespace

    // ------------------------------------------------------------------
    // Circle Planner
    // ------------------------------------------------------------------
    class CirclePathPlannerImpl : public CartesianPathPlanner {
       public:
        explicit CirclePathPlannerImpl(const CirclePathParams& params) : params_(params) {}

        std::string name() const override { return "Circle"; }

        CartesianPathResult plan(const glm::vec3& current_pos, const glm::quat& current_quat) override {
            CartesianPathResult result;
            if (params_.num_points < 3) {
                result.status = "错误: 路径点数太少";
                return result;
            }

            glm::vec3 u, v;
            buildBasisFromNormal(params_.normal, &u, &v);

            const double total_len = 2.0 * M_PI * static_cast<double>(params_.radius);
            const auto arc_samples = planArcLengthWithVp(total_len, static_cast<double>(params_.period_sec), params_.num_points);

            if (arc_samples.empty()) {
                result.status = "错误: vp 规划失败";
                return result;
            }

            const double time_scale =
                (arc_samples.back().time > 1e-9) ? (static_cast<double>(params_.period_sec) / arc_samples.back().time) : 1.0;
            for (const auto& ks : arc_samples) {
                const float t     = static_cast<float>(ks.time * time_scale);
                const float angle = static_cast<float>(2.0 * M_PI * (ks.pos / total_len));
                CartesianWaypoint wp;
                wp.time_sec    = t;
                wp.position    = params_.center + params_.radius * (std::cos(angle) * u + std::sin(angle) * v);
                wp.orientation = params_.lock_orientation ? current_quat : current_quat;
                result.waypoints.push_back(wp);
            }

            result.success = true;
            result.status  = "成功: 生成 " + std::to_string(result.waypoints.size()) + " 个路径点";
            return result;
        }

       private:
        CirclePathParams params_;
    };

    std::unique_ptr<CartesianPathPlanner> makeCirclePlanner(const CirclePathParams& params) {
        return std::make_unique<CirclePathPlannerImpl>(params);
    }

    // ------------------------------------------------------------------
    // Square Planner
    // ------------------------------------------------------------------
    class SquarePathPlannerImpl : public CartesianPathPlanner {
       public:
        explicit SquarePathPlannerImpl(const SquarePathParams& params) : params_(params) {}

        std::string name() const override { return "Square"; }

        CartesianPathResult plan(const glm::vec3& current_pos, const glm::quat& current_quat) override {
            CartesianPathResult result;
            if (params_.num_points < 4) {
                result.status = "错误: 路径点数太少";
                return result;
            }

            glm::vec3 u, v;
            buildBasisFromNormal(params_.normal, &u, &v);

            const float half = params_.side_length * 0.5f;
            const float r    = params_.corner_radius;

            // 4 corners + 4 edges, parameterized by perimeter
            // Perimeter = 4 * (side - 2*r) + 4 * (pi/2 * r) = 4*side - 8*r + 2*pi*r
            const float straight_len = params_.side_length - 2.0f * r;
            const float arc_len      = static_cast<float>(M_PI) * 0.5f * r;
            const float total_len    = 4.0f * straight_len + 4.0f * arc_len;
            const auto arc_samples =
                planArcLengthWithVp(static_cast<double>(total_len), static_cast<double>(params_.period_sec), params_.num_points);
            if (arc_samples.empty()) {
                result.status = "错误: vp 规划失败";
                return result;
            }
            const double time_scale =
                (arc_samples.back().time > 1e-9) ? (static_cast<double>(params_.period_sec) / arc_samples.back().time) : 1.0;

            auto sampleSquare = [&](float s) -> glm::vec3 {
                // s in [0, total_len)
                s = std::fmod(s + total_len, total_len);

                // Segment 0: bottom edge (right to left)
                float seg_len = straight_len;
                if (s < seg_len) {
                    float ratio = s / seg_len;
                    return params_.center + (half - r - ratio * straight_len) * u - (half - r) * v;
                }
                s -= seg_len;

                // Segment 1: bottom-left arc
                seg_len = arc_len;
                if (s < seg_len) {
                    float angle = static_cast<float>(M_PI) + (s / seg_len) * (static_cast<float>(M_PI) * 0.5f);
                    return params_.center + (-half + r) * u + (-half + r) * v + r * (std::cos(angle) * u + std::sin(angle) * v);
                }
                s -= seg_len;

                // Segment 2: left edge (bottom to top)
                seg_len = straight_len;
                if (s < seg_len) {
                    float ratio = s / seg_len;
                    return params_.center + (-half + r) * u + (-half + r + ratio * straight_len) * v;
                }
                s -= seg_len;

                // Segment 3: top-left arc
                seg_len = arc_len;
                if (s < seg_len) {
                    float angle = static_cast<float>(M_PI) * 1.5f + (s / seg_len) * (static_cast<float>(M_PI) * 0.5f);
                    return params_.center + (-half + r) * u + (half - r) * v + r * (std::cos(angle) * u + std::sin(angle) * v);
                }
                s -= seg_len;

                // Segment 4: top edge (left to right)
                seg_len = straight_len;
                if (s < seg_len) {
                    float ratio = s / seg_len;
                    return params_.center + (-half + r + ratio * straight_len) * u + (half - r) * v;
                }
                s -= seg_len;

                // Segment 5: top-right arc
                seg_len = arc_len;
                if (s < seg_len) {
                    float angle = 0.0f + (s / seg_len) * (static_cast<float>(M_PI) * 0.5f);
                    return params_.center + (half - r) * u + (half - r) * v + r * (std::cos(angle) * u + std::sin(angle) * v);
                }
                s -= seg_len;

                // Segment 6: right edge (top to bottom)
                seg_len = straight_len;
                if (s < seg_len) {
                    float ratio = s / seg_len;
                    return params_.center + (half - r) * u + (half - r - ratio * straight_len) * v;
                }
                s -= seg_len;

                // Segment 7: bottom-right arc
                seg_len     = arc_len;
                float angle = static_cast<float>(M_PI) * 0.5f + (s / seg_len) * (static_cast<float>(M_PI) * 0.5f);
                return params_.center + (half - r) * u + (-half + r) * v + r * (std::cos(angle) * u + std::sin(angle) * v);
            };

            for (const auto& ks : arc_samples) {
                float t = static_cast<float>(ks.time * time_scale);
                float s = static_cast<float>(ks.pos);
                CartesianWaypoint wp;
                wp.time_sec    = t;
                wp.position    = sampleSquare(s);
                wp.orientation = params_.lock_orientation ? current_quat : current_quat;
                result.waypoints.push_back(wp);
            }

            result.success = true;
            result.status  = "成功: 生成 " + std::to_string(result.waypoints.size()) + " 个路径点";
            return result;
        }

       private:
        SquarePathParams params_;
    };

    std::unique_ptr<CartesianPathPlanner> makeSquarePlanner(const SquarePathParams& params) {
        return std::make_unique<SquarePathPlannerImpl>(params);
    }

    // ------------------------------------------------------------------
    // Head Bob Planner (pitch oscillation around current pose)
    // ------------------------------------------------------------------
    class HeadBobPlannerImpl : public CartesianPathPlanner {
       public:
        explicit HeadBobPlannerImpl(const HeadBobParams& params) : params_(params) {}

        std::string name() const override { return "HeadBob"; }

        CartesianPathResult plan(const glm::vec3& current_pos, const glm::quat& current_quat) override {
            CartesianPathResult result;
            if (params_.num_points < 2) {
                result.status = "错误: 路径点数太少";
                return result;
            }

            const float dt      = params_.period_sec / static_cast<float>(params_.num_points);
            const float amp_rad = glm::radians(params_.pitch_amplitude_deg);

            for (int i = 0; i <= params_.num_points; ++i) {
                float t     = static_cast<float>(i) * dt;
                float phase = 2.0f * static_cast<float>(M_PI) * static_cast<float>(i) / static_cast<float>(params_.num_points);
                float pitch = amp_rad * std::sin(phase);

                CartesianWaypoint wp;
                wp.time_sec = t;
                wp.position = current_pos;
                // Apply pitch rotation around local X axis
                glm::quat pitch_rot = eulerToQuat(0.0f, pitch, 0.0f);
                wp.orientation      = current_quat * pitch_rot;
                result.waypoints.push_back(wp);
            }

            result.success = true;
            result.status  = "成功: 生成 " + std::to_string(result.waypoints.size()) + " 个路径点";
            return result;
        }

       private:
        HeadBobParams params_;
    };

    std::unique_ptr<CartesianPathPlanner> makeHeadBobPlanner(const HeadBobParams& params) {
        return std::make_unique<HeadBobPlannerImpl>(params);
    }

    // ------------------------------------------------------------------
    // Straight Line Planner (uses vp::StraightTrajectory)
    // ------------------------------------------------------------------
    class StraightPathPlannerImpl : public CartesianPathPlanner {
       public:
        explicit StraightPathPlannerImpl(const StraightPathParams& params) : params_(params) {}

        std::string name() const override { return "Straight"; }

        CartesianPathResult plan(const glm::vec3& current_pos, const glm::quat& current_quat) override {
            CartesianPathResult result;

            // Use vp::StraightTrajectory for straight line planning
            std::vector<double> start_pose = {static_cast<double>(params_.start_pos.x),
                                              static_cast<double>(params_.start_pos.y),
                                              static_cast<double>(params_.start_pos.z),
                                              0.0,
                                              0.0,
                                              0.0};
            std::vector<double> goal_pose  = {static_cast<double>(params_.goal_pos.x),
                                              static_cast<double>(params_.goal_pos.y),
                                              static_cast<double>(params_.goal_pos.z),
                                              0.0,
                                              0.0,
                                              0.0};

            vp::BCs<double> bc;
            bc.start_state.pos = 0.0;
            bc.start_state.vel = 0.0;
            bc.goal_state.pos  = 1.0;
            bc.goal_state.vel  = 0.0;
            bc.max_vel         = params_.max_vel;
            bc.max_acc         = params_.max_acc;
            bc.max_jerk        = params_.max_acc * 2.0;  // Conservative jerk
            bc.delta_t         = params_.delta_t;

            try {
                vp::StraightTrajectory straight_traj(start_pose, goal_pose, {bc}, params_.profile);
                auto traj = straight_traj.getTrajs();

                const size_t n = traj.size();
                for (size_t i = 0; i < n; ++i) {
                    const auto& point = traj[i];
                    CartesianWaypoint wp;
                    wp.time_sec       = static_cast<float>(point[0]);
                    wp.position       = glm::vec3(static_cast<float>(point[1]), static_cast<float>(point[2]), static_cast<float>(point[3]));
                    const float ratio = (n > 1) ? static_cast<float>(i) / static_cast<float>(n - 1) : 1.0f;
                    wp.orientation    = glm::normalize(glm::slerp(params_.start_quat, params_.goal_quat, ratio));
                    result.waypoints.push_back(wp);
                }

                result.success = true;
                result.status  = "成功: 生成 " + std::to_string(result.waypoints.size()) + " 个路径点";
            } catch (const std::exception& ex) {
                result.status = std::string("规划失败: ") + ex.what();
            }

            return result;
        }

       private:
        StraightPathParams params_;
    };

    std::unique_ptr<CartesianPathPlanner> makeStraightPlanner(const StraightPathParams& params) {
        return std::make_unique<StraightPathPlannerImpl>(params);
    }

    // ------------------------------------------------------------------
    // IK Solver for Cartesian Path (serial solve with scene state update)
    // ------------------------------------------------------------------
    JointSpaceTrajectory solveIkForCartesianPath(const CartesianPathResult& cartesian_path, rkv::RobotScene* scene,
                                                 rkv::IkSolver* solver, int chain_index,
                                                 const IkSolveProgressCallback& progress_cb) {
        JointSpaceTrajectory result;
        if (!cartesian_path.success || cartesian_path.waypoints.empty()) {
            result.status = "错误: 笛卡尔路径无效";
            return result;
        }
        if (scene == nullptr) {
            result.status = "错误: 场景未初始化";
            return result;
        }
        if (solver == nullptr) {
            result.status = "错误: IK 求解器未初始化";
            return result;
        }
        if (chain_index < 0 || chain_index >= solver->chainCount()) {
            result.status = "错误: 无效的 IK 链索引";
            return result;
        }

        const auto& chain_status = solver->chainStatus(chain_index);
        if (!chain_status.ready) {
            result.status = "错误: IK 链未就绪: " + chain_status.error;
            return result;
        }

        std::vector<std::string> joint_names;
        std::vector<std::vector<float>> all_joint_positions;
        std::vector<float> times;

        int success_count = 0;
        int fail_count    = 0;
        const int total   = static_cast<int>(cartesian_path.waypoints.size());

        for (size_t i = 0; i < cartesian_path.waypoints.size(); ++i) {
            const auto& wp = cartesian_path.waypoints[i];

            // Report progress before solving
            if (progress_cb) {
                progress_cb(static_cast<int>(i), total, "求解中... " + std::to_string(i) + "/" + std::to_string(total));
            }

            // Build target transform from position + quaternion
            glm::mat4 target_world = glm::mat4_cast(wp.orientation);
            target_world[3]        = glm::vec4(wp.position, 1.0f);

            std::string ik_status;
            bool ik_success = solver->solveSingleChain(scene, chain_index, target_world, &ik_status);

            if (ik_success) {
                ++success_count;

                // On first successful solve, collect joint names
                if (joint_names.empty()) {
                    auto all_joints = scene->getJointInfos();
                    for (const auto& js : all_joints) {
                        joint_names.push_back(js.name);
                    }
                }

                // Collect joint positions
                std::vector<float> positions;
                const auto all_joints = scene->getJointInfos();
                positions.reserve(all_joints.size());
                for (const auto& js : all_joints) {
                    positions.push_back(js.position);
                }

                times.push_back(wp.time_sec);
                all_joint_positions.push_back(std::move(positions));
            } else {
                ++fail_count;
            }
        }

        if (success_count == 0) {
            result.status = "错误: 所有路径点 IK 求解失败";
            return result;
        }

        result.times           = std::move(times);
        result.joint_positions = std::move(all_joint_positions);
        result.joint_names     = std::move(joint_names);
        result.success         = true;
        result.status = "成功: IK 求解 " + std::to_string(success_count) + "/" + std::to_string(cartesian_path.waypoints.size()) + " 点";
        if (fail_count > 0) {
            result.status += " (" + std::to_string(fail_count) + " 点失败)";
        }
        return result;
    }

    // ------------------------------------------------------------------
    // IK Solver for Cartesian Path (Full Body Mode)
    // ------------------------------------------------------------------
    JointSpaceTrajectory solveIkForCartesianPathFullBody(const CartesianPathResult& cartesian_path, rkv::RobotScene* scene,
                                                         rkv::IkSolver* solver, int chain_index,
                                                         const IkSolveProgressCallback& progress_cb) {
        JointSpaceTrajectory result;
        if (!cartesian_path.success || cartesian_path.waypoints.empty()) {
            result.status = "错误: 笛卡尔路径无效";
            return result;
        }
        if (scene == nullptr) {
            result.status = "错误: 场景未初始化";
            return result;
        }
        if (solver == nullptr) {
            result.status = "错误: IK 求解器未初始化";
            return result;
        }
        if (chain_index < 0 || chain_index >= solver->chainCount()) {
            result.status = "错误: 无效的 IK 链索引";
            return result;
        }

        const auto& chain_status = solver->chainStatus(chain_index);
        if (!chain_status.ready) {
            result.status = "错误: IK 链未就绪: " + chain_status.error;
            return result;
        }

        // Get current pose of all chains as their targets (they stay still)
        std::vector<glm::mat4> current_targets;
        for (int i = 0; i < solver->chainCount(); ++i) {
            glm::vec3 pos(0.0f);
            glm::vec3 rpy(0.0f);
            if (solver->fetchTipWorldPose(*scene, i, &pos, &rpy)) {
                glm::mat4 target = glm::mat4_cast(glm::quat(glm::vec3(rpy.x, rpy.y, rpy.z)));
                target[3]        = glm::vec4(pos, 1.0f);
                current_targets.push_back(target);
            } else {
                current_targets.push_back(glm::mat4(1.0f));
            }
        }

        std::vector<std::string> joint_names;
        std::vector<std::vector<float>> all_joint_positions;
        std::vector<float> times;

        int success_count = 0;
        int fail_count    = 0;
        const int total   = static_cast<int>(cartesian_path.waypoints.size());

        for (size_t i = 0; i < cartesian_path.waypoints.size(); ++i) {
            const auto& wp = cartesian_path.waypoints[i];

            // Report progress before solving
            if (progress_cb) {
                progress_cb(static_cast<int>(i), total, "全身求解中... " + std::to_string(i) + "/" + std::to_string(total));
            }

            // Update target for the active chain
            std::vector<glm::mat4> targets               = current_targets;
            targets[static_cast<size_t>(chain_index)]    = glm::mat4_cast(wp.orientation);
            targets[static_cast<size_t>(chain_index)][3] = glm::vec4(wp.position, 1.0f);

            rkv::IkSolveStats stats;
            std::string ik_status;
            bool ik_success = solver->solveFullBody(scene, targets, 1, chain_index, false, false, &stats, &ik_status);

            if (ik_success && stats.active_chain_solved) {
                ++success_count;

                // On first successful solve, collect joint names
                if (joint_names.empty()) {
                    auto all_joints = scene->getJointInfos();
                    for (const auto& js : all_joints) {
                        joint_names.push_back(js.name);
                    }
                }

                // Collect joint positions
                std::vector<float> positions;
                const auto all_joints = scene->getJointInfos();
                positions.reserve(all_joints.size());
                for (const auto& js : all_joints) {
                    positions.push_back(js.position);
                }

                times.push_back(wp.time_sec);
                all_joint_positions.push_back(std::move(positions));
            } else {
                ++fail_count;
            }
        }

        if (success_count == 0) {
            result.status = "错误: 所有路径点全身 IK 求解失败";
            return result;
        }

        result.times           = std::move(times);
        result.joint_positions = std::move(all_joint_positions);
        result.joint_names     = std::move(joint_names);
        result.success         = true;
        result.status =
            "成功: 全身 IK 求解 " + std::to_string(success_count) + "/" + std::to_string(cartesian_path.waypoints.size()) + " 点";
        if (fail_count > 0) {
            result.status += " (" + std::to_string(fail_count) + " 点失败)";
        }
        return result;
    }

    namespace {

        vp::BCs<double> makeJointPTPBoundaryCondition(const JointSpacePTPParams& params, size_t dof_index) {
            vp::BCs<double> bc;
            bc.start_state.pos = static_cast<double>(params.start_positions[dof_index]);
            bc.start_state.vel = 0.0;
            bc.start_state.acc = 0.0;
            bc.goal_state.pos  = static_cast<double>(params.goal_positions[dof_index]);
            bc.goal_state.vel  = 0.0;
            bc.goal_state.acc  = 0.0;
            bc.max_vel         = static_cast<double>(params.max_vel);
            bc.max_acc         = static_cast<double>(params.max_acc);
            bc.max_jerk        = static_cast<double>(params.max_jerk);
            bc.delta_t         = static_cast<double>(params.delta_t);
            return bc;
        }

        bool allJointsStationary(const JointSpacePTPParams& params) {
            for (size_t i = 0; i < params.joint_names.size(); ++i) {
                if (std::fabs(static_cast<double>(params.goal_positions[i] - params.start_positions[i])) >= 1e-6) {
                    return false;
                }
            }
            return true;
        }

        JointSpaceTrajectory makeStationaryTrajectory(const JointSpacePTPParams& params) {
            JointSpaceTrajectory result;
            result.joint_names = params.joint_names;
            result.times.push_back(0.0f);
            result.joint_positions.push_back(params.start_positions);
            result.success     = true;
            result.status      = "成功: 所有关节无运动";
            return result;
        }

        JointSpaceTrajectory planJointSpacePTPHold(const JointSpacePTPParams& params) {
            JointSpaceTrajectory result;

            const size_t n_dofs = params.joint_names.size();
            std::vector<std::vector<vp::KinematicState<double>>> per_dof_trajs;
            per_dof_trajs.reserve(n_dofs);
            double max_total_time = 0.0;

            for (size_t i = 0; i < n_dofs; ++i) {
                const double start_pos = static_cast<double>(params.start_positions[i]);
                const double goal_pos  = static_cast<double>(params.goal_positions[i]);
                const double dp        = std::fabs(goal_pos - start_pos);

                if (dp < 1e-6) {
                    std::vector<vp::KinematicState<double>> fallback;
                    vp::KinematicState<double> s0, s1;
                    s0.time = 0.0;
                    s0.pos  = start_pos;
                    s0.vel  = 0.0;
                    s1.time = std::max(static_cast<double>(params.delta_t), 0.02);
                    s1.pos  = goal_pos;
                    s1.vel  = 0.0;
                    fallback.push_back(s0);
                    fallback.push_back(s1);
                    per_dof_trajs.push_back(fallback);
                    max_total_time = std::max(max_total_time, fallback.back().time);
                    continue;
                }

                const vp::BCs<double> bc = makeJointPTPBoundaryCondition(params, i);
                std::vector<vp::BCs<double>> bc_vec = {bc};
                std::shared_ptr<vp::VelocityPlannerInterface<double>> planner;
                if (params.profile == "TVP") {
                    planner = std::make_shared<vp::TrapezoidalPlanner>(bc_vec, "TVP");
                } else {
                    planner = std::make_shared<vp::DoubleSPlanner>(bc_vec, "DSVP");
                }

                auto kstates = planner->planKStates(false);
                if (kstates.empty() || kstates[0].empty()) {
                    result.status = "错误: DOF " + std::to_string(i) + " 规划返回空轨迹";
                    return result;
                }
                per_dof_trajs.push_back(kstates[0]);
                max_total_time = std::max(max_total_time, kstates[0].back().time);
            }

            if (max_total_time < 1e-9) {
                return makeStationaryTrajectory(params);
            }

            result.joint_names = params.joint_names;
            const double dt    = static_cast<double>(params.delta_t);
            const int n_steps  = static_cast<int>(std::ceil(max_total_time / dt)) + 1;

            for (int step = 0; step < n_steps; ++step) {
                const double t = std::min(static_cast<double>(step) * dt, max_total_time);
                result.times.push_back(static_cast<float>(t));

                std::vector<float> positions;
                positions.reserve(n_dofs);
                for (size_t i = 0; i < n_dofs; ++i) {
                    const auto& traj = per_dof_trajs[i];
                    size_t idx       = 0;
                    while (idx + 1 < traj.size() && traj[idx + 1].time <= t) {
                        ++idx;
                    }
                    if (idx + 1 >= traj.size()) {
                        positions.push_back(static_cast<float>(traj.back().pos));
                    } else {
                        const double t0 = traj[idx].time;
                        const double t1 = traj[idx + 1].time;
                        const double p0 = traj[idx].pos;
                        const double p1 = traj[idx + 1].pos;
                        if (t1 - t0 < 1e-12) {
                            positions.push_back(static_cast<float>(p0));
                        } else {
                            const double ratio = (t - t0) / (t1 - t0);
                            positions.push_back(static_cast<float>(p0 + ratio * (p1 - p0)));
                        }
                    }
                }
                result.joint_positions.push_back(std::move(positions));
            }

            result.success = true;
            result.status  = "成功[先到先停]: 生成 " + std::to_string(result.times.size()) + " 个轨迹点, 时长 " +
                            std::to_string(result.times.back()) + "s";
            return result;
        }

        JointSpaceTrajectory planJointSpacePTPTimeScaling(const JointSpacePTPParams& params) {
            JointSpaceTrajectory result;

            if (allJointsStationary(params)) {
                return makeStationaryTrajectory(params);
            }

            std::vector<vp::BCs<double>> bcs;
            bcs.reserve(params.joint_names.size());
            for (size_t i = 0; i < params.joint_names.size(); ++i) {
                bcs.push_back(makeJointPTPBoundaryCondition(params, i));
            }

            vp::DoubleSMultiPlanner multi_planner(bcs, "MDSVP");
            const auto per_dof_trajs = multi_planner.planKStates(false);
            if (per_dof_trajs.empty() || per_dof_trajs[0].empty()) {
                result.status = "错误: 时间缩放同步规划返回空轨迹";
                return result;
            }

            result.joint_names = params.joint_names;
            const size_t n_points = per_dof_trajs[0].size();
            result.times.reserve(n_points);
            result.joint_positions.reserve(n_points);

            for (size_t point = 0; point < n_points; ++point) {
                result.times.push_back(static_cast<float>(per_dof_trajs[0][point].time));
                std::vector<float> positions;
                positions.reserve(per_dof_trajs.size());
                for (const auto& dof_traj : per_dof_trajs) {
                    positions.push_back(static_cast<float>(dof_traj[point].pos));
                }
                result.joint_positions.push_back(std::move(positions));
            }

            result.success = true;
            result.status  = "成功[时间缩放]: 生成 " + std::to_string(result.times.size()) + " 个轨迹点, 时长 " +
                            std::to_string(result.times.back()) + "s (DSVP)";
            return result;
        }

    }  // namespace

    // ------------------------------------------------------------------
    // Joint-Space Point-to-Point Velocity Planning
    // ------------------------------------------------------------------
    JointSpaceTrajectory planJointSpacePTP(const JointSpacePTPParams& params) {
        JointSpaceTrajectory result;

        if (params.joint_names.empty()) {
            result.status = "错误: 关节名列表为空";
            return result;
        }
        if (params.start_positions.size() != params.joint_names.size() || params.goal_positions.size() != params.joint_names.size()) {
            result.status = "错误: 关节位置维度与关节名数量不匹配";
            return result;
        }

        try {
            if (params.sync_mode == "time_scaling") {
                return planJointSpacePTPTimeScaling(params);
            }
            return planJointSpacePTPHold(params);
        } catch (const std::exception& ex) {
            result.status = std::string("关节空间规划失败: ") + ex.what();
        } catch (...) {
            result.status = "关节空间规划失败: 未知异常";
        }

        return result;
    }

    // ------------------------------------------------------------------
    // Export to CSV
    // ------------------------------------------------------------------
    bool exportTrajectoryToCsv(const JointSpaceTrajectory& traj, const std::string& file_path) {
        if (traj.joint_names.empty() || traj.joint_positions.empty()) {
            return false;
        }

        std::ofstream file(file_path);
        if (!file.is_open()) {
            return false;
        }

        // Header
        file << "time";
        for (const auto& name : traj.joint_names) {
            file << "," << name;
        }
        file << "\n";

        // Data
        for (size_t i = 0; i < traj.times.size() && i < traj.joint_positions.size(); ++i) {
            file << std::fixed << std::setprecision(4) << traj.times[i];
            for (float pos : traj.joint_positions[i]) {
                file << "," << pos;
            }
            file << "\n";
        }

        file.close();
        return true;
    }

}  // namespace kinematic_viewer
