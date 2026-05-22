/**
 * @file straight_trajectory.cpp
 * @brief Cartesian Space Straight Line Trajectory Implementation
 */

#include "geometry_trajectory/straight_trajectory.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace vp {

// Helper function for pose interpolation (linear position + slerp rotation)
static std::vector<double> interpolateTCPPose(const std::vector<double>& from, const std::vector<double>& to, 
                                              double ratio) {
    if (from.size() != 6 || to.size() != 6) {
        throw std::invalid_argument("Pose must have 6 elements [x, y, z, rx, ry, rz]");
    }
    
    std::vector<double> result(6);
    
    // Linear interpolation for position
    for (int i = 0; i < 3; ++i) {
        result[i] = from[i] + ratio * (to[i] - from[i]);
    }
    
    // For orientation (rx, ry, rz), we use linear interpolation as approximation
    // In a full implementation, you would convert to quaternion and use SLERP
    for (int i = 3; i < 6; ++i) {
        result[i] = from[i] + ratio * (to[i] - from[i]);
    }
    
    return result;
}

StraightTrajectory::StraightTrajectory(const std::vector<BCs<double>>& BCs, const std::string& alg)
    : bcs_(BCs) {
    plan_type_ = PlanType::GIVEN_MULTI_BCS;
    setPlanner(BCs, alg);
}

StraightTrajectory::StraightTrajectory(const std::vector<double>& s_pose, const std::vector<double>& g_pose,
                                      const std::vector<BCs<double>>& BCs, const std::string& alg)
    : start_pose_(s_pose), goal_pose_(g_pose), bcs_(BCs) {
    plan_type_ = PlanType::GIVEN_SG_POINTS;
    setPlanner(s_pose, g_pose, BCs, alg);
}

void StraightTrajectory::setPlanner(const std::vector<BCs<double>>& BCs, const std::string& alg) {
    if ((plan_type_ == PlanType::GIVEN_MULTI_BCS && BCs.size() != PLANNER_DIM) ||
        (plan_type_ == PlanType::GIVEN_SG_POINTS && BCs.size() != 1)) {
        throw std::invalid_argument("Boundary conditions size mismatch. Expected " + 
                                   std::to_string(plan_type_ == PlanType::GIVEN_MULTI_BCS ? PLANNER_DIM : 1) +
                                   ", got " + std::to_string(BCs.size()));
    }
    
    velocity_planner_ = std::make_shared<MultiVelocityPlanner>(BCs, alg);
}

void StraightTrajectory::setPlanner(const std::vector<double>& s_pose, const std::vector<double>& g_pose,
                                   const std::vector<BCs<double>>& BCs, const std::string& alg) {
    start_pose_ = s_pose;
    goal_pose_  = g_pose;
    bcs_        = BCs;
    
    velocity_planner_ = std::make_shared<MultiVelocityPlanner>(BCs, alg);
    plan_type_ = PlanType::GIVEN_SG_POINTS;
}

std::vector<std::vector<KinematicState<double>>> StraightTrajectory::getKStates() const {
    if (!velocity_planner_) {
        throw std::runtime_error("Velocity planner not initialized");
    }
    return velocity_planner_->getKStates();
}

std::vector<std::vector<double>> StraightTrajectory::getTrajs() const {
    if (!velocity_planner_) {
        throw std::runtime_error("Velocity planner not initialized");
    }
    
    if (plan_type_ == PlanType::GIVEN_MULTI_BCS) {
        // Return joint-space trajectory directly
        return velocity_planner_->getTrajs();
    } else if (plan_type_ == PlanType::GIVEN_SG_POINTS) {
        // Cartesian space straight line with pose interpolation
        
        // Get normalized trajectory from velocity planner
        auto normalized_trajs = velocity_planner_->getTrajs();
        auto traj_num = normalized_trajs.size();
        
        if (traj_num == 0) {
            return {};
        }
        
        // Output format: [time, x, y, z, rx, ry, rz, vx, vy, vz, wx, wy, wz, ax, ay, az, ...]
        // Total: 1 (time) + 6*3 (pos, vel, acc) = 19 columns
        std::vector<std::vector<double>> cartesian_trajs(traj_num, std::vector<double>(1 + PLANNER_DIM * 3, 0.0));
        
        // Step 1: Interpolate poses
        for (size_t i = 0; i < traj_num; ++i) {
            double ratio = normalized_trajs[i][1];  // Normalized position [0, 1]
            
            // Interpolate Cartesian pose
            auto interpolated_pose = interpolateTCPPose(start_pose_, goal_pose_, ratio);
            
            // Store time and pose
            cartesian_trajs[i][0] = normalized_trajs[i][0];  // Time
            std::copy(interpolated_pose.begin(), interpolated_pose.end(), 
                     cartesian_trajs[i].begin() + 1);
        }
        
        // Step 2: Calculate velocities (numerical differentiation)
        double max_linear_vel = 0.0;
        double max_angular_vel = 0.0;
        
        for (size_t i = 1; i < traj_num; ++i) {
            double dt = cartesian_trajs[i][0] - cartesian_trajs[i-1][0];
            if (dt < 1e-6) continue;
            
            for (int j = 0; j < PLANNER_DIM; ++j) {
                // Velocity
                cartesian_trajs[i][1 + PLANNER_DIM + j] = 
                    (cartesian_trajs[i][1 + j] - cartesian_trajs[i-1][1 + j]) / dt;
            }
            
            // Calculate linear and angular velocity magnitudes
            double lin_vel = std::sqrt(
                std::pow(cartesian_trajs[i][7], 2) + 
                std::pow(cartesian_trajs[i][8], 2) + 
                std::pow(cartesian_trajs[i][9], 2)
            );
            double ang_vel = std::sqrt(
                std::pow(cartesian_trajs[i][10], 2) + 
                std::pow(cartesian_trajs[i][11], 2) + 
                std::pow(cartesian_trajs[i][12], 2)
            );
            
            max_linear_vel = std::max(max_linear_vel, lin_vel);
            max_angular_vel = std::max(max_angular_vel, ang_vel);
        }
        
        // Step 3: Calculate accelerations
        double max_linear_acc = 0.0;
        double max_angular_acc = 0.0;
        
        for (size_t i = 1; i < traj_num; ++i) {
            double dt = cartesian_trajs[i][0] - cartesian_trajs[i-1][0];
            if (dt < 1e-6) continue;
            
            for (int j = 0; j < PLANNER_DIM; ++j) {
                // Acceleration
                cartesian_trajs[i][1 + 2*PLANNER_DIM + j] = 
                    (cartesian_trajs[i][1 + PLANNER_DIM + j] - 
                     cartesian_trajs[i-1][1 + PLANNER_DIM + j]) / dt;
            }
            
            // Calculate linear and angular acceleration magnitudes
            double lin_acc = std::sqrt(
                std::pow(cartesian_trajs[i][13], 2) + 
                std::pow(cartesian_trajs[i][14], 2) + 
                std::pow(cartesian_trajs[i][15], 2)
            );
            double ang_acc = std::sqrt(
                std::pow(cartesian_trajs[i][16], 2) + 
                std::pow(cartesian_trajs[i][17], 2) + 
                std::pow(cartesian_trajs[i][18], 2)
            );
            
            max_linear_acc = std::max(max_linear_acc, lin_acc);
            max_angular_acc = std::max(max_angular_acc, ang_acc);
        }
        
        // Log statistics (optional, can be removed in production)
        // std::cout << "Max linear velocity: " << max_linear_vel << " m/s" << std::endl;
        // std::cout << "Max angular velocity: " << max_angular_vel << " rad/s" << std::endl;
        // std::cout << "Max linear acceleration: " << max_linear_acc << " m/s²" << std::endl;
        // std::cout << "Max angular acceleration: " << max_angular_acc << " rad/s²" << std::endl;
        
        return cartesian_trajs;
    }
    
    return {};
}

std::vector<std::vector<double>> StraightTrajectory::getRawTrajs() const {
    if (!velocity_planner_) {
        throw std::runtime_error("Velocity planner not initialized");
    }
    return velocity_planner_->getTrajs();
}

std::vector<double> StraightTrajectory::interpolatePose(const std::vector<double>& from, 
                                                        const std::vector<double>& to, 
                                                        double ratio) const {
    return interpolateTCPPose(from, to, ratio);
}

}  // namespace vp
