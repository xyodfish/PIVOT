/**
 * @file straight_trajectory.h
 * @brief Cartesian Space Straight Line Trajectory Planner
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_STRAIGHT_TRAJECTORY_H
#define VP_STRAIGHT_TRAJECTORY_H

#include "../velocity_planner_interface.h"
#include "../multi_velocity_planner.h"
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace vp {

/**
 * @brief Cartesian space straight line trajectory planner
 * 
 * Generates a straight-line path in Cartesian space between start and goal poses,
 * with velocity profile planning using trapezoidal or S-curve profiles.
 * 
 * The trajectory includes:
 * - Position interpolation (linear)
 * - Orientation interpolation (spherical linear interpolation for rotation)
 * - Velocity and acceleration profiling
 */
class StraightTrajectory {
   public:
    /**
     * @brief Planning mode enumeration
     */
    enum class PlanType {
        GIVEN_MULTI_BCS = 0,   ///< Multiple boundary conditions (6-DOF)
        GIVEN_SG_POINTS,       ///< Start and goal poses given
        NO_PARAMS_CONS         ///< No parameters set
    };

    StraightTrajectory() = default;

    /**
     * @brief Construct with multiple boundary conditions
     * @param BCs Boundary conditions for each DOF
     * @param alg Algorithm identifier ("TVP" or "DSVP")
     */
    StraightTrajectory(const std::vector<BCs<double>>& BCs, const std::string& alg);

    /**
     * @brief Construct with start/goal poses and boundary conditions
     * @param s_pose Start pose [x, y, z, rx, ry, rz]
     * @param g_pose Goal pose [x, y, z, rx, ry, rz]
     * @param BCs Boundary conditions (single DOF for normalized time)
     * @param alg Algorithm identifier
     */
    StraightTrajectory(const std::vector<double>& s_pose, const std::vector<double>& g_pose,
                      const std::vector<BCs<double>>& BCs, const std::string& alg);

    ~StraightTrajectory() = default;

    /**
     * @brief Get kinematic states for all DOFs
     * @return Vector of kinematic states [time, pos, vel, acc, jerk] for each DOF
     */
    std::vector<std::vector<KinematicState<double>>> getKStates() const;

    /**
     * @brief Get complete trajectory with Cartesian poses
     * @return Trajectory points [time, x, y, z, rx, ry, rz, vx, vy, vz, wx, wy, wz, ax, ay, az, ...]
     */
    std::vector<std::vector<double>> getTrajs() const;

    /**
     * @brief Set planner with multiple boundary conditions
     * @param BCs Boundary conditions
     * @param alg Algorithm identifier
     */
    void setPlanner(const std::vector<BCs<double>>& BCs, const std::string& alg);

    /**
     * @brief Set planner with start/goal poses
     * @param s_pose Start pose
     * @param g_pose Goal pose
     * @param BCs Boundary conditions
     * @param alg Algorithm identifier
     */
    void setPlanner(const std::vector<double>& s_pose, const std::vector<double>& g_pose,
                   const std::vector<BCs<double>>& BCs, const std::string& alg);

    /**
     * @brief Get raw normalized trajectory from velocity planner
     * @return Normalized trajectory [time, normalized_pos, vel, acc, jerk]
     */
    std::vector<std::vector<double>> getRawTrajs() const;

    /**
     * @brief Get planning mode
     * @return Current planning mode
     */
    PlanType getPlanType() const { return plan_type_; }

   private:
    /**
     * @brief Interpolate TCP pose between start and goal
     * @param from Start pose [x, y, z, rx, ry, rz]
     * @param to Goal pose
     * @param ratio Interpolation ratio [0, 1]
     * @return Interpolated pose
     */
    std::vector<double> interpolatePose(const std::vector<double>& from, const std::vector<double>& to, 
                                       double ratio) const;

    PlanType plan_type_{PlanType::NO_PARAMS_CONS};
    std::vector<BCs<double>> bcs_;
    std::vector<double> start_pose_;
    std::vector<double> goal_pose_;
    
    std::shared_ptr<MultiVelocityPlanner> velocity_planner_;
    
    static constexpr unsigned int PLANNER_DIM = 6;  ///< 6-DOF robot
};

}  // namespace vp

#endif  // VP_STRAIGHT_TRAJECTORY_H
