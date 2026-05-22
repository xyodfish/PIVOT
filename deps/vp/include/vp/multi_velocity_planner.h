/**
 * @file multi_velocity_planner.h
 * @brief Multi-DOF Velocity Planner with Factory Pattern
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_MULTI_VELOCITY_PLANNER_H
#define VP_MULTI_VELOCITY_PLANNER_H

#include <memory>
#include <string>
#include "velocity_planner_interface.h"

namespace vp {

/**
 * @brief Multi-DOF velocity planner that manages multiple single-DOF planners
 */
class MultiVelocityPlanner {
   public:
    /**
     * @brief Construct with boundary conditions and algorithm type
     * @param BCs Boundary conditions for each DOF
     * @param algorithm Algorithm type ("TVP" for trapezoidal, "DSVP" for double-S)
     */
    MultiVelocityPlanner(const std::vector<BCs<double>>& BCs, const std::string& algorithm);

    /**
     * @brief Construct with boundary conditions, algorithm, and parameters
     * @param BCs Boundary conditions
     * @param algorithm Algorithm type
     * @param params Planner parameters
     */
    MultiVelocityPlanner(const std::vector<BCs<double>>& BCs, const std::string& algorithm,
                        std::shared_ptr<VpParams3D> params);

    ~MultiVelocityPlanner();

    /**
     * @brief Get kinematic states at time t
     * @param isNormalized Use normalized time if true
     * @return Kinematic states for all DOFs
     */
    std::vector<std::vector<KinematicState<double>>> getKStates(bool isNormalized = false);

    /**
     * @brief Get trajectory positions
     * @param isNormalized Use normalized time if true
     * @return Position trajectories
     */
    std::vector<std::vector<double>> getTrajs(bool isNormalized = false);

    /**
     * @brief Get position vector
     * @return Positions for all DOFs
     */
    std::vector<std::vector<double>> getPosVec();

    /**
     * @brief Get end trajectory positions
     * @param isNormalized Use normalized time if true
     * @return End positions
     */
    std::vector<double> getEndTraj(bool isNormalized = false);

    /**
     * @brief Initialize default boundary conditions
     * @param BCs Boundary conditions to initialize
     * @param vel Maximum velocity
     * @param acc Maximum acceleration
     * @param jerk Maximum jerk
     * @param dt Time step
     */
    static void initDefaultBCs(std::vector<BCs<double>>& BCs, double vel, double acc, double jerk, double dt);

   private:
    /**
     * @brief Create velocity planner instance
     * @param BCs Boundary conditions
     * @param algo Algorithm type
     * @param params Planner parameters
     * @return Shared pointer to planner interface
     */
    std::shared_ptr<VelocityPlannerInterface<double>> createVelocityPlanner(
        const std::vector<BCs<double>>& BCs, const std::string& algo, std::shared_ptr<VpParams3D> params);

    std::shared_ptr<VelocityPlannerInterface<double>> planner_;  ///< Active planner
    std::shared_ptr<VpParams3D> params_;                         ///< Planner parameters
};

}  // namespace vp

#endif  // VP_MULTI_VELOCITY_PLANNER_H
