/**
 * @file trapezoidal_planner.h
 * @brief Trapezoidal Velocity Profile Planner
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_TRAPEZOIDAL_PLANNER_H
#define VP_TRAPEZOIDAL_PLANNER_H

#include "velocity_planner_interface.h"

namespace vp {

/**
 * @brief Trapezoidal velocity profile planner
 * 
 * Implements a classic trapezoidal velocity profile with:
 * - Constant acceleration phase
 * - Constant velocity phase (optional)
 * - Constant deceleration phase
 */
class TrapezoidalPlanner : public VelocityPlannerInterface<double> {
   public:
    TrapezoidalPlanner() = default;

    /**
     * @brief Construct with boundary conditions
     * @param BC Boundary conditions
     * @param name Planner identifier
     */
    TrapezoidalPlanner(std::vector<BCs<double>> BC, const std::string& name);

    ~TrapezoidalPlanner() override;

    /**
     * @brief Get kinematic state at time t
     * @param t Query time
     * @param isNormalized Use normalized time if true
     * @return Kinematic states for all DOFs
     */
    std::vector<KinematicState<double>> getKState(double t, bool isNormalized = false) override;

    /**
     * @brief Plan complete trajectory
     * @param isNormalized Use normalized time if true
     * @return Trajectory as kinematic states
     */
    std::vector<std::vector<KinematicState<double>>> planKStates(bool isNormalized = false) override;

    /**
     * @brief Plan position trajectory
     * @param isNormalized Use normalized time if true
     * @return Position trajectory
     */
    std::vector<std::vector<double>> planTrajs(bool isNormalized = false) override;

    /**
     * @brief Set boundary conditions
     * @param BC New boundary conditions
     */
    void setBoundaryConditions(BCs<double>& BC);

    /**
     * @brief Get end trajectory positions
     * @param isNormalized Use normalized time if true
     * @return End positions
     */
    std::vector<double> getEndTraj(bool isNormalized = false) override { return {}; }

   private:
    /**
     * @brief Initialize planner parameters
     * @param BC Boundary conditions
     */
    void initParams(BCs<double>& BC);

    // Trajectory parameters
    double p0_{0.0};      ///< Start position
    double p1_{0.0};      ///< Goal position
    double aa_{0.0};      ///< Acceleration amplitude
    double ad_{0.0};      ///< Deceleration amplitude
    double v0_{0.0};      ///< Start velocity
    double v1_{0.0};      ///< Goal velocity
    double vmax_{0.0};    ///< Maximum velocity
    double dt_{0.0};      ///< Time step
    double t0_{0.0};      ///< Start time
    double tTotal_{0.0};  ///< Total trajectory time

    unsigned int step_{0};  ///< Number of steps

    BCs<double> BC_;                        ///< Boundary conditions
    std::vector<KinematicState<double>> traj_;  ///< Generated trajectory

    double dp_{0.0};  ///< Position difference
};

}  // namespace vp

#endif  // VP_TRAPEZOIDAL_PLANNER_H
