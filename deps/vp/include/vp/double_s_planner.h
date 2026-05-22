/**
 * @file double_s_planner.h
 * @brief Double-S Velocity Profile Planner (S-curve)
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_DOUBLE_S_PLANNER_H
#define VP_DOUBLE_S_PLANNER_H

#include "velocity_planner_interface.h"
#include <memory>
#include <vector>

// Forward declarations
namespace vp {
    class ConstantJerkTrajectory;
    class PiecewiseTrajectory;
}

namespace vp {

/**
 * @brief Double-S velocity profile planner with jerk limitation
 * 
 * Implements a smooth S-curve velocity profile with 7 phases:
 * 1. Positive jerk (increasing acceleration)
 * 2. Constant acceleration
 * 3. Negative jerk (decreasing acceleration)
 * 4. Constant velocity
 * 5. Negative jerk (increasing deceleration)
 * 6. Constant deceleration
 * 7. Positive jerk (decreasing deceleration)
 */
class DoubleSPlanner : public VelocityPlannerInterface<double> {
   public:
    DoubleSPlanner() = default;

    /**
     * @brief Construct with boundary conditions
     * @param BCs Boundary conditions for each DOF
     * @param name Planner identifier
     */
    DoubleSPlanner(std::vector<BCs<double>> BCs, const std::string& name);

    /**
     * @brief Construct with gamma parameter for time scaling
     * @param BCs Boundary conditions
     * @param name Planner identifier
     * @param gamma Time scaling factor (0-1)
     */
    DoubleSPlanner(std::vector<BCs<double>> BCs, const std::string& name, double gamma);

    ~DoubleSPlanner() override;

    /**
     * @brief Get kinematic state at time t
     * @param t Query time
     * @param isNormalized Use normalized time if true
     * @return Kinematic states for all DOFs
     */
    std::vector<KinematicState<double>> getKState(double t, bool isNormalized = false) override;

    /**
     * @brief Get trajectory at time t
     * @param t Query time
     * @param isNormalized Use normalized time if true
     * @return Position, velocity, acceleration, jerk
     */
    std::vector<double> getTraj(double t, bool isNormalized = false);

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
     * @brief Set maximum velocity
     * @param vel Maximum velocity
     */
    void setMaxVel(double vel);

    /**
     * @brief Set maximum acceleration
     * @param acc Maximum acceleration
     */
    void setMaxAcc(double acc);

    /**
     * @brief Set maximum jerk
     * @param jerk Maximum jerk
     */
    void setMaxJerk(double jerk);

    /**
     * @brief Get boundary conditions
     * @return Current boundary conditions
     */
    BCs<double> getBoundaryCondition() { return BC_; }

    /**
     * @brief Check if planning is needed
     * @return True if planning is needed
     */
    bool needsPlanning() const { return needs_plan_; }

    /**
     * @brief Get end trajectory positions
     * @param isNormalized Use normalized time if true
     * @return End positions
     */
    std::vector<double> getEndTraj(bool isNormalized = false) override;

    // Public members for compatibility
    double tTotal{0.0};   ///< Total trajectory time
    double dt{0.0};       ///< Time step
    double aLimitA{0.0};  ///< Acceleration limit (acceleration phase)
    double aLimitD{0.0};  ///< Deceleration limit
    double vLimit{0.0};   ///< Velocity limit
    double dp{0.0};       ///< Position difference
    std::vector<double> t; ///< Phase times

   private:
    /**
     * @brief Initialize planner parameters
     */
    void initParams();

    /**
     * @brief Calculate trajectory with gradually reducing acceleration
     * @param BC Boundary conditions
     * @param gamma Reduction factor
     */
    void calWithGradReducingAcc(BCs<double>& BC, double gamma);

    /**
     * @brief Splice trajectory segments together
     */
    void splicingTrajectorySegments();

    BCs<double> BC_;                        ///< Boundary conditions
    bool needs_plan_{true};                 ///< Flag indicating if planning is needed
    double gamma_{0.95};                    ///< Time scaling factor
    int m_sigma{1};                         ///< Direction factor (+1 or -1)
    
    // Phase durations
    double tAcc{0.0}, tDec{0.0};
    
    // Trajectory segments
    std::shared_ptr<PiecewiseTrajectory> trajectory_;
    std::vector<std::shared_ptr<ConstantJerkTrajectory>> cjt_sptrs;
};

}  // namespace vp

#endif  // VP_DOUBLE_S_PLANNER_H
