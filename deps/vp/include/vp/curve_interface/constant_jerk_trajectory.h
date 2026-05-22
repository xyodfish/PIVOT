/**
 * @file constant_jerk_trajectory.h
 * @brief Constant Jerk Trajectory Segment
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_CONSTANT_JERK_TRAJECTORY_H
#define VP_CONSTANT_JERK_TRAJECTORY_H

#include "curve_interface.h"

namespace vp {

/**
 * @brief Trajectory segment with constant jerk
 * 
 * Models motion with:
 * - Constant jerk (3rd derivative of position)
 * - Linearly changing acceleration
 * - Quadratically changing velocity
 * - Cubically changing position
 */
class ConstantJerkTrajectory : public CurveInterface {
   public:
    ConstantJerkTrajectory() = default;

    /**
     * @brief Construct constant jerk trajectory
     * @param p0 Initial position
     * @param v0 Initial velocity
     * @param a0 Initial acceleration
     * @param jerk Constant jerk value
     * @param duration Time duration of segment
     */
    ConstantJerkTrajectory(const double p0, const double v0, const double a0, 
                          const double jerk, const double duration);

    ~ConstantJerkTrajectory() override = default;

    /**
     * @brief Evaluate trajectory at parameter t
     * @param t Time parameter
     * @param order Derivative order (0=pos, 1=vel, 2=acc, 3=jerk)
     * @return Evaluated value
     */
    double evaluate(const double t, const unsigned int order = 0) const override;

    /**
     * @brief Get parameter length (duration)
     * @return Duration of segment
     */
    double paramLength() const override;

    // Getters
    double startPosition() const { return p0_; }
    double startVelocity() const { return v0_; }
    double startAcceleration() const { return a0_; }
    double endPosition() const { return p1_; }
    double endVelocity() const { return v1_; }
    double endAcceleration() const { return a1_; }
    double jerk() const { return jerk_; }
    double duration() const { return duration_; }

   private:
    double p0_{0.0}, v0_{0.0}, a0_{0.0};  // Initial conditions
    double p1_{0.0}, v1_{0.0}, a1_{0.0};  // Final conditions
    double duration_{0.0};                 // Time duration
    double jerk_{0.0};                     // Constant jerk
};

}  // namespace vp

#endif  // VP_CONSTANT_JERK_TRAJECTORY_H
