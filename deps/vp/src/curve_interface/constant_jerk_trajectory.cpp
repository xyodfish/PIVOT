/**
 * @file constant_jerk_trajectory.cpp
 * @brief Constant Jerk Trajectory Implementation
 */

#include "curve_interface/constant_jerk_trajectory.h"
#include <cmath>

namespace vp {

/**
 * @brief Construct constant jerk trajectory
 * 
 * Equations:
 * - p(t) = p0 + v0*t + 0.5*a0*t² + j0*t³/6
 * - v(t) = v0 + a0*t + 0.5*j0*t²
 * - a(t) = a0 + j0*t
 * - j(t) = j0
 */
ConstantJerkTrajectory::ConstantJerkTrajectory(const double p0, const double v0, const double a0,
                                               const double jerk, const double duration)
    : p0_(p0), v0_(v0), a0_(a0), jerk_(jerk), duration_(duration) {
    // Calculate end conditions
    p1_ = evaluate(duration_, 0);
    v1_ = evaluate(duration_, 1);
    a1_ = evaluate(duration_, 2);
}

double ConstantJerkTrajectory::evaluate(const double t, const unsigned int order) const {
    switch (order) {
        case 0: {  // Position
            return p0_ + v0_ * t + 0.5 * a0_ * std::pow(t, 2) + jerk_ * std::pow(t, 3) / 6.0;
        }
        case 1: {  // Velocity
            return v0_ + a0_ * t + 0.5 * jerk_ * t * t;
        }
        case 2: {  // Acceleration
            return a0_ + jerk_ * t;
        }
        case 3: {  // Jerk
            return jerk_;
        }
        default:
            return 0.0;
    }
}

double ConstantJerkTrajectory::paramLength() const {
    return duration_;
}

}  // namespace vp
