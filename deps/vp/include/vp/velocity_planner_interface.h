/**
 * @file velocity_planner_interface.h
 * @brief Velocity Planning Module - Core Interfaces
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_VELOCITY_PLANNER_INTERFACE_H
#define VP_VELOCITY_PLANNER_INTERFACE_H

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace vp {

/**
 * @brief Kinematic State at a specific time
 * @tparam T Numeric type (double, float, etc.)
 */
template <typename T>
struct KinematicState {
    T time;   ///< Time stamp
    T pos;    ///< Position
    T vel;    ///< Velocity
    T acc;    ///< Acceleration
    T jerk;   ///< Jerk
    T theta;  ///< Orientation (optional)
};

/**
 * @brief Boundary Conditions for trajectory planning
 * @tparam T Numeric type
 */
template <typename T>
struct BoundaryConditions {
    KinematicState<T> start_state;  ///< Start state
    KinematicState<T> goal_state;   ///< Goal state
    T max_acc;                       ///< Maximum acceleration
    T max_vel;                       ///< Maximum velocity
    T max_jerk;                      ///< Maximum jerk
    T delta_t;                       ///< Time step
};

/**
 * @brief Alias for boundary conditions
 */
template <typename T>
using BCs = BoundaryConditions<T>;

/**
 * @brief Base class for velocity planner parameters
 * @tparam Args Parameter types
 */
template <typename... Args>
class VpParamsBase {
   public:
    VpParamsBase() = default;  // Default constructor
    explicit VpParamsBase(Args&... args) : params(std::forward<Args&>(args)...) {}
    std::tuple<Args...> params;
};

/**
 * @brief 3D velocity planner parameters (vel, acc, jerk)
 */
using VpParams3D = VpParamsBase<double, double, double>;

/**
 * @brief Extended velocity planner parameters
 */
class VpParams : public VpParams3D {
   public:
    explicit VpParams(double& param1, double& param2, double& param3)
        : VpParams3D(param1, param2, param3) {}
};

/**
 * @brief Abstract interface for velocity planners
 * @tparam T Numeric type
 */
template <typename T>
class VelocityPlannerInterface {
   public:
    VelocityPlannerInterface() = default;
    virtual ~VelocityPlannerInterface() = default;

    /**
     * @brief Get kinematic states at specified time
     * @param time Query time
     * @param isNormalized If true, return normalized time [0, 1]
     * @return Vector of kinematic states for all DOFs
     */
    virtual std::vector<KinematicState<T>> getKState(T time, bool isNormalized = false) = 0;

    /**
     * @brief Plan complete trajectory (kinematic states)
     * @param isNormalized If true, use normalized time
     * @return Trajectory as vector of kinematic states per DOF
     */
    virtual std::vector<std::vector<KinematicState<T>>> planKStates(bool isNormalized = false) = 0;

    /**
     * @brief Plan complete trajectory (position values only)
     * @param isNormalized If true, use normalized time
     * @return Trajectory as vector of positions per DOF
     */
    virtual std::vector<std::vector<T>> planTrajs(bool isNormalized = false) = 0;

    /**
     * @brief Get end position of trajectory
     * @param isNormalized If true, use normalized time
     * @return End positions for all DOFs
     */
    virtual std::vector<T> getEndTraj(bool isNormalized = false) = 0;

    /**
     * @brief Get time span of trajectory
     * @return Vector containing total time
     */
    virtual std::vector<std::vector<T>> getTimeSpan() {
        std::vector<T> span;
        for (const auto& traj : trajs_) {
            span.push_back(traj[0]);
        }
        return {span};
    }

    /**
     * @brief Get position vector
     * @return Vector of positions
     */
    virtual std::vector<std::vector<T>> getPosVec() {
        std::vector<T> pos;
        for (const auto& traj : trajs_) {
            pos.push_back(traj[1]);
        }
        return {pos};
    }

    /**
     * @brief Get internal trajectory data
     * @return Reference to trajectory data
     */
    std::vector<std::vector<T>>& getTrajs() { return trajs_; }

   protected:
    std::vector<std::vector<T>> trajs_;  ///< Internal trajectory storage
};

}  // namespace vp

#endif  // VP_VELOCITY_PLANNER_INTERFACE_H
