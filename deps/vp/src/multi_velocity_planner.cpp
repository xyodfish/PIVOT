/**
 * @file multi_velocity_planner.cpp
 * @brief Multi-DOF Velocity Planner Implementation
 */

#include "multi_velocity_planner.h"
#include "trapezoidal_planner.h"
#include "double_s_planner.h"
#include <stdexcept>

namespace vp {

MultiVelocityPlanner::MultiVelocityPlanner(const std::vector<BCs<double>>& BCs, const std::string& algorithm)
    : params_(std::make_shared<VpParams3D>()) {
    planner_ = createVelocityPlanner(BCs, algorithm, params_);
}

MultiVelocityPlanner::MultiVelocityPlanner(const std::vector<BCs<double>>& BCs, const std::string& algorithm,
                                          std::shared_ptr<VpParams3D> params)
    : params_(params) {
    planner_ = createVelocityPlanner(BCs, algorithm, params_);
}

MultiVelocityPlanner::~MultiVelocityPlanner() = default;

std::shared_ptr<VelocityPlannerInterface<double>> MultiVelocityPlanner::createVelocityPlanner(
    const std::vector<BCs<double>>& BCs, const std::string& algo, std::shared_ptr<VpParams3D> params) {
    
    if (algo == "TVP") {
        return std::make_shared<TrapezoidalPlanner>(BCs, algo);
    } else if (algo == "DSVP") {
        // Extract gamma from params if available
        double gamma = 0.95;  // Default value
        if (params && std::tuple_size<decltype(params->params)>::value >= 3) {
            gamma = std::get<2>(params->params);
        }
        return std::make_shared<DoubleSPlanner>(BCs, algo, gamma);
    } else {
        throw std::invalid_argument("Invalid velocity planner algorithm: " + algo + 
                                   ". Supported: TVP (Trapezoidal), DSVP (Double-S)");
    }
}

std::vector<std::vector<KinematicState<double>>> MultiVelocityPlanner::getKStates(bool isNormalized) {
    if (!planner_) {
        throw std::runtime_error("Planner not initialized");
    }
    return planner_->planKStates(isNormalized);
}

std::vector<std::vector<double>> MultiVelocityPlanner::getTrajs(bool isNormalized) {
    if (!planner_) {
        throw std::runtime_error("Planner not initialized");
    }
    return planner_->planTrajs(isNormalized);
}

std::vector<std::vector<double>> MultiVelocityPlanner::getPosVec() {
    if (!planner_) {
        throw std::runtime_error("Planner not initialized");
    }
    return planner_->getPosVec();
}

std::vector<double> MultiVelocityPlanner::getEndTraj(bool isNormalized) {
    if (!planner_) {
        throw std::runtime_error("Planner not initialized");
    }
    return planner_->getEndTraj(isNormalized);
}

void MultiVelocityPlanner::initDefaultBCs(std::vector<BCs<double>>& BCs, double vel, double acc, double jerk, double dt) {
    for (auto& bc : BCs) {
        bc.start_state.time = 0;
        bc.start_state.pos  = 0;
        bc.start_state.vel  = 0;
        bc.start_state.acc  = 0;
        bc.start_state.jerk = 0;
        
        bc.goal_state.time = 0;
        bc.goal_state.pos  = 1.0;  // Normalized goal
        bc.goal_state.vel  = 0;
        bc.goal_state.acc  = 0;
        bc.goal_state.jerk = 0;
        
        bc.max_vel  = vel;
        bc.max_acc  = acc;
        bc.max_jerk = jerk;
        bc.delta_t  = dt;
    }
}

}  // namespace vp
