/**
 * @file double_s_planner.cpp
 * @brief Double-S Velocity Profile Planner (S-Curve) Implementation
 * 
 * Implements a smooth S-curve velocity profile with 7 phases:
 * 1. Increasing acceleration (positive jerk)
 * 2. Constant acceleration
 * 3. Decreasing acceleration (negative jerk to zero)
 * 4. Constant velocity
 * 5. Increasing deceleration (negative jerk)
 * 6. Constant deceleration
 * 7. Decreasing deceleration (positive jerk to zero)
 */

#include "double_s_planner.h"
#include "curve_interface/constant_jerk_trajectory.h"
#include "curve_interface/piecewise_trajectory.h"
#include "planner_exception.h"
#include <cmath>
#include <algorithm>
#include <memory>

namespace vp {

// Time stage enums
enum TimeStage { TJ1 = 0, T_A = 1, T_V = 2, TJ2 = 3, T_D = 4 };
constexpr int TIME_SEGMENT_NUM = 5;
constexpr int TRAJ_SEGMENT_NUM = 7;
constexpr double DEFAULT_GAMMA = 0.95;

DoubleSPlanner::DoubleSPlanner(std::vector<BCs<double>> BCs, const std::string& name) 
    : gamma_(DEFAULT_GAMMA) {
    if (BCs.empty()) {
        throw PlannerException(1009);
    }
    BC_ = BCs[0];
    initParams();
}

DoubleSPlanner::DoubleSPlanner(std::vector<BCs<double>> BCs, const std::string& name, double gamma)
    : gamma_((gamma > 0.8 && gamma < 1.0) ? gamma : DEFAULT_GAMMA) {
    if (BCs.empty()) {
        throw PlannerException(1009);
    }
    BC_ = BCs[0];
    initParams();
}

DoubleSPlanner::~DoubleSPlanner() = default;

void DoubleSPlanner::initParams() {
    // Check if planning is needed
    if (std::fabs(BC_.start_state.pos - BC_.goal_state.pos) < 1e-6) {
        needs_plan_ = false;
        dt = BC_.delta_t;
        tTotal = 0.0;
        return;
    }

    needs_plan_ = true;
    dt = BC_.delta_t;
    dp = std::fabs(BC_.goal_state.pos - BC_.start_state.pos);

    // Direction factor
    m_sigma = (BC_.start_state.pos > BC_.goal_state.pos) ? -1 : 1;

    // Normalize boundary conditions
    BC_.start_state.pos = m_sigma * BC_.start_state.pos;
    BC_.goal_state.pos  = m_sigma * BC_.goal_state.pos;
    BC_.start_state.vel = m_sigma * BC_.start_state.vel;
    BC_.goal_state.vel  = m_sigma * BC_.goal_state.vel;

    // Resize time vectors
    t.resize(TIME_SEGMENT_NUM);
    cjt_sptrs.resize(TRAJ_SEGMENT_NUM);
    trajectory_ = std::make_shared<PiecewiseTrajectory>();

    /**
     * Case 1: v_lim = v_max (can reach maximum velocity)
     */
    
    // Acceleration phase
    if (((BC_.max_vel - BC_.start_state.vel) * BC_.max_jerk) < std::pow(BC_.max_acc, 2)) {
        // Not reach max_acc
        t[TJ1] = std::pow((BC_.max_vel - BC_.start_state.vel) / BC_.max_jerk, 0.5);
        t[T_A] = 2 * t[TJ1];
    } else {
        // Reach max_acc
        t[TJ1] = BC_.max_acc / BC_.max_jerk;
        t[T_A] = t[TJ1] + (BC_.max_vel - BC_.start_state.vel) / BC_.max_acc;
    }

    // Deceleration phase
    if (((BC_.max_vel - BC_.goal_state.vel) * BC_.max_jerk) < std::pow(BC_.max_acc, 2)) {
        // Not reach max_acc
        t[TJ2] = std::pow((BC_.max_vel - BC_.goal_state.vel) / BC_.max_jerk, 0.5);
        t[T_D] = 2 * t[TJ2];
    } else {
        // Reach max_acc
        t[TJ2] = BC_.max_acc / BC_.max_jerk;
        t[T_D] = t[TJ2] + (BC_.max_vel - BC_.goal_state.vel) / BC_.max_acc;
    }

    // Constant velocity phase
    t[T_V] = (BC_.goal_state.pos - BC_.start_state.pos) / BC_.max_vel 
             - t[T_A] / 2 * (1 + BC_.start_state.vel / BC_.max_vel)
             - t[T_D] / 2 * (1 + BC_.goal_state.vel / BC_.max_vel);

    if (t[T_V] >= 0) {
        // Can reach max velocity
        tTotal = t[T_A] + t[T_V] + t[T_D];
    } else {
        // Cannot reach max velocity, need to reduce acceleration
        calWithGradReducingAcc(BC_, gamma_);

        // Handle special cases
        if (t[T_A] < 0) {
            // Only deceleration
            t[T_A] = 0;
            t[TJ1] = 0;
            t[T_V] = 0;
            t[T_D] = 2 * (BC_.goal_state.pos - BC_.start_state.pos) / (BC_.goal_state.vel + BC_.start_state.vel);
            t[TJ2] = (BC_.max_jerk * (BC_.goal_state.pos - BC_.start_state.pos) -
                     std::sqrt(BC_.max_jerk * BC_.max_jerk * std::pow(BC_.goal_state.pos - BC_.start_state.pos, 2) +
                              std::pow(BC_.goal_state.vel + BC_.start_state.vel, 2) * 
                              (BC_.goal_state.vel - BC_.start_state.vel))) /
                    (BC_.max_jerk * (BC_.goal_state.vel + BC_.start_state.vel));
            tTotal = t[T_D];
        } else if (t[T_D] < 0) {
            // Only acceleration
            t[T_D] = 0;
            t[TJ2] = 0;
            t[T_V] = 0;
            t[T_A] = 2 * (BC_.goal_state.pos - BC_.start_state.pos) / (BC_.goal_state.vel + BC_.start_state.vel);
            t[TJ1] = (BC_.max_jerk * (BC_.goal_state.pos - BC_.start_state.pos) -
                     std::sqrt(BC_.max_jerk * BC_.max_jerk * std::pow(BC_.goal_state.pos - BC_.start_state.pos, 2) -
                              std::pow(BC_.goal_state.vel + BC_.start_state.vel, 2) * 
                              (BC_.goal_state.vel - BC_.start_state.vel))) /
                    (BC_.max_jerk * (BC_.goal_state.vel + BC_.start_state.vel));
            tTotal = t[T_A];
        } else {
            // No constant velocity phase
            t[T_V] = 0;
            tTotal = t[T_A] + t[T_D];
        }
    }

    // Calculate additional parameters
    tAcc = t[T_A] - 2 * t[TJ1];
    tDec = t[T_D] - 2 * t[TJ2];

    aLimitA = BC_.max_jerk * t[TJ1];
    aLimitD = -BC_.max_jerk * t[TJ2];
    vLimit  = BC_.start_state.vel + (t[T_A] - t[TJ1]) * aLimitA;

    // Splice trajectory segments
    splicingTrajectorySegments();
}

void DoubleSPlanner::calWithGradReducingAcc(BCs<double>& BC, double gamma) {
    double max_acc = BC.max_acc;
    double t_j, delta;
    double t_gamma = (gamma > 0.8 && gamma < 1.0) ? gamma : DEFAULT_GAMMA;

    while (true) {
        t[TJ1] = max_acc / BC.max_jerk;
        t[TJ2] = t[TJ1];
        t_j    = t[TJ1];

        delta = std::pow(max_acc, 4) / std::pow(BC.max_jerk, 2) +
                2 * (std::pow(BC.start_state.vel, 2) + std::pow(BC.goal_state.vel, 2)) +
                max_acc * (4 * (BC.goal_state.pos - BC.start_state.pos) -
                           2 * max_acc / BC.max_jerk * (BC.goal_state.vel + BC.start_state.vel));
        
        t[T_A] = (std::pow(max_acc, 2) / BC.max_jerk - 2 * BC.start_state.vel + std::sqrt(delta)) / (2 * max_acc);
        t[T_D] = (std::pow(max_acc, 2) / BC.max_jerk - 2 * BC.goal_state.vel + std::sqrt(delta)) / (2 * max_acc);

        if ((2 * t_j <= t[T_A]) && (2 * t_j <= t[T_D])) {
            BC.max_acc = max_acc;
            break;
        }

        max_acc = max_acc * t_gamma;
    }
}

void DoubleSPlanner::splicingTrajectorySegments(bool force) {
    if (!needs_plan_) {
        return;
    }
    if (!force && trajectory_ && trajectory_->getSegmentsNum() == TRAJ_SEGMENT_NUM) {
        return;
    }

    if (!trajectory_) {
        trajectory_ = std::make_shared<PiecewiseTrajectory>();
    }

    // Clear existing trajectory
    trajectory_->clear();

    double last_time = tTotal - (t[TJ1] * 2 + tAcc + t[T_V] + t[TJ2] * 2 + tDec);
    if (last_time <= 0) {
        last_time = t[TJ2];
    }

    std::vector<double> time_vec = {t[TJ1], tAcc, t[TJ1], t[T_V], t[TJ2], tDec, last_time};
    std::vector<double> jerk_vec = {BC_.max_jerk, 0, -BC_.max_jerk, 0, -BC_.max_jerk, 0, BC_.max_jerk};

    // First segment
    cjt_sptrs[0] = std::make_shared<ConstantJerkTrajectory>(
        BC_.start_state.pos, BC_.start_state.vel, BC_.start_state.acc, jerk_vec[0], time_vec[0]);
    trajectory_->appendSegment(cjt_sptrs[0]);

    // Remaining segments
    for (unsigned int i = 1; i < cjt_sptrs.size(); ++i) {
        double p = cjt_sptrs[i - 1]->evaluate(time_vec[i - 1], 0);
        double v = cjt_sptrs[i - 1]->evaluate(time_vec[i - 1], 1);
        double a = cjt_sptrs[i - 1]->evaluate(time_vec[i - 1], 2);
        
        cjt_sptrs[i] = std::make_shared<ConstantJerkTrajectory>(p, v, a, jerk_vec[i], time_vec[i]);
        trajectory_->appendSegment(cjt_sptrs[i]);
    }
}

std::vector<KinematicState<double>> DoubleSPlanner::getKState(double time, bool isNormalized) {
    KinematicState<double> state;

    if (!needs_plan_) {
        state.pos  = BC_.goal_state.pos;
        state.vel  = 0;
        state.acc  = 0;
        state.jerk = 0;
        state.time = time;
        return {state};
    }

    state.time = time;
    state.pos  = m_sigma * trajectory_->evaluate(time, 0);
    state.vel  = m_sigma * trajectory_->evaluate(time, 1);
    state.acc  = m_sigma * trajectory_->evaluate(time, 2);
    state.jerk = m_sigma * trajectory_->evaluate(time, 3);

    if (isNormalized) {
        state.pos = (state.pos - BC_.start_state.pos) / dp;
    }

    return {state};
}

std::vector<double> DoubleSPlanner::getTraj(double time, bool isNormalized) {
    std::vector<double> traj(5);  // [time, pos, vel, acc, jerk]
    traj[0] = time;

    if (!needs_plan_) {
        traj[1] = BC_.goal_state.pos;
        traj[2] = 0;
        traj[3] = 0;
        traj[4] = 0;
        return traj;
    }

    traj[1] = m_sigma * trajectory_->evaluate(time, 0);
    traj[2] = m_sigma * trajectory_->evaluate(time, 1);
    traj[3] = m_sigma * trajectory_->evaluate(time, 2);
    traj[4] = m_sigma * trajectory_->evaluate(time, 3);

    if (isNormalized) {
        traj[1] = (traj[1] - BC_.start_state.pos) / dp;
    }

    return traj;
}

std::vector<std::vector<KinematicState<double>>> DoubleSPlanner::planKStates(bool isNormalized) {
    std::vector<KinematicState<double>> states;

    for (double t = 0; t < tTotal; t += dt) {
        auto state_vec = getKState(t, isNormalized);
        states.push_back(state_vec[0]);
    }

    return {states};
}

std::vector<std::vector<double>> DoubleSPlanner::planTrajs(bool isNormalized) {
    std::vector<std::vector<double>> trajs;

    splicingTrajectorySegments();

    double t;
    for (t = 0; t < tTotal; t += dt) {
        trajs.push_back(getTraj(t, isNormalized));
    }

    // Add final point
    if (t - dt < tTotal) {
        trajs.push_back(getTraj(tTotal - 1e-5, isNormalized));
    }

    trajs_ = trajs;
    return trajs;
}

std::vector<double> DoubleSPlanner::getEndTraj(bool isNormalized) {
    std::vector<double> traj(4);  // [pos, vel, acc, jerk]

    if (!needs_plan_) {
        traj[0] = BC_.goal_state.pos;
        traj[1] = 0;
        traj[2] = 0;
        traj[3] = 0;
        return traj;
    }

    traj[0] = m_sigma * trajectory_->evaluate(tTotal, 0);
    traj[1] = m_sigma * trajectory_->evaluate(tTotal, 1);
    traj[2] = m_sigma * trajectory_->evaluate(tTotal, 2);
    traj[3] = m_sigma * trajectory_->evaluate(tTotal, 3);

    if (isNormalized) {
        traj[0] = (traj[0] - BC_.start_state.pos) / dp;
    }

    return traj;
}

void DoubleSPlanner::setMaxVel(double vel) {
    BC_.max_vel = vel;
    initParams();  // Reinitialize with new constraint
}

void DoubleSPlanner::setMaxAcc(double acc) {
    BC_.max_acc = acc;
    initParams();
}

void DoubleSPlanner::setMaxJerk(double jerk) {
    BC_.max_jerk = jerk;
    initParams();
}

void DoubleSPlanner::setLimitsOnly(double vel, double acc, double jerk) {
    BC_.max_vel  = vel;
    BC_.max_acc  = acc;
    BC_.max_jerk = jerk;
}

void DoubleSPlanner::syncToGivenTime(double max_time, double alpha, double beta) {
    if (!needs_plan_ || max_time < 1e-9) {
        return;
    }

    const double h = dp;
    if (h < 1e-9) {
        return;
    }

    const double denom_vel = (1.0 - alpha) * max_time;
    const double denom_acc = (1.0 - alpha) * alpha * (1.0 - beta) * std::pow(max_time, 2);
    const double denom_jerk =
        std::pow(alpha, 2) * beta * (1.0 - alpha) * (1.0 - beta) * std::pow(max_time, 3);

    if (denom_vel < 1e-12 || denom_acc < 1e-12 || denom_jerk < 1e-12) {
        return;
    }

    setLimitsOnly(h / denom_vel, h / denom_acc, h / denom_jerk);

    t[T_A]  = alpha * max_time;
    t[TJ1]  = beta * t[T_A];
    t[T_D]  = t[T_A];
    t[TJ2]  = t[TJ1];
    t[T_V]  = max_time - t[T_A] - t[T_D];
    tTotal  = max_time;
    tAcc    = t[T_A] - 2.0 * t[TJ1];
    tDec    = t[T_D] - 2.0 * t[TJ2];
    aLimitA = BC_.max_jerk * t[TJ1];
    aLimitD = -BC_.max_jerk * t[TJ2];
    vLimit  = BC_.start_state.vel + (t[T_A] - t[TJ1]) * aLimitA;

    rebuildTrajectory();
}

void DoubleSPlanner::setTotalTime(double total_time) {
    tTotal = std::max(0.0, total_time);
}

void DoubleSPlanner::rebuildTrajectory() {
    splicingTrajectorySegments(true);
}

}  // namespace vp
