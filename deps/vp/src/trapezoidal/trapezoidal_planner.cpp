/**
 * @file trapezoidal_planner.cpp
 * @brief Trapezoidal Velocity Profile Planner Implementation
 */

#include "trapezoidal_planner.h"
#include "planner_exception.h"
#include <cmath>
#include <stdexcept>

namespace vp {

TrapezoidalPlanner::TrapezoidalPlanner(std::vector<BCs<double>> BC, const std::string& name) {
    if (BC.empty()) {
        throw PlannerException(1009);  // Empty input value
    }
    
    BC_ = BC[0];
    setBoundaryConditions(BC_);
}

TrapezoidalPlanner::~TrapezoidalPlanner() = default;

void TrapezoidalPlanner::setBoundaryConditions(BCs<double>& BC) {
    initParams(BC);
}

void TrapezoidalPlanner::initParams(BCs<double>& BC) {
    p0_   = BC.start_state.pos;
    p1_   = BC.goal_state.pos;
    v0_   = BC.start_state.vel;
    v1_   = BC.goal_state.vel;
    vmax_ = BC.max_vel;
    aa_   = BC.max_acc;
    ad_   = -aa_;
    dt_   = BC.delta_t;
    t0_   = 0.0;

    dp_ = std::fabs(p1_ - p0_);

    if (dp_ < 1e-6) {
        throw PlannerException(1002);  // Calculated pose is empty
    }
}

std::vector<KinematicState<double>> TrapezoidalPlanner::getKState(double t, bool isNormalized) {
    if (t < 0 || t > tTotal_) {
        throw PlannerException(1008);  // Value extends boundary
    }

    size_t index = static_cast<size_t>(t / dt_);
    if (index >= traj_.size()) {
        index = traj_.size() - 1;
    }

    return {traj_[index]};
}

std::vector<std::vector<KinematicState<double>>> TrapezoidalPlanner::planKStates(bool isNormalized) {
    std::vector<KinematicState<double>> traj;
    KinematicState<double> state;

    double h  = std::abs(p0_ - p1_);
    double vf = std::sqrt((2.0 * aa_ * ad_ * h - aa_ * std::pow(v1_, 2) + ad_ * std::pow(v0_, 2)) / (ad_ - aa_));

    double vv = (vf < vmax_) ? vf : vmax_;  // Cruise velocity

    // Acceleration phase
    double T_a = std::fabs((vv - v0_) / aa_);
    double L_a = v0_ * T_a + 0.5 * aa_ * std::pow(T_a, 2);

    // Constant velocity phase
    double T_v = std::fabs((h - (std::pow(vv, 2) - std::pow(v0_, 2)) / (2.0 * aa_) -
                            (std::pow(v1_, 2) - std::pow(vv, 2)) / (2.0 * ad_)) / vv);
    double L_v = vv * T_v;

    // Deceleration phase
    double T_d = std::fabs((v1_ - vv) / ad_);
    double L_d = vv * T_d + 0.5 * ad_ * std::pow(T_d, 2);

    tTotal_ = T_a + T_v + T_d;

    // Normalized parameters
    double S1 = std::abs(L_a);
    double S2 = std::abs(L_d);
    double T1 = T_a;
    double T3 = T_d;
    double T2 = tTotal_ - T_d;

    if (isNormalized) {
        S1 /= h;
        S2 /= h;
        T1 /= tTotal_;
        T3 /= tTotal_;
        T2 /= tTotal_;
    }

    double acc1 = (T1 != 0) ? (2 * S1 / std::pow(T1, 2)) : 0;
    double acc2 = (T3 != 0) ? (2 * S2 / std::pow(T3, 2)) : 0;

    step_ = static_cast<unsigned int>(tTotal_ / dt_);

    for (unsigned i = 0; i < step_; ++i) {
        double t_current = i * dt_;
        double t_value   = isNormalized ? (i * dt_ / tTotal_) : (i * dt_);

        double p, v;

        if (t_current >= 0 && t_current < T_a) {
            // Acceleration phase
            p = 0.5 * acc1 * std::pow(t_value, 2);
            v = acc1 * t_value;
        } else if (t_current >= T_a && t_current < (T_a + T_v)) {
            // Constant velocity phase
            if (T1 == 0) {
                p = acc2 * T3 * t_value;
                v = acc2 * T3;
            } else {
                p = 0.5 * acc1 * std::pow(T1, 2) + acc1 * T1 * (t_value - T1);
                v = acc1 * T1;
            }
        } else {
            // Deceleration phase
            if (T1 == 0) {
                p = acc2 * T3 * (T2 - T1) + acc2 * T3 * (t_value - T2) - 0.5 * acc2 * std::pow(t_value - T2, 2);
                v = acc2 * T3 - acc2 * t_value;
            } else {
                p = 0.5 * acc1 * std::pow(T1, 2) + acc1 * T1 * (T2 - T1) + 
                    acc1 * T1 * (t_value - T2) - 0.5 * acc2 * std::pow(t_value - T2, 2);
                v = acc1 * T1 - acc2 * (t_value - T2);
            }
        }

        state.time = t_current;
        state.pos  = p;
        state.vel  = v;
        state.acc  = 0;  // TODO: Calculate acceleration
        state.jerk = 0;  // TODO: Calculate jerk

        traj.push_back(state);
    }

    traj_ = traj;
    return {traj};
}

std::vector<std::vector<double>> TrapezoidalPlanner::planTrajs(bool isNormalized) {
    std::vector<double> point(3);  // [time, normalized_pos, velocity]
    std::vector<std::vector<double>> trajs;

    double h  = std::abs(p0_ - p1_);
    double vf = std::sqrt((2.0 * aa_ * ad_ * h - aa_ * std::pow(v1_, 2) + ad_ * std::pow(v0_, 2)) / (ad_ - aa_));
    double vv = (vf < vmax_) ? vf : vmax_;

    // Phase calculations
    double T_a = std::fabs((vv - v0_) / aa_);
    double L_a = v0_ * T_a + 0.5 * aa_ * std::pow(T_a, 2);

    double T_v = std::fabs((h - (std::pow(vv, 2) - std::pow(v0_, 2)) / (2.0 * aa_) -
                            (std::pow(v1_, 2) - std::pow(vv, 2)) / (2.0 * ad_)) / vv);

    double T_d = std::fabs((v1_ - vv) / ad_);
    double L_d = vv * T_d + 0.5 * ad_ * std::pow(T_d, 2);

    tTotal_ = T_a + T_v + T_d;

    double S1 = std::abs(L_a);
    double S2 = std::abs(L_d);
    double T1 = T_a;
    double T3 = T_d;
    double T2 = tTotal_ - T_d;

    double acc1 = (T1 != 0) ? (2 * S1 / std::pow(T1, 2)) : 0;
    double acc2 = (T3 != 0) ? (2 * S2 / std::pow(T3, 2)) : 0;

    for (double t = 0; t < tTotal_; t += dt_) {
        double t_current = t;
        double t_value   = t;

        double p, v;

        if (t_current >= 0 && t_current < T_a) {
            p = 0.5 * acc1 * std::pow(t_value, 2);
            v = acc1 * t_value;
        } else if (t_current >= T_a && t_current < (T_a + T_v)) {
            if (T1 == 0) {
                p = acc2 * T3 * t_value;
                v = acc2 * T3;
            } else {
                p = 0.5 * acc1 * std::pow(T1, 2) + acc1 * T1 * (t_value - T1);
                v = acc1 * T1;
            }
        } else {
            if (T1 == 0) {
                p = acc2 * T3 * (T2 - T1) + acc2 * T3 * (t_value - T2) - 0.5 * acc2 * std::pow(t_value - T2, 2);
                v = acc2 * T3 - acc2 * t_value;
            } else {
                p = 0.5 * acc1 * std::pow(T1, 2) + acc1 * T1 * (T2 - T1) + 
                    acc1 * T1 * (t_value - T2) - 0.5 * acc2 * std::pow(t_value - T2, 2);
                v = acc1 * T1 - acc2 * (t_value - T2);
            }
        }

        point[0] = t_current;
        point[1] = p / dp_;
        point[2] = v;

        trajs.push_back(point);
    }

    // Add final point
    if (trajs.empty() || trajs.back()[0] < tTotal_) {
        point[0] = tTotal_;
        point[1] = 1.0;
        point[2] = trajs.empty() ? 0 : trajs.back()[2];
        trajs.push_back(point);
    }

    trajs_ = trajs;
    return trajs;
}

}  // namespace vp
