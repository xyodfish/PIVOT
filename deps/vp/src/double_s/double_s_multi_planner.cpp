/**
 * @file double_s_multi_planner.cpp
 * @brief Multi-DOF Double-S planner with time-synchronized arrival
 */

#include "double_s_multi_planner.h"

#include <cmath>
#include <stdexcept>

namespace vp {

namespace {
constexpr double kTimeSyncTolerance = 0.01;
constexpr int kTJ1                  = 0;
constexpr int kTA                   = 1;
}  // namespace

DoubleSMultiPlanner::~DoubleSMultiPlanner() = default;

DoubleSMultiPlanner::DoubleSMultiPlanner(const std::vector<BCs<double>>& bcs, const std::string& name)
    : DoubleSMultiPlanner(bcs, name, 1.0 / 3.0, 0.2) {}

DoubleSMultiPlanner::DoubleSMultiPlanner(const std::vector<BCs<double>>& bcs, const std::string& name, double alpha,
                                         double beta) {
    if (bcs.empty()) {
        throw std::invalid_argument("DoubleSMultiPlanner requires at least one boundary condition");
    }

    alpha_ = alpha;
    beta_  = beta;
    planners_.reserve(bcs.size());
    for (const auto& bc : bcs) {
        std::vector<BCs<double>> single_bc;
        single_bc.push_back(bc);
        planners_.push_back(std::make_shared<DoubleSPlanner>(std::move(single_bc), name));
    }
    planner_size_ = planners_.size();

    if (planner_size_ > 1) {
        const double max_time = getMultiPlannerMaxTime();
        setPlannerByGivenTime(max_time, alpha_, beta_);
    }
}

double DoubleSMultiPlanner::getMultiPlannerMaxTime() const {
    double max_time = 0.0;
    for (const auto& planner : planners_) {
        max_time = std::max(max_time, planner->tTotal);
    }
    return max_time;
}

void DoubleSMultiPlanner::setPlannerByGivenTime(double max_time, double alpha, double beta) {
    for (auto& planner : planners_) {
        const BCs<double> bc = planner->getBoundaryCondition();

        if (std::fabs(planner->tTotal - max_time) > kTimeSyncTolerance) {
            planner->setTotalTime(max_time);
            if (planner->needsPlanning()) {
                planner->syncToGivenTime(max_time, alpha, beta);
            }
        } else if (planner->needsPlanning()) {
            planner->aLimitA = bc.max_jerk * planner->t[kTJ1];
            planner->aLimitD = -bc.max_jerk * planner->t[kTJ1];
            planner->vLimit  = bc.start_state.vel + (planner->t[kTA] - planner->t[kTJ1]) * planner->aLimitA;
        } else {
            planner->setTotalTime(max_time);
        }
    }
}

std::vector<KinematicState<double>> DoubleSMultiPlanner::getKState(double t, bool isNormalized) {
    std::vector<KinematicState<double>> states;
    states.reserve(planner_size_);
    for (const auto& planner : planners_) {
        auto state_vec = planner->getKState(t, isNormalized);
        if (!state_vec.empty()) {
            states.push_back(state_vec.front());
        }
    }
    return states;
}

std::vector<std::vector<KinematicState<double>>> DoubleSMultiPlanner::planKStates(bool isNormalized) {
    std::vector<std::vector<KinematicState<double>>> traj;
    traj.reserve(planner_size_);
    for (const auto& planner : planners_) {
        auto dof_traj = planner->planKStates(isNormalized);
        if (!dof_traj.empty()) {
            traj.push_back(dof_traj.front());
        }
    }
    return traj;
}

std::vector<std::vector<double>> DoubleSMultiPlanner::planTrajs(bool isNormalized) {
    const auto kstates    = planKStates(isNormalized);
    if (kstates.empty() || kstates[0].empty()) {
        return {};
    }

    const size_t traj_num = kstates[0].size();
    std::vector<std::vector<double>> trajs(traj_num, std::vector<double>(3 * planner_size_ + 1, 0.0));

    for (size_t i = 0; i < traj_num; ++i) {
        trajs[i][0] = kstates[0][i].time;
        for (size_t j = 0; j < planner_size_; ++j) {
            trajs[i][1 + j]                               = kstates[j][i].pos;
            trajs[i][1 + planner_size_ + j]             = kstates[j][i].vel;
            trajs[i][1 + 2 * planner_size_ + j]           = kstates[j][i].acc;
        }
    }

    trajs_ = trajs;
    return trajs;
}

std::vector<double> DoubleSMultiPlanner::getEndTraj(bool isNormalized) {
    std::vector<double> end_positions;
    end_positions.reserve(planner_size_);
    for (const auto& planner : planners_) {
        auto end_traj = planner->getEndTraj(isNormalized);
        if (!end_traj.empty()) {
            end_positions.push_back(end_traj.front());
        }
    }
    return end_positions;
}

}  // namespace vp
