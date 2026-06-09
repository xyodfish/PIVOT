/**
 * @file double_s_multi_planner.h
 * @brief Multi-DOF Double-S planner with time-synchronized arrival
 */

#ifndef VP_DOUBLE_S_MULTI_PLANNER_H
#define VP_DOUBLE_S_MULTI_PLANNER_H

#include "double_s_planner.h"
#include "velocity_planner_interface.h"

#include <memory>
#include <string>
#include <vector>

namespace vp {

/**
 * @brief Multi-DOF Double-S planner that synchronizes all joints to arrive together
 *
 * Fast joints are time-scaled (reduced vel/acc/jerk) to match the slowest joint.
 */
class DoubleSMultiPlanner : public VelocityPlannerInterface<double> {
   public:
    DoubleSMultiPlanner() = default;
    DoubleSMultiPlanner(const std::vector<BCs<double>>& bcs, const std::string& name);
    DoubleSMultiPlanner(const std::vector<BCs<double>>& bcs, const std::string& name, double alpha, double beta);
    ~DoubleSMultiPlanner() override;

    std::vector<KinematicState<double>> getKState(double t, bool isNormalized = false) override;
    std::vector<std::vector<KinematicState<double>>> planKStates(bool isNormalized = false) override;
    std::vector<std::vector<double>> planTrajs(bool isNormalized = false) override;
    std::vector<double> getEndTraj(bool isNormalized = false) override;

   private:
    double getMultiPlannerMaxTime() const;
    void setPlannerByGivenTime(double max_time, double alpha, double beta);

    std::vector<std::shared_ptr<DoubleSPlanner>> planners_;
    size_t planner_size_{0};
    double alpha_{1.0 / 3.0};
    double beta_{0.2};
};

}  // namespace vp

#endif  // VP_DOUBLE_S_MULTI_PLANNER_H
