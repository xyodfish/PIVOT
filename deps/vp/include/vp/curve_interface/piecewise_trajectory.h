/**
 * @file piecewise_trajectory.h
 * @brief Piecewise Trajectory (composed of multiple segments)
 * @version 1.0.0
 * @date 2024
 * 
 * @copyright Apache License 2.0
 */

#ifndef VP_PIECEWISE_TRAJECTORY_H
#define VP_PIECEWISE_TRAJECTORY_H

#include <memory>
#include <vector>
#include "curve_interface.h"

namespace vp {

/**
 * @brief A trajectory composed of multiple curve segments
 * 
 * Allows building complex trajectories by concatenating simpler segments.
 * Each segment can be any implementation of CurveInterface.
 */
class PiecewiseTrajectory : public CurveInterface {
   public:
    PiecewiseTrajectory() = default;
    ~PiecewiseTrajectory() override = default;

    /**
     * @brief Evaluate the piecewise trajectory at time t
     * @param t Time parameter
     * @param order Derivative order
     * @return Evaluated value
     */
    double evaluate(const double t, const unsigned int order = 0) const override;

    /**
     * @brief Get total parameter length
     * @return Sum of all segment lengths
     */
    double paramLength() const override;

    /**
     * @brief Append a new segment to the trajectory
     * @param trajectory Shared pointer to curve segment
     */
    void appendSegment(const std::shared_ptr<CurveInterface> trajectory);

    /**
     * @brief Remove the last segment
     */
    void popSegment();

    /**
     * @brief Get number of segments
     * @return Segment count
     */
    size_t getSegmentsNum() const;

    /**
     * @brief Clear all segments
     */
    void clear();

    /**
     * @brief Check if trajectory is empty
     * @return True if no segments
     */
    bool empty() const;

   private:
    std::vector<std::shared_ptr<CurveInterface>> trajectory_segments_;
    std::vector<double> cumulative_lengths_;  // Cumulative parameter lengths
};

}  // namespace vp

#endif  // VP_PIECEWISE_TRAJECTORY_H
