/**
 * @file piecewise_trajectory.cpp
 * @brief Piecewise Trajectory Implementation
 */

#include "curve_interface/piecewise_trajectory.h"
#include <algorithm>
#include <cmath>

namespace vp {

double PiecewiseTrajectory::evaluate(const double t, const unsigned int order) const {
    if (trajectory_segments_.empty()) {
        return 0.0;
    }

    // If t exceeds total length, evaluate at the last segment's end
    if (t > cumulative_lengths_.back()) {
        double t_offset = t - cumulative_lengths_[cumulative_lengths_.size() - 2];
        return trajectory_segments_.back()->evaluate(t_offset, order);
    }

    // Find which segment contains time t
    auto it_lower = std::lower_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(), t);
    size_t index  = static_cast<size_t>(it_lower - cumulative_lengths_.begin());

    // Calculate time offset within the segment
    double t_offset = 0.0;
    if (index != 0) {
        t_offset = cumulative_lengths_[index - 1];
    }

    return trajectory_segments_[index]->evaluate(t - t_offset, order);
}

double PiecewiseTrajectory::paramLength() const {
    if (cumulative_lengths_.empty()) {
        return 0.0;
    }
    return cumulative_lengths_.back();
}

void PiecewiseTrajectory::appendSegment(const std::shared_ptr<CurveInterface> trajectory) {
    if (!trajectory) {
        return;
    }

    if (trajectory_segments_.empty()) {
        trajectory_segments_.push_back(trajectory);
    } else {
        // Check continuity (optional validation)
        double p1 = trajectory->evaluate(0.0, 0);
        double v1 = trajectory->evaluate(0.0, 1);
        double a1 = trajectory->evaluate(0.0, 2);

        auto last_segment     = trajectory_segments_.back();
        double last_length = last_segment->paramLength();

        double p0 = last_segment->evaluate(last_length, 0);
        double v0 = last_segment->evaluate(last_length, 1);
        double a0 = last_segment->evaluate(last_length, 2);

        // Optional: warn about discontinuities
        // In production code, you might want to throw exceptions for large discontinuities
        constexpr double CONTINUITY_TOLERANCE = 1e-4;
        
        if (std::fabs(p0 - p1) > CONTINUITY_TOLERANCE) {
            // Position discontinuity detected
        }
        if (std::fabs(v0 - v1) > CONTINUITY_TOLERANCE) {
            // Velocity discontinuity detected
        }
        if (std::fabs(a0 - a1) > CONTINUITY_TOLERANCE) {
            // Acceleration discontinuity detected
        }

        trajectory_segments_.push_back(trajectory);
    }

    // Update cumulative lengths
    double accumulated_length = trajectory->paramLength();
    if (!cumulative_lengths_.empty()) {
        accumulated_length += cumulative_lengths_.back();
    }
    cumulative_lengths_.push_back(accumulated_length);
}

void PiecewiseTrajectory::popSegment() {
    if (trajectory_segments_.empty()) {
        return;
    }
    trajectory_segments_.pop_back();
    cumulative_lengths_.pop_back();
}

void PiecewiseTrajectory::clear() {
    trajectory_segments_.clear();
    cumulative_lengths_.clear();
}

bool PiecewiseTrajectory::empty() const {
    return trajectory_segments_.empty() || cumulative_lengths_.empty();
}

size_t PiecewiseTrajectory::getSegmentsNum() const {
    return trajectory_segments_.size();
}

}  // namespace vp
