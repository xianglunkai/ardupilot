#pragma once

#include <vector>
#include "discretized_trajectory.h"

namespace planning {

class PathData : public std::vector<PathPoint> {
 public:
    PathData() = default;

    explicit PathData(std::vector<PathPoint> path_points);

    float Length() const;

    PathPoint Evaluate(const float path_s) const;

    PathPoint EvaluateReverse(const float path_s) const;

 protected:
    std::vector<PathPoint>::const_iterator QueryLowerBound(
        const float path_s) const;
    std::vector<PathPoint>::const_iterator QueryUpperBound(
        const float path_s) const;
};

} // namespace planning