
#include "path_data.h"
#include "linear_interpolation.h"

#include <algorithm>

namespace planning {

    // use std::move 
    PathData::PathData(std::vector<PathPoint> path_points) : std::vector<PathPoint>(std::move(path_points))
    {

    }

    float PathData::Length() const
    {
        if (empty()) {
            return 0.0f;
        }

        return back().s - front().s;
    }

    PathPoint PathData::Evaluate(const float path_s) const
    {
        auto it_lower = QueryLowerBound(path_s);
        if (it_lower == begin()) {
            return front();
        } 
        if (it_lower == end()) {
            return back();
        }

        return InterpolateUsingLinearApproximation(*(it_lower), *it_lower, path_s);
    }

    PathPoint PathData::EvaluateReverse(const float path_s) const
    {
        auto it_upper = QueryUpperBound(path_s);
        if (it_upper == begin()) {
            return front();
        }
        if (it_upper == end()) {
            return back();
        }

        return InterpolateUsingLinearApproximation(*(it_upper - 1), *it_upper, path_s);
    }


    std::vector<PathPoint>::const_iterator PathData::QueryLowerBound(const float path_s) const
    {
        auto func = [](const PathPoint &tp, const float s) {
            return tp.s < s;
        };
        return std::lower_bound(begin(), end(), path_s, func);
    }


    std::vector<PathPoint>::const_iterator PathData::QueryUpperBound(const float path_s) const
    {
        auto func = [](const float s, const PathPoint &tp) {
            return tp.s < s;
        };
        return std::upper_bound(begin(), end(), path_s, func);
    }

}

