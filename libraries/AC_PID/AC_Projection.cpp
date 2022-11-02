#include "AC_Projection.h"
#include <AP_Math/AP_Math.h>

void Projection::init(const double tolerance, const double limit)
{
    _tolerance = tolerance;
    _limit = limit;
}

Eigen::VectorXd  Projection::update(Eigen::VectorXd p, Eigen::VectorXd dp)
{
    Eigen::VectorXd gradient = 2.0 * (1 + _tolerance) / (_tolerance * sq(_limit)) * p;

    const double convex_function = ((1+ _tolerance) * sq(p.norm()) - sq(_limit)) / (_tolerance * sq(_limit));

    if (convex_function > 0 && dp.transpose() * gradient > 0) {
        return dp - (gradient * gradient.transpose() * dp * convex_function) / (gradient.transpose() * gradient);
    } else {
        return dp;
    }
}