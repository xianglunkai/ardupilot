#pragma once

#include <eigen/include/eigen3/Eigen/Core>

class Projection {
public:
    Projection() = default;
    virtual ~Projection() = default;
    
    void init(const double tolerance, const double limit);

    // return parameter derivatives by projective revise
    Eigen::VectorXd update(Eigen::VectorXd p, Eigen::VectorXd dp);

private:
    double _tolerance{0.5};
    double _limit{10};
};