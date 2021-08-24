//
// Created by francesco on 25.07.21.
//

#include "TimeNDStateValidityChecker.h"

namespace nd {

TimeNDStateValidityChecker::TimeNDStateValidityChecker(const ompl::base::SpaceInformationPtr &si, int d,
                                                       std::vector<TimeNDConstraint> constraints,
                                                       ob::RealVectorBounds bounds, double agentRadius)
        : StateValidityChecker(si), d_(d), constraints_(std::move(constraints)),
          bounds_(std::move(bounds)), agentRadius_(agentRadius) {}

bool TimeNDStateValidityChecker::isValid(const ompl::base::State *state) const {

    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double *values = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;

    if (!isInBounds(values))
        return false;

    return hasCollision(values, t);
}

bool TimeNDStateValidityChecker::isInBounds(const double *values) const {
    for (int i = 0; i < d_; ++i) {
        if (values[i] < bounds_.low[i] + agentRadius_ || values[i] > bounds_.high[i] - agentRadius_)
            return false;
    }
    return true;
}

bool TimeNDStateValidityChecker::hasCollision(const double *values, double t) const {
    int i1 = std::floor(t);
    int i2 = i1 + 1;
    double frac = t - i1;
    std::vector<double> direction(d_);
    std::vector<double> res(d_);
    for (auto &obstacle : constraints_) {
        // calculate obstacle position
        std::vector<double> v1 = obstacle.path[i1 % 8];
        std::vector<double> v2 = obstacle.path[i2 % 8];
        std::transform (v1.begin(), v1.end(), v2.begin(), direction.begin(), [] (auto &a, auto &b) {
            return b - a;
        });
        std::transform (v1.begin(), v1.end(), direction.begin(), res.begin(), [frac] (auto &a, auto &b) {
            return a + frac * b;
        });

        // calculate distance
        double dist = 0.0;
        for (int i = 0; i < d_; ++i) {
            dist += (values[i] - res[i]) * (values[i] - res[i]);
        }
        if (sqrt(dist) < agentRadius_ + obstacle.radius)
            return false;
    }

    return true;
}
}

