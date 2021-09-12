//
// Created by francesco on 11.09.21.
//

#include "TimeNDNarrowPassageValidityChecker.h"

namespace nd {


TimeNDNarrowPassageValidityChecker::TimeNDNarrowPassageValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                                       const ob::RealVectorBounds &bounds)
        : StateValidityChecker(si), bounds_(bounds) {

    double xMin = 0.4;
    double xMax = 0.6;
//    double width = xMax - xMin;
//
//    double tStep = sqrt(d_);
//    double tMin = sqrt(d_ * pow(xMin, 2));
//    double tMax = tMin + sqrt(d_ * pow(width, 2));
//
//    constraints_.emplace_back(xMin, xMax, 0, tMin - eps);
//    constraints_.emplace_back(xMin, xMax, tMax - eps, tStep - eps);
//    tMin = tStep + sqrt(d_ * pow(2 * width, 2));
//    constraints_.emplace_back(xMin, xMax, tMin - eps, 2 * tStep - eps);
//    tMin = 2 * tStep + sqrt(d_ * pow(4 * width, 2));
//    constraints_.emplace_back(xMin, xMax, tMin - eps, 4 * tStep - eps);

    double gap = 0.2 / sqrt(d_);
    double eps = 0.85 * gap;
    constraints_.emplace_back(xMin, xMax, 0, 0.4);
    constraints_.emplace_back(xMin, xMax, eps + 0.4 + gap, 1);
    constraints_.emplace_back(xMin, xMax, eps + 1 + 2 * gap, 2);
    constraints_.emplace_back(xMin, xMax, eps + 2 + 3 * gap, 4);

}

bool TimeNDNarrowPassageValidityChecker::isValid(const ompl::base::State *state) const {
    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    double *values = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values;

    if (!isInBounds(values))
        return false;

    for (auto &c : constraints_) {
        if (t >= c.tMin && t <= c.tMax && values[0] >= c.xMin && values[0] <= c.xMax)
            return false;
        if (values[0] < 0.5) {
            for (int i = 1; i < d_; ++i) {
                if (values[i] > 0.5)
                    return false;
            }
        }
    }
    return true;
}

bool TimeNDNarrowPassageValidityChecker::isInBounds(const double *values) const {
    for (int i = 0; i < d_; ++i) {
        if (values[i] < bounds_.low[i] || values[i] > bounds_.high[i])
            return false;
    }
    return true;
}
}
