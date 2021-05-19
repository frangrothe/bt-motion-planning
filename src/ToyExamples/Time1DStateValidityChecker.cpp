//
// Created by francesco on 19.05.21.
//

#include "Time1DStateValidityChecker.h"

namespace t1d {

Time1DStateValidityChecker::Time1DStateValidityChecker(const ob::SpaceInformationPtr &si)
        : StateValidityChecker(si) {}

Time1DStateValidityChecker::Time1DStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                       std::vector<Constraint> constraints)
        : StateValidityChecker(si), constraints_(std::move(constraints)) {}

bool Time1DStateValidityChecker::isValid(const ob::State *state) const {
    double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    /*
     * forbid the segment from x lower bound to x upper bound in respect to the time from t lower bound to t upper bound
     */
    for (auto const &c : constraints_) {
        if ((x >= c.x_lb && x <= c.x_ub) && (t >= c.t_lb && t <= c.t_ub)) {
            return false;
        }
    }

    return true;

//    return std::none_of(constraints_.cbegin(), constraints_.cend(), [x, t] (Constraint c) {
//        return x >= c.x_lb && x <= c.x_ub && t >= c.t_lb && t <= c.t_ub;
//    });

}
}


