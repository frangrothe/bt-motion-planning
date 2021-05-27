//
// Created by francesco on 27.05.21.
//

#include "SpaceTimeStateValidityChecker.h"

namespace SpaceTime {

SpaceTimeStateValidityChecker::SpaceTimeStateValidityChecker(const ompl::base::SpaceInformationPtr &si)
        : StateValidityChecker(si) {}

SpaceTimeStateValidityChecker::SpaceTimeStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                             std::vector<Constraint> constraints)
        : StateValidityChecker(si), constraints_(std::move(constraints)) {}

bool SpaceTimeStateValidityChecker::isValid(const ompl::base::State *state) const {
    double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    return std::none_of(constraints_.cbegin(), constraints_.cend(), [x, t] (Constraint c) {
        return x >= c.x_lb && x <= c.x_ub && t >= c.t_lb && t <= c.t_ub;
    });
}
}