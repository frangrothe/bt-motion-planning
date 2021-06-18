//
// Created by francesco on 27.05.21.
//

#include "SpaceTimeStateValidityChecker.h"

namespace time_2d {

SpaceTimeStateValidityChecker::SpaceTimeStateValidityChecker(const ompl::base::SpaceInformationPtr &si)
        : StateValidityChecker(si) {}

SpaceTimeStateValidityChecker::SpaceTimeStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                             std::vector<Constraint2D> constraints)
        : StateValidityChecker(si), constraints_(std::move(constraints)) {}

bool SpaceTimeStateValidityChecker::isValid(const ompl::base::State *state) const {
    double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    for (const auto& c : constraints_) {
        double xlb, xub, ylb, yub;
        std::tie(xlb, xub, ylb, yub) = c.getBoundsForTime(t);

        if (x >= xlb && x <= xub && y >= ylb && y <= yub) return false;
    }

    return true;
//    return std::none_of(constraints_.cbegin(), constraints_.cend(), [x, t] (Constraint c) {
//        return x >= c.x_lb && x <= c.x_ub && t >= c.t_lb && t <= c.t_ub;
//    });
}
}