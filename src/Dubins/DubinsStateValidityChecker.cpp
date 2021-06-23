//
// Created by francesco on 21.06.21.
//

#include "DubinsStateValidityChecker.h"

#include <utility>


namespace dubins {


DubinsStateValidityChecker::DubinsStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                                       std::vector<DubinsConstraint> constraints)
        : StateValidityChecker(si), constraints_(std::move(constraints)) {}

bool DubinsStateValidityChecker::isValid(const ompl::base::State *state) const {

//    double x = state->as<ob::SE2StateSpace::StateType>()->getX();
//    double y = state->as<ob::SE2StateSpace::StateType>()->getY();
//    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

//    double x = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getX();
//    double y = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getY();
//    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

//    for (const auto& c : constraints_) {
//        double xlb, xub, ylb, yub;
//        std::tie(xlb, xub, ylb, yub) = c.getBoundsForTime(t);
//
//        if (x >= xlb && x <= xub && y >= ylb && y <= yub) return false;
//    }

    return true;
}
}