//
// Created by francesco on 21.06.21.
//

#include "DubinsStateValidityChecker.h"




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

//    double x = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getX();
//    double y = state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->getY();
//    double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


    return isInBounds(state);
}

bool DubinsStateValidityChecker::isInBounds(const ompl::base::State *state) const {
    bool inTimeBound = si_->getStateSpace().get()->as<space_time::AnimationStateSpace>()->getTimeComponent()->satisfiesBounds(
            state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1));
    bool inSpaceBounds = si_->getStateSpace().get()->as<space_time::AnimationStateSpace>()->getSpaceComponent()
            ->as<ob::SE2StateSpace>()->as<ob::RealVectorStateSpace>(0)->satisfiesBounds(
                    state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->as<ob::RealVectorStateSpace::StateType>(0));

    return inTimeBound && inSpaceBounds;
}
}