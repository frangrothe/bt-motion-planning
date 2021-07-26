//
// Created by francesco on 25.07.21.
//

#include "TimeNDStateValidityChecker.h"

namespace nd {

TimeNDStateValidityChecker::TimeNDStateValidityChecker(const ompl::base::SpaceInformationPtr &si, int d,
                                                       double constraintTime) : StateValidityChecker(si), d_(d),
                                                                                constraintTime_(constraintTime) {}

bool TimeNDStateValidityChecker::isValid(const ompl::base::State *state) const {

    auto t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
    if (t > constraintTime_)
        return true;

//    for (int i = 0; i < d_; ++i) {
//        double v = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[i];
//        if (v < 0.8 || v > 0.9)
//            return true;
//    }
    double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    if (x < 0.8 || x > 0.9)
        return true;

    return false;

}
}

