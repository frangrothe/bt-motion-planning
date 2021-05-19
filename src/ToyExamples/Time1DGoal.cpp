//
// Created by francesco on 19.05.21.
//


#include "Time1DGoal.h"

namespace t1d {

Time1DGoal::Time1DGoal(const ompl::base::SpaceInformationPtr &si, double x) : Goal(si), x_(x) {}

bool Time1DGoal::isSatisfied(const ompl::base::State *st) const {
    auto currentX = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];

    return currentX >= x_;
}
}