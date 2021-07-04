//
// Created by francesco on 04.07.21.
//

#include "VectorSpaceGoalRegion.h"

namespace time_1d {


VectorSpaceGoalRegion::VectorSpaceGoalRegion(const ompl::base::SpaceInformationPtr &si, double minX, double maxX)
        : GoalSampleableRegion(si), minX_(minX), maxX_(maxX), sampler_(&(*si), minX, maxX) {}

double VectorSpaceGoalRegion::distanceGoal(const ompl::base::State *st) const {
    double x = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    return x < minX_ ? minX_ - x : x > maxX_ ? x - maxX_ : 0;
}

void VectorSpaceGoalRegion::sampleGoal(ompl::base::State *st) const {

    st->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = 0;
    if (minX_ == maxX_) {
        st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = minX_;
    }
    else {
        sampler_.sample(st);
    }
}

unsigned int VectorSpaceGoalRegion::maxSampleCount() const {
    return minX_ == maxX_ ? 1u : std::numeric_limits<unsigned int>::max();
}
}
