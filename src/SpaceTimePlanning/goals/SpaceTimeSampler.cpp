//
// Created by francesco on 27.05.21.
//

#include "SpaceTimeSampler.h"

namespace SpaceTime {

SpaceTimeSampler::SpaceTimeSampler(const ompl::base::SpaceInformation *si, double x,
                                                         double y, double minTime, double maxTime)
        : ValidStateSampler(si), x_(x), y_(y), minTime_(minTime), maxTime_(maxTime) {}

bool SpaceTimeSampler::sample(ompl::base::State *state) {

    double t = rng_.uniformReal(minTime_,maxTime_);

    state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x_;
    state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y_;
    state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = t;
    return true;
}

bool SpaceTimeSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near,
                                             const double distance) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
}
}