//
// Created by francesco on 27.05.21.
//

#include "Time2DSampler.h"

namespace time_2d {

Time2DSampler::Time2DSampler(const ompl::base::SpaceInformation *si, double x,
                                                         double y, double minTime, double maxTime)
        : ValidStateSampler(si), x_(x), y_(y), minTime_(minTime), maxTime_(maxTime) {}

bool Time2DSampler::sample(ompl::base::State *state) {

    double t = rng_.uniformReal(minTime_,maxTime_);

    state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x_;
    state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y_;
    state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = t;
    return true;
}

bool Time2DSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near,
                                             const double distance) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
}
}