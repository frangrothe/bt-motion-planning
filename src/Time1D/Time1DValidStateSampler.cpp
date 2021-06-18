//
// Created by francesco on 20.05.21.
//



#include "Time1DValidStateSampler.h"

namespace time_1d {

Time1DValidStateSampler::Time1DValidStateSampler(const ompl::base::SpaceInformation *si, double minX, double maxX,
                                                 double minTime, double maxTime) : ValidStateSampler(si), minX_(minX),
                                                                                   maxX_(maxX), minTime_(minTime),
                                                                                   maxTime_(maxTime) {}

bool Time1DValidStateSampler::sample(ob::State *state) {
    double x = rng_.uniformReal(minX_,maxX_);
    double t = rng_.uniformReal(minTime_,maxTime_);

    state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
    state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = t;
    return true;
}

bool
Time1DValidStateSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
}


}