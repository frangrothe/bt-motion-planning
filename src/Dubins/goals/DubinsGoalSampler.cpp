//
// Created by francesco on 21.06.21.
//


#include "DubinsGoalSampler.h"

namespace dubins {

DubinsGoalSampler::DubinsGoalSampler(const ompl::base::SpaceInformation *si, double x, double y, double yaw, double minTime,
                                     double maxTime) : ValidStateSampler(si), x_(x), y_(y), yaw_(yaw), minTime_(minTime),
                                                       maxTime_(maxTime) {}

bool DubinsGoalSampler::sample(ompl::base::State *state) {
    double t = rng_.uniformReal(minTime_, maxTime_);

    state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->setXY(x_, y_);
    state->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0)->setYaw(yaw_);
    state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position = t;
    return true;
}

bool DubinsGoalSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance) {
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
}
}