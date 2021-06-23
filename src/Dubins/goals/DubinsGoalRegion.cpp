//
// Created by francesco on 21.06.21.
//

#include "DubinsGoalRegion.h"

namespace dubins {


DubinsGoalRegion::DubinsGoalRegion(const ompl::base::SpaceInformationPtr &si, double x, double y, double yaw, double minTime,
                                   double maxTime) : GoalSampleableRegion(si), x_(x), y_(y), yaw_(yaw), minTime_(minTime),
                                                     maxTime_(maxTime), sampler_(&(*si), x, y, yaw, minTime, maxTime) {
    goalState_ = si_->allocState();
    sampler_.sample(goalState_);
}

double DubinsGoalRegion::distanceGoal(const ompl::base::State *st) const {
    return si_->getStateSpace().get()->as<space_time::AnimationStateSpace>()->distanceSpace(st, goalState_);
}

void DubinsGoalRegion::sampleGoal(ompl::base::State *st) const {
    sampler_.sample(st);
}

unsigned int DubinsGoalRegion::maxSampleCount() const {
    return (int) ((maxTime_ - minTime_) / granularityTime_);
}
}