//
// Created by francesco on 27.05.21.
//

#include "Time2DGoalRegion.h"

namespace time_2d {


Time2DGoalRegion::Time2DGoalRegion(const ompl::base::SpaceInformationPtr &si, double x, double y,
                                         double minTime, double maxTime) : GoalSampleableRegion(si), x_(x),
                                                                           y_(y), minTime_(minTime),
                                                                           maxTime_(maxTime),
                                                                           sampler_(&(*si), x, y, minTime, maxTime) {}

void Time2DGoalRegion::sampleGoal(ob::State *st) const {
    sampler_.sample(st);
}

unsigned int Time2DGoalRegion::maxSampleCount() const {
    return (int) ((maxTime_ - minTime_) / granularityTime_);
}

double Time2DGoalRegion::distanceGoal(const ob::State *st) const {
    double deltaX = fabs(x_- st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0]);
    double deltaY = fabs(y_- st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1]);

    return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}
}