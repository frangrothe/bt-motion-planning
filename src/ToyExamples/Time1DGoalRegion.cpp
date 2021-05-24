//
// Created by francesco on 20.05.21.
//


#include "Time1DGoalRegion.h"

namespace t1d {

Time1DGoalRegion::Time1DGoalRegion(const ob::SpaceInformationPtr &si, double minX, double maxX, double minTime,
                                   double maxTime) : GoalSampleableRegion(si) , minX_(minX), maxX_(maxX),
                                                     minTime_(minTime), maxTime_(maxTime),
                                                     sampler_(&(*si), minX, maxX, minTime, maxTime) {}

void Time1DGoalRegion::sampleGoal(ob::State *st) const{
    sampler_.sample(st);
}

unsigned int Time1DGoalRegion::maxSampleCount() const {
    int nXSamples = (int) ((maxX_ - minX_) / granularityX_);
    int nTimeSamples = (int) ((maxTime_ - minTime_) / granularityTime_);
    return nXSamples * nTimeSamples;
}

double Time1DGoalRegion::distanceGoal(const ob::State *st) const {
    double deltaX = minX_ - st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    return deltaX > 0 ? deltaX : 0.0;
}
}

