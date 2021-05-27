//
// Created by francesco on 27.05.21.
//

#include "SpaceTimeGoalRegion.h"

namespace SpaceTime {


SpaceTimeGoalRegion::SpaceTimeGoalRegion(const ompl::base::SpaceInformationPtr &si, double minX, double maxX,
                                         double minTime, double maxTime) : GoalSampleableRegion(si), minX_(minX),
                                                                           maxX_(maxX), minTime_(minTime),
                                                                           maxTime_(maxTime),
                                                                           sampler_(&(*si), minX, maxX, minTime, maxTime) {}

void SpaceTimeGoalRegion::sampleGoal(ob::State *st) const {
    sampler_.sample(st);
}

unsigned int SpaceTimeGoalRegion::maxSampleCount() const {
    int nXSamples = (int) ((maxX_ - minX_) / granularityX_);
    int nTimeSamples = (int) ((maxTime_ - minTime_) / granularityTime_);
    return nXSamples * nTimeSamples;
}

double SpaceTimeGoalRegion::distanceGoal(const ob::State *st) const {
    double deltaX = minX_ - st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    return deltaX > 0 ? deltaX : 0.0;
}


}