//
// Created by francesco on 20.05.21.
//

#ifndef BT_ROBOTICS_TIME1DGOALREGION_H
#define BT_ROBOTICS_TIME1DGOALREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/StateSampler.h>

#include "Time1DValidStateSampler.h"

namespace ob = ompl::base;
namespace time_1d {

class Time1DGoalRegion : public ob::GoalSampleableRegion {

public:
    Time1DGoalRegion(const ob::SpaceInformationPtr &si, double minX, double maxX, double minTime,
                     double maxTime);

    void sampleGoal(ob::State *st) const override;
    unsigned int maxSampleCount() const override;
    double distanceGoal(const ob::State *st) const override;

private:
    double minX_;
    double maxX_;
    double minTime_;
    double maxTime_;
    double granularityX_ = 0.01; // only used to calculate max sample count
    double granularityTime_ = 0.01; // only used to calculate max sample count

    mutable Time1DValidStateSampler sampler_; // Valid State Sampler to sample goal states
};
}




#endif //BT_ROBOTICS_TIME1DGOALREGION_H
