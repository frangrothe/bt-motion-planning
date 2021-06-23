//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_TIME2DGOALREGION_H
#define BT_ROBOTICS_TIME2DGOALREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/StateSampler.h>

#include "Time2DSampler.h"

namespace ob = ompl::base;
namespace time_2d {

class Time2DGoalRegion : public ob::GoalSampleableRegion {

public:
    Time2DGoalRegion(const ompl::base::SpaceInformationPtr &si, double x, double y, double minTime,
                        double maxTime);

    void sampleGoal(ob::State *st) const override;
    unsigned int maxSampleCount() const override;
    double distanceGoal(const ob::State *st) const override;

private:
    double x_;
    double y_;
    double minTime_;
    double maxTime_;
    double granularityTime_ = 0.01; // only used to calculate max sample count

    mutable Time2DSampler sampler_; // Valid State Sampler to sample goal states
};
}




#endif //BT_ROBOTICS_TIME2DGOALREGION_H
