//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_SPACETIMEGOALREGION_H
#define BT_ROBOTICS_SPACETIMEGOALREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/StateSampler.h>

#include "SpaceTimeSampler.h"

namespace ob = ompl::base;
namespace SpaceTime {

class SpaceTimeGoalRegion : public ob::GoalSampleableRegion {

public:
    SpaceTimeGoalRegion(const ompl::base::SpaceInformationPtr &si, double minX, double maxX, double minTime,
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

    mutable SpaceTimeSampler sampler_; // Valid State Sampler to sample goal states
};
}




#endif //BT_ROBOTICS_SPACETIMEGOALREGION_H
