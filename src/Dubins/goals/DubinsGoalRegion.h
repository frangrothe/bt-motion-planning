//
// Created by francesco on 21.06.21.
//

#ifndef BT_ROBOTICS_DUBINSGOALREGION_H
#define BT_ROBOTICS_DUBINSGOALREGION_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>

#include "DubinsGoalSampler.h"
#include "../../SpaceTimePlanning/AnimationStateSpace.h"

namespace ob = ompl::base;
namespace dubins {

class DubinsGoalRegion : public ob::GoalSampleableRegion {

public:
    DubinsGoalRegion(const ompl::base::SpaceInformationPtr &si, double x, double y, double yaw, double minTime, double maxTime);

    double distanceGoal(const ompl::base::State *st) const override;

    void sampleGoal(ompl::base::State *st) const override;

    unsigned int maxSampleCount() const override;

private:
    double x_;
    double y_;
    double yaw_;
    double minTime_;
    double maxTime_;
    double granularityTime_ = 0.01; // only used to calculate max sample count
    ompl::RNG rng_;

    mutable DubinsGoalSampler sampler_; // Valid State Sampler to sample goal states
    ompl::base::State * goalState_;

};
}




#endif //BT_ROBOTICS_DUBINSGOALREGION_H
