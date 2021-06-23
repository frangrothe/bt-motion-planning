//
// Created by francesco on 21.06.21.
//

#ifndef BT_ROBOTICS_DUBINSGOALSAMPLER_H
#define BT_ROBOTICS_DUBINSGOALSAMPLER_H

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>


namespace ob = ompl::base;
namespace dubins {

class DubinsGoalSampler : public ob::ValidStateSampler {

public:
    DubinsGoalSampler(const ompl::base::SpaceInformation *si, double x, double y, double yaw, double minTime, double maxTime);

    bool sample(ompl::base::State *state) override;

    bool sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance) override;

protected:
    ompl::RNG rng_;

private:
    double x_;
    double y_;
    double yaw_;
    double minTime_;
    double maxTime_;

};
}




#endif //BT_ROBOTICS_DUBINSGOALSAMPLER_H
