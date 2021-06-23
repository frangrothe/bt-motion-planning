//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_TIME2DSAMPLER_H
#define BT_ROBOTICS_TIME2DSAMPLER_H

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Exception.h>

namespace ob = ompl::base;
namespace time_2d {

class Time2DSampler : public ob::ValidStateSampler {

public:
    Time2DSampler(const ompl::base::SpaceInformation *si, double x, double y, double minTime,
                                double maxTime);

    bool sample(ompl::base::State *state) override;
    bool sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance) override;

protected:
    ompl::RNG rng_;

private:
    double x_;
    double y_;
    double minTime_;
    double maxTime_;
};
}




#endif //BT_ROBOTICS_TIME2DSAMPLER_H
