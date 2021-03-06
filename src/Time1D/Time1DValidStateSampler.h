//
// Created by francesco on 20.05.21.
//

#ifndef BT_ROBOTICS_TIME1DVALIDSTATESAMPLER_H
#define BT_ROBOTICS_TIME1DVALIDSTATESAMPLER_H

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Exception.h>

namespace ob = ompl::base;
namespace time_1d {

class Time1DValidStateSampler : public ob::ValidStateSampler {

public:
    Time1DValidStateSampler(const ompl::base::SpaceInformation *si, double minX, double maxX, double minTime,
                            double maxTime);

    bool sample(ompl::base::State *state) override;
    bool sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance) override;

protected:
    ompl::RNG rng_;

private:
    double minX_;
    double maxX_;
    double minTime_;
    double maxTime_;
};
}




#endif //BT_ROBOTICS_TIME1DVALIDSTATESAMPLER_H
