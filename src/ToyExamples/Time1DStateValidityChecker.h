//
// Created by francesco on 19.05.21.
//

#ifndef BT_ROBOTICS_TIME1DSTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_TIME1DSTATEVALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "../structs/Constraint.h"

namespace ob = ompl::base;
namespace t1d {

class Time1DStateValidityChecker : public ob::StateValidityChecker{

public:
    explicit Time1DStateValidityChecker(const ob::SpaceInformationPtr &si);
    Time1DStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<Constraint> constraints);

    bool isValid(const ob::State *state) const override;

private:
    std::vector<Constraint> constraints_;
};
}




#endif //BT_ROBOTICS_TIME1DSTATEVALIDITYCHECKER_H
