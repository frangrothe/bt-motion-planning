//
// Created by francesco on 25.07.21.
//

#ifndef BT_ROBOTICS_TIMENDSTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_TIMENDSTATEVALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

namespace ob = ompl::base;
namespace nd {

class TimeNDStateValidityChecker : public ob::StateValidityChecker {
public:

    TimeNDStateValidityChecker(const ompl::base::SpaceInformationPtr &si, int d, double constraintTime);

    bool isValid(const ompl::base::State *state) const override;

private:
    int d_;
    double constraintTime_;

};
}




#endif //BT_ROBOTICS_TIMENDSTATEVALIDITYCHECKER_H
