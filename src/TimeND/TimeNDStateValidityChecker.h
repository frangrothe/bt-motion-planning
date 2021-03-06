//
// Created by francesco on 25.07.21.
//

#ifndef BT_ROBOTICS_TIMENDSTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_TIMENDSTATEVALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include "TimeNDConstraint.h"

namespace ob = ompl::base;
namespace nd {

class TimeNDStateValidityChecker : public ob::StateValidityChecker {
public:

    TimeNDStateValidityChecker(const ompl::base::SpaceInformationPtr &si, int d,
                               std::vector<TimeNDConstraint> constraints, ob::RealVectorBounds bounds,
                               double agentRadius);

    bool isValid(const ompl::base::State *state) const override;

private:
    int d_;

    std::vector<TimeNDConstraint> constraints_;
    ob::RealVectorBounds bounds_;
    double agentRadius_;

    bool isInBounds(const double *values) const;
    bool hasCollision(const double *values, double t) const;
};
}




#endif //BT_ROBOTICS_TIMENDSTATEVALIDITYCHECKER_H
