//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_TIME2DSTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_TIME2DSTATEVALIDITYCHECKER_H

#include <vector>
#include <utility>
#include <algorithm>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "Constraint.h"

namespace ob = ompl::base;
namespace time_2d {

class Time2DStateValidityChecker : public ob::StateValidityChecker {
public:
    Time2DStateValidityChecker(const ompl::base::SpaceInformationPtr &si, ob::RealVectorBounds bounds, std::vector<Constraint> constraints,
                               double agentWidth = 0.1, double agentHeight = 0.1);

    bool isValid(const ompl::base::State *state) const override;

private:
    std::vector<Constraint> constraints_;
    ob::RealVectorBounds bounds_;
    double agentWidth_;
    double agentHeight_;

    bool isInBounds(double minX, double maxX, double minY, double maxY) const;
};
}




#endif //BT_ROBOTICS_TIME2DSTATEVALIDITYCHECKER_H
