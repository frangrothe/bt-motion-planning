//
// Created by francesco on 21.06.21.
//

#ifndef BT_ROBOTICS_DUBINSSTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_DUBINSSTATEVALIDITYCHECKER_H

#include <vector>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "DubinsConstraint.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"

namespace ob = ompl::base;
namespace dubins {

class DubinsStateValidityChecker : public ob::StateValidityChecker {

public:
    DubinsStateValidityChecker(const ompl::base::SpaceInformationPtr &si, std::vector<DubinsConstraint> constraints);

    bool isValid(const ompl::base::State *state) const override;

private:
    std::vector<DubinsConstraint> constraints_;
};
}




#endif //BT_ROBOTICS_DUBINSSTATEVALIDITYCHECKER_H
