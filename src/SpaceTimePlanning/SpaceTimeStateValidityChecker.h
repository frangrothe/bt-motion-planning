//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_SPACETIMESTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_SPACETIMESTATEVALIDITYCHECKER_H

#include <vector>
#include <utility>
#include <algorithm>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "../structs/Constraint.h"

namespace ob = ompl::base;
namespace SpaceTime {

class SpaceTimeStateValidityChecker : public ob::StateValidityChecker {
public:
    explicit SpaceTimeStateValidityChecker(const ompl::base::SpaceInformationPtr &si);

    SpaceTimeStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                  std::vector<Constraint> constraints);

    bool isValid(const ompl::base::State *state) const override;

private:
    std::vector<Constraint> constraints_;
};
}




#endif //BT_ROBOTICS_SPACETIMESTATEVALIDITYCHECKER_H
