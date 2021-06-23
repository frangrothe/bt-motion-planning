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
    explicit Time2DStateValidityChecker(const ompl::base::SpaceInformationPtr &si);

    Time2DStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                  std::vector<Constraint> constraints);

    bool isValid(const ompl::base::State *state) const override;

private:
    std::vector<Constraint> constraints_;
};
}




#endif //BT_ROBOTICS_TIME2DSTATEVALIDITYCHECKER_H
