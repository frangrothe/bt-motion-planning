//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_SPACETIMEMOTIONVALIDATOR_H
#define BT_ROBOTICS_SPACETIMEMOTIONVALIDATOR_H

#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <queue>

#include "../SpaceTimePlanning/AnimationStateSpace.h"

namespace ob = ompl::base;
namespace time_2d {

class SpaceTimeMotionValidator : public ob::MotionValidator {

public:
    explicit SpaceTimeMotionValidator(const ompl::base::SpaceInformationPtr &si, unsigned int dim, double vMax = 1.0);

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override;

private:
    double vMax_;
    ob::StateSpace *stateSpace_;
    ob::RealVectorStateSpace vectorStateSpace_;
};
}




#endif //BT_ROBOTICS_SPACETIMEMOTIONVALIDATOR_H
