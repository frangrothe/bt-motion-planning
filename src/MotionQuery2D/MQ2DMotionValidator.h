//
// Created by francesco on 19.07.21.
//

#ifndef BT_ROBOTICS_MQ2DMOTIONVALIDATOR_H
#define BT_ROBOTICS_MQ2DMOTIONVALIDATOR_H

#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <queue>

#include "../SpaceTimePlanning/AnimationStateSpace.h"

namespace ob = ompl::base;
namespace query2d {

class MQ2DMotionValidator : public ob::MotionValidator {

public:
    MQ2DMotionValidator(const ompl::base::SpaceInformationPtr &si, unsigned int dim, double vMax = 1.0);

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override;

private:
    double vMax_;
    ob::StateSpace *stateSpace_;
    ob::RealVectorStateSpace vectorStateSpace_;
};
}




#endif //BT_ROBOTICS_MQ2DMOTIONVALIDATOR_H
