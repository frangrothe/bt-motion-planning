//
// Created by francesco on 19.05.21.
//

#ifndef BT_ROBOTICS_TIME1DMOTIONVALIDATOR_H
#define BT_ROBOTICS_TIME1DMOTIONVALIDATOR_H

#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include <queue>

namespace ob = ompl::base;
namespace time_1d {

class Time1DMotionValidator : public ob::MotionValidator {

public:
    Time1DMotionValidator(const ob::SpaceInformationPtr &si, double maxSpeed);

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override;

private:
    double maxSpeed_; // in m/s
    ob::StateSpace *stateSpace_;

};

}




#endif //BT_ROBOTICS_TIME1DMOTIONVALIDATOR_H
