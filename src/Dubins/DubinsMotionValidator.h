//
// Created by francesco on 21.06.21.
//

#ifndef BT_ROBOTICS_DUBINSMOTIONVALIDATOR_H
#define BT_ROBOTICS_DUBINSMOTIONVALIDATOR_H

#include <queue>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/util/Exception.h>
#include <ompl/base/SpaceInformation.h>

#include "../SpaceTimePlanning/AnimationStateSpace.h"

namespace ob = ompl::base;
namespace dubins {

class DubinsMotionValidator : public ob::MotionValidator {

public:
    explicit DubinsMotionValidator(ob::SpaceInformation *si, double vMax = 1.0) : MotionValidator(si), vMax_(vMax) {
        defaultSettings();
    }

    explicit DubinsMotionValidator(const ob::SpaceInformationPtr &si, double vMax = 1.0) : MotionValidator(si), vMax_(vMax) {
        defaultSettings();
    }

    ~DubinsMotionValidator() override = default;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override;

private:
    double vMax_;
    void defaultSettings();
    ob::DubinsStateSpace *stateSpace_;

};
}




#endif //BT_ROBOTICS_DUBINSMOTIONVALIDATOR_H
