//
// Created by francesco on 21.06.21.
//



#include <ompl/base/ScopedState.h>
#include "DubinsMotionValidator.h"

namespace dubins {


void DubinsMotionValidator::defaultSettings() {

    stateSpace_ = si_->getStateSpace().get()->as<space_time::AnimationStateSpace>()->getSpaceComponent()->as<ob::DubinsStateSpace>();
    if (stateSpace_ == nullptr)
        throw ompl::Exception("No state space for motion validator");
}

bool DubinsMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    /*
     * Check if motion is forward in time and is not exceeding the speed limit
     */
    auto deltaSpace = si_->getStateSpace()->as<space_time::AnimationStateSpace>()->distanceSpace(s1, s2);

    auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position
                  - s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    if (!(deltaT > 0 && deltaSpace / deltaT <= vMax_)) {
        invalid_++;
        return false;
    }

    bool result = true, firstTime = true;
    ob::DubinsStateSpace::DubinsPath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

//        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool DubinsMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                        std::pair<ob::State *, double> &lastValid) const {
    return checkMotion(s1, s2);
}
}