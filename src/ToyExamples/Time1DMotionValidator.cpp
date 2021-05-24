//
// Created by francesco on 19.05.21.
//

#include "Time1DMotionValidator.h"

namespace t1d {

Time1DMotionValidator::Time1DMotionValidator(const ob::SpaceInformationPtr &si, double maxSpeed)
        : MotionValidator(si), maxSpeed_(maxSpeed), stateSpace_(si_->getStateSpace().get()) {}

bool Time1DMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const {
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2)) {
        invalid_++;
        return false;
    }

    /*
     * Check if motion is forward in time and is not exceeding the speed limit
     */
    auto deltaX = abs(s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0]
                  - s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0]);

    auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position
                  - s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    if (!(deltaT > 0 && deltaX / deltaT <= maxSpeed_)) {
        invalid_++;
        return false;
    }

    /*
     * Check if the path between s1 and s2 only goes through a valid are
     * by taking samples and checking them for validity
     * see https://ompl.kavrakilab.org/DiscreteMotionValidator_8cpp_source.html
     */
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
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
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

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

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool Time1DMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                        std::pair<ob::State *, double> &lastValid) const {
    return checkMotion(s1, s2);
}
}

