//
// Created by francesco on 25.07.21.
//

#include "TimeNDMotionValidator.h"

namespace nd {

TimeNDMotionValidator::TimeNDMotionValidator(const ompl::base::SpaceInformationPtr &si, unsigned int dim, double vMax)
        : MotionValidator(si), stateSpace_(si_->getStateSpace().get()), vMax_(vMax),
          vectorStateSpace_(dim) {}

bool TimeNDMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2)) {
        invalid_++;
        return false;
    }

    /*
     * Check if motion is forward in time and is not exceeding the speed limit
     */
    auto deltaSpace = vectorStateSpace_.distance(s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0),
                                                 s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0));

    auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position
                  - s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    if (!(deltaT > 0 && deltaSpace / deltaT <= vMax_)) {
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

bool TimeNDMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                        std::pair<ob::State *, double> &lastValid) const {
    return checkMotion(s1, s2);
}
}