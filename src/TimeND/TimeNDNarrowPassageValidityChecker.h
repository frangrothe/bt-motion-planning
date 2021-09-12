//
// Created by francesco on 11.09.21.
//

#ifndef BT_ROBOTICS_TIMENDNARROWPASSAGEVALIDITYCHECKER_H
#define BT_ROBOTICS_TIMENDNARROWPASSAGEVALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

namespace ob = ompl::base;
namespace nd {

class TimeNDNarrowPassageValidityChecker : public ob::StateValidityChecker {

public:
    TimeNDNarrowPassageValidityChecker(const ompl::base::SpaceInformationPtr &si, const ob::RealVectorBounds &bounds);

    bool isValid(const ompl::base::State *state) const override;

protected:
    struct NarrowConstraint {
    public:
        NarrowConstraint(double xMin, double xMax, double tMin, double tMax) : xMin(xMin), xMax(xMax),
                                                                               tMin(tMin), tMax(tMax) {};
        double xMin;
        double xMax;
        double tMin;
        double tMax;
    };

    int d_ = 8;

    std::vector<NarrowConstraint> constraints_;
    ob::RealVectorBounds bounds_;

    bool isInBounds(const double *values) const;

};
}




#endif //BT_ROBOTICS_TIMENDNARROWPASSAGEVALIDITYCHECKER_H
