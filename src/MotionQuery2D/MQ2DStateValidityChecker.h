//
// Created by francesco on 19.07.21.
//

#ifndef BT_ROBOTICS_MQ2DSTATEVALIDITYCHECKER_H
#define BT_ROBOTICS_MQ2DSTATEVALIDITYCHECKER_H

#include <utility>
#include <cmath>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "StaticConstraint.h"
#include "Point.h"

namespace ob = ompl::base;
namespace query2d {

class MQ2DStateValidityChecker : public ob::StateValidityChecker {

public:
    MQ2DStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                             std::vector<StaticConstraint> staticConstraints,
                             std::vector<std::vector<Point>> pathConstraints, ob::RealVectorBounds bounds,
                             double agentRadius);

    bool isValid(const ompl::base::State *state) const override;

private:
    std::vector<StaticConstraint> staticConstraints_;
    std::vector<std::vector<Point>> pathConstraints_;
    ob::RealVectorBounds bounds_;
    double agentRadius_;

    bool isInBounds(double x, double y) const;

};
}




#endif //BT_ROBOTICS_MQ2DSTATEVALIDITYCHECKER_H
