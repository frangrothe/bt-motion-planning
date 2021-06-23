//
// Created by francesco on 21.06.21.
//

#include <vector>
#include <iostream>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "../SpaceTimePlanning/AnimationStateSpace.h"
#include "../SpaceTimePlanning/SpaceTimeRRT.h"

#include "DubinsStateValidityChecker.h"
#include "DubinsMotionValidator.h"
#include "DubinsConstraint.h"
#include "goals/DubinsGoalRegion.h"

#ifndef BT_ROBOTICS_DUBINSPLANNER_H
#define BT_ROBOTICS_DUBINSPLANNER_H

namespace ob = ompl::base;
namespace dubins {

class DubinsPlanner {

public:
    explicit DubinsPlanner(std::string filename);

    void planMotion();

private:
    double xBoundLow_ = -1.0;
    double xBoundHigh_ = 11.0;
    double yBoundLow_ = 0.0;
    double yBoundHigh_ = 2.0;
    double timeBoundLow_ = 0.0;
    double timeBoundHigh_ = 30.0;
    double vMax_ = 1.0;

    double xStart_ = 0.5;
    double yStart_ = 1.5;
    double yawStart_ = 0; // value element of (-Pi, Pi)
    double xGoal_ = 10.0;
    double yGoal_ = 1.5;
    double yawGoal_ = 0.0;
    double minTime_ = 0.0; // minimum time for the goal to be able to be reached. Calculated during planning

    double plannerRange_ = 1.0;
    double solveTime_ = 2.0;

    std::string filename_;
    std::string delim_ = ",";

    std::vector<DubinsConstraint> constraints_{

            // Static obstacle
            {[](double x, double y, double t) {
                return std::make_tuple(x, y);
            }, 1.5, 1.5, 0.5, 0.25}
    };

};
}




#endif //BT_ROBOTICS_DUBINSPLANNER_H
