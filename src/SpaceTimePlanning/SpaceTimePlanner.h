//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_SPACETIMEPLANNER_H
#define BT_ROBOTICS_SPACETIMEPLANNER_H

#include <vector>
#include <iostream>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "../structs/Constraint.h"
#include "../auxillary.h"
#include "spaces/AnimationStateSpace.h"
#include "SpaceTimeStateValidityChecker.h"
#include "SpaceTimeMotionValidator.h"
#include "goals/SpaceTimeGoalRegion.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace SpaceTime {

class SpaceTimePlanner {

public:
    explicit SpaceTimePlanner(std::string filename);

    void planMotion();

private:
    double xBoundLow_ = 0.0;
    double xBoundHigh_ = 2.0;
    double timeBoundLow_ = 0.0;
    double timeBoundHigh_ = 2.5;

    double xStart_ = 0.0;
    double xGoalRegionLeft_ = 1.0;
    double xGoalRegionRight_ = 1.1;
    double minTime_ = 0.0; // minimum time for the goal to be able to be reached. Calculated during planning

    std::vector<Constraint> constraints_ {
            {0.4, 0.8, 0.8, 1.2},
            {0.05, 0.15, 0.3, 0.6}
    };
    double vMax_ = 1.0; // 1 m/s
    double solveTime_ = 1.0; // in seconds
    double plannerRange_ = 0.5;

    std::string filename_;

};
}




#endif //BT_ROBOTICS_SPACETIMEPLANNER_H
