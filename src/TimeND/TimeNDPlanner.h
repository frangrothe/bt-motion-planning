//
// Created by francesco on 25.07.21.
//

#ifndef BT_ROBOTICS_TIMENDPLANNER_H
#define BT_ROBOTICS_TIMENDPLANNER_H

#include <vector>
#include <iostream>
#include <fstream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>

#include "../SpaceTimePlanning/SpaceTimeRRT.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"
#include "TimeNDStateValidityChecker.h"
#include "TimeNDMotionValidator.h"
#include "TimeNDGoal.h"

namespace ob =ompl::base;
namespace nd {

class TimeNDPlanner {

public:
    void planMotion();

private:
    int d_ = 1000; // dimensions
    double solveTime_ = 10.0;
    double timeWeight_ = 0.5;
    double constraintTime_ = 200; // up to which time the goal region is blocked
    int batchSize_ = 200;
    double plannerRangeFactor_ = 0.2;

};
}




#endif //BT_ROBOTICS_TIMENDPLANNER_H
