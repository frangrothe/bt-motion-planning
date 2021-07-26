//
// Created by francesco on 19.05.21.
//

#ifndef BT_ROBOTICS_TIME1DPLANNER_H
#define BT_ROBOTICS_TIME1DPLANNER_H

#include <vector>
#include <iostream>
#include <fstream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSpace.h>
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include <ompl/base/ProblemDefinition.h>


#include "Time1DStateValidityChecker.h"
#include "Time1DMotionValidator.h"
#include "Time1DGoalRegion.h"
#include "goals/VectorSpaceGoalRegion.h"

#include "../SpaceTimePlanning/SpaceTimeRRT.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace time_1d {

class Time1DPlanner {

public:
    explicit Time1DPlanner(std::string filename);

    void planMotion();

private:
    double xBoundLow_ = -2.0;
    double xBoundHigh_ = 2.0;
    double timeBoundLow_ = 0.0;
    double timeBoundHigh_ = 3.0;

    double xStart_ = 0.0;
    std::vector<std::pair<double, double>> goalRegions_ = {
            {1.0, 1.05}
//            {-1.2, -1.2}
    };

    std::vector<Constraint> constraints_ {
//            {0.05, 0.15, 0.16, 0.35},
//            {0.5, 0.6, 0.51, 0.6},
//            {0.5, 0.6, 0.71, 0.8},
//            {0.5, 0.6, 1.01, 1.1},
            {0.9, 1.1, 0.0, 30.0}
    };
    double vMax_ = 1.0; // 1 m/s
    double timeWeight_ = 0.5; // compared to distance weight used in distance function. [0,1]
    double solveTime_ = 30.0; // in seconds
    double plannerRange_ = 0.2;

    std::string filename_;
    std::string delim_ = ",";

    void writeSamplesToCSV(const ob::PlannerData &data);
    void writePathToCSV(const ob::PathPtr &);
    void writeConstraintsToCSV();
    void writeGoalRegionToCSV();

};
}




#endif //BT_ROBOTICS_TIME1DPLANNER_H
