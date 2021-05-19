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
#include "ompl/geometric/planners/rrt/RRT.h"

#include "Time1DStateValidityChecker.h"
#include "Time1DMotionValidator.h"
#include "Time1DGoal.h"
#include "structs/Constraint.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace t1d {

class Time1DPlanner {

public:
    explicit Time1DPlanner(std::string filename);

    void planMotion();

private:
    double xBoundLow_ = 0.0;
    double xBoundHigh_ = 2.0;
    double timeBoundLow_ = 0.0;
    double timeBoundHigh_ = 2.5;

    double xStart_ = 0.0;
    double xGoal_ = 1.0;

    std::vector<Constraint> constraints_ {
            {0.4, 0.8, 0.8, 1.2},
            {0.05, 0.15, 0.3, 0.6}
    };
    double maxSpeed_ = 1.0; // 1 m/s
    double goalBias_ = 0.05; // [0, 1] probability that the goal is sampled
    double solveTime_ = 2.0; // in seconds

    std::string filename_;

    void writeResultToCSV(const ob::PlannerData &data);
    void writeConstraintsToCSV();

};
}




#endif //BT_ROBOTICS_TIME1DPLANNER_H
