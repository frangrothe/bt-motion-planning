//
// Created by francesco on 27.05.21.
//

#ifndef BT_ROBOTICS_TIME2DPLANNER_H
#define BT_ROBOTICS_TIME2DPLANNER_H

#include <vector>
#include <iostream>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "Constraint.h"
#include "../auxillary.h"
#include "../json.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"
#include "../SpaceTimePlanning/SpaceTimeRRT.h"
#include "Time2DStateValidityChecker.h"
#include "Time2DMotionValidator.h"
#include "goals/Time2DGoalRegion.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace time_2d {

class Time2DPlanner {

public:
    explicit Time2DPlanner(std::string filename);

    void planMotion();
    void test();

private:
    double xBoundLow_ = 0.0;
    double xBoundHigh_ = 2.0;
    double yBoundLow_ = 0.0;
    double yBoundHigh_ = 2.0;
    double timeBoundLow_ = 0.0;
    double timeBoundHigh_ = 2.5;

    double xStart_ = 0.0;
    double yStart_ = 0.0;
    double xGoal_ = 1.0;
    double yGoal_ = 1.0;
    double minTime_ = 0.0; // minimum time for the goal to be able to be reached. Calculated during planning

//    std::vector<Constraint> constraints_ {
//            {0.4, 0.8, 0.8, 1.2},
//            {0.05, 0.15, 0.3, 0.6}
//    };

    std::vector<Constraint> constraints_{
            // Movement between x = 0 and x = 1
            {[](double x, double y, double t) {
                x = x + 0.5 * sin(t * M_PI);
                return std::make_tuple(x, y);
            }, 0.5,  0.7,  0.1,  0.1},

            // Static obstacle
            {[](double x, double y, double t) {
                return std::make_tuple(x, y);
            }, 0.35, 0.3, 0.25, 0.25}
    };

//    // constant acceleration movement
//    {[](double x, double y, double t) {
//            double u = -0.1; // initial velocity
//            double a = -0.15; // constant acceleration
//            return std::make_tuple(x + u * t + 0.5 * a * t * t, y + u * t + 0.5 * a * t * t);
//        }, 0.3, 0.6, 0.1, 0.1},
//
//    // Movement between x = 0 and x = 1
//    {[](double x, double y, double t) {
//            x = x + 0.5 * sin(t * M_PI);
//            return std::make_tuple(x, y);
//        }, 0.5, 1.0, 0.1, 0.1},

    double vMax_ = 1.0; // 1 m/s
    double solveTime_ = 1.0; // in seconds
    double plannerRange_ = 0.5;

    std::string filename_;
    std::string delim_ = ",";

    void writeSamplesToCSV(const ob::PlannerData &data);
    void writePathToCSV(const ob::PathPtr &pathPtr);
    void writeConstraintsToJSON();
    void writeGoalRegionToCSV();

};
}




#endif //BT_ROBOTICS_TIME2DPLANNER_H
