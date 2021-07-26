//
// Created by francesco on 19.07.21.
//

#ifndef BT_ROBOTICS_MOTIONQUERY2DPLANNER_H
#define BT_ROBOTICS_MOTIONQUERY2DPLANNER_H

#include <vector>
#include <iostream>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/GoalSpace.h>

#include "../json.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"
#include "../SpaceTimePlanning/SpaceTimeRRT.h"
#include "MQ2DStateValidityChecker.h"
#include "MQ2DMotionValidator.h"
#include "MQ2DGoal.h"

namespace query2d {

class MotionQuery2DPlanner {

public:
    MotionQuery2DPlanner(double xStart, double yStart, double xGoal, double yGoal);
    void planMotion();

    std::vector<Point> getPathConstraint();
    void setPathConstraints(const std::vector<std::vector<Point>>& pathConstraints)
    {
        pathConstraints_ = pathConstraints;
    }

    void writeConstraintsToJSON(const std::string& filename);
    void writeSolutionToJSON(const std::string& filename, int index);


private:
    double xBoundLow_ = 0.0;
    double xBoundHigh_ = 50.0;
    double yBoundLow_ = 0.0;
    double yBoundHigh_ = 50.0;

    double radius_ = 1.0;
    double vMax_ = 1.0; // 1 m/s
    double solveTime_ = 2.0; // in seconds
    double timeWeight_ = 0.5; // weight of time for the distance calculation, ~ [0,1]
    double plannerRange_ = 2.5;

    double xStart_;
    double yStart_;
    double xGoal_;
    double yGoal_;

    std::vector<StaticConstraint> staticConstraints_ {{25.0, 25.0, 10.0}};
    std::vector<std::vector<Point>> pathConstraints_{};

    ob::PathPtr solutionPath_;
};
}




#endif //BT_ROBOTICS_MOTIONQUERY2DPLANNER_H
