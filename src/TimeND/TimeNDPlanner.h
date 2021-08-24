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
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include "../SpaceTimePlanning/SpaceTimeRRT.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"
#include "../SpaceTimePlanning/MinimizeArrivalTime.h"
#include "TimeNDStateValidityChecker.h"
#include "TimeNDMotionValidator.h"
#include "TimeNDGoal.h"
#include "../json.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace nd {

class TimeNDPlanner {

public:
    enum Planners {
        SpaceTimeRRT,
        RRTConnect,
        RRTStar
    };

    explicit TimeNDPlanner(int d);

    void plan();
    void benchmark();
    void loadConfigFromJSON(const std::string& filename);
    void setPlanner(Planners planner) {plannerType_ = planner;}
    void setSolveTime(double t) {solveTime_ = t;}
    void setUpperTimeBound(double utb) {upperTimeBound_ = utb;}

private:
    int d_; // dimensions
    double solveTime_ = 10.0;
    double timeWeight_ = 0.5;
    int batchSize_ = 200;
    double plannerRangeFactor_ = 0.2;
    double upperTimeBound_ = -std::numeric_limits<double>::infinity();

    std::vector<double> startPosition_;
    std::vector<double> goalPosition_;
    double agentRadius_ = 0.1;
    std::vector<TimeNDConstraint> constraints_{};

    std::string filename_;
    void writeSolutionToJSON(const ompl::base::PathPtr &pathPtr);

    Planners plannerType_ = SpaceTimeRRT;

    og::SimpleSetup createSimpleSetup();
    ompl::base::PlannerPtr createRRTConnect(const ompl::base::SpaceInformationPtr &si);
    ompl::base::PlannerPtr createRRTStar(const ompl::base::SpaceInformationPtr &si);
    ompl::base::PlannerPtr createSpaceTimeRRT(const ompl::base::SpaceInformationPtr &si);

};
}




#endif //BT_ROBOTICS_TIMENDPLANNER_H
