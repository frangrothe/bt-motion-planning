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
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/tools/benchmark/Benchmark.h>


#include "Time1DStateValidityChecker.h"
#include "Time1DMotionValidator.h"
#include "Time1DGoalRegion.h"
#include "goals/VectorSpaceGoalRegion.h"

#include "../SpaceTimePlanning/SpaceTimeRRT.h"
#include "../SpaceTimePlanning/AnimationStateSpace.h"
#include "../SpaceTimePlanning/MinimizeArrivalTime.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace time_1d {

class Time1DPlanner {

public:
    explicit Time1DPlanner(std::string filename);

    void planMotion();
    void benchmark();

    enum Planners {
        SpaceTimeRRT,
        RRTConnect,
        RRTStar
    };

    void setUpperTimeBound(double tb)
    {
        timeBoundHigh_ = tb;
    }

    void setPlanner(Planners planner)
    {
        plannerType_ = planner;
    }

private:
    double xBoundLow_ = -2.0;
    double xBoundHigh_ = 2.0;
    double timeBoundLow_ = 0.0;
    double timeBoundHigh_ = -2.0;

    double initialTimeBoundFactor_ = 2;

    double xStart_ = 0.0;
    std::vector<std::pair<double, double>> goalRegions_ = {
            {1.0, 1.001},
            {-1.001, -1.0}
    };

    std::vector<Constraint> constraints_ {
            {0.4, 0.6, 0.0, 2.4},
            {0.4, 0.6, 3.0, 4.0},
            {-0.8, -0.6, 2.6, 3.0},
            {-0.4, -0.2, 0.401, 2.0}
    };
    double vMax_ = 1.0; // 1 m/s
    double timeWeight_ = 0.5; // compared to distance weight used in distance function. [0,1]
    double solveTime_ = 2.0; // in seconds
    double plannerRange_ = 0.2;
    Planners plannerType_ = SpaceTimeRRT;

    std::string filename_;
    std::string delim_ = ",";

    og::SimpleSetup createSimpleSetup();
    ompl::base::PlannerPtr createRRTConnect(const ompl::base::SpaceInformationPtr &si);
    ompl::base::PlannerPtr createRRTStar(const ompl::base::SpaceInformationPtr &si);
    ompl::base::PlannerPtr createSpaceTimeRRT(const ompl::base::SpaceInformationPtr &si);

    void RecordTimeSpaceTimeRRT(const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run);
    void RecordBestCost(const ob::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run);

    void writeSamplesToCSV(const ob::PlannerData &data);
    void writePathToCSV(const ob::PathPtr &);
    void writeConstraintsToCSV();
    void writeGoalRegionToCSV();

};
}




#endif //BT_ROBOTICS_TIME1DPLANNER_H
