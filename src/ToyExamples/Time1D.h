//
// Created by francesco on 12.05.21.
//

#ifndef BT_ROBOTICS_TIME1D_H
#define BT_ROBOTICS_TIME1D_H
#include <iostream>
#include <fstream>


#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include <ompl/geometric/SimpleSetup.h>
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerDataGraph.h"

#include "../auxillary.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace tb1d {

bool isStateValid(const ob::State *state);
void plan(const std::string&);
void writeToCSV(const ob::PlannerData &data, const std::string&);

}

#endif //BT_ROBOTICS_TIME1D_H
