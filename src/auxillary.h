//
// Created by francesco on 17.05.21.
//

#ifndef BT_ROBOTICS_AUXILLARY_H
#define BT_ROBOTICS_AUXILLARY_H

#include <iostream>
#include <string>
#include <iomanip>
#include <ctime>
#include <sstream>

#include <ompl/base/PlannerData.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "structs/Constraint.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace auxillary {

const std::string delim_ = ",";

// Get current date/time, format is MM-DD.HH:mm:ss
std::string currentDateTime();

void writeSamplesToCSV(const ob::PlannerData &data, const std::string& filename);
void writePathToCSV(const ob::PathPtr &, const std::string& filename);
void writeConstraintsToCSV(const std::vector<Constraint>& constraints, const std::string& filename);
void writeGoalRegionToCSV(double xLb, double xUb, double tLb, double tUb, const std::string& filename);
}

#endif //BT_ROBOTICS_AUXILLARY_H
