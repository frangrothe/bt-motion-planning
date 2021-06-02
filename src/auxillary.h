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
#include "structs/Constraint2D.h"
#include "../json.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace auxillary {

const std::string delim_ = ",";

// Get current date/time, format is MM-DD.HH:mm:ss
std::string currentDateTime();

void writeSamplesToCSV1D(const ob::PlannerData &data, const std::string& filename);
void writeSamplesToCSV2D(const ob::PlannerData &data, const std::string& filename);
void writePathToCSV1D(const ob::PathPtr &, const std::string& filename);
void writePathToCSV2D(const ob::PathPtr &, const std::string& filename);
void writeConstraintsToCSV(const std::vector<Constraint>& constraints, const std::string& filename);
void writeConstraintsToJSON(const std::vector<Constraint2D>& constraints, const std::string& filename);
void writeGoalRegionToCSV1D(double xLb, double xUb, double tLb, double tUb, const std::string& filename);
void writeGoalToCSV2D(double x, double y, double tLow, double tHigh, const std::string &filename);
}

#endif //BT_ROBOTICS_AUXILLARY_H
