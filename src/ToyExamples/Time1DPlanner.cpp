//
// Created by francesco on 19.05.21.
//

#include <ompl/base/ProblemDefinition.h>
#include "Time1DPlanner.h"

namespace t1d {

Time1DPlanner::Time1DPlanner(std::string filename) : filename_(std::move(filename)) {}

void Time1DPlanner::planMotion() {

    /*
     * (1) OMPL: State space creation
     * State-Time Space
     *
     * space[0] : x-coordinate
     * space[1] : time
     */

    auto r1State(std::make_shared<ob::RealVectorStateSpace>(1));
    auto timeState(std::make_shared<ob::TimeStateSpace>());
    auto space = r1State + timeState;

    // Set the bounds for R1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(xBoundLow_);
    bounds.setHigh(xBoundHigh_);
    r1State->setBounds(bounds);

    // Set the bounds for Time
    timeState->setBounds(timeBoundLow_, timeBoundHigh_);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<Time1DStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<Time1DMotionValidator>(si, maxSpeed_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    ob::ScopedState<> start(space);
    start[0] = xStart_; // r1State

    minTime_ = (xGoalRegionLeft_ - xStart_) / maxSpeed_; // minimum time at which the goal can be reached
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<Time1DGoalRegion>(si, xGoalRegionLeft_, xGoalRegionRight_, minTime_, timeBoundHigh_));

    //(3) Planner
//    auto planner(std::make_shared<og::RRTConnect>(si, true));
    auto planner(std::make_shared<TimeRRT>(si, maxSpeed_));
    planner->setRange(plannerRange_);
    planner->setProblemDefinition(pdef);
    planner->setup();

    //(4) Planner executes
    ob::PlannerStatus solved = planner->ob::Planner::solve(solveTime_);

    if (solved)
    {
        ob::PlannerData data(si);
        planner->getPlannerData(data);

        std::cout << "\nFound solution:" << std::endl;

        // print the path to screen
        ob::PathPtr path = pdef->getSolutionPath();
        path->print(std::cout);

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (pdef->hasExactSolution()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (pdef->hasApproximateSolution()? "yes" : "no") << std::endl;

        writeSamplesToCSV(data);
        writePathToCSV(path);
        writeConstraintsToCSV();
        writeGoalRegionToCSV();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void Time1DPlanner::writeSamplesToCSV(const ob::PlannerData &data) {

    std::ofstream outfile ("data/" + filename_ + "/samples.csv");

    outfile << "x" << csvDelim_ << "time" << csvDelim_ << "incoming edge" << csvDelim_ << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        std::stringstream ssIn;
        for (int j = 0; j < inEdgeIndexes.size(); ++j) {
            if (j < inEdgeIndexes.size() - 1) {
                ssIn << inEdgeIndexes.at(j) << "#";
            }
            else {
                ssIn << inEdgeIndexes.at(j);
            }
        }
        std::string inEdges = inEdgeIndexes.empty()? "-1" : ssIn.str();

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ssOut;
        for (int j = 0; j < outEdgeIndexes.size(); ++j) {
            if (j < outEdgeIndexes.size() - 1) {
                ssOut << outEdgeIndexes.at(j) << "#";
            }
            else {
                ssOut << outEdgeIndexes.at(j);
            }
        }
        std::string outEdges = outEdgeIndexes.empty()? "-1" : ssOut.str();

        // write node data to csv
        outfile << x << csvDelim_ << t << csvDelim_ << inEdges << csvDelim_ << outEdges << "\n";

    }

    outfile.close();
}

void Time1DPlanner::writePathToCSV(const ob::PathPtr &pathPointer) {
    std::ofstream outfile ("data/" + filename_ + "/path.csv");

    outfile << "x" << csvDelim_ << "time\n";

    auto path = pathPointer->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << csvDelim_ << t << "\n";
    }

    outfile.close();
}

void Time1DPlanner::writeConstraintsToCSV() {

    std::ofstream outfile ("data/" + filename_ + "/constraints.csv");

    outfile << "x lb" << csvDelim_ << "x ub" << csvDelim_ << "t lb" << csvDelim_ << "t ub\n";
    for (auto c : constraints_) {
        outfile << c.x_lb << csvDelim_ << c.x_ub << csvDelim_ << c.t_lb << csvDelim_ << c.t_ub << "\n";
    }

    outfile.close();
}

void Time1DPlanner::writeGoalRegionToCSV() {
    std::ofstream outfile ("data/" + filename_ + "/goal.csv");

    outfile << "x lb" << csvDelim_ << "x ub" << csvDelim_ << "t lb" << csvDelim_ << "t ub\n";
    outfile << xGoalRegionLeft_ << csvDelim_ << xGoalRegionRight_ << csvDelim_ << minTime_ << csvDelim_ << timeBoundHigh_ << "\n";

    outfile.close();
}
}

