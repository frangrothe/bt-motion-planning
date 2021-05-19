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
    ob::GoalPtr goal = std::make_shared<Time1DGoal>(si, xGoal_);

    pdef->addStartState(start);
    pdef->setGoal(goal);

    //(3) Planner
    auto planner(std::make_shared<og::RRT>(si));
    planner->setGoalBias(goalBias_); // with probability x the goal will be sampled
    planner->setProblemDefinition(pdef);
    planner->setup();

    //(4) Planner executes
    ob::PlannerStatus solved = planner->ob::Planner::solve(solveTime_);

    if (solved)
    {
        ob::PathPtr path = pdef->getSolutionPath();

        ob::PlannerData data(si);
        planner->getPlannerData(data);

        std::cout << "\nFound solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (pdef->hasExactSolution()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (pdef->hasApproximateSolution()? "yes" : "no") << std::endl;

        writeResultToCSV(data);
        writeConstraintsToCSV();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void Time1DPlanner::writeResultToCSV(const ob::PlannerData &data) {

    std::string delim = ","; // csv separator
    std::ofstream outfile ("data/" + filename_ + ".csv");

    outfile << "x" << delim << "time" << delim << "incoming edge" << delim << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        long int inEdge = inEdgeIndexes.empty()? -1 : (long int) inEdgeIndexes.at(0);

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ss;
        for (int j = 0; j < outEdgeIndexes.size(); ++j) {
            if (j < outEdgeIndexes.size() - 1) {
                ss << outEdgeIndexes.at(j) << "#";
            }
            else {
                ss << outEdgeIndexes.at(j);
            }
        }
        std::string outEdges = outEdgeIndexes.empty()? "-1" : ss.str();

        // write node data to csv
        outfile << x << delim << t << delim << inEdge << delim << outEdges << "\n";

    }

    outfile.close();
}

void Time1DPlanner::writeConstraintsToCSV() {

    std::string delim = ","; // csv separator
    std::ofstream outfile ("data/" + filename_ + "_constraints.csv");

    outfile << "x lb" << delim << "x ub" << delim << "t lb" << delim << "t_ub\n";
    for (auto c : constraints_) {
        outfile << c.x_lb << delim << c.x_ub << delim << c.t_lb << delim << c.t_ub << "\n";
    }

    outfile.close();
}
}

