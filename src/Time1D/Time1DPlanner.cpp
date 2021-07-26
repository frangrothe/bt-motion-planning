//
// Created by francesco on 19.05.21.
//


#include "Time1DPlanner.h"

namespace time_1d {

Time1DPlanner::Time1DPlanner(std::string filename) : filename_(std::move(filename)) {}

void Time1DPlanner::planMotion() {

    /*
     * (1) OMPL: State space creation
     * State-Time Space
     *
     * space[0] : x-coordinate
     * space[1] : time
     */

//    auto r1State(std::make_shared<ob::RealVectorStateSpace>(1));
//    auto timeState(std::make_shared<ob::TimeStateSpace>());
//    auto space = r1State + timeState;

    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(1);
    auto space = std::make_shared<space_time::AnimationStateSpace>(vectorSpace, vMax_, timeWeight_);

    // Set the bounds for R1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(xBoundLow_);
    bounds.setHigh(xBoundHigh_);
    vectorSpace->setBounds(bounds);

    // Set the bounds for Time
//    space->setTimeBounds(timeBoundLow_, timeBoundHigh_);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<Time1DStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<Time1DMotionValidator>(si, vMax_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    ob::ScopedState<> start(space);
    start[0] = xStart_; // r1State

    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<VectorSpaceGoalRegion>(si, goalRegions_));

    //(3) Planner
//    auto planner(std::make_shared<og::RRTConnect>(si));
    auto planner(std::make_shared<space_time::SpaceTimeRRT>(si));
    planner->setRange(plannerRange_);
    planner->setRewiringToKNearest();
    planner->setBatchSize(100);
//    planner->setOptimize(false);
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

    outfile << "x" << delim_ << "time" << delim_ << "incoming edge" << delim_ << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        std::stringstream ssIn;
        for (auto edgeIndex : inEdgeIndexes) {
            ssIn << "#" << edgeIndex;
        }
        std::string inEdges = inEdgeIndexes.empty()? "#" : ssIn.str();

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ssOut;
        for (auto edgeIndex : outEdgeIndexes) {
            ssOut << "#" << edgeIndex;
        }
        std::string outEdges = outEdgeIndexes.empty()? "#" : ssOut.str();

        // write node data to csv
        outfile << x << delim_ << t << delim_ << inEdges << delim_ << outEdges << "\n";

    }

    outfile.close();
}

void Time1DPlanner::writePathToCSV(const ob::PathPtr &pathPointer) {
    std::ofstream outfile ("data/" + filename_ + "/path.csv");

    outfile << "x" << delim_ << "time\n";

    auto path = pathPointer->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << delim_ << t << "\n";
    }

    outfile.close();
}

void Time1DPlanner::writeConstraintsToCSV() {

    std::ofstream outfile ("data/" + filename_ + "/constraints.csv");

    outfile << "x lb" << delim_ << "x ub" << delim_ << "t lb" << delim_ << "t ub\n";
    for (auto c : constraints_) {
        outfile << c.x_lb << delim_ << c.x_ub << delim_ << c.t_lb << delim_ << c.t_ub << "\n";
    }

    outfile.close();
}

void Time1DPlanner::writeGoalRegionToCSV() {
    std::ofstream outfile ("data/" + filename_ + "/goal.csv");

    outfile << "x lb" << delim_ << "x ub" << delim_ << "t lb" << delim_ << "t ub\n";
    for (auto &g : goalRegions_) {
        double minTime = std::fabs((g.first - xStart_)) / vMax_; // minimum time at which the goal can be reached
        outfile << g.first << delim_ << g.second << delim_ << minTime << delim_ << timeBoundHigh_ << "\n";
    }


    outfile.close();
}
}

