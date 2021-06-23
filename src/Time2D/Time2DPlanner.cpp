//
// Created by francesco on 27.05.21.
//

#include "Time2DPlanner.h"

namespace time_2d {

Time2DPlanner::Time2DPlanner(std::string filename) : filename_(std::move(filename)) {}

void Time2DPlanner::planMotion() {

    //(1) Define State Space
    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    auto stateSpace = std::make_shared<space_time::AnimationStateSpace>(vectorSpace);

    // Set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, xBoundLow_);
    bounds.setHigh(0, xBoundHigh_);
    bounds.setLow(1, yBoundLow_);
    bounds.setHigh(1, yBoundHigh_);

    vectorSpace->setBounds(bounds);
//    stateSpace->getSpaceComponent()->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    stateSpace->setTimeBounds(timeBoundLow_, timeBoundHigh_);
    for (auto &c : constraints_) {
        c.setBounds(xBoundLow_, xBoundHigh_, yBoundLow_, yBoundHigh_);
    }

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(stateSpace);
    si->setStateValidityChecker(std::make_shared<Time2DStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<Time2DMotionValidator>(si, 2, vMax_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    ob::ScopedState<> start(stateSpace);
    start[0] = xStart_; // r1State
    start[1] = yStart_;

    minTime_ = sqrt(pow(fabs(xGoal_ - xStart_), 2) + pow(fabs(yGoal_ - yStart_), 2)) / vMax_; // minimum time at which the goal can be reached
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<Time2DGoalRegion>(si, xGoal_, yGoal_, minTime_, timeBoundHigh_));

    //(3) Planner
//    auto planner(std::make_shared<og::RRTConnect>(si));
    auto planner(std::make_shared<space_time::SpaceTimeRRT>(si));
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
        writeConstraintsToJSON();
        writeGoalRegionToCSV();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void Time2DPlanner::test() {
}

void Time2DPlanner::writeSamplesToCSV(const ob::PlannerData &data) {
    std::ofstream outfile ("data/" + filename_ + "/samples.csv");

    outfile << "x" << delim_ << "y" << delim_ << "time" << delim_ << "incoming edge" << delim_ << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double y = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
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
        outfile << x << delim_ << y << delim_ << t << delim_ << inEdges << delim_ << outEdges << "\n";

    }

    outfile.close();
}

void Time2DPlanner::writePathToCSV(const ompl::base::PathPtr &pathPtr) {
    std::ofstream outfile ("data/" + filename_ + "/path.csv");

    outfile << "x" << delim_ << "y" << delim_ << "time\n";

    auto path = pathPtr->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << delim_ << y << delim_ << t << "\n";
    }

    outfile.close();
}

void Time2DPlanner::writeConstraintsToJSON() {
    using json = nlohmann::json;
    json j;

    int n = 20; // number of time samples to generate planes for visualization
    double tLow = 0.0;
    double tHigh = 2.5;

    double tDiff = (tHigh - tLow) / (n - 1);
    std::vector<double> ts(n);
    std::generate(ts.begin(), ts.end(), [i = 0, &tLow, &tDiff] () mutable { return tLow + i++ * tDiff; });

    std::vector<std::vector<std::vector<std::vector<double>>>> jsonVector;
    for (const auto &c : constraints_) {
        std::vector<std::vector<std::vector<double>>> cVector;
        for (auto t : ts) {
            double xlb, xub, ylb, yub;
            std::tie(xlb, xub, ylb, yub) = c.getBoundsForTime(t);
            std::vector<std::vector<double>> v {
                    {xlb, yub, t},
                    {xlb, ylb, t},
                    {xub, ylb, t},
                    {xub, yub, t}
            };
            cVector.push_back(v);
        }
        jsonVector.push_back(cVector);
    }

    j = jsonVector;
    // write prettified JSON to another file
    std::ofstream outfile("data/" + filename_ + "/constraints.json");
    outfile << std::setw(4) << j << std::endl;

}

void Time2DPlanner::writeGoalRegionToCSV() {
    std::ofstream outfile ("data/" + filename_ + "/goal.csv");

    outfile << "x" << delim_ << "y" << delim_ << "tLow" << delim_ << "tHigh\n";
    outfile << xGoal_ << delim_ << yGoal_ << delim_ << timeBoundLow_ << delim_ << timeBoundHigh_ << "\n";

    outfile.close();
}
}