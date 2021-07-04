//
// Created by francesco on 27.05.21.
//

#include "Time2DPlanner.h"

namespace time_2d {

Time2DPlanner::Time2DPlanner(std::string filename) : filename_(std::move(filename)) {}

void Time2DPlanner::planMotion() {

    //(1) Define State Space
    auto vectorSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    auto stateSpace = std::make_shared<space_time::AnimationStateSpace>(vectorSpace, vMax_, timeWeight_);

    // Set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, xBoundLow_);
    bounds.setHigh(0, xBoundHigh_);
    bounds.setLow(1, yBoundLow_);
    bounds.setHigh(1, yBoundHigh_);

    vectorSpace->setBounds(bounds);
    stateSpace->setTimeBounds(timeBoundLow_, timeBoundHigh_);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(stateSpace);
    si->setStateValidityChecker(std::make_shared<Time2DStateValidityChecker>(si, bounds, constraints_, width_, height_));
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
        writeBoundsToJSON();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void Time2DPlanner::loadConfiguration1() {

    xBoundLow_ = 0.0;
    xBoundHigh_ = 10.0;
    yBoundLow_ = 0.0;
    yBoundHigh_ = 10.0;
    timeBoundLow_ = 0.0;
    timeBoundHigh_ = 30.0;

    xStart_ = 0.5;
    yStart_ = 5.0;
    xGoal_ = 9.5;
    yGoal_ = 5.0;
    width_ = 0.25;
    height_ = 0.25;

    // 4 rectangles moving for the first 10 seconds. The 1st and 3rd moving up, the 2nd and 4th moving down.
    constraints_ = {
            Constraint{[](double x, double y, double t) {
                if (t > 10) t = 10.0;
                return std::make_tuple(x, y + 0.5 * t);
            }, 2.0, 3.0, 0.5, 2.0},
            Constraint{[](double x, double y, double t) {
                if (t > 10) t = 10.0;
                return std::make_tuple(x, y - 0.5 * t);
            }, 4.0,  8.0,  0.5,  2.0},
            Constraint{[](double x, double y, double t) {
                if (t > 10) t = 10.0;
                return std::make_tuple(x, y + 0.5 * t);
            }, 6.0,  1.0,  0.5,  2.0},
            Constraint{[](double x, double y, double t) {
                if (t > 10) t = 10.0;
                if (t <= 1) return std::make_tuple(x, y);
                return std::make_tuple(x, y - 0.5 * (t - 1));
            }, 8.0,  9.0,  0.5,  2.0}
    };

    vMax_ = 1.0;
    solveTime_ = 1.0;
    timeWeight_ = 0.5;
    plannerRange_ = 0.5;
}

void Time2DPlanner::loadConfiguration2() {

    xBoundLow_ = -1.0;
    xBoundHigh_ = 21.0;
    yBoundLow_ = 0.0;
    yBoundHigh_ = 2.0;
    timeBoundLow_ = 0.0;
    timeBoundHigh_ = 15.0;

    xStart_ = 1.0;
    yStart_ = 1.5;
    xGoal_ = 19.0;
    yGoal_ = 1.5;
    width_ = 2.0;
    height_ = 0.8;

    // 2 rectangles moving, the first from left to right in front of the agent,
    // the 2nd in the opposite direction towards the agent
    double xgoal = xGoal_;
    double xstart = xStart_;
    constraints_ = {
            Constraint{[xgoal](double x, double y, double t) {
                auto newX = std::min(xgoal, x + t);
                return std::make_tuple(newX, y);
            }, 4.0, 1.5, 2.0, 0.8},
            Constraint{[xstart](double x, double y, double t) {
                auto newX = std::max(xstart, x - t);
                return std::make_tuple(newX, y);
            }, 19.0, 0.5, 2.0, 0.8}
    };

    vMax_ = 2.5;
    solveTime_ = 2.0;
    timeWeight_ = 0.5;
    plannerRange_ = 0.5;
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
    double tLow = timeBoundLow_;

    double tDiff = (timeBoundHigh_ - timeBoundLow_) / (n - 1);
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

void Time2DPlanner::writeBoundsToJSON() {
    using json = nlohmann::json;
    json j;
    j["x"] = {xBoundLow_, xBoundHigh_};
    j["y"] = {yBoundLow_, yBoundHigh_};
    j["t"] = {timeBoundLow_, timeBoundHigh_};

    // write prettified JSON to another file
    std::ofstream outfile("data/" + filename_ + "/bounds.json");
    outfile << std::setw(4) << j << std::endl;
}
}