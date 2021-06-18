//
// Created by francesco on 27.05.21.
//

#include "SpaceTimePlanner.h"

namespace time_2d {

SpaceTimePlanner::SpaceTimePlanner(std::string filename) : filename_(std::move(filename)) {}

void SpaceTimePlanner::planMotion() {

    //(1) Define State Space
    auto space = std::make_shared<space_time::AnimationStateSpace>(2);
    // Set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, xBoundLow_);
    bounds.setHigh(0, xBoundHigh_);
    bounds.setLow(1, yBoundLow_);
    bounds.setHigh(1, yBoundHigh_);
    space->setVectorBounds(bounds);
    space->setTimeBounds(timeBoundLow_, timeBoundHigh_);
    for (auto &c : constraints_) {
        c.setBounds(xBoundLow_, xBoundHigh_, yBoundLow_, yBoundHigh_);
    }

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<SpaceTimeStateValidityChecker>(si, constraints_));
    si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si, 2, vMax_));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    ob::ScopedState<> start(space);
    start[0] = xStart_; // r1State
    start[1] = yStart_;

    minTime_ = sqrt(pow(fabs(xGoal_ - xStart_), 2) + pow(fabs(yGoal_ - yStart_), 2)) / vMax_; // minimum time at which the goal can be reached
    pdef->addStartState(start);
    pdef->setGoal(std::make_shared<SpaceTimeGoalRegion>(si, xGoal_, yGoal_, minTime_, timeBoundHigh_));

    //(3) Planner
    auto planner(std::make_shared<og::RRTConnect>(si));
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

        auxillary::writeSamplesToCSV2D(data, filename_);
        auxillary::writePathToCSV2D(path, filename_);
        auxillary::writeConstraintsToJSON(constraints_, filename_);
        auxillary::writeGoalToCSV2D(xGoal_, yGoal_, timeBoundLow_, timeBoundHigh_, filename_);
//        auxillary::writePathToCSV1D(path, filename_);
//        auxillary::writeConstraintsToCSV(constraints_, filename_);
//        auxillary::writeGoalRegionToCSV1D(xGoalRegionLeft_, xGoalRegionRight_, minTime_, timeBoundHigh_, filename_);
    }
    else
        std::cout << "No solution found" << std::endl;

}

void SpaceTimePlanner::test() {
}
}